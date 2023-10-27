// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Google LLC
 * Author: Mostafa Saleh <smostafa@google.com>
 */
#include <linux/of_platform.h>
#include <linux/arm-smccc.h>
#include <linux/iommu.h>
#include <linux/pci.h>
#include <linux/rbtree.h>

#define FEAUTRE_PGSIZE_BITMAP		0x1

struct pviommu {
	struct iommu_device		iommu;
	u32				id;
	u32				version;
	u32				pgsize_bitmap;
};

struct pviommu_mapping {
	u64				iova;
	u64				ipa;
	u64				size; /* bytes*/
	struct rb_node			node;
};

struct pviommu_domain {
	struct pviommu			*pv;
	struct iommu_domain		domain;
	unsigned long			id; /* pKVM domain ID. */
	struct mutex			mutex;
	struct rb_root			root; /* IOVA => IPA map*/
};

struct pviommu_master {
	struct device			*dev;
	struct pviommu			*iommu;
	struct pviommu_domain		*domain;
	u32				ssid_bits;
};

/*
 * Returns upper bound
 */
static struct pviommu_mapping *pviommu_find_mapping(phys_addr_t iova,
						    struct pviommu_domain *domain)
{
	struct pviommu_mapping *this, *up = NULL;
	struct rb_node *root = domain->root.rb_node;

	while (root) {
		this = container_of(root, struct pviommu_mapping, node);
		if (this->iova < iova) {
			up = this;
			root = root->rb_left;
		} else if (this->iova > iova) {
			root = root->rb_right;
		} else {
			return this;
		}
	}
	return up;
}

static int pviommu_insert_mapping(phys_addr_t iova, phys_addr_t ipa, u64 size,
				  struct pviommu_domain *domain)
{
	struct pviommu_mapping *this;
	struct rb_node **new = &domain->root.rb_node;
	struct rb_node *parent = NULL;

	while (*new) {
		this = container_of(*new, struct pviommu_mapping, node);
		parent = *new;
		if (this->iova + this->size <= iova)
			new = &((*new)->rb_left);
		else if (this->iova >= iova + size)
			new = &((*new)->rb_right);
		else
			return -EEXIST;
	}
	this = kzalloc(sizeof(*this), GFP_KERNEL);
	if (!this)
		return -ENOMEM;

	this->iova = iova;
	this->ipa = ipa;
	this->size = size;
	rb_link_node(&this->node, parent, new);
	rb_insert_color(&this->node, &domain->root);

	return 0;
}

static int pviommu_remove_mapping(phys_addr_t iova, u64 size,
				  struct pviommu_domain *domain)
{
	struct pviommu_mapping *this = pviommu_find_mapping(iova, domain);
	struct pviommu_mapping temp;
	u64 shift;
	int ret = 0;

	if (!this || (this->iova + this->size <= iova))
		return -ENOENT;

	temp = *this;
	rb_erase(&this->node, &domain->root);
	kfree(this);
	/*
	 * Now we removed the existing entry which may have been intersecting
	 * for small part only which mean we need to re-insert this range.
	 */
	if (iova + size < temp.iova + temp.size) {
		shift = iova + size - temp.iova;
		ret = pviommu_insert_mapping(temp.iova + shift, temp.ipa + shift,
					     temp.size - shift, domain);
	}
	if (ret)
		return ret;
	if (temp.iova < iova) {
		ret = pviommu_insert_mapping(temp.iova,  temp.ipa,
					     iova - temp.iova, domain);
	}
	if (ret)
		return ret;
	/* If found range was shorter. */
	if (iova + size > temp.iova + temp.size) {
		ret = pviommu_remove_mapping(temp.iova + temp.size, iova + size
					     - temp.iova - temp.size, domain);
	}
	if (ret)
		return ret;

	return 0;
}

static int pviommu_map_pages(struct iommu_domain *domain, unsigned long iova,
			     phys_addr_t paddr, size_t pgsize, size_t pgcount,
			     int prot, gfp_t gfp, size_t *mapped)
{
	int ret;
	struct pviommu_domain *pv_domain = container_of(domain, struct pviommu_domain, domain);
	struct arm_smccc_res res;

	arm_smccc_1_1_hvc(ARM_SMCCC_VENDOR_HYP_KVM_IOMMU_MAP_FUNC_ID,
			  pv_domain->id, iova, paddr, pgsize, pgcount, prot, &res);
	ret = res.a0;
	if (ret) {
		pr_err("Failed to map pages %llx => %lx %d\n", paddr, iova, ret);
		*mapped = 0;
		return ret;
	}

	*mapped = pgsize * pgcount;
	ret = pviommu_insert_mapping(iova, paddr, pgsize * pgcount, pv_domain);

	return ret;
}

static size_t pviommu_unmap_pages(struct iommu_domain *domain, unsigned long iova,
				  size_t pgsize, size_t pgcount,
				  struct iommu_iotlb_gather *gather)
{
	int ret;
	struct pviommu_domain *pv_domain = container_of(domain, struct pviommu_domain, domain);
	struct arm_smccc_res res;

	arm_smccc_1_1_hvc(ARM_SMCCC_VENDOR_HYP_KVM_IOMMU_UNMAP_FUNC_ID,
			  pv_domain->id, iova, pgsize, pgcount, &res);
	ret = res.a0;
	if (ret) {
		pr_err("Failed to unmap pages %lx ret %d\n", iova, ret);
		return ret;
	}

	ret = pviommu_remove_mapping(iova, pgsize * pgcount, pv_domain);
	if (ret)
		pr_err("pviommu: unmap: failed to remove %d, iova %lx pgcount %lx",
		       ret, iova, pgcount);

	return pgsize * pgcount;
}

static phys_addr_t pviommu_iova_to_phys(struct iommu_domain *domain, dma_addr_t iova)
{
	struct pviommu_mapping *ret;
	struct pviommu_domain *pv_domain = container_of(domain, struct pviommu_domain, domain);
	phys_addr_t ipa;

	ret = pviommu_find_mapping(iova, pv_domain);

	if (!ret || (ret->iova + ret->size <= iova)) {
		pr_err("pviommu_iova_to_phys for a non existing IOVA %llx\n", iova);
		return 0;
	}
	ipa = ret->iova - iova + ret->ipa;
	return ipa;
}

static void pviommu_domain_free(struct iommu_domain *domain)
{
	struct pviommu_domain *pv_domain = container_of(domain, struct pviommu_domain, domain);
	struct pviommu *pv = pv_domain->pv;
	struct arm_smccc_res res;

	arm_smccc_1_1_hvc(ARM_SMCCC_VENDOR_HYP_KVM_IOMMU_FREE_DOMAIN_FUNC_ID,
			  pv->id, pv_domain->id, &res);
	if (res.a0 < 0)
		pr_err("domain_free failed for, ret %lu\n", res.a0);

	kfree(pv_domain);
}

static int pviommu_domain_finalize(struct pviommu_domain *pv_domain,
				   struct pviommu_master *master)
{
	int ret;
	struct arm_smccc_res res;

	arm_smccc_1_1_hvc(ARM_SMCCC_VENDOR_HYP_KVM_IOMMU_ALLOC_DOMAIN_FUNC_ID,
			  master->iommu->id, &res);
	ret = res.a0;
	pv_domain->id = res.a1;
	pv_domain->pv = master->iommu;

	return ret;
}

static int pviommu_attach_dev(struct iommu_domain *domain, struct device *dev)
{
	int ret = 0, i;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct pviommu *pv;
	struct pviommu_domain *pv_domain = container_of(domain, struct pviommu_domain, domain);
	struct pviommu_master *master;
	struct arm_smccc_res res;
	u32 sid;

	if (!fwspec)
		return -ENOENT;

	master = dev_iommu_priv_get(dev);
	pv = master->iommu;
	master->domain = pv_domain;

	mutex_lock(&pv_domain->mutex);
	if (!pv_domain->pv) {
		ret = pviommu_domain_finalize(pv_domain, master);
	}
	else if (pv_domain->pv != pv) {
		ret = -EINVAL;
	}
	mutex_unlock(&pv_domain->mutex);

	if (ret) {
		dev_err(dev, "Can't finalize IOMMU domain\n");
		return ret;
	}

	for (i = 0; i < fwspec->num_ids; i++) {
		sid = fwspec->ids[i];
		arm_smccc_1_1_hvc(ARM_SMCCC_VENDOR_HYP_KVM_IOMMU_ATTACH_DEV_FUNC_ID,
				  pv->id, sid, 0 /* PASID */,
				  pv_domain->id, master->ssid_bits, &res);
		ret = res.a0;
		if (ret) {
			dev_err(dev, "Failed to attach_dev with sid %d to IOMMU, err %d\n",
				sid, ret);
			return ret;
		}
	}

	return 0;
}

static void pviommu_detach_dev(struct pviommu_master *master)
{
	int ret = 0, i;
	struct device *dev = master->dev;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct pviommu *pv = master->iommu;
	struct pviommu_domain *pv_domain = master->domain;
	struct arm_smccc_res res;
	u32 sid;

	if (!fwspec)
		return;

	for (i = 0; i < fwspec->num_ids; i++) {
		sid = fwspec->ids[i];
		arm_smccc_1_1_hvc(ARM_SMCCC_VENDOR_HYP_KVM_IOMMU_DETACH_DEV_FUNC_ID,
				  pv->id, sid, 0, pv_domain->id, &res);
		ret = res.a0;
		if (ret)
			dev_err(dev, "Failed to detach_dev with sid %d to IOMMU, err %d\n",
				sid, ret);
	}
}

static struct iommu_domain *pviommu_domain_alloc(unsigned int type)
{
	struct pviommu_domain *pv_domain;

	if (type != IOMMU_DOMAIN_UNMANAGED &&
	    type != IOMMU_DOMAIN_DMA &&
	    type != IOMMU_DOMAIN_IDENTITY)
		return NULL;

	pv_domain = kzalloc(sizeof(*pv_domain), GFP_KERNEL);
	if (!pv_domain)
		return NULL;

	mutex_init(&pv_domain->mutex);
	pv_domain->root = RB_ROOT;

	return &pv_domain->domain;
}

static struct platform_driver pkvm_pviommu_driver;

static struct pviommu *pviommu_get_by_fwnode(struct fwnode_handle *fwnode)
{
	struct device *dev = driver_find_device_by_fwnode(&pkvm_pviommu_driver.driver, fwnode);

	put_device(dev);
	return dev ? dev_get_drvdata(dev) : NULL;
}

static struct iommu_ops pviommu_ops;

static struct iommu_device *pviommu_probe_device(struct device *dev)
{
	struct pviommu_master *master;
	struct pviommu *pv = NULL;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	if (!fwspec || fwspec->ops != &pviommu_ops)
		return ERR_PTR(-ENODEV);

	pv = pviommu_get_by_fwnode(fwspec->iommu_fwnode);
	if (!pv)
		return ERR_PTR(-ENODEV);

	master = kzalloc(sizeof(*master), GFP_KERNEL);
	if (!master)
		return ERR_PTR(-ENOMEM);

	master->dev = dev;
	master->iommu = pv;
	device_property_read_u32(dev, "pasid-num-bits", &master->ssid_bits);
	dev_iommu_priv_set(dev, master);

	return &pv->iommu;
}

static void pviommu_release_device(struct device *dev)
{
	struct pviommu_master *master = dev_iommu_priv_get(dev);

	pviommu_detach_dev(master);
}

static int pviommu_of_xlate(struct device *dev, struct of_phandle_args *args)
{
	return iommu_fwspec_add_ids(dev, args->args, 1);
}

static struct iommu_group *pviommu_device_group(struct device *dev)
{
	if (dev_is_pci(dev))
		return pci_device_group(dev);
	else
		return generic_device_group(dev);
}

static struct iommu_ops pviommu_ops = {
	.device_group		= pviommu_device_group,
	.of_xlate		= pviommu_of_xlate,
	.probe_device		= pviommu_probe_device,
	.release_device		= pviommu_release_device,
	.domain_alloc		= pviommu_domain_alloc,
	.owner			= THIS_MODULE,
	.default_domain_ops = &(const struct iommu_domain_ops) {
		.attach_dev	= pviommu_attach_dev,
		.map_pages	= pviommu_map_pages,
		.unmap_pages	= pviommu_unmap_pages,
		.iova_to_phys	= pviommu_iova_to_phys,
		.free		= pviommu_domain_free,
	}
};

static int pviommu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pviommu *pv = devm_kmalloc(dev, sizeof(*pv), GFP_KERNEL);
	struct device_node *np = pdev->dev.of_node;
	int ret;
	struct arm_smccc_res res;

	ret = of_property_read_u32_index(np, "id", 0, &pv->id);
	if (ret) {
		dev_err(dev, "Error reading id from device tree node\n");
		return ret;
	}

	arm_smccc_1_1_hvc(ARM_SMCCC_VENDOR_HYP_KVM_IOMMU_GET_FEATURE_FUNC_ID,
			  pv->id, FEAUTRE_PGSIZE_BITMAP, &res);
	pv->pgsize_bitmap = pviommu_ops.pgsize_bitmap = res.a0;
	WARN_ON(pv->pgsize_bitmap == SMCCC_RET_NOT_SUPPORTED);

	arm_smccc_1_1_hvc(ARM_SMCCC_VENDOR_HYP_KVM_IOMMU_VERSION_FUNC_ID, &res);
	pv->version = res.a0;
	WARN_ON(pv->version != 0x1000);

	ret = iommu_device_sysfs_add(&pv->iommu, dev, NULL,
				     "pviommu.%pa", &pv->id);
	if (ret) {
		dev_err(dev, "Couldn't add to sysfs %d\n", ret);
		return ret;
	}

	ret = iommu_device_register(&pv->iommu, &pviommu_ops, dev);
	if (ret) {
		dev_err(dev, "Couldn't register %d\n", ret);
		iommu_device_sysfs_remove(&pv->iommu);
	}

	platform_set_drvdata(pdev, pv);

	return ret;
}

static const struct of_device_id pviommu_of_match[] = {
	{ .compatible = "pkvm,pviommu", },
	{ },
};

static struct platform_driver pkvm_pviommu_driver = {
	.probe = pviommu_probe,
	.driver = {
		.name = "pkvm-pviommu",
		.of_match_table = pviommu_of_match,
	},
};

module_platform_driver(pkvm_pviommu_driver);

MODULE_DESCRIPTION("IOMMU API for pKVM paravirtualized IOMMU");
MODULE_AUTHOR("Mostafa Saleh <smostafa@google.com>");
MODULE_LICENSE("GPL v2");
