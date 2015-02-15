/* bnx2x_cmn.c: Broadcom Everest network driver.
 *
 * Copyright (c) 2007-2012 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 *
 * Maintained by: Eilon Greenstein <eilong@broadcom.com>
 * Written by: Eliezer Tamir
 * Based on code from Michael Chan's bnx2 driver
 * UDP CSUM errata workaround by Arik Gendelman
 * Slowpath and fastpath rework by Vladislav Zolotarov
 * Statistics and Link management by Yitchak Gertner
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <net/ipv6.h>
#include <net/ip6_checksum.h>
#include <linux/firmware.h>
#include <linux/prefetch.h>
#include "bnx2x_cmn.h"
#include "bnx2x_init.h"
#include "bnx2x_sp.h"



/**
 * bnx2x_move_fp - move content of the fastpath structure.
 *
 * @bp:		driver handle
 * @from:	source FP index
 * @to:		destination FP index
 *
 * Makes sure the contents of the bp->fp[to].napi is kept
 * intact. This is done by first copying the napi struct from
 * the target to the source, and then mem copying the entire
 * source onto the target
 */
static inline void bnx2x_move_fp(struct bnx2x *bp, int from, int to)
{
	struct bnx2x_fastpath *from_fp = &bp->fp[from];
	struct bnx2x_fastpath *to_fp = &bp->fp[to];

	/* Copy the NAPI object as it has been already initialized */
	from_fp->napi = to_fp->napi;

	/* Move bnx2x_fastpath contents */
	memcpy(to_fp, from_fp, sizeof(*to_fp));
	to_fp->index = to;
}

int load_count[2][3] = { {0} }; /* per-path: 0-common, 1-port0, 2-port1 */

/* free skb in the packet ring at pos idx
 * return idx of last bd freed
 */
static u16 bnx2x_free_tx_pkt(struct bnx2x *bp, struct bnx2x_fp_txdata *txdata,
			     u16 idx, unsigned int *pkts_compl,
			     unsigned int *bytes_compl)
{
	struct sw_tx_bd *tx_buf = &txdata->tx_buf_ring[idx];
	struct eth_tx_start_bd *tx_start_bd;
	struct eth_tx_bd *tx_data_bd;
	struct sk_buff *skb = tx_buf->skb;
	u16 bd_idx = TX_BD(tx_buf->first_bd), new_cons;
	int nbd;

	/* prefetch skb end pointer to speedup dev_kfree_skb() */
	prefetch(&skb->end);

	DP(NETIF_MSG_TX_DONE, "fp[%d]: pkt_idx %d  buff @(%p)->skb %p\n",
	   txdata->txq_index, idx, tx_buf, skb);

	/* unmap first bd */
	tx_start_bd = &txdata->tx_desc_ring[bd_idx].start_bd;
	dma_unmap_single(&bp->pdev->dev, BD_UNMAP_ADDR(tx_start_bd),
			 BD_UNMAP_LEN(tx_start_bd), DMA_TO_DEVICE);


	nbd = le16_to_cpu(tx_start_bd->nbd) - 1;
#ifdef BNX2X_STOP_ON_ERROR
	if ((nbd - 1) > (MAX_SKB_FRAGS + 2)) {
		BNX2X_ERR("BAD nbd!\n");
		bnx2x_panic();
	}
#endif
	new_cons = nbd + tx_buf->first_bd;

	/* Get the next bd */
	bd_idx = TX_BD(NEXT_TX_IDX(bd_idx));

	/* Skip a parse bd... */
	--nbd;
	bd_idx = TX_BD(NEXT_TX_IDX(bd_idx));

	/* ...and the TSO split header bd since they have no mapping */
	if (tx_buf->flags & BNX2X_TSO_SPLIT_BD) {
		--nbd;
		bd_idx = TX_BD(NEXT_TX_IDX(bd_idx));
	}

	/* now free frags */
	while (nbd > 0) {

		tx_data_bd = &txdata->tx_desc_ring[bd_idx].reg_bd;
		dma_unmap_page(&bp->pdev->dev, BD_UNMAP_ADDR(tx_data_bd),
			       BD_UNMAP_LEN(tx_data_bd), DMA_TO_DEVICE);
		if (--nbd)
			bd_idx = TX_BD(NEXT_TX_IDX(bd_idx));
	}

	/* release skb */
	WARN_ON(!skb);
	if (likely(skb)) {
		(*pkts_compl)++;
		(*bytes_compl) += skb->len;
	}

	dev_kfree_skb_any(skb);
	tx_buf->first_bd = 0;
	tx_buf->skb = NULL;

	return new_cons;
}

int bnx2x_tx_int(struct bnx2x *bp, struct bnx2x_fp_txdata *txdata)
{
	struct netdev_queue *txq;
	u16 hw_cons, sw_cons, bd_cons = txdata->tx_bd_cons;
	unsigned int pkts_compl = 0, bytes_compl = 0;

#ifdef BNX2X_STOP_ON_ERROR
	if (unlikely(bp->panic))
		return -1;
#endif

	txq = netdev_get_tx_queue(bp->dev, txdata->txq_index);
	hw_cons = le16_to_cpu(*txdata->tx_cons_sb);
	sw_cons = txdata->tx_pkt_cons;

	while (sw_cons != hw_cons) {
		u16 pkt_cons;

		pkt_cons = TX_BD(sw_cons);

		DP(NETIF_MSG_TX_DONE,
		   "queue[%d]: hw_cons %u  sw_cons %u  pkt_cons %u\n",
		   txdata->txq_index, hw_cons, sw_cons, pkt_cons);

		bd_cons = bnx2x_free_tx_pkt(bp, txdata, pkt_cons,
		    &pkts_compl, &bytes_compl);

		sw_cons++;
	}

	netdev_tx_completed_queue(txq, pkts_compl, bytes_compl);

	txdata->tx_pkt_cons = sw_cons;
	txdata->tx_bd_cons = bd_cons;

	/* Need to make the tx_bd_cons update visible to start_xmit()
	 * before checking for netif_tx_queue_stopped().  Without the
	 * memory barrier, there is a small possibility that
	 * start_xmit() will miss it and cause the queue to be stopped
	 * forever.
	 * On the other hand we need an rmb() here to ensure the proper
	 * ordering of bit testing in the following
	 * netif_tx_queue_stopped(txq) call.
	 */
	smp_mb();

	if (unlikely(netif_tx_queue_stopped(txq))) {
		/* Taking tx_lock() is needed to prevent reenabling the queue
		 * while it's empty. This could have happen if rx_action() gets
		 * suspended in bnx2x_tx_int() after the condition before
		 * netif_tx_wake_queue(), while tx_action (bnx2x_start_xmit()):
		 *
		 * stops the queue->sees fresh tx_bd_cons->releases the queue->
		 * sends some packets consuming the whole queue again->
		 * stops the queue
		 */

		__netif_tx_lock(txq, smp_processor_id());

		if ((netif_tx_queue_stopped(txq)) &&
		    (bp->state == BNX2X_STATE_OPEN) &&
		    (bnx2x_tx_avail(bp, txdata) >= MAX_SKB_FRAGS + 3))
			netif_tx_wake_queue(txq);

		__netif_tx_unlock(txq);
	}
	return 0;
}

static inline void bnx2x_update_last_max_sge(struct bnx2x_fastpath *fp,
					     u16 idx)
{
	u16 last_max = fp->last_max_sge;

	if (SUB_S16(idx, last_max) > 0)
		fp->last_max_sge = idx;
}

static inline void bnx2x_update_sge_prod(struct bnx2x_fastpath *fp,
					 u16 sge_len,
					 struct eth_end_agg_rx_cqe *cqe)
{
	struct bnx2x *bp = fp->bp;
	u16 last_max, last_elem, first_elem;
	u16 delta = 0;
	u16 i;

	if (!sge_len)
		return;

	/* First mark all used pages */
	for (i = 0; i < sge_len; i++)
		BIT_VEC64_CLEAR_BIT(fp->sge_mask,
			RX_SGE(le16_to_cpu(cqe->sgl_or_raw_data.sgl[i])));

	DP(NETIF_MSG_RX_STATUS, "fp_cqe->sgl[%d] = %d\n",
	   sge_len - 1, le16_to_cpu(cqe->sgl_or_raw_data.sgl[sge_len - 1]));

	/* Here we assume that the last SGE index is the biggest */
	prefetch((void *)(fp->sge_mask));
	bnx2x_update_last_max_sge(fp,
		le16_to_cpu(cqe->sgl_or_raw_data.sgl[sge_len - 1]));

	last_max = RX_SGE(fp->last_max_sge);
	last_elem = last_max >> BIT_VEC64_ELEM_SHIFT;
	first_elem = RX_SGE(fp->rx_sge_prod) >> BIT_VEC64_ELEM_SHIFT;

	/* If ring is not full */
	if (last_elem + 1 != first_elem)
		last_elem++;

	/* Now update the prod */
	for (i = first_elem; i != last_elem; i = NEXT_SGE_MASK_ELEM(i)) {
		if (likely(fp->sge_mask[i]))
			break;

		fp->sge_mask[i] = BIT_VEC64_ELEM_ONE_MASK;
		delta += BIT_VEC64_ELEM_SZ;
	}

	if (delta > 0) {
		fp->rx_sge_prod += delta;
		/* clear page-end entries */
		bnx2x_clear_sge_mask_next_elems(fp);
	}

	DP(NETIF_MSG_RX_STATUS,
	   "fp->last_max_sge = %d  fp->rx_sge_prod = %d\n",
	   fp->last_max_sge, fp->rx_sge_prod);
}

/* Set Toeplitz hash value in the skb using the value from the
 * CQE (calculated by HW).
 */
static u32 bnx2x_get_rxhash(const struct bnx2x *bp,
			    const struct eth_fast_path_rx_cqe *cqe)
{
	/* Set Toeplitz hash from CQE */
	if ((bp->dev->features & NETIF_F_RXHASH) &&
	    (cqe->status_flags & ETH_FAST_PATH_RX_CQE_RSS_HASH_FLG))
		return le32_to_cpu(cqe->rss_hash_result);
	return 0;
}

static void bnx2x_tpa_start(struct bnx2x_fastpath *fp, u16 queue,
			    u16 cons, u16 prod,
			    struct eth_fast_path_rx_cqe *cqe)
{
	struct bnx2x *bp = fp->bp;
	struct sw_rx_bd *cons_rx_buf = &fp->rx_buf_ring[cons];
	struct sw_rx_bd *prod_rx_buf = &fp->rx_buf_ring[prod];
	struct eth_rx_bd *prod_bd = &fp->rx_desc_ring[prod];
	dma_addr_t mapping;
	struct bnx2x_agg_info *tpa_info = &fp->tpa_info[queue];
	struct sw_rx_bd *first_buf = &tpa_info->first_buf;

	/* print error if current state != stop */
	if (tpa_info->tpa_state != BNX2X_TPA_STOP)
		BNX2X_ERR("start of bin not in stop [%d]\n", queue);

	/* Try to map an empty data buffer from the aggregation info  */
	mapping = dma_map_single(&bp->pdev->dev,
				 first_buf->data + NET_SKB_PAD,
				 fp->rx_buf_size, DMA_FROM_DEVICE);
	/*
	 *  ...if it fails - move the skb from the consumer to the producer
	 *  and set the current aggregation state as ERROR to drop it
	 *  when TPA_STOP arrives.
	 */

	if (unlikely(dma_mapping_error(&bp->pdev->dev, mapping))) {
		/* Move the BD from the consumer to the producer */
		bnx2x_reuse_rx_data(fp, cons, prod);
		tpa_info->tpa_state = BNX2X_TPA_ERROR;
		return;
	}

	/* move empty data from pool to prod */
	prod_rx_buf->data = first_buf->data;
	dma_unmap_addr_set(prod_rx_buf, mapping, mapping);
	/* point prod_bd to new data */
	prod_bd->addr_hi = cpu_to_le32(U64_HI(mapping));
	prod_bd->addr_lo = cpu_to_le32(U64_LO(mapping));

	/* move partial skb from cons to pool (don't unmap yet) */
	*first_buf = *cons_rx_buf;

	/* mark bin state as START */
	tpa_info->parsing_flags =
		le16_to_cpu(cqe->pars_flags.flags);
	tpa_info->vlan_tag = le16_to_cpu(cqe->vlan_tag);
	tpa_info->tpa_state = BNX2X_TPA_START;
	tpa_info->len_on_bd = le16_to_cpu(cqe->len_on_bd);
	tpa_info->placement_offset = cqe->placement_offset;
	tpa_info->rxhash = bnx2x_get_rxhash(bp, cqe);
	if (fp->mode == TPA_MODE_GRO) {
		u16 gro_size = le16_to_cpu(cqe->pkt_len_or_gro_seg_len);
		tpa_info->full_page =
			SGE_PAGE_SIZE * PAGES_PER_SGE / gro_size * gro_size;
		/*
		 * FW 7.2.16 BUG workaround:
		 * if SGE size is (exactly) multiple gro_size
		 * fw will place one less frag on SGE.
		 * the calculation is done only for potentially
		 * dangerous MTUs.
		 */
		if (unlikely(bp->gro_check))
			if (!(SGE_PAGE_SIZE * PAGES_PER_SGE % gro_size))
				tpa_info->full_page -= gro_size;
		tpa_info->gro_size = gro_size;
	}

#ifdef BNX2X_STOP_ON_ERROR
	fp->tpa_queue_used |= (1 << queue);
#ifdef _ASM_GENERIC_INT_L64_H
	DP(NETIF_MSG_RX_STATUS, "fp->tpa_queue_used = 0x%lx\n",
#else
	DP(NETIF_MSG_RX_STATUS, "fp->tpa_queue_used = 0x%llx\n",
#endif
	   fp->tpa_queue_used);
#endif
}

/* Timestamp option length allowed for TPA aggregation:
 *
 *		nop nop kind length echo val
 */
#define TPA_TSTAMP_OPT_LEN	12
/**
 * bnx2x_set_lro_mss - calculate the approximate value of the MSS
 *
 * @bp:			driver handle
 * @parsing_flags:	parsing flags from the START CQE
 * @len_on_bd:		total length of the first packet for the
 *			aggregation.
 *
 * Approximate value of the MSS for this aggregation calculated using
 * the first packet of it.
 */
static inline u16 bnx2x_set_lro_mss(struct bnx2x *bp, u16 parsing_flags,
				    u16 len_on_bd)
{
	/*
	 * TPA arrgregation won't have either IP options or TCP options
	 * other than timestamp or IPv6 extension headers.
	 */
	u16 hdrs_len = ETH_HLEN + sizeof(struct tcphdr);

	if (GET_FLAG(parsing_flags, PARSING_FLAGS_OVER_ETHERNET_PROTOCOL) ==
	    PRS_FLAG_OVERETH_IPV6)
		hdrs_len += sizeof(struct ipv6hdr);
	else /* IPv4 */
		hdrs_len += sizeof(struct iphdr);


	/* Check if there was a TCP timestamp, if there is it's will
	 * always be 12 bytes length: nop nop kind length echo val.
	 *
	 * Otherwise FW would close the aggregation.
	 */
	if (parsing_flags & PARSING_FLAGS_TIME_STAMP_EXIST_FLAG)
		hdrs_len += TPA_TSTAMP_OPT_LEN;

	return len_on_bd - hdrs_len;
}

static int bnx2x_fill_frag_skb(struct bnx2x *bp, struct bnx2x_fastpath *fp,
			       struct bnx2x_agg_info *tpa_info,
			       u16 pages,
			       struct sk_buff *skb,
			       struct eth_end_agg_rx_cqe *cqe,
			       u16 cqe_idx)
{
	struct sw_rx_page *rx_pg, old_rx_pg;
	u32 i, frag_len, frag_size;
	int err, j, frag_id = 0;
	u16 len_on_bd = tpa_info->len_on_bd;
	u16 full_page = 0, gro_size = 0;

	frag_size = le16_to_cpu(cqe->pkt_len) - len_on_bd;

	if (fp->mode == TPA_MODE_GRO) {
		gro_size = tpa_info->gro_size;
		full_page = tpa_info->full_page;
	}

	/* This is needed in order to enable forwarding support */
	if (frag_size) {
		skb_shinfo(skb)->gso_size = bnx2x_set_lro_mss(bp,
					tpa_info->parsing_flags, len_on_bd);

		/* set for GRO */
		if (fp->mode == TPA_MODE_GRO)
			skb_shinfo(skb)->gso_type =
			    (GET_FLAG(tpa_info->parsing_flags,
				      PARSING_FLAGS_OVER_ETHERNET_PROTOCOL) ==
						PRS_FLAG_OVERETH_IPV6) ?
				SKB_GSO_TCPV6 : SKB_GSO_TCPV4;
	}


#ifdef BNX2X_STOP_ON_ERROR
	if (pages > min_t(u32, 8, MAX_SKB_FRAGS)*SGE_PAGE_SIZE*PAGES_PER_SGE) {
		BNX2X_ERR("SGL length is too long: %d. CQE index is %d\n",
			  pages, cqe_idx);
		BNX2X_ERR("cqe->pkt_len = %d\n", cqe->pkt_len);
		bnx2x_panic();
		return -EINVAL;
	}
#endif

	/* Run through the SGL and compose the fragmented skb */
	for (i = 0, j = 0; i < pages; i += PAGES_PER_SGE, j++) {
		u16 sge_idx = RX_SGE(le16_to_cpu(cqe->sgl_or_raw_data.sgl[j]));

		/* FW gives the indices of the SGE as if the ring is an array
		   (meaning that "next" element will consume 2 indices) */
		if (fp->mode == TPA_MODE_GRO)
			frag_len = min_t(u32, frag_size, (u32)full_page);
		else /* LRO */
			frag_len = min_t(u32, frag_size,
					 (u32)(SGE_PAGE_SIZE * PAGES_PER_SGE));

		rx_pg = &fp->rx_page_ring[sge_idx];
		old_rx_pg = *rx_pg;

		/* If we fail to allocate a substitute page, we simply stop
		   where we are and drop the whole packet */
		err = bnx2x_alloc_rx_sge(bp, fp, sge_idx);
		if (unlikely(err)) {
			fp->eth_q_stats.rx_skb_alloc_failed++;
			return err;
		}

		/* Unmap the page as we r going to pass it to the stack */
		dma_unmap_page(&bp->pdev->dev,
			       dma_unmap_addr(&old_rx_pg, mapping),
			       SGE_PAGE_SIZE*PAGES_PER_SGE, DMA_FROM_DEVICE);
		/* Add one frag and update the appropriate fields in the skb */
		if (fp->mode == TPA_MODE_LRO)
			skb_fill_page_desc(skb, j, old_rx_pg.page, 0, frag_len);
		else { /* GRO */
			int rem;
			int offset = 0;
			for (rem = frag_len; rem > 0; rem -= gro_size) {
				int len = rem > gro_size ? gro_size : rem;
				skb_fill_page_desc(skb, frag_id++,
						   old_rx_pg.page, offset, len);
				if (offset)
					get_page(old_rx_pg.page);
				offset += len;
			}
		}

		skb->data_len += frag_len;
		skb->truesize += SGE_PAGE_SIZE * PAGES_PER_SGE;
		skb->len += frag_len;

		frag_size -= frag_len;
	}

	return 0;
}

static inline void bnx2x_tpa_stop(struct bnx2x *bp, struct bnx2x_fastpath *fp,
				  struct bnx2x_agg_info *tpa_info,
				  u16 pages,
				  struct eth_end_agg_rx_cqe *cqe,
				  u16 cqe_idx)
{
	struct sw_rx_bd *rx_buf = &tpa_info->first_buf;
	u8 pad = tpa_info->placement_offset;
	u16 len = tpa_info->len_on_bd;
	struct sk_buff *skb = NULL;
	u8 *new_data, *data = rx_buf->data;
	u8 old_tpa_state = tpa_info->tpa_state;

	tpa_info->tpa_state = BNX2X_TPA_STOP;

	/* If we there was an error during the handling of the TPA_START -
	 * drop this aggregation.
	 */
	if (old_tpa_state == BNX2X_TPA_ERROR)
		goto drop;

	/* Try to allocate the new data */
	new_data = kmalloc(fp->rx_buf_size + NET_SKB_PAD, GFP_ATOMIC);

	/* Unmap skb in the pool anyway, as we are going to change
	   pool entry status to BNX2X_TPA_STOP even if new skb allocation
	   fails. */
	dma_unmap_single(&bp->pdev->dev, dma_unmap_addr(rx_buf, mapping),
			 fp->rx_buf_size, DMA_FROM_DEVICE);
	if (likely(new_data))
		skb = build_skb(data);

	if (likely(skb)) {
#ifdef BNX2X_STOP_ON_ERROR
		if (pad + len > fp->rx_buf_size) {
			BNX2X_ERR("skb_put is about to fail...  pad %d  len %d  rx_buf_size %d\n",
				  pad, len, fp->rx_buf_size);
			bnx2x_panic();
			return;
		}
#endif

		skb_reserve(skb, pad + NET_SKB_PAD);
		skb_put(skb, len);
		skb->rxhash = tpa_info->rxhash;

		skb->protocol = eth_type_trans(skb, bp->dev);
		skb->ip_summed = CHECKSUM_UNNECESSARY;

		if (!bnx2x_fill_frag_skb(bp, fp, tpa_info, pages,
					 skb, cqe, cqe_idx)) {
			if (tpa_info->parsing_flags & PARSING_FLAGS_VLAN)
				__vlan_hwaccel_put_tag(skb, tpa_info->vlan_tag);
			napi_gro_receive(&fp->napi, skb);
		} else {
			DP(NETIF_MSG_RX_STATUS,
			   "Failed to allocate new pages - dropping packet!\n");
			dev_kfree_skb_any(skb);
		}


		/* put new data in bin */
		rx_buf->data = new_data;

		return;
	}
	kfree(new_data);
drop:
	/* drop the packet and keep the buffer in the bin */
	DP(NETIF_MSG_RX_STATUS,
	   "Failed to allocate or map a new skb - dropping packet!\n");
	fp->eth_q_stats.rx_skb_alloc_failed++;
}


int bnx2x_rx_int(struct bnx2x_fastpath *fp, int budget)
{
	struct bnx2x *bp = fp->bp;
	u16 bd_cons, bd_prod, bd_prod_fw, comp_ring_cons;
	u16 hw_comp_cons, sw_comp_cons, sw_comp_prod;
	int rx_pkt = 0;

#ifdef BNX2X_STOP_ON_ERROR
	if (unlikely(bp->panic))
		return 0;
#endif

	/* CQ "next element" is of the size of the regular element,
	   that's why it's ok here */
	hw_comp_cons = le16_to_cpu(*fp->rx_cons_sb);
	if ((hw_comp_cons & MAX_RCQ_DESC_CNT) == MAX_RCQ_DESC_CNT)
		hw_comp_cons++;

	bd_cons = fp->rx_bd_cons;
	bd_prod = fp->rx_bd_prod;
	bd_prod_fw = bd_prod;
	sw_comp_cons = fp->rx_comp_cons;
	sw_comp_prod = fp->rx_comp_prod;

	/* Memory barrier necessary as speculative reads of the rx
	 * buffer can be ahead of the index in the status block
	 */
	rmb();

	DP(NETIF_MSG_RX_STATUS,
	   "queue[%d]:  hw_comp_cons %u  sw_comp_cons %u\n",
	   fp->index, hw_comp_cons, sw_comp_cons);

	while (sw_comp_cons != hw_comp_cons) {
		struct sw_rx_bd *rx_buf = NULL;
		struct sk_buff *skb;
		union eth_rx_cqe *cqe;
		struct eth_fast_path_rx_cqe *cqe_fp;
		u8 cqe_fp_flags;
		enum eth_rx_cqe_type cqe_fp_type;
		u16 len, pad, queue;
		u8 *data;

#ifdef BNX2X_STOP_ON_ERROR
		if (unlikely(bp->panic))
			return 0;
#endif

		comp_ring_cons = RCQ_BD(sw_comp_cons);
		bd_prod = RX_BD(bd_prod);
		bd_cons = RX_BD(bd_cons);

		cqe = &fp->rx_comp_ring[comp_ring_cons];
		cqe_fp = &cqe->fast_path_cqe;
		cqe_fp_flags = cqe_fp->type_error_flags;
		cqe_fp_type = cqe_fp_flags & ETH_FAST_PATH_RX_CQE_TYPE;

		DP(NETIF_MSG_RX_STATUS,
		   "CQE type %x  err %x  status %x  queue %x  vlan %x  len %u\n",
		   CQE_TYPE(cqe_fp_flags),
		   cqe_fp_flags, cqe_fp->status_flags,
		   le32_to_cpu(cqe_fp->rss_hash_result),
		   le16_to_cpu(cqe_fp->vlan_tag),
		   le16_to_cpu(cqe_fp->pkt_len_or_gro_seg_len));

		/* is this a slowpath msg? */
		if (unlikely(CQE_TYPE_SLOW(cqe_fp_type))) {
			bnx2x_sp_event(fp, cqe);
			goto next_cqe;
		}

		rx_buf = &fp->rx_buf_ring[bd_cons];
		data = rx_buf->data;

		if (!CQE_TYPE_FAST(cqe_fp_type)) {
			struct bnx2x_agg_info *tpa_info;
			u16 frag_size, pages;
#ifdef BNX2X_STOP_ON_ERROR
			/* sanity check */
			if (fp->disable_tpa &&
			    (CQE_TYPE_START(cqe_fp_type) ||
			     CQE_TYPE_STOP(cqe_fp_type)))
				BNX2X_ERR("START/STOP packet while disable_tpa type %x\n",
					  CQE_TYPE(cqe_fp_type));
#endif

			if (CQE_TYPE_START(cqe_fp_type)) {
				u16 queue = cqe_fp->queue_index;
				DP(NETIF_MSG_RX_STATUS,
				   "calling tpa_start on queue %d\n",
				   queue);

				bnx2x_tpa_start(fp, queue,
						bd_cons, bd_prod,
						cqe_fp);

				goto next_rx;

			}
			queue = cqe->end_agg_cqe.queue_index;
			tpa_info = &fp->tpa_info[queue];
			DP(NETIF_MSG_RX_STATUS,
			   "calling tpa_stop on queue %d\n",
			   queue);

			frag_size = le16_to_cpu(cqe->end_agg_cqe.pkt_len) -
				    tpa_info->len_on_bd;

			if (fp->mode == TPA_MODE_GRO)
				pages = (frag_size + tpa_info->full_page - 1) /
					 tpa_info->full_page;
			else
				pages = SGE_PAGE_ALIGN(frag_size) >>
					SGE_PAGE_SHIFT;

			bnx2x_tpa_stop(bp, fp, tpa_info, pages,
				       &cqe->end_agg_cqe, comp_ring_cons);
#ifdef BNX2X_STOP_ON_ERROR
			if (bp->panic)
				return 0;
#endif

			bnx2x_update_sge_prod(fp, pages, &cqe->end_agg_cqe);
			goto next_cqe;
		}
		/* non TPA */
		len = le16_to_cpu(cqe_fp->pkt_len_or_gro_seg_len);
		pad = cqe_fp->placement_offset;
		dma_sync_single_for_cpu(&bp->pdev->dev,
					dma_unmap_addr(rx_buf, mapping),
					pad + RX_COPY_THRESH,
					DMA_FROM_DEVICE);
		pad += NET_SKB_PAD;
		prefetch(data + pad); /* speedup eth_type_trans() */
		/* is this an error packet? */
		if (unlikely(cqe_fp_flags & ETH_RX_ERROR_FALGS)) {
			DP(NETIF_MSG_RX_ERR | NETIF_MSG_RX_STATUS,
			   "ERROR  flags %x  rx packet %u\n",
			   cqe_fp_flags, sw_comp_cons);
			fp->eth_q_stats.rx_err_discard_pkt++;
			goto reuse_rx;
		}

		/* Since we don't have a jumbo ring
		 * copy small packets if mtu > 1500
		 */
		if ((bp->dev->mtu > ETH_MAX_PACKET_SIZE) &&
		    (len <= RX_COPY_THRESH)) {
			skb = netdev_alloc_skb_ip_align(bp->dev, len);
			if (skb == NULL) {
				DP(NETIF_MSG_RX_ERR | NETIF_MSG_RX_STATUS,
				   "ERROR  packet dropped because of alloc failure\n");
				fp->eth_q_stats.rx_skb_alloc_failed++;
				goto reuse_rx;
			}
			memcpy(skb->data, data + pad, len);
			bnx2x_reuse_rx_data(fp, bd_cons, bd_prod);
		} else {
			if (likely(bnx2x_alloc_rx_data(bp, fp, bd_prod) == 0)) {
				dma_unmap_single(&bp->pdev->dev,
						 dma_unmap_addr(rx_buf, mapping),
						 fp->rx_buf_size,
						 DMA_FROM_DEVICE);
				skb = build_skb(data);
				if (unlikely(!skb)) {
					kfree(data);
					fp->eth_q_stats.rx_skb_alloc_failed++;
					goto next_rx;
				}
				skb_reserve(skb, pad);
			} else {
				DP(NETIF_MSG_RX_ERR | NETIF_MSG_RX_STATUS,
				   "ERROR  packet dropped because of alloc failure\n");
				fp->eth_q_stats.rx_skb_alloc_failed++;
reuse_rx:
				bnx2x_reuse_rx_data(fp, bd_cons, bd_prod);
				goto next_rx;
			}
		}

		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, bp->dev);

		/* Set Toeplitz hash for a none-LRO skb */
		skb->rxhash = bnx2x_get_rxhash(bp, cqe_fp);

		skb_checksum_none_assert(skb);

		if (bp->dev->features & NETIF_F_RXCSUM) {

			if (likely(BNX2X_RX_CSUM_OK(cqe)))
				skb->ip_summed = CHECKSUM_UNNECESSARY;
			else
				fp->eth_q_stats.hw_csum_err++;
		}

		skb_record_rx_queue(skb, fp->rx_queue);

		if (le16_to_cpu(cqe_fp->pars_flags.flags) &
		    PARSING_FLAGS_VLAN)
			__vlan_hwaccel_put_tag(skb,
					       le16_to_cpu(cqe_fp->vlan_tag));
		napi_gro_receive(&fp->napi, skb);


next_rx:
		rx_buf->data = NULL;

		bd_cons = NEXT_RX_IDX(bd_cons);
		bd_prod = NEXT_RX_IDX(bd_prod);
		bd_prod_fw = NEXT_RX_IDX(bd_prod_fw);
		rx_pkt++;
next_cqe:
		sw_comp_prod = NEXT_RCQ_IDX(sw_comp_prod);
		sw_comp_cons = NEXT_RCQ_IDX(sw_comp_cons);

		if (rx_pkt == budget)
			break;
	} /* while */

	fp->rx_bd_cons = bd_cons;
	fp->rx_bd_prod = bd_prod_fw;
	fp->rx_comp_cons = sw_comp_cons;
	fp->rx_comp_prod = sw_comp_prod;

	/* Update producers */
	bnx2x_update_rx_prod(bp, fp, bd_prod_fw, sw_comp_prod,
			     fp->rx_sge_prod);

	fp->rx_pkt += rx_pkt;
	fp->rx_calls++;

	return rx_pkt;
}

static irqreturn_t bnx2x_msix_fp_int(int irq, void *fp_cookie)
{
	struct bnx2x_fastpath *fp = fp_cookie;
	struct bnx2x *bp = fp->bp;
	u8 cos;

	DP(NETIF_MSG_INTR,
	   "got an MSI-X interrupt on IDX:SB [fp %d fw_sd %d igusb %d]\n",
	   fp->index, fp->fw_sb_id, fp->igu_sb_id);
	bnx2x_ack_sb(bp, fp->igu_sb_id, USTORM_ID, 0, IGU_INT_DISABLE, 0);

#ifdef BNX2X_STOP_ON_ERROR
	if (unlikely(bp->panic))
		return IRQ_HANDLED;
#endif

	/* Handle Rx and Tx according to MSI-X vector */
	prefetch(fp->rx_cons_sb);

	for_each_cos_in_tx_queue(fp, cos)
		prefetch(fp->txdata[cos].tx_cons_sb);

	prefetch(&fp->sb_running_index[SM_RX_ID]);
	napi_schedule(&bnx2x_fp(bp, fp->index, napi));

	return IRQ_HANDLED;
}

/* HW Lock for shared dual port PHYs */
void bnx2x_acquire_phy_lock(struct bnx2x *bp)
{
	mutex_lock(&bp->port.phy_mutex);

	if (bp->port.need_hw_lock)
		bnx2x_acquire_hw_lock(bp, HW_LOCK_RESOURCE_MDIO);
}

void bnx2x_release_phy_lock(struct bnx2x *bp)
{
	if (bp->port.need_hw_lock)
		bnx2x_release_hw_lock(bp, HW_LOCK_RESOURCE_MDIO);

	mutex_unlock(&bp->port.phy_mutex);
}

/* calculates MF speed according to current linespeed and MF configuration */
u16 bnx2x_get_mf_speed(struct bnx2x *bp)
{
	u16 line_speed = bp->link_vars.line_speed;
	if (IS_MF(bp)) {
		u16 maxCfg = bnx2x_extract_max_cfg(bp,
						   bp->mf_config[BP_VN(bp)]);

		/* Calculate the current MAX line speed limit for the MF
		 * devices
		 */
		if (IS_MF_SI(bp))
			line_speed = (line_speed * maxCfg) / 100;
		else { /* SD mode */
			u16 vn_max_rate = maxCfg * 100;

			if (vn_max_rate < line_speed)
				line_speed = vn_max_rate;
		}
	}

	return line_speed;
}

/**
 * bnx2x_fill_report_data - fill link report data to report
 *
 * @bp:		driver handle
 * @data:	link state to update
 *
 * It uses a none-atomic bit operations because is called under the mutex.
 */
static inline void bnx2x_fill_report_data(struct bnx2x *bp,
					  struct bnx2x_link_report_data *data)
{
	u16 line_speed = bnx2x_get_mf_speed(bp);

	memset(data, 0, sizeof(*data));

	/* Fill the report data: efective line speed */
	data->line_speed = line_speed;

	/* Link is down */
	if (!bp->link_vars.link_up || (bp->flags & MF_FUNC_DIS))
		__set_bit(BNX2X_LINK_REPORT_LINK_DOWN,
			  &data->link_report_flags);

	/* Full DUPLEX */
	if (bp->link_vars.duplex == DUPLEX_FULL)
		__set_bit(BNX2X_LINK_REPORT_FD, &data->link_report_flags);

	/* Rx Flow Control is ON */
	if (bp->link_vars.flow_ctrl & BNX2X_FLOW_CTRL_RX)
		__set_bit(BNX2X_LINK_REPORT_RX_FC_ON, &data->link_report_flags);

	/* Tx Flow Control is ON */
	if (bp->link_vars.flow_ctrl & BNX2X_FLOW_CTRL_TX)
		__set_bit(BNX2X_LINK_REPORT_TX_FC_ON, &data->link_report_flags);
}

/**
 * bnx2x_link_report - report link status to OS.
 *
 * @bp:		driver handle
 *
 * Calls the __bnx2x_link_report() under the same locking scheme
 * as a link/PHY state managing code to ensure a consistent link
 * reporting.
 */

void bnx2x_link_report(struct bnx2x *bp)
{
	bnx2x_acquire_phy_lock(bp);
	__bnx2x_link_report(bp);
	bnx2x_release_phy_lock(bp);
}

/**
 * __bnx2x_link_report - report link status to OS.
 *
 * @bp:		driver handle
 *
 * None atomic inmlementation.
 * Should be called under the phy_lock.
 */
void __bnx2x_link_report(struct bnx2x *bp)
{
	struct bnx2x_link_report_data cur_data;

	/* reread mf_cfg */
	if (!CHIP_IS_E1(bp))
		bnx2x_read_mf_cfg(bp);

	/* Read the current link report info */
	bnx2x_fill_report_data(bp, &cur_data);

	/* Don't report link down or exactly the same link status twice */
	if (!memcmp(&cur_data, &bp->last_reported_link, sizeof(cur_data)) ||
	    (test_bit(BNX2X_LINK_REPORT_LINK_DOWN,
		      &bp->last_reported_link.link_report_flags) &&
	     test_bit(BNX2X_LINK_REPORT_LINK_DOWN,
		      &cur_data.link_report_flags)))
		return;

	bp->link_cnt++;

	/* We are going to report a new link parameters now -
	 * remember the current data for the next time.
	 */
	memcpy(&bp->last_reported_link, &cur_data, sizeof(cur_data));

	if (test_bit(BNX2X_LINK_REPORT_LINK_DOWN,
		     &cur_data.link_report_flags)) {
		netif_carrier_off(bp->dev);
		netdev_err(bp->dev, "NIC Link is Down\n");
		return;
	} else {
		const char *duplex;
		const char *flow;

		netif_carrier_on(bp->dev);

		if (test_and_clear_bit(BNX2X_LINK_REPORT_FD,
				       &cur_data.link_report_flags))
			duplex = "full";
		else
			duplex = "half";

		/* Handle the FC at the end so that only these flags would be
		 * possibly set. This way we may easily check if there is no FC
		 * enabled.
		 */
		if (cur_data.link_report_flags) {
			if (test_bit(BNX2X_LINK_REPORT_RX_FC_ON,
				     &cur_data.link_report_flags)) {
				if (test_bit(BNX2X_LINK_REPORT_TX_FC_ON,
				     &cur_data.link_report_flags))
					flow = "ON - receive & transmit";
				else
					flow = "ON - receive";
			} else {
				flow = "ON - transmit";
			}
		} else {
			flow = "none";
		}
		netdev_info(bp->dev, "NIC Link is Up, %d Mbps %s duplex, Flow control: %s\n",
			    cur_data.line_speed, duplex, flow);
	}
}

void bnx2x_init_rx_rings(struct bnx2x *bp)
{
	int func = BP_FUNC(bp);
	u16 ring_prod;
	int i, j;

	/* Allocate TPA resources */
	for_each_rx_queue(bp, j) {
		struct bnx2x_fastpath *fp = &bp->fp[j];

		DP(NETIF_MSG_IFUP,
		   "mtu %d  rx_buf_size %d\n", bp->dev->mtu, fp->rx_buf_size);

		if (!fp->disable_tpa) {
			/* Fill the per-aggregtion pool */
			for (i = 0; i < MAX_AGG_QS(bp); i++) {
				struct bnx2x_agg_info *tpa_info =
					&fp->tpa_info[i];
				struct sw_rx_bd *first_buf =
					&tpa_info->first_buf;

				first_buf->data = kmalloc(fp->rx_buf_size + NET_SKB_PAD,
							  GFP_ATOMIC);
				if (!first_buf->data) {
					BNX2X_ERR("Failed to allocate TPA skb pool for queue[%d] - disabling TPA on this queue!\n",
						  j);
					bnx2x_free_tpa_pool(bp, fp, i);
					fp->disable_tpa = 1;
					break;
				}
				dma_unmap_addr_set(first_buf, mapping, 0);
				tpa_info->tpa_state = BNX2X_TPA_STOP;
			}

			/* "next page" elements initialization */
			bnx2x_set_next_page_sgl(fp);

			/* set SGEs bit mask */
			bnx2x_init_sge_ring_bit_mask(fp);

			/* Allocate SGEs and initialize the ring elements */
			for (i = 0, ring_prod = 0;
			     i < MAX_RX_SGE_CNT*NUM_RX_SGE_PAGES; i++) {

				if (bnx2x_alloc_rx_sge(bp, fp, ring_prod) < 0) {
					BNX2X_ERR("was only able to allocate %d rx sges\n",
						  i);
					BNX2X_ERR("disabling TPA for queue[%d]\n",
						  j);
					/* Cleanup already allocated elements */
					bnx2x_free_rx_sge_range(bp, fp,
								ring_prod);
					bnx2x_free_tpa_pool(bp, fp,
							    MAX_AGG_QS(bp));
					fp->disable_tpa = 1;
					ring_prod = 0;
					break;
				}
				ring_prod = NEXT_SGE_IDX(ring_prod);
			}

			fp->rx_sge_prod = ring_prod;
		}
	}

	for_each_rx_queue(bp, j) {
		struct bnx2x_fastpath *fp = &bp->fp[j];

		fp->rx_bd_cons = 0;

		/* Activate BD ring */
		/* Warning!
		 * this will generate an interrupt (to the TSTORM)
		 * must only be done after chip is initialized
		 */
		bnx2x_update_rx_prod(bp, fp, fp->rx_bd_prod, fp->rx_comp_prod,
				     fp->rx_sge_prod);

		if (j != 0)
			continue;

		if (CHIP_IS_E1(bp)) {
			REG_WR(bp, BAR_USTRORM_INTMEM +
			       USTORM_MEM_WORKAROUND_ADDRESS_OFFSET(func),
			       U64_LO(fp->rx_comp_mapping));
			REG_WR(bp, BAR_USTRORM_INTMEM +
			       USTORM_MEM_WORKAROUND_ADDRESS_OFFSET(func) + 4,
			       U64_HI(fp->rx_comp_mapping));
		}
	}
}

static void bnx2x_free_tx_skbs(struct bnx2x *bp)
{
	int i;
	u8 cos;

	for_each_tx_queue(bp, i) {
		struct bnx2x_fastpath *fp = &bp->fp[i];
		for_each_cos_in_tx_queue(fp, cos) {
			struct bnx2x_fp_txdata *txdata = &fp->txdata[cos];
			unsigned pkts_compl = 0, bytes_compl = 0;

			u16 sw_prod = txdata->tx_pkt_prod;
			u16 sw_cons = txdata->tx_pkt_cons;

			while (sw_cons != sw_prod) {
				bnx2x_free_tx_pkt(bp, txdata, TX_BD(sw_cons),
				    &pkts_compl, &bytes_compl);
				sw_cons++;
			}
			netdev_tx_reset_queue(
			    netdev_get_tx_queue(bp->dev, txdata->txq_index));
		}
	}
}

static void bnx2x_free_rx_bds(struct bnx2x_fastpath *fp)
{
	struct bnx2x *bp = fp->bp;
	int i;

	/* ring wasn't allocated */
	if (fp->rx_buf_ring == NULL)
		return;

	for (i = 0; i < NUM_RX_BD; i++) {
		struct sw_rx_bd *rx_buf = &fp->rx_buf_ring[i];
		u8 *data = rx_buf->data;

		if (data == NULL)
			continue;
		dma_unmap_single(&bp->pdev->dev,
				 dma_unmap_addr(rx_buf, mapping),
				 fp->rx_buf_size, DMA_FROM_DEVICE);

		rx_buf->data = NULL;
		kfree(data);
	}
}

static void bnx2x_free_rx_skbs(struct bnx2x *bp)
{
	int j;

	for_each_rx_queue(bp, j) {
		struct bnx2x_fastpath *fp = &bp->fp[j];

		bnx2x_free_rx_bds(fp);

		if (!fp->disable_tpa)
			bnx2x_free_tpa_pool(bp, fp, MAX_AGG_QS(bp));
	}
}

void bnx2x_free_skbs(struct bnx2x *bp)
{
	bnx2x_free_tx_skbs(bp);
	bnx2x_free_rx_skbs(bp);
}

void bnx2x_update_max_mf_config(struct bnx2x *bp, u32 value)
{
	/* load old values */
	u32 mf_cfg = bp->mf_config[BP_VN(bp)];

	if (value != bnx2x_extract_max_cfg(bp, mf_cfg)) {
		/* leave all but MAX value */
		mf_cfg &= ~FUNC_MF_CFG_MAX_BW_MASK;

		/* set new MAX value */
		mf_cfg |= (value << FUNC_MF_CFG_MAX_BW_SHIFT)
				& FUNC_MF_CFG_MAX_BW_MASK;

		bnx2x_fw_command(bp, DRV_MSG_CODE_SET_MF_BW, mf_cfg);
	}
}

/**
 * bnx2x_free_msix_irqs - free previously requested MSI-X IRQ vectors
 *
 * @bp:		driver handle
 * @nvecs:	number of vectors to be released
 */
static void bnx2x_free_msix_irqs(struct bnx2x *bp, int nvecs)
{
	int i, offset = 0;

	if (nvecs == offset)
		return;
	free_irq(bp->msix_table[offset].vector, bp->dev);
	DP(NETIF_MSG_IFDOWN, "released sp irq (%d)\n",
	   bp->msix_table[offset].vector);
	offset++;
#ifdef BCM_CNIC
	if (nvecs == offset)
		return;
	offset++;
#endif

	for_each_eth_queue(bp, i) {
		if (nvecs == offset)
			return;
		DP(NETIF_MSG_IFDOWN, "about to release fp #%d->%d irq\n",
		   i, bp->msix_table[offset].vector);

		free_irq(bp->msix_table[offset++].vector, &bp->fp[i]);
	}
}

void bnx2x_free_irq(struct bnx2x *bp)
{
	if (bp->flags & USING_MSIX_FLAG)
		bnx2x_free_msix_irqs(bp, BNX2X_NUM_ETH_QUEUES(bp) +
				     CNIC_PRESENT + 1);
	else if (bp->flags & USING_MSI_FLAG)
		free_irq(bp->pdev->irq, bp->dev);
	else
		free_irq(bp->pdev->irq, bp->dev);
}

int bnx2x_enable_msix(struct bnx2x *bp)
{
	int msix_vec = 0, i, rc, req_cnt;

	bp->msix_table[msix_vec].entry = msix_vec;
	BNX2X_DEV_INFO("msix_table[0].entry = %d (slowpath)\n",
	   bp->msix_table[0].entry);
	msix_vec++;

#ifdef BCM_CNIC
	bp->msix_table[msix_vec].entry = msix_vec;
	BNX2X_DEV_INFO("msix_table[%d].entry = %d (CNIC)\n",
	   bp->msix_table[msix_vec].entry, bp->msix_table[msix_vec].entry);
	msix_vec++;
#endif
	/* We need separate vectors for ETH queues only (not FCoE) */
	for_each_eth_queue(bp, i) {
		bp->msix_table[msix_vec].entry = msix_vec;
		BNX2X_DEV_INFO("msix_table[%d].entry = %d (fastpath #%u)\n",
			       msix_vec, msix_vec, i);
		msix_vec++;
	}

	req_cnt = BNX2X_NUM_ETH_QUEUES(bp) + CNIC_PRESENT + 1;

	rc = pci_enable_msix(bp->pdev, &bp->msix_table[0], req_cnt);

	/*
	 * reconfigure number of tx/rx queues according to available
	 * MSI-X vectors
	 */
	if (rc >= BNX2X_MIN_MSIX_VEC_CNT) {
		/* how less vectors we will have? */
		int diff = req_cnt - rc;

		BNX2X_DEV_INFO("Trying to use less MSI-X vectors: %d\n", rc);

		rc = pci_enable_msix(bp->pdev, &bp->msix_table[0], rc);

		if (rc) {
			BNX2X_DEV_INFO("MSI-X is not attainable  rc %d\n", rc);
			return rc;
		}
		/*
		 * decrease number of queues by number of unallocated entries
		 */
		bp->num_queues -= diff;

		BNX2X_DEV_INFO("New queue configuration set: %d\n",
				  bp->num_queues);
	} else if (rc) {
		/* fall to INTx if not enough memory */
		if (rc == -ENOMEM)
			bp->flags |= DISABLE_MSI_FLAG;
		BNX2X_DEV_INFO("MSI-X is not attainable  rc %d\n", rc);
		return rc;
	}

	bp->flags |= USING_MSIX_FLAG;

	return 0;
}

static int bnx2x_req_msix_irqs(struct bnx2x *bp)
{
	int i, rc, offset = 0;

	rc = request_irq(bp->msix_table[offset++].vector,
			 bnx2x_msix_sp_int, 0,
			 bp->dev->name, bp->dev);
	if (rc) {
		BNX2X_ERR("request sp irq failed\n");
		return -EBUSY;
	}

#ifdef BCM_CNIC
	offset++;
#endif
	for_each_eth_queue(bp, i) {
		struct bnx2x_fastpath *fp = &bp->fp[i];
		snprintf(fp->name, sizeof(fp->name), "%s-fp-%d",
			 bp->dev->name, i);

		rc = request_irq(bp->msix_table[offset].vector,
				 bnx2x_msix_fp_int, 0, fp->name, fp);
		if (rc) {
			BNX2X_ERR("request fp #%d irq (%d) failed  rc %d\n", i,
			      bp->msix_table[offset].vector, rc);
			bnx2x_free_msix_irqs(bp, offset);
			return -EBUSY;
		}

		offset++;
	}

	i = BNX2X_NUM_ETH_QUEUES(bp);
	offset = 1 + CNIC_PRESENT;
	netdev_info(bp->dev, "using MSI-X  IRQs: sp %d  fp[%d] %d ... fp[%d] %d\n",
	       bp->msix_table[0].vector,
	       0, bp->msix_table[offset].vector,
	       i - 1, bp->msix_table[offset + i - 1].vector);

	return 0;
}

int bnx2x_enable_msi(struct bnx2x *bp)
{
	int rc;

	rc = pci_enable_msi(bp->pdev);
	if (rc) {
		BNX2X_DEV_INFO("MSI is not attainable\n");
		return -1;
	}
	bp->flags |= USING_MSI_FLAG;

	return 0;
}

static int bnx2x_req_irq(struct bnx2x *bp)
{
	unsigned long flags;
	int rc;

	if (bp->flags & USING_MSI_FLAG)
		flags = 0;
	else
		flags = IRQF_SHARED;

	rc = request_irq(bp->pdev->irq, bnx2x_interrupt, flags,
			 bp->dev->name, bp->dev);
	return rc;
}

static inline int bnx2x_setup_irqs(struct bnx2x *bp)
{
	int rc = 0;
	if (bp->flags & USING_MSIX_FLAG) {
		rc = bnx2x_req_msix_irqs(bp);
		if (rc)
			return rc;
	} else {
		bnx2x_ack_int(bp);
		rc = bnx2x_req_irq(bp);
		if (rc) {
			BNX2X_ERR("IRQ request failed  rc %d, aborting\n", rc);
			return rc;
		}
		if (bp->flags & USING_MSI_FLAG) {
			bp->dev->irq = bp->pdev->irq;
			netdev_info(bp->dev, "using MSI  IRQ %d\n",
			       bp->pdev->irq);
		}
	}

	return 0;
}

static inline void bnx2x_napi_enable(struct bnx2x *bp)
{
	int i;

	for_each_rx_queue(bp, i)
		napi_enable(&bnx2x_fp(bp, i, napi));
}

static inline void bnx2x_napi_disable(struct bnx2x *bp)
{
	int i;

	for_each_rx_queue(bp, i)
		napi_disable(&bnx2x_fp(bp, i, napi));
}

void bnx2x_netif_start(struct bnx2x *bp)
{
	if (netif_running(bp->dev)) {
		bnx2x_napi_enable(bp);
		bnx2x_int_enable(bp);
		if (bp->state == BNX2X_STATE_OPEN)
			netif_tx_wake_all_queues(bp->dev);
	}
}

void bnx2x_netif_stop(struct bnx2x *bp, int disable_hw)
{
	bnx2x_int_disable_sync(bp, disable_hw);
	bnx2x_napi_disable(bp);
}

u16 bnx2x_select_queue(struct net_device *dev, struct sk_buff *skb)
{
	struct bnx2x *bp = netdev_priv(dev);

#ifdef BCM_CNIC
	if (!NO_FCOE(bp)) {
		struct ethhdr *hdr = (struct ethhdr *)skb->data;
		u16 ether_type = ntohs(hdr->h_proto);

		/* Skip VLAN tag if present */
		if (ether_type == ETH_P_8021Q) {
			struct vlan_ethhdr *vhdr =
				(struct vlan_ethhdr *)skb->data;

			ether_type = ntohs(vhdr->h_vlan_encapsulated_proto);
		}

		/* If ethertype is FCoE or FIP - use FCoE ring */
		if ((ether_type == ETH_P_FCOE) || (ether_type == ETH_P_FIP))
			return bnx2x_fcoe_tx(bp, txq_index);
	}
#endif
	/* select a non-FCoE queue */
	return __skb_tx_hash(dev, skb, BNX2X_NUM_ETH_QUEUES(bp));
}

void bnx2x_set_num_queues(struct bnx2x *bp)
{
	switch (bp->multi_mode) {
	case ETH_RSS_MODE_DISABLED:
		bp->num_queues = 1;
		break;
	case ETH_RSS_MODE_REGULAR:
		bp->num_queues = bnx2x_calc_num_queues(bp);
		break;

	default:
		bp->num_queues = 1;
		break;
	}

#ifdef BCM_CNIC
	/* override in STORAGE SD mode */
	if (IS_MF_STORAGE_SD(bp))
		bp->num_queues = 1;
#endif
	/* Add special queues */
	bp->num_queues += NON_ETH_CONTEXT_USE;
}

/**
 * bnx2x_set_real_num_queues - configure netdev->real_num_[tx,rx]_queues
 *
 * @bp:		Driver handle
 *
 * We currently support for at most 16 Tx queues for each CoS thus we will
 * allocate a multiple of 16 for ETH L2 rings according to the value of the
 * bp->max_cos.
 *
 * If there is an FCoE L2 queue the appropriate Tx queue will have the next
 * index after all ETH L2 indices.
 *
 * If the actual number of Tx queues (for each CoS) is less than 16 then there
 * will be the holes at the end of each group of 16 ETh L2 indices (0..15,
 * 16..31,...) with indicies that are not coupled with any real Tx queue.
 *
 * The proper configuration of skb->queue_mapping is handled by
 * bnx2x_select_queue() and __skb_tx_hash().
 *
 * bnx2x_setup_tc() takes care of the proper TC mappings so that __skb_tx_hash()
 * will return a proper Tx index if TC is enabled (netdev->num_tc > 0).
 */
static inline int bnx2x_set_real_num_queues(struct bnx2x *bp)
{
	int rc, tx, rx;

	tx = MAX_TXQS_PER_COS * bp->max_cos;
	rx = BNX2X_NUM_ETH_QUEUES(bp);

/* account for fcoe queue */
#ifdef BCM_CNIC
	if (!NO_FCOE(bp)) {
		rx += FCOE_PRESENT;
		tx += FCOE_PRESENT;
	}
#endif

	rc = netif_set_real_num_tx_queues(bp->dev, tx);
	if (rc) {
		BNX2X_ERR("Failed to set real number of Tx queues: %d\n", rc);
		return rc;
	}
	rc = netif_set_real_num_rx_queues(bp->dev, rx);
	if (rc) {
		BNX2X_ERR("Failed to set real number of Rx queues: %d\n", rc);
		return rc;
	}

	DP(NETIF_MSG_IFUP, "Setting real num queues to (tx, rx) (%d, %d)\n",
			  tx, rx);

	return rc;
}

static inline void bnx2x_set_rx_buf_size(struct bnx2x *bp)
{
	int i;

	for_each_queue(bp, i) {
		struct bnx2x_fastpath *fp = &bp->fp[i];
		u32 mtu;

		/* Always use a mini-jumbo MTU for the FCoE L2 ring */
		if (IS_FCOE_IDX(i))
			/*
			 * Although there are no IP frames expected to arrive to
			 * this ring we still want to add an
			 * IP_HEADER_ALIGNMENT_PADDING to prevent a buffer
			 * overrun attack.
			 */
			mtu = BNX2X_FCOE_MINI_JUMBO_MTU;
		else
			mtu = bp->dev->mtu;
		fp->rx_buf_size = BNX2X_FW_RX_ALIGN_START +
				  IP_HEADER_ALIGNMENT_PADDING +
				  ETH_OVREHEAD +
				  mtu +
				  BNX2X_FW_RX_ALIGN_END;
		/* Note : rx_buf_size doesnt take into account NET_SKB_PAD */
	}
}

static inline int bnx2x_init_rss_pf(struct bnx2x *bp)
{
	int i;
	u8 ind_table[T_ETH_INDIRECTION_TABLE_SIZE] = {0};
	u8 num_eth_queues = BNX2X_NUM_ETH_QUEUES(bp);

	/*
	 * Prepare the inital contents fo the indirection table if RSS is
	 * enabled
	 */
	if (bp->multi_mode != ETH_RSS_MODE_DISABLED) {
		for (i = 0; i < sizeof(ind_table); i++)
			ind_table[i] =
				bp->fp->cl_id +
				ethtool_rxfh_indir_default(i, num_eth_queues);
	}

	/*
	 * For 57710 and 57711 SEARCHER configuration (rss_keys) is
	 * per-port, so if explicit configuration is needed , do it only
	 * for a PMF.
	 *
	 * For 57712 and newer on the other hand it's a per-function
	 * configuration.
	 */
	return bnx2x_config_rss_pf(bp, ind_table,
				   bp->port.pmf || !CHIP_IS_E1x(bp));
}

int bnx2x_config_rss_pf(struct bnx2x *bp, u8 *ind_table, bool config_hash)
{
	struct bnx2x_config_rss_params params = {NULL};
	int i;

	/* Although RSS is meaningless when there is a single HW queue we
	 * still need it enabled in order to have HW Rx hash generated.
	 *
	 * if (!is_eth_multi(bp))
	 *      bp->multi_mode = ETH_RSS_MODE_DISABLED;
	 */

	params.rss_obj = &bp->rss_conf_obj;

	__set_bit(RAMROD_COMP_WAIT, &params.ramrod_flags);

	/* RSS mode */
	switch (bp->multi_mode) {
	case ETH_RSS_MODE_DISABLED:
		__set_bit(BNX2X_RSS_MODE_DISABLED, &params.rss_flags);
		break;
	case ETH_RSS_MODE_REGULAR:
		__set_bit(BNX2X_RSS_MODE_REGULAR, &params.rss_flags);
		break;
	case ETH_RSS_MODE_VLAN_PRI:
		__set_bit(BNX2X_RSS_MODE_VLAN_PRI, &params.rss_flags);
		break;
	case ETH_RSS_MODE_E1HOV_PRI:
		__set_bit(BNX2X_RSS_MODE_E1HOV_PRI, &params.rss_flags);
		break;
	case ETH_RSS_MODE_IP_DSCP:
		__set_bit(BNX2X_RSS_MODE_IP_DSCP, &params.rss_flags);
		break;
	default:
		BNX2X_ERR("Unknown multi_mode: %d\n", bp->multi_mode);
		return -EINVAL;
	}

	/* If RSS is enabled */
	if (bp->multi_mode != ETH_RSS_MODE_DISABLED) {
		/* RSS configuration */
		__set_bit(BNX2X_RSS_IPV4, &params.rss_flags);
		__set_bit(BNX2X_RSS_IPV4_TCP, &params.rss_flags);
		__set_bit(BNX2X_RSS_IPV6, &params.rss_flags);
		__set_bit(BNX2X_RSS_IPV6_TCP, &params.rss_flags);

		/* Hash bits */
		params.rss_result_mask = MULTI_MASK;

		memcpy(params.ind_table, ind_table, sizeof(params.ind_table));

		if (config_hash) {
			/* RSS keys */
			for (i = 0; i < sizeof(params.rss_key) / 4; i++)
				params.rss_key[i] = random32();

			__set_bit(BNX2X_RSS_SET_SRCH, &params.rss_flags);
		}
	}

	return bnx2x_config_rss(bp, &params);
}

static inline int bnx2x_init_hw(struct bnx2x *bp, u32 load_code)
{
	struct bnx2x_func_state_params func_params = {NULL};

	/* Prepare parameters for function state transitions */
	__set_bit(RAMROD_COMP_WAIT, &func_params.ramrod_flags);

	func_params.f_obj = &bp->func_obj;
	func_params.cmd = BNX2X_F_CMD_HW_INIT;

	func_params.params.hw_init.load_phase = load_code;

	return bnx2x_func_state_change(bp, &func_params);
}

/*
 * Cleans the object that have internal lists without sending
 * ramrods. Should be run when interrutps are disabled.
 */
static void bnx2x_squeeze_objects(struct bnx2x *bp)
{
	int rc;
	unsigned long ramrod_flags = 0, vlan_mac_flags = 0;
	struct bnx2x_mcast_ramrod_params rparam = {NULL};
	struct bnx2x_vlan_mac_obj *mac_obj = &bp->fp->mac_obj;

	/***************** Cleanup MACs' object first *************************/

	/* Wait for completion of requested */
	__set_bit(RAMROD_COMP_WAIT, &ramrod_flags);
	/* Perform a dry cleanup */
	__set_bit(RAMROD_DRV_CLR_ONLY, &ramrod_flags);

	/* Clean ETH primary MAC */
	__set_bit(BNX2X_ETH_MAC, &vlan_mac_flags);
	rc = mac_obj->delete_all(bp, &bp->fp->mac_obj, &vlan_mac_flags,
				 &ramrod_flags);
	if (rc != 0)
		BNX2X_ERR("Failed to clean ETH MACs: %d\n", rc);

	/* Cleanup UC list */
	vlan_mac_flags = 0;
	__set_bit(BNX2X_UC_LIST_MAC, &vlan_mac_flags);
	rc = mac_obj->delete_all(bp, mac_obj, &vlan_mac_flags,
				 &ramrod_flags);
	if (rc != 0)
		BNX2X_ERR("Failed to clean UC list MACs: %d\n", rc);

	/***************** Now clean mcast object *****************************/
	rparam.mcast_obj = &bp->mcast_obj;
	__set_bit(RAMROD_DRV_CLR_ONLY, &rparam.ramrod_flags);

	/* Add a DEL command... */
	rc = bnx2x_config_mcast(bp, &rparam, BNX2X_MCAST_CMD_DEL);
	if (rc < 0)
		BNX2X_ERR("Failed to add a new DEL command to a multi-cast object: %d\n",
			  rc);

	/* ...and wait until all pending commands are cleared */
	rc = bnx2x_config_mcast(bp, &rparam, BNX2X_MCAST_CMD_CONT);
	while (rc != 0) {
		if (rc < 0) {
			BNX2X_ERR("Failed to clean multi-cast object: %d\n",
				  rc);
			return;
		}

		rc = bnx2x_config_mcast(bp, &rparam, BNX2X_MCAST_CMD_CONT);
	}
}

#ifndef BNX2X_STOP_ON_ERROR
#define LOAD_ERROR_EXIT(bp, label) \
	do { \
		(bp)->state = BNX2X_STATE_ERROR; \
		goto label; \
	} while (0)
#else
#define LOAD_ERROR_EXIT(bp, label) \
	do { \
		(bp)->state = BNX2X_STATE_ERROR; \
		(bp)->panic = 1; \
		return -EBUSY; \
	} while (0)
#endif

bool bnx2x_test_firmware_version(struct bnx2x *bp, bool is_err)
{
	/* build FW version dword */
	u32 my_fw = (BCM_5710_FW_MAJOR_VERSION) +
		    (BCM_5710_FW_MINOR_VERSION << 8) +
		    (BCM_5710_FW_REVISION_VERSION << 16) +
		    (BCM_5710_FW_ENGINEERING_VERSION << 24);

	/* read loaded FW from chip */
	u32 loaded_fw = REG_RD(bp, XSEM_REG_PRAM);

	DP(NETIF_MSG_IFUP, "loaded fw %x, my fw %x\n", loaded_fw, my_fw);

	if (loaded_fw != my_fw) {
		if (is_err)
			BNX2X_ERR("bnx2x with FW %x was already loaded, which mismatches my %x FW. aborting\n",
				  loaded_fw, my_fw);
		return false;
	}

	return true;
}

/* must be called with rtnl_lock */
int bnx2x_nic_load(struct bnx2x *bp, int load_mode)
{
	int port = BP_PORT(bp);
	u32 load_code;
	int i, rc;

#ifdef BNX2X_STOP_ON_ERROR
	if (unlikely(bp->panic)) {
		BNX2X_ERR("Can't load NIC when there is panic\n");
		return -EPERM;
	}
#endif

	bp->state = BNX2X_STATE_OPENING_WAIT4_LOAD;

	/* Set the initial link reported state to link down */
	bnx2x_acquire_phy_lock(bp);
	memset(&bp->last_reported_link, 0, sizeof(bp->last_reported_link));
	__set_bit(BNX2X_LINK_REPORT_LINK_DOWN,
		&bp->last_reported_link.link_report_flags);
	bnx2x_release_phy_lock(bp);

	/* must be called before memory allocation and HW init */
	bnx2x_ilt_set_info(bp);

	/*
	 * Zero fastpath structures preserving invariants like napi, which are
	 * allocated only once, fp index, max_cos, bp pointer.
	 * Also set fp->disable_tpa.
	 */
	DP(NETIF_MSG_IFUP, "num queues: %d", bp->num_queues);
	for_each_queue(bp, i)
		bnx2x_bz_fp(bp, i);


	/* Set the receive queues buffer size */
	bnx2x_set_rx_buf_size(bp);

	if (bnx2x_alloc_mem(bp))
		return -ENOMEM;

	/* As long as bnx2x_alloc_mem() may possibly update
	 * bp->num_queues, bnx2x_set_real_num_queues() should always
	 * come after it.
	 */
	rc = bnx2x_set_real_num_queues(bp);
	if (rc) {
		BNX2X_ERR("Unable to set real_num_queues\n");
		LOAD_ERROR_EXIT(bp, load_error0);
	}

	/* configure multi cos mappings in kernel.
	 * this configuration may be overriden by a multi class queue discipline
	 * or by a dcbx negotiation result.
	 */
	bnx2x_setup_tc(bp->dev, bp->max_cos);

	bnx2x_napi_enable(bp);

	/* set pf load just before approaching the MCP */
	bnx2x_set_pf_load(bp);

	/* Send LOAD_REQUEST command to MCP
	 * Returns the type of LOAD command:
	 * if it is the first port to be initialized
	 * common blocks should be initialized, otherwise - not
	 */
	if (!BP_NOMCP(bp)) {
		/* init fw_seq */
		bp->fw_seq =
			(SHMEM_RD(bp, func_mb[BP_FW_MB_IDX(bp)].drv_mb_header) &
			 DRV_MSG_SEQ_NUMBER_MASK);
		BNX2X_DEV_INFO("fw_seq 0x%08x\n", bp->fw_seq);

		/* Get current FW pulse sequence */
		bp->fw_drv_pulse_wr_seq =
			(SHMEM_RD(bp, func_mb[BP_FW_MB_IDX(bp)].drv_pulse_mb) &
			 DRV_PULSE_SEQ_MASK);
		BNX2X_DEV_INFO("drv_pulse 0x%x\n", bp->fw_drv_pulse_wr_seq);

		load_code = bnx2x_fw_command(bp, DRV_MSG_CODE_LOAD_REQ, 0);
		if (!load_code) {
			BNX2X_ERR("MCP response failure, aborting\n");
			rc = -EBUSY;
			LOAD_ERROR_EXIT(bp, load_error1);
		}
		if (load_code == FW_MSG_CODE_DRV_LOAD_REFUSED) {
			BNX2X_ERR("Driver load refused\n");
			rc = -EBUSY; /* other port in diagnostic mode */
			LOAD_ERROR_EXIT(bp, load_error1);
		}
		if (load_code != FW_MSG_CODE_DRV_LOAD_COMMON_CHIP &&
		    load_code != FW_MSG_CODE_DRV_LOAD_COMMON) {
			/* abort nic load if version mismatch */
			if (!bnx2x_test_firmware_version(bp, true)) {
				rc = -EBUSY;
				LOAD_ERROR_EXIT(bp, load_error2);
			}
		}

	} else {
		int path = BP_PATH(bp);

		DP(NETIF_MSG_IFUP, "NO MCP - load counts[%d]      %d, %d, %d\n",
		   path, load_count[path][0], load_count[path][1],
		   load_count[path][2]);
		load_count[path][0]++;
		load_count[path][1 + port]++;
		DP(NETIF_MSG_IFUP, "NO MCP - new load counts[%d]  %d, %d, %d\n",
		   path, load_count[path][0], load_count[path][1],
		   load_count[path][2]);
		if (load_count[path][0] == 1)
			load_code = FW_MSG_CODE_DRV_LOAD_COMMON;
		else if (load_count[path][1 + port] == 1)
			load_code = FW_MSG_CODE_DRV_LOAD_PORT;
		else
			load_code = FW_MSG_CODE_DRV_LOAD_FUNCTION;
	}

	if ((load_code == FW_MSG_CODE_DRV_LOAD_COMMON) ||
	    (load_code == FW_MSG_CODE_DRV_LOAD_COMMON_CHIP) ||
	    (load_code == FW_MSG_CODE_DRV_LOAD_PORT)) {
		bp->port.pmf = 1;
		/*
		 * We need the barrier to ensure the ordering between the
		 * writing to bp->port.pmf here and reading it from the
		 * bnx2x_periodic_task().
		 */
		smp_mb();
	} else
		bp->port.pmf = 0;

	DP(NETIF_MSG_IFUP, "pmf %d\n", bp->port.pmf);

	/* Init Function state controlling object */
	bnx2x__init_func_obj(bp);

	/* Initialize HW */
	rc = bnx2x_init_hw(bp, load_code);
	if (rc) {
		BNX2X_ERR("HW init failed, aborting\n");
		bnx2x_fw_command(bp, DRV_MSG_CODE_LOAD_DONE, 0);
		LOAD_ERROR_EXIT(bp, load_error2);
	}

	/* Connect to IRQs */
	rc = bnx2x_setup_irqs(bp);
	if (rc) {
		BNX2X_ERR("IRQs setup failed\n");
		bnx2x_fw_command(bp, DRV_MSG_CODE_LOAD_DONE, 0);
		LOAD_ERROR_EXIT(bp, load_error2);
	}

	/* Setup NIC internals and enable interrupts */
	bnx2x_nic_init(bp, load_code);

	/* Init per-function objects */
	bnx2x_init_bp_objs(bp);

	if (((load_code == FW_MSG_CODE_DRV_LOAD_COMMON) ||
	    (load_code == FW_MSG_CODE_DRV_LOAD_COMMON_CHIP)) &&
	    (bp->common.shmem2_base)) {
		if (SHMEM2_HAS(bp, dcc_support))
			SHMEM2_WR(bp, dcc_support,
				  (SHMEM_DCC_SUPPORT_DISABLE_ENABLE_PF_TLV |
				   SHMEM_DCC_SUPPORT_BANDWIDTH_ALLOCATION_TLV));
	}

	bp->state = BNX2X_STATE_OPENING_WAIT4_PORT;
	rc = bnx2x_func_start(bp);
	if (rc) {
		BNX2X_ERR("Function start failed!\n");
		bnx2x_fw_command(bp, DRV_MSG_CODE_LOAD_DONE, 0);
		LOAD_ERROR_EXIT(bp, load_error3);
	}

	/* Send LOAD_DONE command to MCP */
	if (!BP_NOMCP(bp)) {
		load_code = bnx2x_fw_command(bp, DRV_MSG_CODE_LOAD_DONE, 0);
		if (!load_code) {
			BNX2X_ERR("MCP response failure, aborting\n");
			rc = -EBUSY;
			LOAD_ERROR_EXIT(bp, load_error3);
		}
	}

	rc = bnx2x_setup_leading(bp);
	if (rc) {
		BNX2X_ERR("Setup leading failed!\n");
		LOAD_ERROR_EXIT(bp, load_error3);
	}

#ifdef BCM_CNIC
	/* Enable Timer scan */
	REG_WR(bp, TM_REG_EN_LINEAR0_TIMER + port*4, 1);
#endif

	for_each_nondefault_queue(bp, i) {
		rc = bnx2x_setup_queue(bp, &bp->fp[i], 0);
		if (rc) {
			BNX2X_ERR("Queue setup failed\n");
			LOAD_ERROR_EXIT(bp, load_error4);
		}
	}

	rc = bnx2x_init_rss_pf(bp);
	if (rc) {
		BNX2X_ERR("PF RSS init failed\n");
		LOAD_ERROR_EXIT(bp, load_error4);
	}

	/* Now when Clients are configured we are ready to work */
	bp->state = BNX2X_STATE_OPEN;

	/* Configure a ucast MAC */
	rc = bnx2x_set_eth_mac(bp, true);
	if (rc) {
		BNX2X_ERR("Setting Ethernet MAC failed\n");
		LOAD_ERROR_EXIT(bp, load_error4);
	}

	if (bp->pending_max) {
		bnx2x_update_max_mf_config(bp, bp->pending_max);
		bp->pending_max = 0;
	}

	if (bp->port.pmf)
		bnx2x_initial_phy_init(bp, load_mode);

	/* Start fast path */

	/* Initialize Rx filter. */
	netif_addr_lock_bh(bp->dev);
	bnx2x_set_rx_mode(bp->dev);
	netif_addr_unlock_bh(bp->dev);

	/* Start the Tx */
	switch (load_mode) {
	case LOAD_NORMAL:
		/* Tx queue should be only reenabled */
		netif_tx_wake_all_queues(bp->dev);
		break;

	case LOAD_OPEN:
		netif_tx_start_all_queues(bp->dev);
		smp_mb__after_clear_bit();
		break;

	case LOAD_DIAG:
		bp->state = BNX2X_STATE_DIAG;
		break;

	default:
		break;
	}

	if (bp->port.pmf)
		bnx2x_update_drv_flags(bp, 1 << DRV_FLAGS_DCB_CONFIGURED, 0);
	else
		bnx2x__link_status_update(bp);

	/* start the timer */
	mod_timer(&bp->timer, jiffies + bp->current_interval);

#ifdef BCM_CNIC
	/* re-read iscsi info */
	bnx2x_get_iscsi_info(bp);
	bnx2x_setup_cnic_irq_info(bp);
	if (bp->state == BNX2X_STATE_OPEN)
		bnx2x_cnic_notify(bp, CNIC_CTL_START_CMD);
#endif

	/* mark driver is loaded in shmem2 */
	if (SHMEM2_HAS(bp, drv_capabilities_flag)) {
		u32 val;
		val = SHMEM2_RD(bp, drv_capabilities_flag[BP_FW_MB_IDX(bp)]);
		SHMEM2_WR(bp, drv_capabilities_flag[BP_FW_MB_IDX(bp)],
			  val | DRV_FLAGS_CAPABILITIES_LOADED_SUPPORTED |
			  DRV_FLAGS_CAPABILITIES_LOADED_L2);
	}

	/* Wait for all pending SP commands to complete */
	if (!bnx2x_wait_sp_comp(bp, ~0x0UL)) {
		BNX2X_ERR("Timeout waiting for SP elements to complete\n");
		bnx2x_nic_unload(bp, UNLOAD_CLOSE);
		return -EBUSY;
	}

	bnx2x_dcbx_init(bp);
	return 0;

#ifndef BNX2X_STOP_ON_ERROR
load_error4:
#ifdef BCM_CNIC
	/* Disable Timer scan */
	REG_WR(bp, TM_REG_EN_LINEAR0_TIMER + port*4, 0);
#endif
load_error3:
	bnx2x_int_disable_sync(bp, 1);

	/* Clean queueable objects */
	bnx2x_squeeze_objects(bp);

	/* Free SKBs, SGEs, TPA pool and driver internals */
	bnx2x_free_skbs(bp);
	for_each_rx_queue(bp, i)
		bnx2x_free_rx_sge_range(bp, bp->fp + i, NUM_RX_SGE);

	/* Release IRQs */
	bnx2x_free_irq(bp);
load_error2:
	if (!BP_NOMCP(bp)) {
		bnx2x_fw_command(bp, DRV_MSG_CODE_UNLOAD_REQ_WOL_MCP, 0);
		bnx2x_fw_command(bp, DRV_MSG_CODE_UNLOAD_DONE, 0);
	}

	bp->port.pmf = 0;
load_error1:
	bnx2x_napi_disable(bp);
	/* clear pf_load status, as it was already set */
	bnx2x_clear_pf_load(bp);
load_error0:
	bnx2x_free_mem(bp);

	return rc;
#endif /* ! BNX2X_STOP_ON_ERROR */
}

/* must be called with rtnl_lock */
int bnx2x_nic_unload(struct bnx2x *bp, int unload_mode)
{
	int i;
	bool global = false;

	/* mark driver is unloaded in shmem2 */
	if (SHMEM2_HAS(bp, drv_capabilities_flag)) {
		u32 val;
		val = SHMEM2_RD(bp, drv_capabilities_flag[BP_FW_MB_IDX(bp)]);
		SHMEM2_WR(bp, drv_capabilities_flag[BP_FW_MB_IDX(bp)],
			  val & ~DRV_FLAGS_CAPABILITIES_LOADED_L2);
	}

	if ((bp->state == BNX2X_STATE_CLOSED) ||
	    (bp->state == BNX2X_STATE_ERROR)) {
		/* We can get here if the driver has been unloaded
		 * during parity error recovery and is either waiting for a
		 * leader to complete or for other functions to unload and
		 * then ifdown has been issued. In this case we want to
		 * unload and let other functions to complete a recovery
		 * process.
		 */
		bp->recovery_state = BNX2X_RECOVERY_DONE;
		bp->is_leader = 0;
		bnx2x_release_leader_lock(bp);
		smp_mb();

		DP(NETIF_MSG_IFDOWN, "Releasing a leadership...\n");
		BNX2X_ERR("Can't unload in closed or error state\n");
		return -EINVAL;
	}

	/*
	 * It's important to set the bp->state to the value different from
	 * BNX2X_STATE_OPEN and only then stop the Tx. Otherwise bnx2x_tx_int()
	 * may restart the Tx from the NAPI context (see bnx2x_tx_int()).
	 */
	bp->state = BNX2X_STATE_CLOSING_WAIT4_HALT;
	smp_mb();

	/* Stop Tx */
	bnx2x_tx_disable(bp);

#ifdef BCM_CNIC
	bnx2x_cnic_notify(bp, CNIC_CTL_STOP_CMD);
#endif

	bp->rx_mode = BNX2X_RX_MODE_NONE;

	del_timer_sync(&bp->timer);

	/* Set ALWAYS_ALIVE bit in shmem */
	bp->fw_drv_pulse_wr_seq |= DRV_PULSE_ALWAYS_ALIVE;

	bnx2x_drv_pulse(bp);

	bnx2x_stats_handle(bp, STATS_EVENT_STOP);
	bnx2x_save_statistics(bp);

	/* Cleanup the chip if needed */
	if (unload_mode != UNLOAD_RECOVERY)
		bnx2x_chip_cleanup(bp, unload_mode);
	else {
		/* Send the UNLOAD_REQUEST to the MCP */
		bnx2x_send_unload_req(bp, unload_mode);

		/*
		 * Prevent transactions to host from the functions on the
		 * engine that doesn't reset global blocks in case of global
		 * attention once gloabl blocks are reset and gates are opened
		 * (the engine which leader will perform the recovery
		 * last).
		 */
		if (!CHIP_IS_E1x(bp))
			bnx2x_pf_disable(bp);

		/* Disable HW interrupts, NAPI */
		bnx2x_netif_stop(bp, 1);

		/* Release IRQs */
		bnx2x_free_irq(bp);

		/* Report UNLOAD_DONE to MCP */
		bnx2x_send_unload_done(bp);
	}

	/*
	 * At this stage no more interrupts will arrive so we may safly clean
	 * the queueable objects here in case they failed to get cleaned so far.
	 */
	bnx2x_squeeze_objects(bp);

	/* There should be no more pending SP commands at this stage */
	bp->sp_state = 0;

	bp->port.pmf = 0;

	/* Free SKBs, SGEs, TPA pool and driver internals */
	bnx2x_free_skbs(bp);
	for_each_rx_queue(bp, i)
		bnx2x_free_rx_sge_range(bp, bp->fp + i, NUM_RX_SGE);

	bnx2x_free_mem(bp);

	bp->state = BNX2X_STATE_CLOSED;

	/* Check if there are pending parity attentions. If there are - set
	 * RECOVERY_IN_PROGRESS.
	 */
	if (bnx2x_chk_parity_attn(bp, &global, false)) {
		bnx2x_set_reset_in_progress(bp);

		/* Set RESET_IS_GLOBAL if needed */
		if (global)
			bnx2x_set_reset_global(bp);
	}


	/* The last driver must disable a "close the gate" if there is no
	 * parity attention or "process kill" pending.
	 */
	if (!bnx2x_clear_pf_load(bp) && bnx2x_reset_is_done(bp, BP_PATH(bp)))
		bnx2x_disable_close_the_gate(bp);

	return 0;
}

int bnx2x_set_power_state(struct bnx2x *bp, pci_power_t state)
{
	u16 pmcsr;

	/* If there is no power capability, silently succeed */
	if (!bp->pm_cap) {
		BNX2X_DEV_INFO("No power capability. Breaking.\n");
		return 0;
	}

	pci_read_config_word(bp->pdev, bp->pm_cap + PCI_PM_CTRL, &pmcsr);

	switch (state) {
	case PCI_D0:
		pci_write_config_word(bp->pdev, bp->pm_cap + PCI_PM_CTRL,
				      ((pmcsr & ~PCI_PM_CTRL_STATE_MASK) |
				       PCI_PM_CTRL_PME_STATUS));

		if (pmcsr & PCI_PM_CTRL_STATE_MASK)
			/* delay required during transition out of D3hot */
			msleep(20);
		break;

	case PCI_D3hot:
		/* If there are other clients above don't
		   shut down the power */
		if (atomic_read(&bp->pdev->enable_cnt) != 1)
			return 0;
		/* Don't shut down the power for emulation and FPGA */
		if (CHIP_REV_IS_SLOW(bp))
			return 0;

		pmcsr &= ~PCI_PM_CTRL_STATE_MASK;
		pmcsr |= 3;

		if (bp->wol)
			pmcsr |= PCI_PM_CTRL_PME_ENABLE;

		pci_write_config_word(bp->pdev, bp->pm_cap + PCI_PM_CTRL,
				      pmcsr);

		/* No more memory access after this point until
		* device is brought back to D0.
		*/
		break;

	default:
		dev_err(&bp->pdev->dev, "Can't support state = %d\n", state);
		return -EINVAL;
	}
	return 0;
}

/*
 * net_device service functions
 */
int bnx2x_poll(struct napi_struct *napi, int budget)
{
	int work_done = 0;
	u8 cos;
	struct bnx2x_fastpath *fp = container_of(napi, struct bnx2x_fastpath,
						 napi);
	struct bnx2x *bp = fp->bp;

	while (1) {
#ifdef BNX2X_STOP_ON_ERROR
		if (unlikely(bp->panic)) {
			napi_complete(napi);
			return 0;
		}
#endif

		for_each_cos_in_tx_queue(fp, cos)
			if (bnx2x_tx_queue_has_work(&fp->txdata[cos]))
				bnx2x_tx_int(bp, &fp->txdata[cos]);


		if (bnx2x_has_rx_work(fp)) {
			work_done += bnx2x_rx_int(fp, budget - work_done);

			/* must not complete if we consumed full budget */
			if (work_done >= budget)
				break;
		}

		/* Fall out from the NAPI loop if needed */
		if (!(bnx2x_has_rx_work(fp) || bnx2x_has_tx_work(fp))) {
#ifdef BCM_CNIC
			/* No need to update SB for FCoE L2 ring as long as
			 * it's connected to the default SB and the SB
			 * has been updated when NAPI was scheduled.
			 */
			if (IS_FCOE_FP(fp)) {
				napi_complete(napi);
				break;
			}
#endif

			bnx2x_update_fpsb_idx(fp);
			/* bnx2x_has_rx_work() reads the status block,
			 * thus we need to ensure that status block indices
			 * have been actually read (bnx2x_update_fpsb_idx)
			 * prior to this check (bnx2x_has_rx_work) so that
			 * we won't write the "newer" value of the status block
			 * to IGU (if there was a DMA right after
			 * bnx2x_has_rx_work and if there is no rmb, the memory
			 * reading (bnx2x_update_fpsb_idx) may be postponed
			 * to right before bnx2x_ack_sb). In this case there
			 * will never be another interrupt until there is
			 * another update of the status block, while there
			 * is still unhandled work.
			 */
			rmb();

			if (!(bnx2x_has_rx_work(fp) || bnx2x_has_tx_work(fp))) {
				napi_complete(napi);
				/* Re-enable interrupts */
				DP(NETIF_MSG_RX_STATUS,
				   "Update index to %d\n", fp->fp_hc_idx);
				bnx2x_ack_sb(bp, fp->igu_sb_id, USTORM_ID,
					     le16_to_cpu(fp->fp_hc_idx),
					     IGU_INT_ENABLE, 1);
				break;
			}
		}
	}

	return work_done;
}

/* we split the first BD into headers and data BDs
 * to ease the pain of our fellow microcode engineers
 * we use one mapping for both BDs
 * So far this has only been observed to happen
 * in Other Operating Systems(TM)
 */
static noinline u16 bnx2x_tx_split(struct bnx2x *bp,
				   struct bnx2x_fp_txdata *txdata,
				   struct sw_tx_bd *tx_buf,
				   struct eth_tx_start_bd **tx_bd, u16 hlen,
				   u16 bd_prod, int nbd)
{
	struct eth_tx_start_bd *h_tx_bd = *tx_bd;
	struct eth_tx_bd *d_tx_bd;
	dma_addr_t mapping;
	int old_len = le16_to_cpu(h_tx_bd->nbytes);

	/* first fix first BD */
	h_tx_bd->nbd = cpu_to_le16(nbd);
	h_tx_bd->nbytes = cpu_to_le16(hlen);

	DP(NETIF_MSG_TX_QUEUED,	"TSO split header size is %d (%x:%x) nbd %d\n",
	   h_tx_bd->nbytes, h_tx_bd->addr_hi, h_tx_bd->addr_lo, h_tx_bd->nbd);

	/* now get a new data BD
	 * (after the pbd) and fill it */
	bd_prod = TX_BD(NEXT_TX_IDX(bd_prod));
	d_tx_bd = &txdata->tx_desc_ring[bd_prod].reg_bd;

	mapping = HILO_U64(le32_to_cpu(h_tx_bd->addr_hi),
			   le32_to_cpu(h_tx_bd->addr_lo)) + hlen;

	d_tx_bd->addr_hi = cpu_to_le32(U64_HI(mapping));
	d_tx_bd->addr_lo = cpu_to_le32(U64_LO(mapping));
	d_tx_bd->nbytes = cpu_to_le16(old_len - hlen);

	/* this marks the BD as one that has no individual mapping */
	tx_buf->flags |= BNX2X_TSO_SPLIT_BD;

	DP(NETIF_MSG_TX_QUEUED,
	   "TSO split data size is %d (%x:%x)\n",
	   d_tx_bd->nbytes, d_tx_bd->addr_hi, d_tx_bd->addr_lo);

	/* update tx_bd */
	*tx_bd = (struct eth_tx_start_bd *)d_tx_bd;

	return bd_prod;
}

static inline u16 bnx2x_csum_fix(unsigned char *t_header, u16 csum, s8 fix)
{
	if (fix > 0)
		csum = (u16) ~csum_fold(csum_sub(csum,
				csum_partial(t_header - fix, fix, 0)));

	else if (fix < 0)
		csum = (u16) ~csum_fold(csum_add(csum,
				csum_partial(t_header, -fix, 0)));

	return swab16(csum);
}

static inline u32 bnx2x_xmit_type(struct bnx2x *bp, struct sk_buff *skb)
{
	u32 rc;

	if (skb->ip_summed != CHECKSUM_PARTIAL)
		rc = XMIT_PLAIN;

	else {
		if (vlan_get_protocol(skb) == htons(ETH_P_IPV6)) {
			rc = XMIT_CSUM_V6;
			if (ipv6_hdr(skb)->nexthdr == IPPROTO_TCP)
				rc |= XMIT_CSUM_TCP;

		} else {
			rc = XMIT_CSUM_V4;
			if (ip_hdr(skb)->protocol == IPPROTO_TCP)
				rc |= XMIT_CSUM_TCP;
		}
	}

	if (skb_is_gso_v6(skb))
		rc |= XMIT_GSO_V6 | XMIT_CSUM_TCP | XMIT_CSUM_V6;
	else if (skb_is_gso(skb))
		rc |= XMIT_GSO_V4 | XMIT_CSUM_V4 | XMIT_CSUM_TCP;

	return rc;
}

#if (MAX_SKB_FRAGS >= MAX_FETCH_BD - 3)
/* check if packet requires linearization (packet is too fragmented)
   no need to check fragmentation if page size > 8K (there will be no
   violation to FW restrictions) */
static int bnx2x_pkt_req_lin(struct bnx2x *bp, struct sk_buff *skb,
			     u32 xmit_type)
{
	int to_copy = 0;
	int hlen = 0;
	int first_bd_sz = 0;

	/* 3 = 1 (for linear data BD) + 2 (for PBD and last BD) */
	if (skb_shinfo(skb)->nr_frags >= (MAX_FETCH_BD - 3)) {

		if (xmit_type & XMIT_GSO) {
			unsigned short lso_mss = skb_shinfo(skb)->gso_size;
			/* Check if LSO packet needs to be copied:
			   3 = 1 (for headers BD) + 2 (for PBD and last BD) */
			int wnd_size = MAX_FETCH_BD - 3;
			/* Number of windows to check */
			int num_wnds = skb_shinfo(skb)->nr_frags - wnd_size;
			int wnd_idx = 0;
			int frag_idx = 0;
			u32 wnd_sum = 0;

			/* Headers length */
			hlen = (int)(skb_transport_header(skb) - skb->data) +
				tcp_hdrlen(skb);

			/* Amount of data (w/o headers) on linear part of SKB*/
			first_bd_sz = skb_headlen(skb) - hlen;

			wnd_sum  = first_bd_sz;

			/* Calculate the first sum - it's special */
			for (frag_idx = 0; frag_idx < wnd_size - 1; frag_idx++)
				wnd_sum +=
					skb_frag_size(&skb_shinfo(skb)->frags[frag_idx]);

			/* If there was data on linear skb data - check it */
			if (first_bd_sz > 0) {
				if (unlikely(wnd_sum < lso_mss)) {
					to_copy = 1;
					goto exit_lbl;
				}

				wnd_sum -= first_bd_sz;
			}

			/* Others are easier: run through the frag list and
			   check all windows */
			for (wnd_idx = 0; wnd_idx <= num_wnds; wnd_idx++) {
				wnd_sum +=
			  skb_frag_size(&skb_shinfo(skb)->frags[wnd_idx + wnd_size - 1]);

				if (unlikely(wnd_sum < lso_mss)) {
					to_copy = 1;
					break;
				}
				wnd_sum -=
					skb_frag_size(&skb_shinfo(skb)->frags[wnd_idx]);
			}
		} else {
			/* in non-LSO too fragmented packet should always
			   be linearized */
			to_copy = 1;
		}
	}

exit_lbl:
	if (unlikely(to_copy))
		DP(NETIF_MSG_TX_QUEUED,
		   "Linearization IS REQUIRED for %s packet. num_frags %d  hlen %d  first_bd_sz %d\n",
		   (xmit_type & XMIT_GSO) ? "LSO" : "non-LSO",
		   skb_shinfo(skb)->nr_frags, hlen, first_bd_sz);

	return to_copy;
}
#endif

static inline void bnx2x_set_pbd_gso_e2(struct sk_buff *skb, u32 *parsing_data,
					u32 xmit_type)
{
	*parsing_data |= (skb_shinfo(skb)->gso_size <<
			      ETH_TX_PARSE_BD_E2_LSO_MSS_SHIFT) &
			      ETH_TX_PARSE_BD_E2_LSO_MSS;
	if ((xmit_type & XMIT_GSO_V6) &&
	    (ipv6_hdr(skb)->nexthdr == NEXTHDR_IPV6))
		*parsing_data |= ETH_TX_PARSE_BD_E2_IPV6_WITH_EXT_HDR;
}

/**
 * bnx2x_set_pbd_gso - update PBD in GSO case.
 *
 * @skb:	packet skb
 * @pbd:	parse BD
 * @xmit_type:	xmit flags
 */
static inline void bnx2x_set_pbd_gso(struct sk_buff *skb,
				     struct eth_tx_parse_bd_e1x *pbd,
				     u32 xmit_type)
{
	pbd->lso_mss = cpu_to_le16(skb_shinfo(skb)->gso_size);
	pbd->tcp_send_seq = swab32(tcp_hdr(skb)->seq);
	pbd->tcp_flags = pbd_tcp_flags(skb);

	if (xmit_type & XMIT_GSO_V4) {
		pbd->ip_id = swab16(ip_hdr(skb)->id);
		pbd->tcp_pseudo_csum =
			swab16(~csum_tcpudp_magic(ip_hdr(skb)->saddr,
						  ip_hdr(skb)->daddr,
						  0, IPPROTO_TCP, 0));

	} else
		pbd->tcp_pseudo_csum =
			swab16(~csum_ipv6_magic(&ipv6_hdr(skb)->saddr,
						&ipv6_hdr(skb)->daddr,
						0, IPPROTO_TCP, 0));

	pbd->global_data |= ETH_TX_PARSE_BD_E1X_PSEUDO_CS_WITHOUT_LEN;
}

/**
 * bnx2x_set_pbd_csum_e2 - update PBD with checksum and return header length
 *
 * @bp:			driver handle
 * @skb:		packet skb
 * @parsing_data:	data to be updated
 * @xmit_type:		xmit flags
 *
 * 57712 related
 */
static inline  u8 bnx2x_set_pbd_csum_e2(struct bnx2x *bp, struct sk_buff *skb,
	u32 *parsing_data, u32 xmit_type)
{
	*parsing_data |=
			((((u8 *)skb_transport_header(skb) - skb->data) >> 1) <<
			ETH_TX_PARSE_BD_E2_TCP_HDR_START_OFFSET_W_SHIFT) &
			ETH_TX_PARSE_BD_E2_TCP_HDR_START_OFFSET_W;

	if (xmit_type & XMIT_CSUM_TCP) {
		*parsing_data |= ((tcp_hdrlen(skb) / 4) <<
			ETH_TX_PARSE_BD_E2_TCP_HDR_LENGTH_DW_SHIFT) &
			ETH_TX_PARSE_BD_E2_TCP_HDR_LENGTH_DW;

		return skb_transport_header(skb) + tcp_hdrlen(skb) - skb->data;
	} else
		/* We support checksum offload for TCP and UDP only.
		 * No need to pass the UDP header length - it's a constant.
		 */
		return skb_transport_header(skb) +
				sizeof(struct udphdr) - skb->data;
}

static inline void bnx2x_set_sbd_csum(struct bnx2x *bp, struct sk_buff *skb,
	struct eth_tx_start_bd *tx_start_bd, u32 xmit_type)
{
	tx_start_bd->bd_flags.as_bitfield |= ETH_TX_BD_FLAGS_L4_CSUM;

	if (xmit_type & XMIT_CSUM_V4)
		tx_start_bd->bd_flags.as_bitfield |=
					ETH_TX_BD_FLAGS_IP_CSUM;
	else
		tx_start_bd->bd_flags.as_bitfield |=
					ETH_TX_BD_FLAGS_IPV6;

	if (!(xmit_type & XMIT_CSUM_TCP))
		tx_start_bd->bd_flags.as_bitfield |= ETH_TX_BD_FLAGS_IS_UDP;
}

/**
 * bnx2x_set_pbd_csum - update PBD with checksum and return header length
 *
 * @bp:		driver handle
 * @skb:	packet skb
 * @pbd:	parse BD to be updated
 * @xmit_type:	xmit flags
 */
static inline u8 bnx2x_set_pbd_csum(struct bnx2x *bp, struct sk_buff *skb,
	struct eth_tx_parse_bd_e1x *pbd,
	u32 xmit_type)
{
	u8 hlen = (skb_network_header(skb) - skb->data) >> 1;

	/* for now NS flag is not used in Linux */
	pbd->global_data =
		(hlen | ((skb->protocol == cpu_to_be16(ETH_P_8021Q)) <<
			 ETH_TX_PARSE_BD_E1X_LLC_SNAP_EN_SHIFT));

	pbd->ip_hlen_w = (skb_transport_header(skb) -
			skb_network_header(skb)) >> 1;

	hlen += pbd->ip_hlen_w;

	/* We support checksum offload for TCP and UDP only */
	if (xmit_type & XMIT_CSUM_TCP)
		hlen += tcp_hdrlen(skb) / 2;
	else
		hlen += sizeof(struct udphdr) / 2;

	pbd->total_hlen_w = cpu_to_le16(hlen);
	hlen = hlen*2;

	if (xmit_type & XMIT_CSUM_TCP) {
		pbd->tcp_pseudo_csum = swab16(tcp_hdr(skb)->check);

	} else {
		s8 fix = SKB_CS_OFF(skb); /* signed! */

		DP(NETIF_MSG_TX_QUEUED,
		   "hlen %d  fix %d  csum before fix %x\n",
		   le16_to_cpu(pbd->total_hlen_w), fix, SKB_CS(skb));

		/* HW bug: fixup the CSUM */
		pbd->tcp_pseudo_csum =
			bnx2x_csum_fix(skb_transport_header(skb),
				       SKB_CS(skb), fix);

		DP(NETIF_MSG_TX_QUEUED, "csum after fix %x\n",
		   pbd->tcp_pseudo_csum);
	}

	return hlen;
}

/* called with netif_tx_lock
 * bnx2x_tx_int() runs without netif_tx_lock unless it needs to call
 * netif_wake_queue()
 */
netdev_tx_t bnx2x_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct bnx2x *bp = netdev_priv(dev);

	struct bnx2x_fastpath *fp;
	struct netdev_queue *txq;
	struct bnx2x_fp_txdata *txdata;
	struct sw_tx_bd *tx_buf;
	struct eth_tx_start_bd *tx_start_bd, *first_bd;
	struct eth_tx_bd *tx_data_bd, *total_pkt_bd = NULL;
	struct eth_tx_parse_bd_e1x *pbd_e1x = NULL;
	struct eth_tx_parse_bd_e2 *pbd_e2 = NULL;
	u32 pbd_e2_parsing_data = 0;
	u16 pkt_prod, bd_prod;
	int nbd, txq_index, fp_index, txdata_index;
	dma_addr_t mapping;
	u32 xmit_type = bnx2x_xmit_type(bp, skb);
	int i;
	u8 hlen = 0;
	__le16 pkt_size = 0;
	struct ethhdr *eth;
	u8 mac_type = UNICAST_ADDRESS;

#ifdef BNX2X_STOP_ON_ERROR
	if (unlikely(bp->panic))
		return NETDEV_TX_BUSY;
#endif

	txq_index = skb_get_queue_mapping(skb);
	txq = netdev_get_tx_queue(dev, txq_index);

	BUG_ON(txq_index >= MAX_ETH_TXQ_IDX(bp) + FCOE_PRESENT);

	/* decode the fastpath index and the cos index from the txq */
	fp_index = TXQ_TO_FP(txq_index);
	txdata_index = TXQ_TO_COS(txq_index);

#ifdef BCM_CNIC
	/*
	 * Override the above for the FCoE queue:
	 *   - FCoE fp entry is right after the ETH entries.
	 *   - FCoE L2 queue uses bp->txdata[0] only.
	 */
	if (unlikely(!NO_FCOE(bp) && (txq_index ==
				      bnx2x_fcoe_tx(bp, txq_index)))) {
		fp_index = FCOE_IDX;
		txdata_index = 0;
	}
#endif

	/* enable this debug print to view the transmission queue being used
	DP(NETIF_MSG_TX_QUEUED, "indices: txq %d, fp %d, txdata %d\n",
	   txq_index, fp_index, txdata_index); */

	/* locate the fastpath and the txdata */
	fp = &bp->fp[fp_index];
	txdata = &fp->txdata[txdata_index];

	/* enable this debug print to view the tranmission details
	DP(NETIF_MSG_TX_QUEUED,
	   "transmitting packet cid %d fp index %d txdata_index %d tx_data ptr %p fp pointer %p\n",
	   txdata->cid, fp_index, txdata_index, txdata, fp); */

	if (unlikely(bnx2x_tx_avail(bp, txdata) <
		     (skb_shinfo(skb)->nr_frags + 3))) {
		fp->eth_q_stats.driver_xoff++;
		netif_tx_stop_queue(txq);
		BNX2X_ERR("BUG! Tx ring full when queue awake!\n");
		return NETDEV_TX_BUSY;
	}

	DP(NETIF_MSG_TX_QUEUED,
	   "queue[%d]: SKB: summed %x  protocol %x protocol(%x,%x) gso type %x  xmit_type %x\n",
	   txq_index, skb->ip_summed, skb->protocol, ipv6_hdr(skb)->nexthdr,
	   ip_hdr(skb)->protocol, skb_shinfo(skb)->gso_type, xmit_type);

	eth = (struct ethhdr *)skb->data;

	/* set flag according to packet type (UNICAST_ADDRESS is default)*/
	if (unlikely(is_multicast_ether_addr(eth->h_dest))) {
		if (is_broadcast_ether_addr(eth->h_dest))
			mac_type = BROADCAST_ADDRESS;
		else
			mac_type = MULTICAST_ADDRESS;
	}

#if (MAX_SKB_FRAGS >= MAX_FETCH_BD - 3)
	/* First, check if we need to linearize the skb (due to FW
	   restrictions). No need to check fragmentation if page size > 8K
	   (there will be no violation to FW restrictions) */
	if (bnx2x_pkt_req_lin(bp, skb, xmit_type)) {
		/* Statistics of linearization */
		bp->lin_cnt++;
		if (skb_linearize(skb) != 0) {
			DP(NETIF_MSG_TX_QUEUED,
			   "SKB linearization failed - silently dropping this SKB\n");
			dev_kfree_skb_any(skb);
			return NETDEV_TX_OK;
		}
	}
#endif
	/* Map skb linear data for DMA */
	mapping = dma_map_single(&bp->pdev->dev, skb->data,
				 skb_headlen(skb), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(&bp->pdev->dev, mapping))) {
		DP(NETIF_MSG_TX_QUEUED,
		   "SKB mapping failed - silently dropping this SKB\n");
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	/*
	Please read carefully. First we use one BD which we mark as start,
	then we have a parsing info BD (used for TSO or xsum),
	and only then we have the rest of the TSO BDs.
	(don't forget to mark the last one as last,
	and to unmap only AFTER you write to the BD ...)
	And above all, all pdb sizes are in words - NOT DWORDS!
	*/

	/* get current pkt produced now - advance it just before sending packet
	 * since mapping of pages may fail and cause packet to be dropped
	 */
	pkt_prod = txdata->tx_pkt_prod;
	bd_prod = TX_BD(txdata->tx_bd_prod);

	/* get a tx_buf and first BD
	 * tx_start_bd may be changed during SPLIT,
	 * but first_bd will always stay first
	 */
	tx_buf = &txdata->tx_buf_ring[TX_BD(pkt_prod)];
	tx_start_bd = &txdata->tx_desc_ring[bd_prod].start_bd;
	first_bd = tx_start_bd;

	tx_start_bd->bd_flags.as_bitfield = ETH_TX_BD_FLAGS_START_BD;
	SET_FLAG(tx_start_bd->general_data, ETH_TX_START_BD_ETH_ADDR_TYPE,
		 mac_type);

	/* header nbd */
	SET_FLAG(tx_start_bd->general_data, ETH_TX_START_BD_HDR_NBDS, 1);

	/* remember the first BD of the packet */
	tx_buf->first_bd = txdata->tx_bd_prod;
	tx_buf->skb = skb;
	tx_buf->flags = 0;

	DP(NETIF_MSG_TX_QUEUED,
	   "sending pkt %u @%p  next_idx %u  bd %u @%p\n",
	   pkt_prod, tx_buf, txdata->tx_pkt_prod, bd_prod, tx_start_bd);

	if (vlan_tx_tag_present(skb)) {
		tx_start_bd->vlan_or_ethertype =
		    cpu_to_le16(vlan_tx_tag_get(skb));
		tx_start_bd->bd_flags.as_bitfield |=
		    (X_ETH_OUTBAND_VLAN << ETH_TX_BD_FLAGS_VLAN_MODE_SHIFT);
	} else
		tx_start_bd->vlan_or_ethertype = cpu_to_le16(pkt_prod);

	/* turn on parsing and get a BD */
	bd_prod = TX_BD(NEXT_TX_IDX(bd_prod));

	if (xmit_type & XMIT_CSUM)
		bnx2x_set_sbd_csum(bp, skb, tx_start_bd, xmit_type);

	if (!CHIP_IS_E1x(bp)) {
		pbd_e2 = &txdata->tx_desc_ring[bd_prod].parse_bd_e2;
		memset(pbd_e2, 0, sizeof(struct eth_tx_parse_bd_e2));
		/* Set PBD in checksum offload case */
		if (xmit_type & XMIT_CSUM)
			hlen = bnx2x_set_pbd_csum_e2(bp, skb,
						     &pbd_e2_parsing_data,
						     xmit_type);
		if (IS_MF_SI(bp)) {
			/*
			 * fill in the MAC addresses in the PBD - for local
			 * switching
			 */
			bnx2x_set_fw_mac_addr(&pbd_e2->src_mac_addr_hi,
					      &pbd_e2->src_mac_addr_mid,
					      &pbd_e2->src_mac_addr_lo,
					      eth->h_source);
			bnx2x_set_fw_mac_addr(&pbd_e2->dst_mac_addr_hi,
					      &pbd_e2->dst_mac_addr_mid,
					      &pbd_e2->dst_mac_addr_lo,
					      eth->h_dest);
		}
	} else {
		pbd_e1x = &txdata->tx_desc_ring[bd_prod].parse_bd_e1x;
		memset(pbd_e1x, 0, sizeof(struct eth_tx_parse_bd_e1x));
		/* Set PBD in checksum offload case */
		if (xmit_type & XMIT_CSUM)
			hlen = bnx2x_set_pbd_csum(bp, skb, pbd_e1x, xmit_type);

	}

	/* Setup the data pointer of the first BD of the packet */
	tx_start_bd->addr_hi = cpu_to_le32(U64_HI(mapping));
	tx_start_bd->addr_lo = cpu_to_le32(U64_LO(mapping));
	nbd = 2; /* start_bd + pbd + frags (updated when pages are mapped) */
	tx_start_bd->nbytes = cpu_to_le16(skb_headlen(skb));
	pkt_size = tx_start_bd->nbytes;

	DP(NETIF_MSG_TX_QUEUED,
	   "first bd @%p  addr (%x:%x)  nbd %d  nbytes %d  flags %x  vlan %x\n",
	   tx_start_bd, tx_start_bd->addr_hi, tx_start_bd->addr_lo,
	   le16_to_cpu(tx_start_bd->nbd), le16_to_cpu(tx_start_bd->nbytes),
	   tx_start_bd->bd_flags.as_bitfield,
	   le16_to_cpu(tx_start_bd->vlan_or_ethertype));

	if (xmit_type & XMIT_GSO) {

		DP(NETIF_MSG_TX_QUEUED,
		   "TSO packet len %d  hlen %d  total len %d  tso size %d\n",
		   skb->len, hlen, skb_headlen(skb),
		   skb_shinfo(skb)->gso_size);

		tx_start_bd->bd_flags.as_bitfield |= ETH_TX_BD_FLAGS_SW_LSO;

		if (unlikely(skb_headlen(skb) > hlen))
			bd_prod = bnx2x_tx_split(bp, txdata, tx_buf,
						 &tx_start_bd, hlen,
						 bd_prod, ++nbd);
		if (!CHIP_IS_E1x(bp))
			bnx2x_set_pbd_gso_e2(skb, &pbd_e2_parsing_data,
					     xmit_type);
		else
			bnx2x_set_pbd_gso(skb, pbd_e1x, xmit_type);
	}

	/* Set the PBD's parsing_data field if not zero
	 * (for the chips newer than 57711).
	 */
	if (pbd_e2_parsing_data)
		pbd_e2->parsing_data = cpu_to_le32(pbd_e2_parsing_data);

	tx_data_bd = (struct eth_tx_bd *)tx_start_bd;

	/* Handle fragmented skb */
	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		mapping = skb_frag_dma_map(&bp->pdev->dev, frag, 0,
					   skb_frag_size(frag), DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(&bp->pdev->dev, mapping))) {
			unsigned int pkts_compl = 0, bytes_compl = 0;

			DP(NETIF_MSG_TX_QUEUED,
			   "Unable to map page - dropping packet...\n");

			/* we need unmap all buffers already mapped
			 * for this SKB;
			 * first_bd->nbd need to be properly updated
			 * before call to bnx2x_free_tx_pkt
			 */
			first_bd->nbd = cpu_to_le16(nbd);
			bnx2x_free_tx_pkt(bp, txdata,
					  TX_BD(txdata->tx_pkt_prod),
					  &pkts_compl, &bytes_compl);
			return NETDEV_TX_OK;
		}

		bd_prod = TX_BD(NEXT_TX_IDX(bd_prod));
		tx_data_bd = &txdata->tx_desc_ring[bd_prod].reg_bd;
		if (total_pkt_bd == NULL)
			total_pkt_bd = &txdata->tx_desc_ring[bd_prod].reg_bd;

		tx_data_bd->addr_hi = cpu_to_le32(U64_HI(mapping));
		tx_data_bd->addr_lo = cpu_to_le32(U64_LO(mapping));
		tx_data_bd->nbytes = cpu_to_le16(skb_frag_size(frag));
		le16_add_cpu(&pkt_size, skb_frag_size(frag));
		nbd++;

		DP(NETIF_MSG_TX_QUEUED,
		   "frag %d  bd @%p  addr (%x:%x)  nbytes %d\n",
		   i, tx_data_bd, tx_data_bd->addr_hi, tx_data_bd->addr_lo,
		   le16_to_cpu(tx_data_bd->nbytes));
	}

	DP(NETIF_MSG_TX_QUEUED, "last bd @%p\n", tx_data_bd);

	/* update with actual num BDs */
	first_bd->nbd = cpu_to_le16(nbd);

	bd_prod = TX_BD(NEXT_TX_IDX(bd_prod));

	/* now send a tx doorbell, counting the next BD
	 * if the packet contains or ends with it
	 */
	if (TX_BD_POFF(bd_prod) < nbd)
		nbd++;

	/* total_pkt_bytes should be set on the first data BD if
	 * it's not an LSO packet and there is more than one
	 * data BD. In this case pkt_size is limited by an MTU value.
	 * However we prefer to set it for an LSO packet (while we don't
	 * have to) in order to save some CPU cycles in a none-LSO
	 * case, when we much more care about them.
	 */
	if (total_pkt_bd != NULL)
		total_pkt_bd->total_pkt_bytes = pkt_size;

	if (pbd_e1x)
		DP(NETIF_MSG_TX_QUEUED,
		   "PBD (E1X) @%p  ip_data %x  ip_hlen %u  ip_id %u  lso_mss %u  tcp_flags %x  xsum %x  seq %u  hlen %u\n",
		   pbd_e1x, pbd_e1x->global_data, pbd_e1x->ip_hlen_w,
		   pbd_e1x->ip_id, pbd_e1x->lso_mss, pbd_e1x->tcp_flags,
		   pbd_e1x->tcp_pseudo_csum, pbd_e1x->tcp_send_seq,
		    le16_to_cpu(pbd_e1x->total_hlen_w));
	if (pbd_e2)
		DP(NETIF_MSG_TX_QUEUED,
		   "PBD (E2) @%p  dst %x %x %x src %x %x %x parsing_data %x\n",
		   pbd_e2, pbd_e2->dst_mac_addr_hi, pbd_e2->dst_mac_addr_mid,
		   pbd_e2->dst_mac_addr_lo, pbd_e2->src_mac_addr_hi,
		   pbd_e2->src_mac_addr_mid, pbd_e2->src_mac_addr_lo,
		   pbd_e2->parsing_data);
	DP(NETIF_MSG_TX_QUEUED, "doorbell: nbd %d  bd %u\n", nbd, bd_prod);

	netdev_tx_sent_queue(txq, skb->len);

	txdata->tx_pkt_prod++;
	/*
	 * Make sure that the BD data is updated before updating the producer
	 * since FW might read the BD right after the producer is updated.
	 * This is only applicable for weak-ordered memory model archs such
	 * as IA-64. The following barrier is also mandatory since FW will
	 * assumes packets must have BDs.
	 */
	wmb();

	txdata->tx_db.data.prod += nbd;
	barrier();

	DOORBELL(bp, txdata->cid, txdata->tx_db.raw);

	mmiowb();

	txdata->tx_bd_prod += nbd;

	if (unlikely(bnx2x_tx_avail(bp, txdata) < MAX_SKB_FRAGS + 3)) {
		netif_tx_stop_queue(txq);

		/* paired memory barrier is in bnx2x_tx_int(), we have to keep
		 * ordering of set_bit() in netif_tx_stop_queue() and read of
		 * fp->bd_tx_cons */
		smp_mb();

		fp->eth_q_stats.driver_xoff++;
		if (bnx2x_tx_avail(bp, txdata) >= MAX_SKB_FRAGS + 3)
			netif_tx_wake_queue(txq);
	}
	txdata->tx_pkt++;

	return NETDEV_TX_OK;
}

/**
 * bnx2x_setup_tc - routine to configure net_device for multi tc
 *
 * @netdev: net device to configure
 * @tc: number of traffic classes to enable
 *
 * callback connected to the ndo_setup_tc function pointer
 */
int bnx2x_setup_tc(struct net_device *dev, u8 num_tc)
{
	int cos, prio, count, offset;
	struct bnx2x *bp = netdev_priv(dev);

	/* setup tc must be called under rtnl lock */
	ASSERT_RTNL();

	/* no traffic classes requested. aborting */
	if (!num_tc) {
		netdev_reset_tc(dev);
		return 0;
	}

	/* requested to support too many traffic classes */
	if (num_tc > bp->max_cos) {
		BNX2X_ERR("support for too many traffic classes requested: %d. max supported is %d\n",
			  num_tc, bp->max_cos);
		return -EINVAL;
	}

	/* declare amount of supported traffic classes */
	if (netdev_set_num_tc(dev, num_tc)) {
		BNX2X_ERR("failed to declare %d traffic classes\n", num_tc);
		return -EINVAL;
	}

	/* configure priority to traffic class mapping */
	for (prio = 0; prio < BNX2X_MAX_PRIORITY; prio++) {
		netdev_set_prio_tc_map(dev, prio, bp->prio_to_cos[prio]);
		DP(BNX2X_MSG_SP | NETIF_MSG_IFUP,
		   "mapping priority %d to tc %d\n",
		   prio, bp->prio_to_cos[prio]);
	}


	/* Use this configuration to diffrentiate tc0 from other COSes
	   This can be used for ets or pfc, and save the effort of setting
	   up a multio class queue disc or negotiating DCBX with a switch
	netdev_set_prio_tc_map(dev, 0, 0);
	DP(BNX2X_MSG_SP, "mapping priority %d to tc %d\n", 0, 0);
	for (prio = 1; prio < 16; prio++) {
		netdev_set_prio_tc_map(dev, prio, 1);
		DP(BNX2X_MSG_SP, "mapping priority %d to tc %d\n", prio, 1);
	} */

	/* configure traffic class to transmission queue mapping */
	for (cos = 0; cos < bp->max_cos; cos++) {
		count = BNX2X_NUM_ETH_QUEUES(bp);
		offset = cos * MAX_TXQS_PER_COS;
		netdev_set_tc_queue(dev, cos, count, offset);
		DP(BNX2X_MSG_SP | NETIF_MSG_IFUP,
		   "mapping tc %d to offset %d count %d\n",
		   cos, offset, count);
	}

	return 0;
}

/* called with rtnl_lock */
int bnx2x_change_mac_addr(struct net_device *dev, void *p)
{
	struct sockaddr *addr = p;
	struct bnx2x *bp = netdev_priv(dev);
	int rc = 0;

	if (!bnx2x_is_valid_ether_addr(bp, addr->sa_data)) {
		BNX2X_ERR("Requested MAC address is not valid\n");
		return -EINVAL;
	}

#ifdef BCM_CNIC
	if (IS_MF_STORAGE_SD(bp) && !is_zero_ether_addr(addr->sa_data)) {
		BNX2X_ERR("Can't configure non-zero address on iSCSI or FCoE functions in MF-SD mode\n");
		return -EINVAL;
	}
#endif

	if (netif_running(dev))  {
		rc = bnx2x_set_eth_mac(bp, false);
		if (rc)
			return rc;
	}

	dev->addr_assign_type &= ~NET_ADDR_RANDOM;
	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	if (netif_running(dev))
		rc = bnx2x_set_eth_mac(bp, true);

	return rc;
}

static void bnx2x_free_fp_mem_at(struct bnx2x *bp, int fp_index)
{
	union host_hc_status_block *sb = &bnx2x_fp(bp, fp_index, status_blk);
	struct bnx2x_fastpath *fp = &bp->fp[fp_index];
	u8 cos;

	/* Common */
#ifdef BCM_CNIC
	if (IS_FCOE_IDX(fp_index)) {
		memset(sb, 0, sizeof(union host_hc_status_block));
		fp->status_blk_mapping = 0;

	} else {
#endif
		/* status blocks */
		if (!CHIP_IS_E1x(bp))
			BNX2X_PCI_FREE(sb->e2_sb,
				       bnx2x_fp(bp, fp_index,
						status_blk_mapping),
				       sizeof(struct host_hc_status_block_e2));
		else
			BNX2X_PCI_FREE(sb->e1x_sb,
				       bnx2x_fp(bp, fp_index,
						status_blk_mapping),
				       sizeof(struct host_hc_status_block_e1x));
#ifdef BCM_CNIC
	}
#endif
	/* Rx */
	if (!skip_rx_queue(bp, fp_index)) {
		bnx2x_free_rx_bds(fp);

		/* fastpath rx rings: rx_buf rx_desc rx_comp */
		BNX2X_FREE(bnx2x_fp(bp, fp_index, rx_buf_ring));
		BNX2X_PCI_FREE(bnx2x_fp(bp, fp_index, rx_desc_ring),
			       bnx2x_fp(bp, fp_index, rx_desc_mapping),
			       sizeof(struct eth_rx_bd) * NUM_RX_BD);

		BNX2X_PCI_FREE(bnx2x_fp(bp, fp_index, rx_comp_ring),
			       bnx2x_fp(bp, fp_index, rx_comp_mapping),
			       sizeof(struct eth_fast_path_rx_cqe) *
			       NUM_RCQ_BD);

		/* SGE ring */
		BNX2X_FREE(bnx2x_fp(bp, fp_index, rx_page_ring));
		BNX2X_PCI_FREE(bnx2x_fp(bp, fp_index, rx_sge_ring),
			       bnx2x_fp(bp, fp_index, rx_sge_mapping),
			       BCM_PAGE_SIZE * NUM_RX_SGE_PAGES);
	}

	/* Tx */
	if (!skip_tx_queue(bp, fp_index)) {
		/* fastpath tx rings: tx_buf tx_desc */
		for_each_cos_in_tx_queue(fp, cos) {
			struct bnx2x_fp_txdata *txdata = &fp->txdata[cos];

			DP(NETIF_MSG_IFDOWN,
			   "freeing tx memory of fp %d cos %d cid %d\n",
			   fp_index, cos, txdata->cid);

			BNX2X_FREE(txdata->tx_buf_ring);
			BNX2X_PCI_FREE(txdata->tx_desc_ring,
				txdata->tx_desc_mapping,
				sizeof(union eth_tx_bd_types) * NUM_TX_BD);
		}
	}
	/* end of fastpath */
}

void bnx2x_free_fp_mem(struct bnx2x *bp)
{
	int i;
	for_each_queue(bp, i)
		bnx2x_free_fp_mem_at(bp, i);
}

static inline void set_sb_shortcuts(struct bnx2x *bp, int index)
{
	union host_hc_status_block status_blk = bnx2x_fp(bp, index, status_blk);
	if (!CHIP_IS_E1x(bp)) {
		bnx2x_fp(bp, index, sb_index_values) =
			(__le16 *)status_blk.e2_sb->sb.index_values;
		bnx2x_fp(bp, index, sb_running_index) =
			(__le16 *)status_blk.e2_sb->sb.running_index;
	} else {
		bnx2x_fp(bp, index, sb_index_values) =
			(__le16 *)status_blk.e1x_sb->sb.index_values;
		bnx2x_fp(bp, index, sb_running_index) =
			(__le16 *)status_blk.e1x_sb->sb.running_index;
	}
}

static int bnx2x_alloc_fp_mem_at(struct bnx2x *bp, int index)
{
	union host_hc_status_block *sb;
	struct bnx2x_fastpath *fp = &bp->fp[index];
	int ring_size = 0;
	u8 cos;
	int rx_ring_size = 0;

#ifdef BCM_CNIC
	if (!bp->rx_ring_size && IS_MF_STORAGE_SD(bp)) {
		rx_ring_size = MIN_RX_SIZE_NONTPA;
		bp->rx_ring_size = rx_ring_size;
	} else
#endif
	if (!bp->rx_ring_size) {
		u32 cfg = SHMEM_RD(bp,
			     dev_info.port_hw_config[BP_PORT(bp)].default_cfg);

		rx_ring_size = MAX_RX_AVAIL/BNX2X_NUM_RX_QUEUES(bp);

		/* Dercease ring size for 1G functions */
		if ((cfg & PORT_HW_CFG_NET_SERDES_IF_MASK) ==
		    PORT_HW_CFG_NET_SERDES_IF_SGMII)
			rx_ring_size /= 10;

		/* allocate at least number of buffers required by FW */
		rx_ring_size = max_t(int, bp->disable_tpa ? MIN_RX_SIZE_NONTPA :
				     MIN_RX_SIZE_TPA, rx_ring_size);

		bp->rx_ring_size = rx_ring_size;
	} else /* if rx_ring_size specified - use it */
		rx_ring_size = bp->rx_ring_size;

	/* Common */
	sb = &bnx2x_fp(bp, index, status_blk);
#ifdef BCM_CNIC
	if (!IS_FCOE_IDX(index)) {
#endif
		/* status blocks */
		if (!CHIP_IS_E1x(bp))
			BNX2X_PCI_ALLOC(sb->e2_sb,
				&bnx2x_fp(bp, index, status_blk_mapping),
				sizeof(struct host_hc_status_block_e2));
		else
			BNX2X_PCI_ALLOC(sb->e1x_sb,
				&bnx2x_fp(bp, index, status_blk_mapping),
			    sizeof(struct host_hc_status_block_e1x));
#ifdef BCM_CNIC
	}
#endif

	/* FCoE Queue uses Default SB and doesn't ACK the SB, thus no need to
	 * set shortcuts for it.
	 */
	if (!IS_FCOE_IDX(index))
		set_sb_shortcuts(bp, index);

	/* Tx */
	if (!skip_tx_queue(bp, index)) {
		/* fastpath tx rings: tx_buf tx_desc */
		for_each_cos_in_tx_queue(fp, cos) {
			struct bnx2x_fp_txdata *txdata = &fp->txdata[cos];

			DP(NETIF_MSG_IFUP,
			   "allocating tx memory of fp %d cos %d\n",
			   index, cos);

			BNX2X_ALLOC(txdata->tx_buf_ring,
				sizeof(struct sw_tx_bd) * NUM_TX_BD);
			BNX2X_PCI_ALLOC(txdata->tx_desc_ring,
				&txdata->tx_desc_mapping,
				sizeof(union eth_tx_bd_types) * NUM_TX_BD);
		}
	}

	/* Rx */
	if (!skip_rx_queue(bp, index)) {
		/* fastpath rx rings: rx_buf rx_desc rx_comp */
		BNX2X_ALLOC(bnx2x_fp(bp, index, rx_buf_ring),
				sizeof(struct sw_rx_bd) * NUM_RX_BD);
		BNX2X_PCI_ALLOC(bnx2x_fp(bp, index, rx_desc_ring),
				&bnx2x_fp(bp, index, rx_desc_mapping),
				sizeof(struct eth_rx_bd) * NUM_RX_BD);

		BNX2X_PCI_ALLOC(bnx2x_fp(bp, index, rx_comp_ring),
				&bnx2x_fp(bp, index, rx_comp_mapping),
				sizeof(struct eth_fast_path_rx_cqe) *
				NUM_RCQ_BD);

		/* SGE ring */
		BNX2X_ALLOC(bnx2x_fp(bp, index, rx_page_ring),
				sizeof(struct sw_rx_page) * NUM_RX_SGE);
		BNX2X_PCI_ALLOC(bnx2x_fp(bp, index, rx_sge_ring),
				&bnx2x_fp(bp, index, rx_sge_mapping),
				BCM_PAGE_SIZE * NUM_RX_SGE_PAGES);
		/* RX BD ring */
		bnx2x_set_next_page_rx_bd(fp);

		/* CQ ring */
		bnx2x_set_next_page_rx_cq(fp);

		/* BDs */
		ring_size = bnx2x_alloc_rx_bds(fp, rx_ring_size);
		if (ring_size < rx_ring_size)
			goto alloc_mem_err;
	}

	return 0;

/* handles low memory cases */
alloc_mem_err:
	BNX2X_ERR("Unable to allocate full memory for queue %d (size %d)\n",
						index, ring_size);
	/* FW will drop all packets if queue is not big enough,
	 * In these cases we disable the queue
	 * Min size is different for OOO, TPA and non-TPA queues
	 */
	if (ring_size < (fp->disable_tpa ?
				MIN_RX_SIZE_NONTPA : MIN_RX_SIZE_TPA)) {
			/* release memory allocated for this queue */
			bnx2x_free_fp_mem_at(bp, index);
			return -ENOMEM;
	}
	return 0;
}

int bnx2x_alloc_fp_mem(struct bnx2x *bp)
{
	int i;

	/**
	 * 1. Allocate FP for leading - fatal if error
	 * 2. {CNIC} Allocate FCoE FP - fatal if error
	 * 3. {CNIC} Allocate OOO + FWD - disable OOO if error
	 * 4. Allocate RSS - fix number of queues if error
	 */

	/* leading */
	if (bnx2x_alloc_fp_mem_at(bp, 0))
		return -ENOMEM;

#ifdef BCM_CNIC
	if (!NO_FCOE(bp))
		/* FCoE */
		if (bnx2x_alloc_fp_mem_at(bp, FCOE_IDX))
			/* we will fail load process instead of mark
			 * NO_FCOE_FLAG
			 */
			return -ENOMEM;
#endif

	/* RSS */
	for_each_nondefault_eth_queue(bp, i)
		if (bnx2x_alloc_fp_mem_at(bp, i))
			break;

	/* handle memory failures */
	if (i != BNX2X_NUM_ETH_QUEUES(bp)) {
		int delta = BNX2X_NUM_ETH_QUEUES(bp) - i;

		WARN_ON(delta < 0);
#ifdef BCM_CNIC
		/**
		 * move non eth FPs next to last eth FP
		 * must be done in that order
		 * FCOE_IDX < FWD_IDX < OOO_IDX
		 */

		/* move FCoE fp even NO_FCOE_FLAG is on */
		bnx2x_move_fp(bp, FCOE_IDX, FCOE_IDX - delta);
#endif
		bp->num_queues -= delta;
		BNX2X_ERR("Adjusted num of queues from %d to %d\n",
			  bp->num_queues + delta, bp->num_queues);
	}

	return 0;
}

void bnx2x_free_mem_bp(struct bnx2x *bp)
{
	kfree(bp->fp);
	kfree(bp->msix_table);
	kfree(bp->ilt);
}

int __devinit bnx2x_alloc_mem_bp(struct bnx2x *bp)
{
	struct bnx2x_fastpath *fp;
	struct msix_entry *tbl;
	struct bnx2x_ilt *ilt;
	int msix_table_size = 0;

	/*
	 * The biggest MSI-X table we might need is as a maximum number of fast
	 * path IGU SBs plus default SB (for PF).
	 */
	msix_table_size = bp->igu_sb_cnt + 1;

	/* fp array: RSS plus CNIC related L2 queues */
	fp = kcalloc(BNX2X_MAX_RSS_COUNT(bp) + NON_ETH_CONTEXT_USE,
		     sizeof(*fp), GFP_KERNEL);
	if (!fp)
		goto alloc_err;
	bp->fp = fp;

	/* msix table */
	tbl = kcalloc(msix_table_size, sizeof(*tbl), GFP_KERNEL);
	if (!tbl)
		goto alloc_err;
	bp->msix_table = tbl;

	/* ilt */
	ilt = kzalloc(sizeof(*ilt), GFP_KERNEL);
	if (!ilt)
		goto alloc_err;
	bp->ilt = ilt;

	return 0;
alloc_err:
	bnx2x_free_mem_bp(bp);
	return -ENOMEM;

}

int bnx2x_reload_if_running(struct net_device *dev)
{
	struct bnx2x *bp = netdev_priv(dev);

	if (unlikely(!netif_running(dev)))
		return 0;

	bnx2x_nic_unload(bp, UNLOAD_NORMAL);
	return bnx2x_nic_load(bp, LOAD_NORMAL);
}

int bnx2x_get_cur_phy_idx(struct bnx2x *bp)
{
	u32 sel_phy_idx = 0;
	if (bp->link_params.num_phys <= 1)
		return INT_PHY;

	if (bp->link_vars.link_up) {
		sel_phy_idx = EXT_PHY1;
		/* In case link is SERDES, check if the EXT_PHY2 is the one */
		if ((bp->link_vars.link_status & LINK_STATUS_SERDES_LINK) &&
		    (bp->link_params.phy[EXT_PHY2].supported & SUPPORTED_FIBRE))
			sel_phy_idx = EXT_PHY2;
	} else {

		switch (bnx2x_phy_selection(&bp->link_params)) {
		case PORT_HW_CFG_PHY_SELECTION_HARDWARE_DEFAULT:
		case PORT_HW_CFG_PHY_SELECTION_FIRST_PHY:
		case PORT_HW_CFG_PHY_SELECTION_FIRST_PHY_PRIORITY:
		       sel_phy_idx = EXT_PHY1;
		       break;
		case PORT_HW_CFG_PHY_SELECTION_SECOND_PHY:
		case PORT_HW_CFG_PHY_SELECTION_SECOND_PHY_PRIORITY:
		       sel_phy_idx = EXT_PHY2;
		       break;
		}
	}

	return sel_phy_idx;

}
int bnx2x_get_link_cfg_idx(struct bnx2x *bp)
{
	u32 sel_phy_idx = bnx2x_get_cur_phy_idx(bp);
	/*
	 * The selected actived PHY is always after swapping (in case PHY
	 * swapping is enabled). So when swapping is enabled, we need to reverse
	 * the configuration
	 */

	if (bp->link_params.multi_phy_config &
	    PORT_HW_CFG_PHY_SWAPPED_ENABLED) {
		if (sel_phy_idx == EXT_PHY1)
			sel_phy_idx = EXT_PHY2;
		else if (sel_phy_idx == EXT_PHY2)
			sel_phy_idx = EXT_PHY1;
	}
	return LINK_CONFIG_IDX(sel_phy_idx);
}

#if defined(NETDEV_FCOE_WWNN) && defined(BCM_CNIC)
int bnx2x_fcoe_get_wwn(struct net_device *dev, u64 *wwn, int type)
{
	struct bnx2x *bp = netdev_priv(dev);
	struct cnic_eth_dev *cp = &bp->cnic_eth_dev;

	switch (type) {
	case NETDEV_FCOE_WWNN:
		*wwn = HILO_U64(cp->fcoe_wwn_node_name_hi,
				cp->fcoe_wwn_node_name_lo);
		break;
	case NETDEV_FCOE_WWPN:
		*wwn = HILO_U64(cp->fcoe_wwn_port_name_hi,
				cp->fcoe_wwn_port_name_lo);
		break;
	default:
		BNX2X_ERR("Wrong WWN type requested - %d\n", type);
		return -EINVAL;
	}

	return 0;
}
#endif

/* called with rtnl_lock */
int bnx2x_change_mtu(struct net_device *dev, int new_mtu)
{
	struct bnx2x *bp = netdev_priv(dev);

	if (bp->recovery_state != BNX2X_RECOVERY_DONE) {
		BNX2X_ERR("Can't perform change MTU during parity recovery\n");
		return -EAGAIN;
	}

	if ((new_mtu > ETH_MAX_JUMBO_PACKET_SIZE) ||
	    ((new_mtu + ETH_HLEN) < ETH_MIN_PACKET_SIZE)) {
		BNX2X_ERR("Can't support requested MTU size\n");
		return -EINVAL;
	}

	/* This does not race with packet allocation
	 * because the actual alloc size is
	 * only updated as part of load
	 */
	dev->mtu = new_mtu;

	bp->gro_check = bnx2x_need_gro_check(new_mtu);

	return bnx2x_reload_if_running(dev);
}

netdev_features_t bnx2x_fix_features(struct net_device *dev,
				     netdev_features_t features)
{
	struct bnx2x *bp = netdev_priv(dev);

	/* TPA requires Rx CSUM offloading */
	if (!(features & NETIF_F_RXCSUM) || bp->disable_tpa) {
		features &= ~NETIF_F_LRO;
		features &= ~NETIF_F_GRO;
	}

	return features;
}

int bnx2x_set_features(struct net_device *dev, netdev_features_t features)
{
	struct bnx2x *bp = netdev_priv(dev);
	u32 flags = bp->flags;
	bool bnx2x_reload = false;

	if (features & NETIF_F_LRO)
		flags |= TPA_ENABLE_FLAG;
	else
		flags &= ~TPA_ENABLE_FLAG;

	if (features & NETIF_F_GRO)
		flags |= GRO_ENABLE_FLAG;
	else
		flags &= ~GRO_ENABLE_FLAG;

	if (features & NETIF_F_LOOPBACK) {
		if (bp->link_params.loopback_mode != LOOPBACK_BMAC) {
			bp->link_params.loopback_mode = LOOPBACK_BMAC;
			bnx2x_reload = true;
		}
	} else {
		if (bp->link_params.loopback_mode != LOOPBACK_NONE) {
			bp->link_params.loopback_mode = LOOPBACK_NONE;
			bnx2x_reload = true;
		}
	}

	if (flags ^ bp->flags) {
		bp->flags = flags;
		bnx2x_reload = true;
	}

	if (bnx2x_reload) {
		if (bp->recovery_state == BNX2X_RECOVERY_DONE)
			return bnx2x_reload_if_running(dev);
		/* else: bnx2x_nic_load() will be called at end of recovery */
	}

	return 0;
}

void bnx2x_tx_timeout(struct net_device *dev)
{
	struct bnx2x *bp = netdev_priv(dev);

#ifdef BNX2X_STOP_ON_ERROR
	if (!bp->panic)
		bnx2x_panic();
#endif

	smp_mb__before_clear_bit();
	set_bit(BNX2X_SP_RTNL_TX_TIMEOUT, &bp->sp_rtnl_state);
	smp_mb__after_clear_bit();

	/* This allows the netif to be shutdown gracefully before resetting */
	schedule_delayed_work(&bp->sp_rtnl_task, 0);
}

int bnx2x_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct bnx2x *bp;

	if (!dev) {
		dev_err(&pdev->dev, "BAD net device from bnx2x_init_one\n");
		return -ENODEV;
	}
	bp = netdev_priv(dev);

	rtnl_lock();

	pci_save_state(pdev);

	if (!netif_running(dev)) {
		rtnl_unlock();
		return 0;
	}

	netif_device_detach(dev);

	bnx2x_nic_unload(bp, UNLOAD_CLOSE);

	bnx2x_set_power_state(bp, pci_choose_state(pdev, state));

	rtnl_unlock();

	return 0;
}

int bnx2x_resume(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct bnx2x *bp;
	int rc;

	if (!dev) {
		dev_err(&pdev->dev, "BAD net device from bnx2x_init_one\n");
		return -ENODEV;
	}
	bp = netdev_priv(dev);

	if (bp->recovery_state != BNX2X_RECOVERY_DONE) {
		BNX2X_ERR("Handling parity error recovery. Try again later\n");
		return -EAGAIN;
	}

	rtnl_lock();

	pci_restore_state(pdev);

	if (!netif_running(dev)) {
		rtnl_unlock();
		return 0;
	}

	bnx2x_set_power_state(bp, PCI_D0);
	netif_device_attach(dev);

	rc = bnx2x_nic_load(bp, LOAD_OPEN);

	rtnl_unlock();

	return rc;
}


void bnx2x_set_ctx_validation(struct bnx2x *bp, struct eth_context *cxt,
			      u32 cid)
{
	/* ustorm cxt validation */
	cxt->ustorm_ag_context.cdu_usage =
		CDU_RSRVD_VALUE_TYPE_A(HW_CID(bp, cid),
			CDU_REGION_NUMBER_UCM_AG, ETH_CONNECTION_TYPE);
	/* xcontext validation */
	cxt->xstorm_ag_context.cdu_reserved =
		CDU_RSRVD_VALUE_TYPE_A(HW_CID(bp, cid),
			CDU_REGION_NUMBER_XCM_AG, ETH_CONNECTION_TYPE);
}

static inline void storm_memset_hc_timeout(struct bnx2x *bp, u8 port,
					     u8 fw_sb_id, u8 sb_index,
					     u8 ticks)
{

	u32 addr = BAR_CSTRORM_INTMEM +
		   CSTORM_STATUS_BLOCK_DATA_TIMEOUT_OFFSET(fw_sb_id, sb_index);
	REG_WR8(bp, addr, ticks);
	DP(NETIF_MSG_IFUP,
	   "port %x fw_sb_id %d sb_index %d ticks %d\n",
	   port, fw_sb_id, sb_index, ticks);
}

static inline void storm_memset_hc_disable(struct bnx2x *bp, u8 port,
					     u16 fw_sb_id, u8 sb_index,
					     u8 disable)
{
	u32 enable_flag = disable ? 0 : (1 << HC_INDEX_DATA_HC_ENABLED_SHIFT);
	u32 addr = BAR_CSTRORM_INTMEM +
		   CSTORM_STATUS_BLOCK_DATA_FLAGS_OFFSET(fw_sb_id, sb_index);
	u16 flags = REG_RD16(bp, addr);
	/* clear and set */
	flags &= ~HC_INDEX_DATA_HC_ENABLED;
	flags |= enable_flag;
	REG_WR16(bp, addr, flags);
	DP(NETIF_MSG_IFUP,
	   "port %x fw_sb_id %d sb_index %d disable %d\n",
	   port, fw_sb_id, sb_index, disable);
}

void bnx2x_update_coalesce_sb_index(struct bnx2x *bp, u8 fw_sb_id,
				    u8 sb_index, u8 disable, u16 usec)
{
	int port = BP_PORT(bp);
	u8 ticks = usec / BNX2X_BTR;

	storm_memset_hc_timeout(bp, port, fw_sb_id, sb_index, ticks);

	disable = disable ? 1 : (usec ? 0 : 1);
	storm_memset_hc_disable(bp, port, fw_sb_id, sb_index, disable);
}
