#include "msm_vidc_debug.h"
#include "htc_msm_smem.h"

static void *htc_ion_new_client_alloc(void)
{
        struct ion_client *client = NULL;
        client = msm_ion_client_create(-1, "vcodec_alloc");
        if (!client)
                dprintk(VIDC_ERR, "Failed to create smem client\n");
        return client;
};

static void *htc_ion_new_client_import(void)
{
        struct ion_client *client = NULL;
        client = msm_ion_client_create(-1, "vcodec_impor");
        if (!client)
                dprintk(VIDC_ERR, "Failed to create smem client\n");
        return client;
};

void *htc_msm_smem_new_client(enum smem_type mtype,
                struct msm_vidc_platform_resources *res)
{
        struct smem_client *client = NULL;
        void *clnt_alloc = NULL;
        void *clnt_import = NULL;
        switch (mtype) {
        case SMEM_ION:
                clnt_alloc = htc_ion_new_client_alloc();
                clnt_import = htc_ion_new_client_import();
                break;
        default:
                dprintk(VIDC_ERR, "Mem type not supported\n");
                break;
        }
        if (clnt_alloc && clnt_import) {
                client = kzalloc(sizeof(*client), GFP_KERNEL);
                if (client) {
                        client->mem_type = mtype;
                        client->clnt = client->clnt_import = clnt_import;
                        client->clnt_alloc = clnt_alloc;
                        client->res = res;
                        client->inst = NULL;
                }
        } else {
                if (clnt_alloc == NULL) {
                        dprintk(VIDC_ERR, "Failed to create new alloc ion_client\n");
                } else {
                        ion_client_destroy(clnt_alloc);
                }

                if (clnt_import == NULL) {
                        dprintk(VIDC_ERR, "Failed to create new import ion_client\n");
                } else {
                        ion_client_destroy(clnt_import);
                }
        }
        return client;
};
