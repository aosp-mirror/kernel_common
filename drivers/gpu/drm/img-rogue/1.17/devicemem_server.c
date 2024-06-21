/*************************************************************************/ /*!
@File
@Title          Device Memory Management
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Server-side component of the Device Memory Management.
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/
/* our exported API */
#include "devicemem_server.h"
#include "devicemem_utils.h"
#include "devicemem.h"

#include "device.h" /* For device node */
#include "img_types.h"
#include "img_defs.h"
#include "pvr_debug.h"
#include "pvrsrv_error.h"

#include "mmu_common.h"
#include "pdump_km.h"
#include "pmr.h"
#include "physmem.h"
#include "pdumpdesc.h"

#include "allocmem.h"
#include "osfunc.h"
#include "lock.h"

#include "pvrsrv.h" /* for PVRSRVGetPVRSRVData() */

#define DEVMEMCTX_FLAGS_FAULT_ADDRESS_AVAILABLE (1 << 0)
#define DEVMEMHEAP_REFCOUNT_MIN 1
#define DEVMEMHEAP_REFCOUNT_MAX IMG_INT32_MAX
#define DEVMEMRESERVATION_REFCOUNT_MIN 1
#define DEVMEMRESERVATION_REFCOUNT_MAX IMG_INT32_MAX

struct _DEVMEMINT_CTX_
{
	PVRSRV_DEVICE_NODE *psDevNode;

	/* MMU common code needs to have a context. There's a one-to-one
	   correspondence between device memory context and MMU context,
	   but we have the abstraction here so that we don't need to care
	   what the MMU does with its context, and the MMU code need not
	   know about us at all. */
	MMU_CONTEXT *psMMUContext;

	ATOMIC_T hRefCount;

	/* This handle is for devices that require notification when a new
	   memory context is created and they need to store private data that
	   is associated with the context. */
	IMG_HANDLE hPrivData;

	/* Protects access to sProcessNotifyListHead */
	POSWR_LOCK hListLock;

	/* The following tracks UM applications that need to be notified of a
	 * page fault */
	DLLIST_NODE sProcessNotifyListHead;
	/* The following is a node for the list of registered devmem contexts */
	DLLIST_NODE sPageFaultNotifyListElem;

	/* Device virtual address of a page fault on this context */
	IMG_DEV_VIRTADDR sFaultAddress;

	/* General purpose flags */
	IMG_UINT32 ui32Flags;
};

struct _DEVMEMINT_CTX_EXPORT_
{
	DEVMEMINT_CTX *psDevmemCtx;
	PMR *psPMR;
	ATOMIC_T hRefCount;
	DLLIST_NODE sNode;
};

struct _DEVMEMINT_HEAP_
{
	struct _DEVMEMINT_CTX_ *psDevmemCtx;
	IMG_UINT32 uiLog2PageSize;
	IMG_DEV_VIRTADDR sBaseAddr;
	IMG_DEV_VIRTADDR sLastAddr;
	ATOMIC_T uiRefCount;
};

struct _DEVMEMINT_RESERVATION_
{
	struct _DEVMEMINT_HEAP_ *psDevmemHeap;
	IMG_DEV_VIRTADDR sBase;
	IMG_DEVMEM_SIZE_T uiLength;
	/* lock used to guard against potential race when freeing reservation */
	POS_LOCK hLock;
	IMG_INT32 i32RefCount;
};

struct _DEVMEMINT_RESERVATION2_
{
	struct _DEVMEMINT_HEAP_ *psDevmemHeap;
	IMG_DEV_VIRTADDR sBase;
	IMG_DEVMEM_SIZE_T uiLength;
	/* lock used to guard against potential race when freeing reservation */
	POS_LOCK hLock;
	IMG_INT32 i32RefCount;
	PVRSRV_MEMALLOCFLAGS_T uiFlags;

	/* We keep a reference to the single PMR associated with this reservation
	 * once mapped. When creating the reservation this is null. Used in
	 * ChangeSparse to validate parameters. We could have a sparse PMR with
	 * no backing and have it mapped to a reservation.
	 */
	PMR *psMappedPMR;

	/* Array of bitfields of size `uiNumPages / MAP_MASK_SHIFT`.
	 * This array represents the mapping between a PMR (psMappedPMR) and
	 * the reservation. Each bit represents an index of a physical page - a value
	 * 1 means the page is mapped and vice versa.
	 */
	IMG_UINT8 *pui8Map;
	#define MAP_MASK_SHIFT 3
};

struct _DEVMEMINT_MAPPING_
{
	struct _DEVMEMINT_RESERVATION_ *psReservation;
	PMR *psPMR;
	IMG_UINT32 uiNumPages;
};

/*! Object representing a virtual range reservation and mapping between
 * the virtual range and a set of PMRs.
 *
 * The physical allocations may be mapped entirely or partially to the entire
 * or partial virtual range. */
struct _DEVMEMXINT_RESERVATION_
{
	/*! Pointer to a device memory heap this reservation is made on. */
	struct _DEVMEMINT_HEAP_ *psDevmemHeap;
	/*! Base device virtual address of this reservation. */
	IMG_DEV_VIRTADDR sBase;
	/*! Size of this reservation (in bytes). */
	IMG_DEVMEM_SIZE_T uiLength;
	/*! Lock for protecting concurrent operations on the mapping. */
	POS_LOCK hLock;
	/*! Array of PMRs of size `uiNumPages`. This array represents how the
	 *  physical memory is mapped to the virtual range. Each entry in the array
	 *  represents to one device page which means that one PMR may be spread
	 *  across many indices. */
	PMR **ppsPMR;
};

struct _DEVMEMINT_PF_NOTIFY_
{
	IMG_UINT32  ui32PID;
	DLLIST_NODE sProcessNotifyListElem;
};

/** Computes division using log2 of divisor. */
#define LOG2_DIV(x, log2) ((x) >> (log2))

/** Computes modulo of a power of 2. */
#define LOG2_MOD(x, log2) ((x) & ((1 << (log2)) - 1))

static INLINE IMG_UINT32 _DevmemReservationPageCount(DEVMEMINT_RESERVATION2 *psRsrv);

/*************************************************************************/ /*!
@Function       DevmemIntReservationIsIndexMapped
@Description    Checks whether a particular index in the reservation has been
                mapped to a page in psMappedPMR.

@Return         IMG_TRUE if mapped or IMG_FALSE if not.
*/ /**************************************************************************/
static INLINE IMG_BOOL DevmemIntReservationIsIndexMapped(DEVMEMINT_RESERVATION2 *psReservation,
                                                         IMG_UINT32 ui32Index)
{
	IMG_UINT32 ui32MapIndex = LOG2_DIV(ui32Index, MAP_MASK_SHIFT);

	PVR_ASSERT(psReservation != NULL);
	PVR_ASSERT(ui32Index < _DevmemReservationPageCount(psReservation));

	return BIT_ISSET(psReservation->pui8Map[ui32MapIndex], LOG2_MOD(ui32Index, MAP_MASK_SHIFT));
}

/*************************************************************************/ /*!
@Function       DevmemIntReservationSetMappingIndex
@Description    Sets an index of the reservation map to indicate a mapped or
                unmapped PMR page.

@Note           The reservations hLock must be acquired before calling this
                function.

@Return         None
*/ /**************************************************************************/
static void DevmemIntReservationSetMappingIndex(DEVMEMINT_RESERVATION2 *psReservation,
                                                IMG_UINT32 ui32Index,
                                                IMG_BOOL bMap)
{
	IMG_UINT32 ui32MapIndex = LOG2_DIV(ui32Index, MAP_MASK_SHIFT);

	PVR_ASSERT(psReservation != NULL);
	PVR_ASSERT(ui32Index < _DevmemReservationPageCount(psReservation));

	if (bMap)
	{
		BIT_SET(psReservation->pui8Map[ui32MapIndex], LOG2_MOD(ui32Index, MAP_MASK_SHIFT));
	}
	else
	{
		BIT_UNSET(psReservation->pui8Map[ui32MapIndex], LOG2_MOD(ui32Index, MAP_MASK_SHIFT));
	}
}

/*************************************************************************/ /*!
@Function       DevmemIntCtxAcquire
@Description    Acquire a reference to the provided device memory context.
@Return         None
*/ /**************************************************************************/
static INLINE void DevmemIntCtxAcquire(DEVMEMINT_CTX *psDevmemCtx)
{
	OSAtomicIncrement(&psDevmemCtx->hRefCount);
}

/*************************************************************************/ /*!
@Function       DevmemIntCtxRelease
@Description    Release the reference to the provided device memory context.
                If this is the last reference which was taken then the
                memory context will be freed.
@Return         None
*/ /**************************************************************************/
static INLINE void DevmemIntCtxRelease(DEVMEMINT_CTX *psDevmemCtx)
{
	if (OSAtomicDecrement(&psDevmemCtx->hRefCount) == 0)
	{
		/* The last reference has gone, destroy the context */
		PVRSRV_DEVICE_NODE *psDevNode = psDevmemCtx->psDevNode;
		DLLIST_NODE *psNode, *psNodeNext;

		/* If there are any PIDs registered for page fault notification.
		 * Loop through the registered PIDs and free each one */
		dllist_foreach_node(&(psDevmemCtx->sProcessNotifyListHead), psNode, psNodeNext)
		{
			DEVMEMINT_PF_NOTIFY *psNotifyNode =
				IMG_CONTAINER_OF(psNode, DEVMEMINT_PF_NOTIFY, sProcessNotifyListElem);
			dllist_remove_node(psNode);
			OSFreeMem(psNotifyNode);
		}

		/* If this context is in the list registered for a debugger, remove
		 * from that list */
		if (dllist_node_is_in_list(&psDevmemCtx->sPageFaultNotifyListElem))
		{
			dllist_remove_node(&psDevmemCtx->sPageFaultNotifyListElem);
		}

		if (psDevNode->pfnUnregisterMemoryContext)
		{
			psDevNode->pfnUnregisterMemoryContext(psDevmemCtx->hPrivData);
		}
		MMU_ContextDestroy(psDevmemCtx->psMMUContext);

		OSWRLockDestroy(psDevmemCtx->hListLock);

		PVR_DPF((PVR_DBG_MESSAGE, "%s: Freed memory context %p",
				 __func__, psDevmemCtx));
		OSFreeMem(psDevmemCtx);
	}
}

/*************************************************************************/ /*!
@Function       DevmemIntHeapAcquire
@Description    Acquire a reference to the provided device memory heap.
@Return         IMG_TRUE if referenced and IMG_FALSE in case of error
*/ /**************************************************************************/
static INLINE IMG_BOOL DevmemIntHeapAcquire(DEVMEMINT_HEAP *psDevmemHeap)
{
	IMG_BOOL bSuccess = OSAtomicAddUnless(&psDevmemHeap->uiRefCount, 1,
	                                      DEVMEMHEAP_REFCOUNT_MAX);

	if (!bSuccess)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s(): Failed to acquire the device memory "
		         "heap, reference count has overflowed.", __func__));
		return IMG_FALSE;
	}

	return IMG_TRUE;
}

/*************************************************************************/ /*!
@Function       DevmemIntHeapRelease
@Description    Release the reference to the provided device memory heap.
                If this is the last reference which was taken then the
                memory context will be freed.
@Return         None
*/ /**************************************************************************/
static INLINE void DevmemIntHeapRelease(DEVMEMINT_HEAP *psDevmemHeap)
{
	IMG_BOOL bSuccess = OSAtomicSubtractUnless(&psDevmemHeap->uiRefCount, 1,
	                                           DEVMEMHEAP_REFCOUNT_MIN);

	if (!bSuccess)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s(): Failed to acquire the device memory "
		         "heap, reference count has underflowed.", __func__));
	}
}

PVRSRV_ERROR
DevmemIntUnpin(PMR *psPMR)
{
	PVR_UNREFERENCED_PARAMETER(psPMR);
	return PVRSRV_ERROR_NOT_IMPLEMENTED;
}

PVRSRV_ERROR
DevmemIntUnpinInvalidate(DEVMEMINT_MAPPING *psDevmemMapping, PMR *psPMR)
{
	PVR_UNREFERENCED_PARAMETER(psDevmemMapping);
	PVR_UNREFERENCED_PARAMETER(psPMR);
	return PVRSRV_ERROR_NOT_IMPLEMENTED;
}
PVRSRV_ERROR
DevmemIntPin(PMR *psPMR)
{
	PVR_UNREFERENCED_PARAMETER(psPMR);
	return PVRSRV_ERROR_NOT_IMPLEMENTED;
}

PVRSRV_ERROR
DevmemIntPinValidate(DEVMEMINT_MAPPING *psDevmemMapping, PMR *psPMR)
{
	PVR_UNREFERENCED_PARAMETER(psDevmemMapping);
	PVR_UNREFERENCED_PARAMETER(psPMR);
	return PVRSRV_ERROR_NOT_IMPLEMENTED;
}

static IMG_BOOL DevmemIntReservationAcquireUnlocked(DEVMEMINT_RESERVATION2 *psDevmemReservation)
{
#if defined(DEBUG)
	if (!OSLockIsLocked(psDevmemReservation->hLock))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Reservation is not locked", __func__));
		return IMG_FALSE;
	}
#endif

	if (psDevmemReservation->i32RefCount == DEVMEMRESERVATION_REFCOUNT_MAX)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s(): Failed to acquire the device memory "
		         "reservation, reference count has overflowed.", __func__));

		return IMG_FALSE;
	}

	psDevmemReservation->i32RefCount++;

	return IMG_TRUE;
}

static void DevmemIntReservationReleaseUnlocked(DEVMEMINT_RESERVATION2 *psDevmemReservation)
{
#if defined(DEBUG)
	if (!OSLockIsLocked(psDevmemReservation->hLock))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Reservation is not locked", __func__));
		return;
	}
#endif

	if (psDevmemReservation->i32RefCount == DEVMEMRESERVATION_REFCOUNT_MIN)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s(): Failed to release the device memory "
		         "reservation, reference count has underflowed.", __func__));

		/* for better debugging */
		PVR_ASSERT(psDevmemReservation->i32RefCount == DEVMEMRESERVATION_REFCOUNT_MIN);

		return;
	}

	psDevmemReservation->i32RefCount--;
}

/*************************************************************************/ /*!
@Function       DevmemIntReservationAcquire
@Description    Acquire a reference to the provided device memory reservation.
@Return         IMG_TRUE if referenced and IMG_FALSE in case of error
*/ /**************************************************************************/
IMG_BOOL DevmemIntReservationAcquire(DEVMEMINT_RESERVATION2 *psDevmemReservation)
{
	IMG_BOOL bSuccess;

	OSLockAcquire(psDevmemReservation->hLock);
	bSuccess = DevmemIntReservationAcquireUnlocked(psDevmemReservation);
	OSLockRelease(psDevmemReservation->hLock);

	return bSuccess;
}

/*************************************************************************/ /*!
@Function       DevmemIntReservationRelease
@Description    Release the reference to the provided device memory reservation.
                If this is the last reference which was taken then the
                reservation will be freed.
@Return         None.
*/ /**************************************************************************/
void DevmemIntReservationRelease(DEVMEMINT_RESERVATION2 *psDevmemReservation)
{
	OSLockAcquire(psDevmemReservation->hLock);
	DevmemIntReservationReleaseUnlocked(psDevmemReservation);
	OSLockRelease(psDevmemReservation->hLock);
}

/*************************************************************************/ /*!
@Function       DevmemServerGetImportHandle
@Description    For given exportable memory descriptor returns PMR handle.
@Return         Memory is exportable - Success
                PVRSRV_ERROR failure code
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemServerGetImportHandle(DEVMEM_MEMDESC *psMemDesc,
                            IMG_HANDLE *phImport)
{
	PVRSRV_ERROR eError;

	if ((GetImportProperties(psMemDesc->psImport) & DEVMEM_PROPERTIES_EXPORTABLE) == 0)
	{
		PVR_GOTO_WITH_ERROR(eError, PVRSRV_ERROR_DEVICEMEM_CANT_EXPORT_SUBALLOCATION, e0);
	}

	/* A new handle means a new import tracking the PMR.
	 * Hence the source PMR memory layout should be marked fixed
	 * to make sure the importer view of the memory is the same as
	 * the exporter throughout its lifetime */
	PMR_SetLayoutFixed((PMR *)psMemDesc->psImport->hPMR, IMG_TRUE);

	*phImport = psMemDesc->psImport->hPMR;
	return PVRSRV_OK;

e0:
	return eError;
}

/*************************************************************************/ /*!
@Function       DevmemServerGetHeapHandle
@Description    For given reservation returns the Heap handle.
@Return         PVRSRV_ERROR failure code
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemServerGetHeapHandle(DEVMEMINT_RESERVATION2 *psReservation,
                          IMG_HANDLE *phHeap)
{
	if (psReservation == NULL || phHeap == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	*phHeap = psReservation->psDevmemHeap;

	return PVRSRV_OK;
}

/*************************************************************************/ /*!
@Function       DevmemServerGetContext
@Description    For given heap returns the context.
@Return         PVRSRV_ERROR failure code
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemServerGetContext(DEVMEMINT_HEAP *psDevmemHeap,
					   DEVMEMINT_CTX **ppsDevmemCtxPtr)
{
	if (psDevmemHeap == NULL || ppsDevmemCtxPtr == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	*ppsDevmemCtxPtr = psDevmemHeap->psDevmemCtx;

	return PVRSRV_OK;
}

/*************************************************************************/ /*!
@Function       DevmemServerGetPrivData
@Description    For given context returns the private data handle.
@Return         PVRSRV_ERROR failure code
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemServerGetPrivData(DEVMEMINT_CTX *psDevmemCtx,
						IMG_HANDLE *phPrivData)
{
	if (psDevmemCtx == NULL || phPrivData == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	*phPrivData = psDevmemCtx->hPrivData;

	return PVRSRV_OK;
}

/*************************************************************************/ /*!
@Function       DevmemIntCtxCreate
@Description    Creates and initialises a device memory context.
@Return         valid Device Memory context handle - Success
                PVRSRV_ERROR failure code
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemIntCtxCreate(CONNECTION_DATA *psConnection,
                   PVRSRV_DEVICE_NODE *psDeviceNode,
                   IMG_BOOL bKernelMemoryCtx,
                   DEVMEMINT_CTX **ppsDevmemCtxPtr,
                   IMG_HANDLE *hPrivData,
                   IMG_UINT32 *pui32CPUCacheLineSize)
{
	PVRSRV_ERROR eError;
	DEVMEMINT_CTX *psDevmemCtx;
	IMG_HANDLE hPrivDataInt = NULL;
	MMU_DEVICEATTRIBS *psMMUDevAttrs = psDeviceNode->pfnGetMMUDeviceAttributes(psDeviceNode,
	                                                                           bKernelMemoryCtx);

	PVR_DPF((PVR_DBG_MESSAGE, "%s", __func__));

	/* Only allow request for a kernel context that comes from a direct bridge
	 * (psConnection == NULL). Only the FW/KM Ctx is created over the direct bridge. */
	PVR_LOG_RETURN_IF_INVALID_PARAM(!bKernelMemoryCtx || psConnection == NULL,
	                                "bKernelMemoryCtx && psConnection");

	/*
	 * Ensure that we are safe to perform unaligned accesses on memory
	 * we mark write-combine, as the compiler might generate
	 * instructions operating on this memory which require this
	 * assumption to be true.
	 */
	PVR_ASSERT(OSIsWriteCombineUnalignedSafe());

	/* allocate a Devmem context */
	psDevmemCtx = OSAllocMem(sizeof(*psDevmemCtx));
	PVR_LOG_GOTO_IF_NOMEM(psDevmemCtx, eError, fail_alloc);

	OSAtomicWrite(&psDevmemCtx->hRefCount, 1);
	psDevmemCtx->psDevNode = psDeviceNode;

	/* Call down to MMU context creation */

	eError = MMU_ContextCreate(psConnection,
	                           psDeviceNode,
	                           &psDevmemCtx->psMMUContext,
	                           psMMUDevAttrs);
	PVR_LOG_GOTO_IF_ERROR(eError, "MMU_ContextCreate", fail_mmucontext);

	if (psDeviceNode->pfnRegisterMemoryContext)
	{
		eError = psDeviceNode->pfnRegisterMemoryContext(psDeviceNode, psDevmemCtx->psMMUContext, &hPrivDataInt);
		PVR_LOG_GOTO_IF_ERROR(eError, "pfnRegisterMemoryContext", fail_register);
	}

	/* Store the private data as it is required to unregister the memory context */
	psDevmemCtx->hPrivData = hPrivDataInt;
	*hPrivData = hPrivDataInt;
	*ppsDevmemCtxPtr = psDevmemCtx;

	/* Pass the CPU cache line size through the bridge to the user mode as it can't be queried in user mode.*/
	*pui32CPUCacheLineSize = OSCPUCacheAttributeSize(OS_CPU_CACHE_ATTRIBUTE_LINE_SIZE);

	/* Initialise the PID notify list */
	OSWRLockCreate(&psDevmemCtx->hListLock);
	dllist_init(&(psDevmemCtx->sProcessNotifyListHead));
	psDevmemCtx->sPageFaultNotifyListElem.psNextNode = NULL;
	psDevmemCtx->sPageFaultNotifyListElem.psPrevNode = NULL;

	/* Initialise page fault address */
	psDevmemCtx->sFaultAddress.uiAddr = 0ULL;

	/* Initialise flags */
	psDevmemCtx->ui32Flags = 0;

	return PVRSRV_OK;

fail_register:
	MMU_ContextDestroy(psDevmemCtx->psMMUContext);
fail_mmucontext:
	OSFreeMem(psDevmemCtx);
fail_alloc:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/*************************************************************************/ /*!
@Function       DevmemIntHeapCreate
@Description    Creates and initialises a device memory heap.
@Return         valid Device Memory heap handle - Success
                PVRSRV_ERROR failure code
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemIntHeapCreate(DEVMEMINT_CTX *psDevmemCtx,
                    IMG_DEV_VIRTADDR sHeapBaseAddr,
                    IMG_DEVMEM_SIZE_T uiHeapLength,
                    IMG_UINT32 uiLog2DataPageSize,
                    DEVMEMINT_HEAP **ppsDevmemHeapPtr)
{
	DEVMEMINT_HEAP *psDevmemHeap;
	PVRSRV_ERROR eError;
	IMG_DEV_VIRTADDR sBlueprintHeapBaseAddr;
	IMG_DEVMEM_SIZE_T uiBlueprintHeapLength;
	IMG_DEVMEM_SIZE_T uiBlueprintResRgnLength;
	IMG_UINT32 ui32BlueprintLog2DataPageSize;
	IMG_UINT32 ui32BlueprintLog2ImportAlignment;
	IMG_UINT32 ui32NumHeapConfigsOut;
	IMG_UINT32 ui32NumHeapsOut;
	IMG_BOOL bHeapParamsValidated = IMG_FALSE;

	IMG_INT uiHeapConfigIndex, uiHeapIndex;

	PVR_DPF((PVR_DBG_MESSAGE, "%s", __func__));

	/* allocate a Devmem context */
	psDevmemHeap = OSAllocMem(sizeof(*psDevmemHeap));
	PVR_LOG_RETURN_IF_NOMEM(psDevmemHeap, "psDevmemHeap");

	psDevmemHeap->psDevmemCtx = psDevmemCtx;

	DevmemIntCtxAcquire(psDevmemHeap->psDevmemCtx);

	OSAtomicWrite(&psDevmemHeap->uiRefCount, 1);

	/* getting number of heaps and heap configs */
	eError = HeapCfgHeapConfigCount(NULL, psDevmemHeap->psDevmemCtx->psDevNode, &ui32NumHeapConfigsOut);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Cannot retrieve the number of heap configs.\n", __func__));
		eError = PVRSRV_ERROR_FAILED_TO_RETRIEVE_HEAPINFO;
		goto ErrorCtxRelease;
	}

	for(uiHeapConfigIndex = 0; uiHeapConfigIndex < ui32NumHeapConfigsOut; uiHeapConfigIndex++)
	{
		eError = HeapCfgHeapCount(NULL, psDevmemHeap->psDevmemCtx->psDevNode, uiHeapConfigIndex, &ui32NumHeapsOut);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Failed to retrieve the number of heaps for heap config index %d.\n",
					 __func__, uiHeapConfigIndex));
			eError = PVRSRV_ERROR_FAILED_TO_RETRIEVE_HEAPINFO;
			goto ErrorCtxRelease;
		}
		else
		{
			for(uiHeapIndex = 0; uiHeapIndex < ui32NumHeapsOut; uiHeapIndex++)
			{
				/* Check page size and base addr match the heap blueprint */
				eError = HeapCfgHeapDetails(NULL,
											psDevmemHeap->psDevmemCtx->psDevNode,
											uiHeapConfigIndex,
											uiHeapIndex,
											0, NULL,
											&sBlueprintHeapBaseAddr,
											&uiBlueprintHeapLength,
											&uiBlueprintResRgnLength,
											&ui32BlueprintLog2DataPageSize,
											&ui32BlueprintLog2ImportAlignment);
				if (eError == PVRSRV_OK)
				{
					if ((ui32BlueprintLog2DataPageSize == uiLog2DataPageSize) &&
						(sHeapBaseAddr.uiAddr == sBlueprintHeapBaseAddr.uiAddr) &&
						(uiBlueprintHeapLength == uiHeapLength))
					{
						bHeapParamsValidated = IMG_TRUE; 
						break;
					}
				}
				else
				{
					PVR_DPF((PVR_DBG_ERROR, "%s: HeapCfgHeapDetails call fail with Heap Config Index: %d, and Heap Index: %d.\n"
							,__func__, uiHeapConfigIndex, uiHeapIndex));
					eError = PVRSRV_ERROR_FAILED_TO_RETRIEVE_HEAPINFO;
					goto ErrorCtxRelease;
				}
			}
		}

		if (bHeapParamsValidated == IMG_TRUE)
		{
			break;
		}
	}

	if (bHeapParamsValidated != IMG_TRUE)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s:The passed parameters do not match with any heap blueprint.\n",
					 __func__));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto ErrorCtxRelease;
	}

	psDevmemHeap->uiLog2PageSize = uiLog2DataPageSize;
	psDevmemHeap->sBaseAddr = sHeapBaseAddr;
	/* Store the last accessible address as our LastAddr. We can access
	 * every address between sHeapBaseAddr and sHeapBaseAddr + HeapLength - 1
	 */
	psDevmemHeap->sLastAddr.uiAddr = sHeapBaseAddr.uiAddr + uiBlueprintHeapLength - 1;

	*ppsDevmemHeapPtr = psDevmemHeap;

	return PVRSRV_OK;

ErrorCtxRelease:
	DevmemIntCtxRelease(psDevmemHeap->psDevmemCtx);
	OSFreeMem(psDevmemHeap);

	return eError;
}

PVRSRV_ERROR DevmemIntAllocDefBackingPage(PVRSRV_DEVICE_NODE *psDevNode,
                                            PVRSRV_DEF_PAGE *psDefPage,
                                            IMG_INT	uiInitValue,
                                            IMG_CHAR *pcDefPageName,
                                            IMG_BOOL bInitPage)
{
	IMG_UINT32 ui32RefCnt;
	PVRSRV_ERROR eError = PVRSRV_OK;

	OSLockAcquire(psDefPage->psPgLock);

	/* We know there will not be 4G number of sparse PMR's */
	ui32RefCnt = OSAtomicIncrement(&psDefPage->atRefCounter);

	if (1 == ui32RefCnt)
	{
		IMG_DEV_PHYADDR	sDevPhysAddr = {0};

#if defined(PDUMP)
		PDUMPCOMMENT(psDevNode, "Alloc %s page object", pcDefPageName);
#endif

		/* Allocate the dummy page required for sparse backing */
		eError = DevPhysMemAlloc(psDevNode,
		                         (1 << psDefPage->ui32Log2PgSize),
		                         0,
		                         uiInitValue,
		                         bInitPage,
#if defined(PDUMP)
		                         psDevNode->psMMUDevAttrs->pszMMUPxPDumpMemSpaceName,
		                         pcDefPageName,
		                         &psDefPage->hPdumpPg,
#endif
		                         &psDefPage->sPageHandle,
		                         &sDevPhysAddr);
		if (PVRSRV_OK != eError)
		{
			OSAtomicDecrement(&psDefPage->atRefCounter);
		}
		else
		{
			psDefPage->ui64PgPhysAddr = sDevPhysAddr.uiAddr;
		}
	}

	OSLockRelease(psDefPage->psPgLock);

	return eError;
}

void DevmemIntFreeDefBackingPage(PVRSRV_DEVICE_NODE *psDevNode,
                                   PVRSRV_DEF_PAGE *psDefPage,
                                   IMG_CHAR *pcDefPageName)
{
	IMG_UINT32 ui32RefCnt;

	ui32RefCnt = OSAtomicRead(&psDefPage->atRefCounter);

	/* For the cases where the dummy page allocation fails due to lack of memory
	 * The refcount can still be 0 even for a sparse allocation */
	if (0 != ui32RefCnt)
	{
		OSLockAcquire(psDefPage->psPgLock);

		/* We know there will not be 4G number of sparse PMR's */
		ui32RefCnt = OSAtomicDecrement(&psDefPage->atRefCounter);

		if (0 == ui32RefCnt)
		{
			PDUMPCOMMENT(psDevNode, "Free %s page object", pcDefPageName);

			/* Free the dummy page when refcount reaches zero */
			DevPhysMemFree(psDevNode,
#if defined(PDUMP)
			               psDefPage->hPdumpPg,
#endif
			               &psDefPage->sPageHandle);

#if defined(PDUMP)
			psDefPage->hPdumpPg = NULL;
#endif
			psDefPage->ui64PgPhysAddr = MMU_BAD_PHYS_ADDR;
		}

		OSLockRelease(psDefPage->psPgLock);
	}

}

PVRSRV_ERROR
DevmemIntMapPages(DEVMEMINT_RESERVATION *psReservation,
                  PMR *psPMR,
                  IMG_UINT32 ui32PageCount,
                  IMG_UINT32 ui32PhysicalPgOffset,
                  PVRSRV_MEMALLOCFLAGS_T uiFlags,
                  IMG_DEV_VIRTADDR sDevVAddrBase)
{
	PVR_UNREFERENCED_PARAMETER(psReservation);
	PVR_UNREFERENCED_PARAMETER(psPMR);
	PVR_UNREFERENCED_PARAMETER(ui32PageCount);
	PVR_UNREFERENCED_PARAMETER(ui32PhysicalPgOffset);
	PVR_UNREFERENCED_PARAMETER(uiFlags);
	PVR_UNREFERENCED_PARAMETER(sDevVAddrBase);

	return PVRSRV_ERROR_NOT_IMPLEMENTED;
}

PVRSRV_ERROR
DevmemIntUnmapPages(DEVMEMINT_RESERVATION *psReservation,
                    IMG_DEV_VIRTADDR sDevVAddrBase,
                    IMG_UINT32 ui32PageCount)
{
	PVR_UNREFERENCED_PARAMETER(psReservation);
	PVR_UNREFERENCED_PARAMETER(sDevVAddrBase);
	PVR_UNREFERENCED_PARAMETER(ui32PageCount);

	return PVRSRV_ERROR_NOT_IMPLEMENTED;
}

static INLINE IMG_UINT32
_DevmemXReservationPageCount(DEVMEMXINT_RESERVATION *psRsrv)
{
	return psRsrv->uiLength >> psRsrv->psDevmemHeap->uiLog2PageSize;
}

static INLINE IMG_DEV_VIRTADDR
_DevmemXReservationPageAddress(DEVMEMXINT_RESERVATION *psRsrv, IMG_UINT32 uiVirtPageOffset)
{
	IMG_DEV_VIRTADDR sAddr = {
		.uiAddr = psRsrv->sBase.uiAddr + (uiVirtPageOffset << psRsrv->psDevmemHeap->uiLog2PageSize)
	};

	return sAddr;
}

static INLINE PVRSRV_ERROR ReserveRangeParamValidation(DEVMEMINT_HEAP *psDevmemHeap,
                                                       IMG_DEV_VIRTADDR sAllocationDevVAddr,
                                                       IMG_DEVMEM_SIZE_T uiAllocationSize)
{
	IMG_DEV_VIRTADDR sLastReserveAddr;
	IMG_UINT64 ui64InvalidSizeMask = (1 << psDevmemHeap->uiLog2PageSize) - 1;

	PVR_LOG_RETURN_IF_INVALID_PARAM(psDevmemHeap != NULL, "psDevmemHeap");

	sLastReserveAddr.uiAddr = sAllocationDevVAddr.uiAddr + uiAllocationSize - 1;

	/* Check that the requested address is not less than the base address of the heap. */
	if (sAllocationDevVAddr.uiAddr < psDevmemHeap->sBaseAddr.uiAddr)
	{
		PVR_LOG_VA(PVR_DBG_ERROR,
			"sAllocationDevVAddr ("IMG_DEV_VIRTADDR_FMTSPEC") is invalid! "
			"Must be greater or equal to "IMG_DEV_VIRTADDR_FMTSPEC,
			sAllocationDevVAddr.uiAddr,
			psDevmemHeap->sBaseAddr.uiAddr);
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Check the allocation size is valid (must be page granular). */
	if ((uiAllocationSize & ui64InvalidSizeMask) != 0 || uiAllocationSize == 0)
	{
		PVR_LOG_VA(PVR_DBG_ERROR,
			"uiAllocationSize ("IMG_DEVMEM_SIZE_FMTSPEC") is invalid! Must a multiple of %u and greater than 0",
			uiAllocationSize,
			1 << psDevmemHeap->uiLog2PageSize);
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	if (uiAllocationSize > PMR_MAX_SUPPORTED_SIZE)
	{
		PVR_LOG_VA(PVR_DBG_ERROR,
			"uiAllocationSize must be less than or equal to the max PMR size ("
			IMG_DEVMEM_SIZE_FMTSPEC")",
			PMR_MAX_SUPPORTED_SIZE);

		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Check that requested address + size fits in our heap. */
	if (sLastReserveAddr.uiAddr > psDevmemHeap->sLastAddr.uiAddr)
	{
		PVR_LOG_VA(PVR_DBG_ERROR,
			"sAllocationDevVAddr ("IMG_DEV_VIRTADDR_FMTSPEC") is invalid! "
			"Must be lower than "IMG_DEV_VIRTADDR_FMTSPEC,
			sAllocationDevVAddr.uiAddr,
			psDevmemHeap->sLastAddr.uiAddr - uiAllocationSize + 1);
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	return PVRSRV_OK;
}

PVRSRV_ERROR
DevmemXIntReserveRange(DEVMEMINT_HEAP *psDevmemHeap,
                       IMG_DEV_VIRTADDR sAllocationDevVAddr,
                       IMG_DEVMEM_SIZE_T uiAllocationSize,
                       DEVMEMXINT_RESERVATION **ppsRsrv)
{
	DEVMEMXINT_RESERVATION *psRsrv;
	IMG_UINT32 uiNumPages;
	PVRSRV_ERROR eError;

	PVR_LOG_RETURN_IF_INVALID_PARAM(ppsRsrv != NULL, "ppsRsrv");

	eError = ReserveRangeParamValidation(psDevmemHeap,
	                                     sAllocationDevVAddr,
	                                     uiAllocationSize);
	PVR_LOG_RETURN_IF_ERROR(eError, "ReserveRangeParamValidation");


	if (!DevmemIntHeapAcquire(psDevmemHeap))
	{
		PVR_GOTO_WITH_ERROR(eError, PVRSRV_ERROR_REFCOUNT_OVERFLOW, ErrorReturnError);
	}

	uiNumPages = uiAllocationSize >> psDevmemHeap->uiLog2PageSize;
	psRsrv = OSAllocZMem(sizeof(*psRsrv->ppsPMR) * uiNumPages + sizeof(*psRsrv));
	PVR_LOG_GOTO_IF_NOMEM(psRsrv, eError, ErrorUnreferenceHeap);

	eError = OSLockCreate(&psRsrv->hLock);
	PVR_LOG_GOTO_IF_ERROR(eError, "OSLockCreate", ErrorFreeReservation);

	psRsrv->sBase = sAllocationDevVAddr;
	psRsrv->uiLength = uiAllocationSize;
	psRsrv->ppsPMR = IMG_OFFSET_ADDR(psRsrv, sizeof(*psRsrv));

	eError = MMU_Alloc(psDevmemHeap->psDevmemCtx->psMMUContext,
	                   uiAllocationSize,
	                   &uiAllocationSize,
	                   0, /* IMG_UINT32 uiProtFlags */
	                   0, /* alignment is n/a since we supply devvaddr */
	                   &sAllocationDevVAddr,
	                   psDevmemHeap->uiLog2PageSize);
	PVR_GOTO_IF_ERROR(eError, ErrorDestroyLock);

	/* since we supplied the virt addr, MMU_Alloc shouldn't have
	   chosen a new one for us */
	PVR_ASSERT(sAllocationDevVAddr.uiAddr == psRsrv->sBase.uiAddr);

	psRsrv->psDevmemHeap = psDevmemHeap;
	*ppsRsrv = psRsrv;

	return PVRSRV_OK;

ErrorDestroyLock:
	OSLockDestroy(psRsrv->hLock);
ErrorFreeReservation:
	OSFreeMem(psRsrv);
ErrorUnreferenceHeap:
	DevmemIntHeapRelease(psDevmemHeap);
ErrorReturnError:
	return eError;
}

PVRSRV_ERROR
DevmemXIntUnreserveRange(DEVMEMXINT_RESERVATION *psRsrv)
{
	IMG_UINT32 i;

	MMU_Free(psRsrv->psDevmemHeap->psDevmemCtx->psMMUContext,
	         psRsrv->sBase,
	         psRsrv->uiLength,
	         psRsrv->psDevmemHeap->uiLog2PageSize);

	/* No need to lock the mapping here since this is a handle destruction path which can not be
	 * executed while there are outstanding handle lookups, i.e. other operations are performed
	 * on the mapping. Bridge and handle framework also make sure this path can also not be executed
	 * concurrently. */

	for (i = 0; i < _DevmemXReservationPageCount(psRsrv); i++)
	{
		if (psRsrv->ppsPMR[i] != NULL)
		{
			PMRUnrefPMR2(psRsrv->ppsPMR[i]);
		}
	}

	/* Don't bother with refcount on reservation, as a reservation only ever
	 * holds one mapping, so we directly decrement the refcount on the heap
	 * instead.
	 * Function will print an error if the heap could not be unreferenced. */
	DevmemIntHeapRelease(psRsrv->psDevmemHeap);

	OSLockDestroy(psRsrv->hLock);
	OSFreeMem(psRsrv);

	return PVRSRV_OK;
}

static INLINE PVRSRV_ERROR
DevmemValidateFlags(PMR *psPMR, PVRSRV_MEMALLOCFLAGS_T uiMapFlags)
{
	PMR_FLAGS_T uiPMRFlags = PMR_Flags(psPMR);
	PVRSRV_ERROR eError = PVRSRV_OK;

	if (PVRSRV_CHECK_SHARED_BUFFER(uiPMRFlags) && PVRSRV_CHECK_GPU_READABLE(uiMapFlags) && !PVRSRV_CHECK_GPU_READABLE(uiPMRFlags))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: PMR is not GPU readable.", __func__));
		PVR_GOTO_WITH_ERROR(eError, PVRSRV_ERROR_INVALID_FLAGS, ErrorReturnError);
	}

	if (PVRSRV_CHECK_SHARED_BUFFER(uiPMRFlags) && PVRSRV_CHECK_GPU_WRITEABLE(uiMapFlags) && !PVRSRV_CHECK_GPU_WRITEABLE(uiPMRFlags))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: PMR is not GPU writeable.", __func__));
		PVR_GOTO_WITH_ERROR(eError, PVRSRV_ERROR_INVALID_FLAGS, ErrorReturnError);
	}

ErrorReturnError:
	return eError;
}

PVRSRV_ERROR
DevmemXIntMapPages(DEVMEMXINT_RESERVATION *psRsrv,
                   PMR *psPMR,
                   IMG_UINT32 uiPageCount,
                   IMG_UINT32 uiPhysPageOffset,
                   PVRSRV_MEMALLOCFLAGS_T uiFlags,
                   IMG_UINT32 uiVirtPageOffset)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 uiPMRMaxChunkCount = PMRGetMaxChunkCount(psPMR);
	DEVMEMINT_HEAP *psDevmemHeap = psRsrv->psDevmemHeap;
	IMG_UINT32 uiLog2PageSize = psDevmemHeap->uiLog2PageSize;
	IMG_UINT32 i;

	PVR_LOG_RETURN_IF_INVALID_PARAM((uiPageCount + uiPhysPageOffset) <= uiPMRMaxChunkCount, "uiPageCount+uiPhysPageOffset");

	/* The range is not valid for the given virtual descriptor */
	PVR_LOG_RETURN_IF_FALSE((uiVirtPageOffset + uiPageCount) <= _DevmemXReservationPageCount(psRsrv),
	                        "mapping offset out of range", PVRSRV_ERROR_DEVICEMEM_OUT_OF_RANGE);
	PVR_LOG_RETURN_IF_FALSE((uiFlags & ~PVRSRV_MEMALLOCFLAGS_DEVMEMX_VIRTUAL_MASK) == 0,
	                        "invalid flags", PVRSRV_ERROR_INVALID_FLAGS);

	if (uiLog2PageSize > PMR_GetLog2Contiguity(psPMR))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Device heap and PMR have incompatible "
		        "Log2Contiguity (%u - %u). PMR contiguity must be a multiple "
		        "of the heap contiguity!", __func__, uiLog2PageSize,
		        PMR_GetLog2Contiguity(psPMR)));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	eError = DevmemValidateFlags(psPMR, uiFlags);
	PVR_LOG_RETURN_IF_ERROR(eError, "DevmemValidateFlags");

	OSLockAcquire(psRsrv->hLock);

	eError = MMU_MapPages(psDevmemHeap->psDevmemCtx->psMMUContext,
	                      uiFlags,
	                      _DevmemXReservationPageAddress(psRsrv, uiVirtPageOffset),
	                      psPMR,
	                      uiPhysPageOffset,
	                      uiPageCount,
	                      NULL,
	                      psDevmemHeap->uiLog2PageSize);
	PVR_GOTO_IF_ERROR(eError, ErrUnlock);

	for (i = uiVirtPageOffset; i < (uiVirtPageOffset + uiPageCount); i++)
	{
		PMRRefPMR2(psPMR);

		if (psRsrv->ppsPMR[i] != NULL)
		{
			PMRUnrefPMR2(psRsrv->ppsPMR[i]);
		}

		psRsrv->ppsPMR[i] = psPMR;
	}

	OSLockRelease(psRsrv->hLock);

	return PVRSRV_OK;

ErrUnlock:
	OSLockRelease(psRsrv->hLock);

	return eError;
}

PVRSRV_ERROR
DevmemXIntUnmapPages(DEVMEMXINT_RESERVATION *psRsrv,
                     IMG_UINT32 uiVirtPageOffset,
                     IMG_UINT32 uiPageCount)
{
	DEVMEMINT_HEAP *psDevmemHeap = psRsrv->psDevmemHeap;
	IMG_UINT32 i;

	PVR_LOG_RETURN_IF_FALSE((uiVirtPageOffset + uiPageCount) <= _DevmemXReservationPageCount(psRsrv),
	                        "mapping offset out of range", PVRSRV_ERROR_DEVICEMEM_OUT_OF_RANGE);

	OSLockAcquire(psRsrv->hLock);

	/* Unmap the pages and mark them invalid in the MMU PTE */
	MMU_UnmapPages(psDevmemHeap->psDevmemCtx->psMMUContext,
	               0,
	               _DevmemXReservationPageAddress(psRsrv, uiVirtPageOffset),
	               uiPageCount,
	               NULL,
	               psDevmemHeap->uiLog2PageSize,
	               0);

	for (i = uiVirtPageOffset; i < (uiVirtPageOffset + uiPageCount); i++)
	{
		if (psRsrv->ppsPMR[i] != NULL)
		{
			PMRUnrefPMR2(psRsrv->ppsPMR[i]);
			psRsrv->ppsPMR[i] = NULL;
		}
	}

	OSLockRelease(psRsrv->hLock);

	return PVRSRV_OK;
}

static INLINE IMG_UINT32
_DevmemReservationPageCount(DEVMEMINT_RESERVATION2 *psRsrv)
{
	return psRsrv->uiLength >> psRsrv->psDevmemHeap->uiLog2PageSize;
}

PVRSRV_ERROR
DevmemIntMapPMR(DEVMEMINT_HEAP *psDevmemHeap,
                DEVMEMINT_RESERVATION *psReservation,
                PMR *psPMR,
                PVRSRV_MEMALLOCFLAGS_T uiMapFlags,
                DEVMEMINT_MAPPING **ppsMappingPtr)
{
	PVR_UNREFERENCED_PARAMETER(psDevmemHeap);
	PVR_UNREFERENCED_PARAMETER(psReservation);
	PVR_UNREFERENCED_PARAMETER(psPMR);
	PVR_UNREFERENCED_PARAMETER(uiMapFlags);
	PVR_UNREFERENCED_PARAMETER(ppsMappingPtr);

	return PVRSRV_ERROR_NOT_IMPLEMENTED;
}

PVRSRV_ERROR
DevmemIntMapPMR2(DEVMEMINT_HEAP *psDevmemHeap,
                DEVMEMINT_RESERVATION2 *psReservation,
                PMR *psPMR)
{
	PVRSRV_ERROR eError;
	/* number of pages (device pages) that allocation spans */
	IMG_UINT32 ui32NumDevPages;
	/* device virtual address of start of allocation */
	IMG_DEV_VIRTADDR sAllocationDevVAddr;
	/* and its length */
	IMG_DEVMEM_SIZE_T uiAllocationSize;
	IMG_UINT32 uiLog2HeapContiguity = psReservation->psDevmemHeap->uiLog2PageSize;
	PVRSRV_MEMALLOCFLAGS_T uiMapFlags = psReservation->uiFlags;
	IMG_BOOL bIsSparse = IMG_FALSE, bNeedBacking = IMG_FALSE;
	PVRSRV_DEVICE_NODE *psDevNode = psReservation->psDevmemHeap->psDevmemCtx->psDevNode;
	PMR_FLAGS_T uiPMRFlags;
	PVRSRV_DEF_PAGE *psDefPage;
	IMG_CHAR *pszPageName;
	IMG_DEV_PHYADDR *psDevPAddr;
	IMG_BOOL *pbValid;
	IMG_DEVMEM_SIZE_T uiPMRLogicalSize;
	IMG_UINT32 i;

	PVR_UNREFERENCED_PARAMETER(psDevmemHeap);

	PMR_LogicalSize(psPMR, &uiPMRLogicalSize);

	PVR_LOG_RETURN_IF_INVALID_PARAM(psReservation->psMappedPMR == NULL, "psReservation");
	PVR_LOG_RETURN_IF_INVALID_PARAM(uiPMRLogicalSize == psReservation->uiLength, "psPMR logical size");

	if (uiLog2HeapContiguity > PMR_GetLog2Contiguity(psPMR))
	{
		PVR_DPF((PVR_DBG_ERROR,
		         "%s: Device heap and PMR have incompatible contiguity (%u - %u). "
		         "Heap contiguity must be a multiple of the heap contiguity!",
		         __func__,
		         uiLog2HeapContiguity,
		         PMR_GetLog2Contiguity(psPMR) ));
		PVR_GOTO_WITH_ERROR(eError, PVRSRV_ERROR_INVALID_PARAMS, ErrorReturnError);
	}

	eError = DevmemValidateFlags(psPMR, uiMapFlags);
	PVR_LOG_GOTO_IF_ERROR(eError, "DevmemValidateFlags", ErrorReturnError);

	OSLockAcquire(psReservation->hLock);

	if (!DevmemIntReservationAcquireUnlocked(psReservation))
	{
		PVR_GOTO_WITH_ERROR(eError, PVRSRV_ERROR_REFCOUNT_OVERFLOW, ErrorReturnError);
	}

	uiAllocationSize = psReservation->uiLength;

	ui32NumDevPages = 0xffffffffU & ( ( (uiAllocationSize - 1) >> uiLog2HeapContiguity) + 1);
	PVR_ASSERT((IMG_DEVMEM_SIZE_T) ui32NumDevPages << uiLog2HeapContiguity == uiAllocationSize);

	eError = PMRLockSysPhysAddresses(psPMR);
	PVR_GOTO_IF_ERROR(eError, ErrorUnreference);

	sAllocationDevVAddr = psReservation->sBase;

	/*Check if the PMR that needs to be mapped is sparse */
	bIsSparse = PMR_IsSparse(psPMR);
	if (bIsSparse)
	{
		/*Get the flags*/
		uiPMRFlags = PMR_Flags(psPMR);
		bNeedBacking = PVRSRV_IS_SPARSE_DUMMY_BACKING_REQUIRED(uiPMRFlags);

		if (bNeedBacking)
		{
			IMG_INT uiInitValue;

			if (PVRSRV_IS_SPARSE_ZERO_BACKING_REQUIRED(uiPMRFlags))
			{
				psDefPage = &psDevNode->sDevZeroPage;
				uiInitValue = PVR_ZERO_PAGE_INIT_VALUE;
				pszPageName = DEV_ZERO_PAGE;
			}
			else
			{
				psDefPage = &psDevNode->sDummyPage;
				uiInitValue = PVR_DUMMY_PAGE_INIT_VALUE;
				pszPageName = DUMMY_PAGE;
			}

			/* Error is logged with in the function if any failures.
			 * As the allocation fails we need to fail the map request and
			 * return appropriate error
			 *
			 * Allocation of dummy/zero page is done after locking the pages for PMR physically
			 * By implementing this way, the best case path of dummy/zero page being most likely to be
			 * allocated after physically locking down pages, is considered.
			 * If the dummy/zero page allocation fails, we do unlock the physical address and the impact
			 * is a bit more in on demand mode of operation */
			eError = DevmemIntAllocDefBackingPage(psDevNode,
			                                      psDefPage,
			                                      uiInitValue,
			                                      pszPageName,
			                                      IMG_TRUE);
			PVR_GOTO_IF_ERROR(eError, ErrorUnlockPhysAddr);
		}

		/* N.B. We pass mapping permission flags to MMU_MapPages and let
		 * it reject the mapping if the permissions on the PMR are not compatible. */
		eError = MMU_MapPages(psReservation->psDevmemHeap->psDevmemCtx->psMMUContext,
		                      uiMapFlags,
		                      sAllocationDevVAddr,
		                      psPMR,
		                      0,
		                      ui32NumDevPages,
		                      NULL,
		                      uiLog2HeapContiguity);
		PVR_GOTO_IF_ERROR(eError, ErrorFreeDefBackingPage);

		psDevPAddr = OSAllocMem(ui32NumDevPages * sizeof(IMG_DEV_PHYADDR));
		PVR_LOG_GOTO_IF_NOMEM(psDevPAddr, eError, ErrorUnmapSparseMap);

		pbValid = OSAllocMem(ui32NumDevPages * sizeof(IMG_BOOL));
		PVR_LOG_GOTO_IF_NOMEM(pbValid, eError, ErrorFreePAddrMappingArray);

		/* Determine which entries of the PMR are valid */
		eError = PMR_DevPhysAddr(psPMR,
		                         uiLog2HeapContiguity,
		                         ui32NumDevPages,
		                         0,
		                         psDevPAddr,
		                         pbValid);
		PVR_GOTO_IF_ERROR(eError, ErrorFreeValidArray);

		for (i = 0; i < ui32NumDevPages; i++)
		{
			if (DevmemIntReservationIsIndexMapped(psReservation, i))
			{
				PMRUnrefPMR2(psReservation->psMappedPMR);
				DevmemIntReservationSetMappingIndex(psReservation, i, IMG_FALSE);
			}

			if (pbValid[i])
			{
				PMRRefPMR2(psPMR);
				DevmemIntReservationSetMappingIndex(psReservation, i, IMG_TRUE);
			}
		}

		OSFreeMem(psDevPAddr);
		OSFreeMem(pbValid);
	}
	else
	{
		eError = MMU_MapPMRFast(psReservation->psDevmemHeap->psDevmemCtx->psMMUContext,
		                        sAllocationDevVAddr,
		                        psPMR,
		                        (IMG_DEVMEM_SIZE_T) ui32NumDevPages << uiLog2HeapContiguity,
		                        uiMapFlags,
		                        uiLog2HeapContiguity);
		PVR_GOTO_IF_ERROR(eError, ErrorUnlockPhysAddr);
	}

	psReservation->psMappedPMR = psPMR;

	OSLockRelease(psReservation->hLock);

	return PVRSRV_OK;

ErrorFreeValidArray:
	OSFreeMem(pbValid);
ErrorFreePAddrMappingArray:
	OSFreeMem(psDevPAddr);
ErrorUnmapSparseMap:
	MMU_UnmapPages(psReservation->psDevmemHeap->psDevmemCtx->psMMUContext,
			0,
			sAllocationDevVAddr,
			ui32NumDevPages,
			NULL,
			uiLog2HeapContiguity,
			0);
ErrorFreeDefBackingPage:
	if (bNeedBacking)
	{
		/*if the mapping failed, the allocated dummy ref count need
		 * to be handled accordingly */
		DevmemIntFreeDefBackingPage(psDevNode,
		                            psDefPage,
		                            pszPageName);
	}
ErrorUnlockPhysAddr:
	{
		PVRSRV_ERROR eError1 = PVRSRV_OK;
		eError1 = PMRUnlockSysPhysAddresses(psPMR);
		PVR_LOG_IF_ERROR(eError1, "PMRUnlockSysPhysAddresses");
	}

ErrorUnreference:
	/* if fails there's not much to do (the function will print an error) */
	DevmemIntReservationReleaseUnlocked(psReservation);

	OSLockRelease(psReservation->hLock);

ErrorReturnError:
	PVR_ASSERT (eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR
DevmemIntUnreserveRangeAndUnmapPMR2(DEVMEMINT_RESERVATION2 *psReservation)
{
	PVRSRV_ERROR eError;

	if (psReservation->psMappedPMR != NULL)
	{
		eError = DevmemIntUnmapPMR2(psReservation);
		PVR_GOTO_IF_ERROR(eError, ErrorReturnError);
	}

	eError = DevmemIntUnreserveRange2(psReservation);
	PVR_GOTO_IF_ERROR(eError, ErrorReturnError);

	return PVRSRV_OK;

ErrorReturnError:
	return eError;
}

PVRSRV_ERROR
DevmemIntGetReservationData(DEVMEMINT_RESERVATION2* psReservation, PMR** ppsPMR, IMG_DEV_VIRTADDR* psDevVAddr)
{
	/* Reservation might not have a PMR if a mapping was not yet performed */
	if (psReservation->psMappedPMR == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevVAddr->uiAddr = psReservation->sBase.uiAddr;
	*ppsPMR = psReservation->psMappedPMR;
	return PVRSRV_OK;
}

PVRSRV_ERROR
DevmemIntUnmapPMR(DEVMEMINT_MAPPING *psMapping)
{
	PVR_UNREFERENCED_PARAMETER(psMapping);

	return PVRSRV_ERROR_NOT_IMPLEMENTED;
}

PVRSRV_ERROR
DevmemIntUnmapPMR2(DEVMEMINT_RESERVATION2 *psReservation)
{
	PVRSRV_ERROR eError;
	DEVMEMINT_HEAP *psDevmemHeap = psReservation->psDevmemHeap;
	/* device virtual address of start of allocation */
	IMG_DEV_VIRTADDR sAllocationDevVAddr;
	/* number of pages (device pages) that allocation spans */
	IMG_UINT32 ui32NumDevPages;
	IMG_BOOL bIsSparse = IMG_FALSE, bNeedBacking = IMG_FALSE;
	IMG_UINT32 i;

	PVR_RETURN_IF_INVALID_PARAM(psReservation->psMappedPMR != NULL);

	ui32NumDevPages = _DevmemReservationPageCount(psReservation);
	sAllocationDevVAddr = psReservation->sBase;

	OSLockAcquire(psReservation->hLock);
	bIsSparse = PMR_IsSparse(psReservation->psMappedPMR);

	if (bIsSparse)
	{
		/*Get the flags*/
		PMR_FLAGS_T uiPMRFlags = PMR_Flags(psReservation->psMappedPMR);
		bNeedBacking = PVRSRV_IS_SPARSE_DUMMY_BACKING_REQUIRED(uiPMRFlags);

		if (bNeedBacking)
		{
			if (PVRSRV_IS_SPARSE_ZERO_BACKING_REQUIRED(uiPMRFlags))
			{
				DevmemIntFreeDefBackingPage(psDevmemHeap->psDevmemCtx->psDevNode,
											&psDevmemHeap->psDevmemCtx->psDevNode->sDevZeroPage,
											DEV_ZERO_PAGE);
			}
			else
			{
				DevmemIntFreeDefBackingPage(psDevmemHeap->psDevmemCtx->psDevNode,
											&psDevmemHeap->psDevmemCtx->psDevNode->sDummyPage,
											DUMMY_PAGE);
			}
		}

		MMU_UnmapPages (psDevmemHeap->psDevmemCtx->psMMUContext,
				0,
				sAllocationDevVAddr,
				ui32NumDevPages,
				NULL,
				psDevmemHeap->uiLog2PageSize,
				0);
		/* We are unmapping the whole PMR */
		for (i = 0; i < ui32NumDevPages; i++)
		{
			if (DevmemIntReservationIsIndexMapped(psReservation, i))
			{
				/* All PMRs in the range should be the same, set local PMR
				 * for Unlocking phys addrs later */
				PMRUnrefPMR2(psReservation->psMappedPMR);
				DevmemIntReservationSetMappingIndex(psReservation, i, IMG_FALSE);
			}
		}
	}
	else
	{
		MMU_UnmapPMRFast(psDevmemHeap->psDevmemCtx->psMMUContext,
		                 sAllocationDevVAddr,
		                 ui32NumDevPages,
		                 psDevmemHeap->uiLog2PageSize);
	}

	eError = PMRUnlockSysPhysAddresses(psReservation->psMappedPMR);
	PVR_ASSERT(eError == PVRSRV_OK);

	psReservation->psMappedPMR = NULL;

	DevmemIntReservationReleaseUnlocked(psReservation);

	OSLockRelease(psReservation->hLock);

	return PVRSRV_OK;
}


PVRSRV_ERROR
DevmemIntReserveRange(DEVMEMINT_HEAP *psDevmemHeap,
                      IMG_DEV_VIRTADDR sAllocationDevVAddr,
                      IMG_DEVMEM_SIZE_T uiAllocationSize,
                      DEVMEMINT_RESERVATION **ppsReservationPtr)
{
	PVR_UNREFERENCED_PARAMETER(psDevmemHeap);
	PVR_UNREFERENCED_PARAMETER(sAllocationDevVAddr);
	PVR_UNREFERENCED_PARAMETER(uiAllocationSize);
	PVR_UNREFERENCED_PARAMETER(ppsReservationPtr);

	return PVRSRV_ERROR_NOT_IMPLEMENTED;
}

PVRSRV_ERROR
DevmemIntReserveRange2(DEVMEMINT_HEAP *psDevmemHeap,
                      IMG_DEV_VIRTADDR sAllocationDevVAddr,
                      IMG_DEVMEM_SIZE_T uiAllocationSize,
                      PVRSRV_MEMALLOCFLAGS_T uiFlags,
                      DEVMEMINT_RESERVATION2 **ppsReservationPtr)
{
	PVRSRV_ERROR eError;
	DEVMEMINT_RESERVATION2 *psReservation;
	IMG_UINT32 uiNumPages;
	IMG_UINT64 ui64MapSize;

	PVR_ASSERT(ppsReservationPtr != NULL);

	eError = ReserveRangeParamValidation(psDevmemHeap,
	                                     sAllocationDevVAddr,
	                                     uiAllocationSize);
	PVR_LOG_RETURN_IF_ERROR(eError, "ReserveRangeParamValidation");

	if (!DevmemIntHeapAcquire(psDevmemHeap))
	{
		PVR_GOTO_WITH_ERROR(eError, PVRSRV_ERROR_REFCOUNT_OVERFLOW,
		                    ErrorReturnError);
	}

	uiNumPages = uiAllocationSize >> psDevmemHeap->uiLog2PageSize;

	/* allocate memory to record the reservation info */
	ui64MapSize = sizeof(*psReservation->pui8Map) * LOG2_DIV(uiNumPages, MAP_MASK_SHIFT);
	if (LOG2_MOD(uiNumPages, MAP_MASK_SHIFT) != 0)
	{
		ui64MapSize += 1;
	}

	psReservation = OSAllocZMem(sizeof(*psReservation) + ui64MapSize);
	PVR_LOG_GOTO_IF_NOMEM(psReservation, eError, ErrorUnreference);

	/* Create lock */
	eError = OSLockCreate(&psReservation->hLock);
	PVR_LOG_GOTO_IF_ERROR(eError, "OSLockCreate", ErrorFreeReservation);

	/* Initialise refcount */
	psReservation->i32RefCount = 1;

	psReservation->uiFlags = uiFlags;
	psReservation->sBase = sAllocationDevVAddr;
	psReservation->uiLength = uiAllocationSize;
	psReservation->pui8Map = IMG_OFFSET_ADDR(psReservation, sizeof(*psReservation));

	eError = MMU_Alloc(psDevmemHeap->psDevmemCtx->psMMUContext,
	                   uiAllocationSize,
	                   &uiAllocationSize,
	                   0, /* IMG_UINT32 uiProtFlags */
	                   0, /* alignment is n/a since we supply devvaddr */
	                   &sAllocationDevVAddr,
	                   psDevmemHeap->uiLog2PageSize);
	PVR_GOTO_IF_ERROR(eError, ErrorDestroyLock);

	/* since we supplied the virt addr, MMU_Alloc shouldn't have
	   chosen a new one for us */
	PVR_ASSERT(sAllocationDevVAddr.uiAddr == psReservation->sBase.uiAddr);

	psReservation->psDevmemHeap = psDevmemHeap;
	*ppsReservationPtr = psReservation;

	return PVRSRV_OK;

	/*
	 *  error exit paths follow
	 */

ErrorDestroyLock:
	OSLockDestroy(psReservation->hLock);

ErrorFreeReservation:
	OSFreeMem(psReservation);

ErrorUnreference:
	/* if fails there's not much to do (the function will print an error) */
	DevmemIntHeapRelease(psDevmemHeap);

ErrorReturnError:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR
DevmemIntUnreserveRange(DEVMEMINT_RESERVATION *psReservation)
{
	PVR_UNREFERENCED_PARAMETER(psReservation);

	return PVRSRV_ERROR_NOT_IMPLEMENTED;
}

PVRSRV_ERROR
DevmemIntUnreserveRange2(DEVMEMINT_RESERVATION2 *psReservation)
{
	IMG_UINT32 i;
	DEVMEMINT_HEAP *psDevmemHeap = psReservation->psDevmemHeap;

	OSLockAcquire(psReservation->hLock);

	if (psReservation->i32RefCount != DEVMEMRESERVATION_REFCOUNT_MIN)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s called but still has existing references "
		         "(%d), free existing reservations & mappings first.", __func__,
		         psReservation->i32RefCount));

		OSLockRelease(psReservation->hLock);

		return PVRSRV_ERROR_RETRY;
	}

	OSLockRelease(psReservation->hLock);

	MMU_Free(psDevmemHeap->psDevmemCtx->psMMUContext,
	         psReservation->sBase,
	         psReservation->uiLength,
	         psDevmemHeap->uiLog2PageSize);

	for (i = 0; i < _DevmemReservationPageCount(psReservation); i++)
	{
		if (DevmemIntReservationIsIndexMapped(psReservation, i))
		{
			PMRUnrefPMR2(psReservation->psMappedPMR);
			DevmemIntReservationSetMappingIndex(psReservation, i, IMG_FALSE);
		}
	}

	OSLockDestroy(psReservation->hLock);
	OSFreeMem(psReservation);

	DevmemIntHeapRelease(psDevmemHeap);

	return PVRSRV_OK;
}


PVRSRV_ERROR
DevmemIntHeapDestroy(DEVMEMINT_HEAP *psDevmemHeap)
{
	if (OSAtomicRead(&psDevmemHeap->uiRefCount) != DEVMEMHEAP_REFCOUNT_MIN)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s called but still has existing references "
		         "(%d), free existing reservations & mappings first.", __func__,
		         OSAtomicRead(&psDevmemHeap->uiRefCount)));

		/*
		 * Try again later when you've freed all the memory
		 *
		 * Note:
		 * We don't expect the application to retry (after all this call would
		 * succeed if the client had freed all the memory which it should have
		 * done before calling this function). However, given there should be
		 * an associated handle, when the handle base is destroyed it will free
		 * any allocations leaked by the client and then it will retry this call,
		 * which should then succeed.
		 */
		return PVRSRV_ERROR_RETRY;
	}

	PVR_ASSERT(OSAtomicRead(&psDevmemHeap->uiRefCount) == DEVMEMHEAP_REFCOUNT_MIN);

	DevmemIntCtxRelease(psDevmemHeap->psDevmemCtx);

	PVR_DPF((PVR_DBG_MESSAGE, "%s: Freed heap %p", __func__, psDevmemHeap));
	OSFreeMem(psDevmemHeap);

	return PVRSRV_OK;
}
PVRSRV_ERROR
DevmemIntChangeSparse(DEVMEMINT_HEAP *psDevmemHeap,
                      PMR *psPMR,
                      IMG_UINT32 ui32AllocPageCount,
                      IMG_UINT32 *pai32AllocIndices,
                      IMG_UINT32 ui32FreePageCount,
                      IMG_UINT32 *pai32FreeIndices,
                      SPARSE_MEM_RESIZE_FLAGS uiSparseFlags,
                      PVRSRV_MEMALLOCFLAGS_T uiFlags,
                      IMG_DEV_VIRTADDR sDevVAddrBase,
                      IMG_UINT64 sCpuVAddrBase)
{
	PVR_UNREFERENCED_PARAMETER(psDevmemHeap);
	PVR_UNREFERENCED_PARAMETER(psPMR);
	PVR_UNREFERENCED_PARAMETER(ui32AllocPageCount);
	PVR_UNREFERENCED_PARAMETER(pai32AllocIndices);
	PVR_UNREFERENCED_PARAMETER(ui32FreePageCount);
	PVR_UNREFERENCED_PARAMETER(uiSparseFlags);
	PVR_UNREFERENCED_PARAMETER(uiFlags);
	PVR_UNREFERENCED_PARAMETER(sDevVAddrBase);
	PVR_UNREFERENCED_PARAMETER(sCpuVAddrBase);

	return PVRSRV_ERROR_NOT_IMPLEMENTED;
}

static PVRSRV_ERROR
DevmemIntValidateSparsePMRIndices(IMG_UINT32 ui32PMRLogicalChunkCount,
                                  IMG_UINT32 *paui32LogicalIndices,
                                  IMG_UINT32 ui32LogicalIndiceCount)
{
	IMG_UINT32 i;
	IMG_UINT8 *paui8TrackedIndices;
	IMG_UINT32 ui32AllocSize;
	PVRSRV_ERROR eError = PVRSRV_OK;

	PVR_ASSERT(ui32PMRLogicalChunkCount != 0);
	PVR_ASSERT(paui32LogicalIndices != NULL);
	PVR_ASSERT(ui32LogicalIndiceCount != 0 && ui32LogicalIndiceCount <= ui32PMRLogicalChunkCount);

	ui32AllocSize = LOG2_DIV(ui32PMRLogicalChunkCount, 3);
	if (LOG2_MOD(ui32PMRLogicalChunkCount, 3) != 0)
	{
		++ui32AllocSize;
	}

	paui8TrackedIndices = OSAllocZMem(ui32AllocSize);
	if (paui8TrackedIndices == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	for (i = 0; i < ui32LogicalIndiceCount; i++)
	{
		IMG_UINT32 ui32LogicalIndex = paui32LogicalIndices[i];

		if (ui32LogicalIndex >= ui32PMRLogicalChunkCount)
		{
			PVR_DPF((PVR_DBG_ERROR,
			         "%s: Reservation index %u is OOB",
			         __func__,
			         ui32LogicalIndex));

			eError = PVRSRV_ERROR_DEVICEMEM_OUT_OF_RANGE;
			break;
		}

		if (BIT_ISSET(paui8TrackedIndices[LOG2_DIV(ui32LogicalIndex, 3)], LOG2_MOD(ui32LogicalIndex, 3)))
		{
			PVR_DPF((PVR_DBG_ERROR,
			         "%s: Duplicate index found: %u",
			         __func__,
			         ui32LogicalIndex));

			eError = PVRSRV_ERROR_PMR_INVALID_MAP_INDEX_ARRAY;
			break;
		}

		BIT_SET(paui8TrackedIndices[LOG2_DIV(ui32LogicalIndex, 3)], LOG2_MOD(ui32LogicalIndex, 3));
	}

	OSFreeMem(paui8TrackedIndices);
	return eError;
}

PVRSRV_ERROR
DevmemIntChangeSparse2(DEVMEMINT_HEAP *psDevmemHeap,
                       PMR *psPMR,
                       IMG_UINT32 ui32AllocPageCount,
                       IMG_UINT32 *pai32AllocIndices,
                       IMG_UINT32 ui32FreePageCount,
                       IMG_UINT32 *pai32FreeIndices,
                       SPARSE_MEM_RESIZE_FLAGS uiSparseFlags,
                       DEVMEMINT_RESERVATION2 *psReservation,
                       IMG_UINT64 sCpuVAddrBase)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	IMG_UINT32 uiLog2PMRContiguity;
	IMG_UINT32 uiLog2HeapContiguity;
	IMG_UINT32 uiOrderDiff;
	PVRSRV_MEMALLOCFLAGS_T uiFlags;

	IMG_UINT32 *pai32MapIndices;
	IMG_UINT32 *pai32UnmapIndices;
	IMG_UINT32 uiMapPageCount;
	IMG_UINT32 uiUnmapPageCount;

	IMG_UINT64 ui64PMRLogicalSize;
	IMG_UINT32 ui32LogicalChunkCount;

	PVR_UNREFERENCED_PARAMETER(psDevmemHeap);

	PMR_LogicalSize(psPMR, &ui64PMRLogicalSize);
	ui32LogicalChunkCount = ui64PMRLogicalSize >> PMR_GetLog2Contiguity(psPMR);

	/* Ensure a PMR has been mapped to this reservation. */
	PVR_LOG_RETURN_IF_INVALID_PARAM(psReservation->psMappedPMR != NULL, "psReservation");

	{
		IMG_UINT64 ui64PMRUID;
		IMG_UINT64 ui64ResPMRUID;

		/* Check the PMR that was used during mapping is being used now. */
		(void)PMRGetUID(psPMR, &ui64PMRUID);
		(void)PMRGetUID(psReservation->psMappedPMR, &ui64ResPMRUID);
		if (ui64ResPMRUID != ui64PMRUID)
		{
			PVR_DPF((PVR_DBG_ERROR,
				"%s: Reservation doesn't represent virtual range associated"
				" with given mapped PMR", __func__));
			return PVRSRV_ERROR_INVALID_PARAMS;
		}
	}

	psPMR = psReservation->psMappedPMR;

	if (!PMR_IsSparse(psPMR))
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Given PMR is not Sparse",
				__func__));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	uiLog2PMRContiguity = PMR_GetLog2Contiguity(psPMR);
	uiLog2HeapContiguity = psReservation->psDevmemHeap->uiLog2PageSize;

	/* This is check is made in DevmemIntMapPMR - no need to do it again in release. */
	PVR_ASSERT(uiLog2HeapContiguity <= uiLog2PMRContiguity);

	if (uiSparseFlags & SPARSE_RESIZE_ALLOC)
	{
		PVR_LOG_RETURN_IF_INVALID_PARAM(ui32AllocPageCount != 0, "ui32AllocPageCount");
		PVR_LOG_RETURN_IF_FALSE(ui32AllocPageCount <= ui32LogicalChunkCount,
		                        "ui32AllocPageCount is invalid",
		                        PVRSRV_ERROR_PMR_BAD_MAPPINGTABLE_SIZE);
	}

	if (uiSparseFlags & SPARSE_RESIZE_FREE)
	{
		PVR_LOG_RETURN_IF_INVALID_PARAM(ui32FreePageCount != 0, "ui32FreePageCount");
		PVR_LOG_RETURN_IF_FALSE(ui32FreePageCount <= ui32LogicalChunkCount,
		                        "ui32FreePageCount is invalid",
		                        PVRSRV_ERROR_PMR_BAD_MAPPINGTABLE_SIZE);
	}

	if (PMR_IsMemLayoutFixed(psPMR) || PMR_IsCpuMapped(psPMR))
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: This PMR layout cannot be changed - PMR_IsMemLayoutFixed()=%c, _PMR_IsMapped()=%c",
				__func__,
				PMR_IsMemLayoutFixed(psPMR) ? 'Y' : 'n',
				PMR_IsCpuMapped(psPMR) ? 'Y' : 'n'));
		return PVRSRV_ERROR_PMR_NOT_PERMITTED;
	}

	uiFlags = psReservation->uiFlags;
	eError = DevmemValidateFlags(psPMR, uiFlags);
	PVR_LOG_GOTO_IF_ERROR(eError, "DevmemValidateFlags", e0);

	pai32MapIndices = pai32AllocIndices;
	pai32UnmapIndices = pai32FreeIndices;
	uiMapPageCount = ui32AllocPageCount;
	uiUnmapPageCount = ui32FreePageCount;

	OSLockAcquire(psReservation->hLock);

	/*
	 * The order of steps in which this request is done is given below. The order of
	 * operations is very important in this case:
	 *
	 * 1. The parameters are validated in function PMR_ChangeSparseMem below.
	 *    A successful response indicates all the parameters are correct.
	 *    In failure case we bail out from here without processing further.
	 * 2. On success, get the PMR specific operations done. this includes page alloc, page free
	 *    and the corresponding PMR status changes.
	 *    when this call fails, it is ensured that the state of the PMR before is
	 *    not disturbed. If it succeeds, then we can go ahead with the subsequent steps.
	 * 3. Invalidate the GPU page table entries for the pages to be freed.
	 * 4. Write the GPU page table entries for the pages that got allocated.
	 * 5. Change the corresponding CPU space map.
	 *
	 * The above steps can be selectively controlled using flags.
	 */
	if (uiSparseFlags & SPARSE_RESIZE_BOTH)
	{
		/* Pre check free indices against reservation given */
		if (uiSparseFlags & SPARSE_RESIZE_FREE)
		{
			eError = DevmemIntValidateSparsePMRIndices(ui32LogicalChunkCount,
			                                           pai32FreeIndices,
			                                           ui32FreePageCount);
			PVR_LOG_GOTO_IF_ERROR(eError, "DevmemIntValidateSparsePMRIndices", e0);
		}

		/* Pre check alloc indices against reservation given */
		if (uiSparseFlags & SPARSE_RESIZE_ALLOC)
		{
			eError = DevmemIntValidateSparsePMRIndices(ui32LogicalChunkCount,
			                                           pai32AllocIndices,
			                                           ui32AllocPageCount);
			PVR_LOG_GOTO_IF_ERROR(eError, "DevmemIntValidateSparsePMRIndices", e0);
		}

		uiOrderDiff = uiLog2PMRContiguity - uiLog2HeapContiguity;

		/* Special case:
		 * Adjust indices if we map into a heap that uses smaller page sizes
		 * than the physical allocation itself.
		 * The incoming parameters are all based on the page size of the PMR
		 * but the mapping functions expects parameters to be in terms of heap page sizes. */
		if (uiOrderDiff != 0)
		{
			IMG_UINT32 uiPgIdx, uiPgOffset;
			IMG_UINT32 uiPagesPerOrder = 1 << uiOrderDiff;

			uiMapPageCount = (uiMapPageCount << uiOrderDiff);
			uiUnmapPageCount = (uiUnmapPageCount << uiOrderDiff);

			pai32MapIndices = OSAllocMem(uiMapPageCount * sizeof(*pai32MapIndices));
			PVR_GOTO_IF_NOMEM(pai32MapIndices, eError, e0);

			pai32UnmapIndices = OSAllocMem(uiUnmapPageCount * sizeof(*pai32UnmapIndices));
			if (!pai32UnmapIndices)
			{
				OSFreeMem(pai32MapIndices);
				PVR_GOTO_WITH_ERROR(eError, PVRSRV_ERROR_OUT_OF_MEMORY, e0);
			}

			/* Every chunk index needs to be translated from physical indices
			* into heap based indices. */
			for (uiPgIdx = 0; uiPgIdx < ui32AllocPageCount; uiPgIdx++)
			{
				for (uiPgOffset = 0; uiPgOffset < uiPagesPerOrder; uiPgOffset++)
				{
					pai32MapIndices[uiPgIdx*uiPagesPerOrder + uiPgOffset] =
							pai32AllocIndices[uiPgIdx]*uiPagesPerOrder + uiPgOffset;
				}
			}

			for (uiPgIdx = 0; uiPgIdx < ui32FreePageCount; uiPgIdx++)
			{
				for (uiPgOffset = 0; uiPgOffset < uiPagesPerOrder; uiPgOffset++)
				{
					pai32UnmapIndices[uiPgIdx*uiPagesPerOrder + uiPgOffset] =
							pai32FreeIndices[uiPgIdx]*uiPagesPerOrder + uiPgOffset;
				}
			}
		}

		/* Validate the virtual indices to be freed. */
		if (uiSparseFlags & SPARSE_RESIZE_FREE)
		{
			IMG_UINT32 i;
			for (i = 0; i < uiUnmapPageCount; i++)
			{
				IMG_BOOL bIsMapped = DevmemIntReservationIsIndexMapped(psReservation,
				                                                       pai32UnmapIndices[i]);
				if (!bIsMapped)
				{
					PVR_DPF((PVR_DBG_ERROR,
						"%s: Reservation index %u is not mapped into the reservation",
						__func__,
						pai32UnmapIndices[i]));
					PVR_GOTO_WITH_ERROR(eError,
					                    PVRSRV_ERROR_DEVICEMEM_NO_MAPPING,
					                    e1);
				}
			}
		}

		/* Validate the virtual indices to be allocated. */
		if (uiSparseFlags & SPARSE_RESIZE_ALLOC)
		{
			IMG_UINT32 i;
			for (i = 0; i < uiMapPageCount; i++)
			{
				IMG_BOOL bIsMapped = DevmemIntReservationIsIndexMapped(psReservation,
				                                                       pai32MapIndices[i]);
				if (bIsMapped)
				{
					PVR_DPF((PVR_DBG_ERROR,
						"%s: Reservation index %u is mapped into the reservation",
						__func__,
						pai32MapIndices[i]));
					PVR_GOTO_WITH_ERROR(eError,
					                    PVRSRV_ERROR_DEVICEMEM_ALREADY_MAPPED,
					                    e1);
				}
			}
		}

		/* Do the PMR specific changes first */
		eError = PMR_ChangeSparseMem(psPMR,
		                             ui32AllocPageCount,
		                             pai32AllocIndices,
		                             ui32FreePageCount,
		                             pai32FreeIndices,
		                             uiSparseFlags);
		if (PVRSRV_OK != eError)
		{
			PVR_DPF((PVR_DBG_MESSAGE,
					"%s: Failed to do PMR specific changes.",
					__func__));
			goto e1;
		}

		/* Invalidate the page table entries for the free pages.
		 * Optimisation later would be not to touch the ones that gets re-mapped */
		if (uiSparseFlags & SPARSE_RESIZE_FREE)
		{
			PMR_FLAGS_T uiPMRFlags;
			IMG_UINT32 i;

			/*Get the flags*/
			uiPMRFlags = PMR_Flags(psPMR);

			/* Unmap the pages and mark them invalid in the MMU PTE */
			MMU_UnmapPages (psReservation->psDevmemHeap->psDevmemCtx->psMMUContext,
			                uiFlags,
			                psReservation->sBase,
			                uiUnmapPageCount,
			                pai32UnmapIndices,
			                uiLog2HeapContiguity,
			                uiPMRFlags);

			for (i = 0; i < uiUnmapPageCount; i++)
			{
				IMG_UINT32 uiIndex = pai32UnmapIndices[i];

				if (DevmemIntReservationIsIndexMapped(psReservation, uiIndex))
				{
					PMRUnrefPMR2(psReservation->psMappedPMR);
					DevmemIntReservationSetMappingIndex(psReservation,
					                                    uiIndex,
					                                    IMG_FALSE);
				}
			}
		}

		/* Wire the pages tables that got allocated */
		if (uiSparseFlags & SPARSE_RESIZE_ALLOC)
		{
			IMG_UINT32 i;
			/* Map the pages and mark them Valid in the MMU PTE */
			eError = MMU_MapPages (psReservation->psDevmemHeap->psDevmemCtx->psMMUContext,
			                       uiFlags,
			                       psReservation->sBase,
			                       psPMR,
			                       0,
			                       uiMapPageCount,
			                       pai32MapIndices,
			                       uiLog2HeapContiguity);
			if (PVRSRV_OK != eError)
			{
				PVR_DPF((PVR_DBG_MESSAGE,
						"%s: Failed to map alloc indices.",
						__func__));
				goto e1;
			}

			for (i = 0; i < uiMapPageCount; i++)
			{
				IMG_UINT32 uiIndex = pai32MapIndices[i];

				if (!DevmemIntReservationIsIndexMapped(psReservation, uiIndex))
				{
					PMRRefPMR2(psReservation->psMappedPMR);
					DevmemIntReservationSetMappingIndex(psReservation,
					                                    uiIndex,
					                                    IMG_TRUE);
				}
			}
		}
	}

#ifndef PVRSRV_UNMAP_ON_SPARSE_CHANGE
	/* Do the changes in sparse on to the CPU virtual map accordingly */
	if (uiSparseFlags & SPARSE_MAP_CPU_ADDR)
	{
		if (sCpuVAddrBase != 0)
		{
			eError = PMR_ChangeSparseMemCPUMap(psPMR,
			                                   sCpuVAddrBase,
			                                   ui32AllocPageCount,
			                                   pai32AllocIndices,
			                                   ui32FreePageCount,
			                                   pai32FreeIndices);
			if (PVRSRV_OK != eError)
			{
				PVR_DPF((PVR_DBG_MESSAGE,
						"%s: Failed to map to CPU addr space.",
						__func__));
				goto e1;
			}
		}
	}
#endif

e1:
	if (pai32MapIndices != pai32AllocIndices)
	{
		OSFreeMem(pai32MapIndices);
	}
	if (pai32UnmapIndices != pai32FreeIndices)
	{
		OSFreeMem(pai32UnmapIndices);
	}
e0:
	OSLockRelease(psReservation->hLock);
	return eError;
}

/*************************************************************************/ /*!
@Function       DevmemIntCtxDestroy
@Description    Destroy that created by DevmemIntCtxCreate
@Input          psDevmemCtx   Device Memory context
@Return         cannot fail.
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemIntCtxDestroy(DEVMEMINT_CTX *psDevmemCtx)
{
	/*
	   We can't determine if we should be freeing the context here
	   as a refcount!=1 could be due to either the fact that heap(s)
	   remain with allocations on them, or that this memory context
	   has been exported.
	   As the client couldn't do anything useful with this information
	   anyway and the fact that the refcount will ensure we only
	   free the context when _all_ references have been released
	   don't bother checking and just return OK regardless.
	   */
	DevmemIntCtxRelease(psDevmemCtx);
	return PVRSRV_OK;
}

PVRSRV_ERROR DevmemIntIsVDevAddrValid(CONNECTION_DATA * psConnection,
                                      PVRSRV_DEVICE_NODE *psDevNode,
                                      DEVMEMINT_CTX *psDevMemContext,
                                      IMG_DEV_VIRTADDR sDevAddr)
{
	IMG_UINT32 i, j, uiLog2HeapPageSize = 0;
	DEVICE_MEMORY_INFO *psDinfo = &psDevNode->sDevMemoryInfo;
	DEVMEM_HEAP_CONFIG *psConfig = psDinfo->psDeviceMemoryHeapConfigArray;

	IMG_BOOL bFound = IMG_FALSE;

	for (i = 0;
		 i < psDinfo->uiNumHeapConfigs && !bFound;
		 i++)
	{
		for (j = 0;
			 j < psConfig[i].uiNumHeaps  && !bFound;
			 j++)
		{
			IMG_DEV_VIRTADDR uiBase =
					psConfig[i].psHeapBlueprintArray[j].sHeapBaseAddr;
			IMG_DEVMEM_SIZE_T uiSize =
					psConfig[i].psHeapBlueprintArray[j].uiHeapLength;

			if ((sDevAddr.uiAddr >= uiBase.uiAddr) &&
				(sDevAddr.uiAddr < (uiBase.uiAddr + uiSize)))
			{
				uiLog2HeapPageSize =
						psConfig[i].psHeapBlueprintArray[j].uiLog2DataPageSize;
				bFound = IMG_TRUE;
			}
		}
	}

	if (uiLog2HeapPageSize == 0)
	{
		return PVRSRV_ERROR_INVALID_GPU_ADDR;
	}

	return MMU_IsVDevAddrValid(psDevMemContext->psMMUContext,
	                           uiLog2HeapPageSize,
	                           sDevAddr) ? PVRSRV_OK : PVRSRV_ERROR_INVALID_GPU_ADDR;
}

PVRSRV_ERROR
DevmemIntFlushDevSLCRange(DEVMEMINT_CTX *psDevMemContext,
                          IMG_DEV_VIRTADDR sDevVAddr,
                          IMG_DEVMEM_SIZE_T uiSize,
                          IMG_BOOL bInvalidate)
{
	PVRSRV_DEVICE_NODE *psDevNode = psDevMemContext->psDevNode;
	MMU_CONTEXT *psMMUContext = psDevMemContext->psMMUContext;

	if (psDevNode->pfnDevSLCFlushRange)
	{
		return psDevNode->pfnDevSLCFlushRange(psDevNode,
		                                      psMMUContext,
		                                      sDevVAddr,
		                                      uiSize,
		                                      bInvalidate);
	}

	return PVRSRV_ERROR_NOT_SUPPORTED;
}

PVRSRV_ERROR
DevmemIntInvalidateFBSCTable(DEVMEMINT_CTX *psDevMemContext,
                             IMG_UINT64 ui64FBSCEntryMask)
{
	PVRSRV_DEVICE_NODE *psDevNode = psDevMemContext->psDevNode;
	MMU_CONTEXT *psMMUContext = psDevMemContext->psMMUContext;

	if (psDevNode->pfnInvalFBSCTable)
	{
		return psDevNode->pfnInvalFBSCTable(psDevNode,
		                                    psMMUContext,
		                                    ui64FBSCEntryMask);
	}

	return PVRSRV_ERROR_NOT_SUPPORTED;
}

PVRSRV_ERROR DevmemIntGetFaultAddress(CONNECTION_DATA *psConnection,
                                      PVRSRV_DEVICE_NODE *psDevNode,
                                      DEVMEMINT_CTX *psDevMemContext,
                                      IMG_DEV_VIRTADDR *psFaultAddress)
{
	if ((psDevMemContext->ui32Flags & DEVMEMCTX_FLAGS_FAULT_ADDRESS_AVAILABLE) == 0)
	{
		return PVRSRV_ERROR_RESOURCE_UNAVAILABLE;
	}

	*psFaultAddress = psDevMemContext->sFaultAddress;
	psDevMemContext->ui32Flags &= ~DEVMEMCTX_FLAGS_FAULT_ADDRESS_AVAILABLE;

	return PVRSRV_OK;
}

static POSWR_LOCK g_hExportCtxListLock;
static DLLIST_NODE g_sExportCtxList;

PVRSRV_ERROR
DevmemIntInit(void)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	dllist_init(&g_sExportCtxList);

	eError = OSWRLockCreate(&g_hExportCtxListLock);

	return eError;
}

PVRSRV_ERROR
DevmemIntDeInit(void)
{
	PVR_ASSERT(dllist_is_empty(&g_sExportCtxList));

	OSWRLockDestroy(g_hExportCtxListLock);

	return PVRSRV_OK;
}

PVRSRV_ERROR
DevmemIntExportCtx(DEVMEMINT_CTX *psContext,
                   PMR *psPMR,
                   DEVMEMINT_CTX_EXPORT **ppsContextExport)
{
	DEVMEMINT_CTX_EXPORT *psCtxExport;

	psCtxExport = OSAllocMem(sizeof(DEVMEMINT_CTX_EXPORT));
	PVR_LOG_RETURN_IF_NOMEM(psCtxExport, "psCtxExport");

	DevmemIntCtxAcquire(psContext);
	PMRRefPMR(psPMR);
	/* Now that the source PMR is exported, the layout
	 * can't change as there could be outstanding importers
	 * This is to make sure both exporter and importers view of
	 * the memory is same */
	PMR_SetLayoutFixed(psPMR, IMG_TRUE);
	psCtxExport->psDevmemCtx = psContext;
	psCtxExport->psPMR = psPMR;
	OSWRLockAcquireWrite(g_hExportCtxListLock);
	dllist_add_to_tail(&g_sExportCtxList, &psCtxExport->sNode);
	OSWRLockReleaseWrite(g_hExportCtxListLock);

	*ppsContextExport = psCtxExport;

	return PVRSRV_OK;
}

PVRSRV_ERROR
DevmemIntUnexportCtx(DEVMEMINT_CTX_EXPORT *psContextExport)
{
	PMRUnrefPMR(psContextExport->psPMR);
	DevmemIntCtxRelease(psContextExport->psDevmemCtx);
	OSWRLockAcquireWrite(g_hExportCtxListLock);
	dllist_remove_node(&psContextExport->sNode);
	OSWRLockReleaseWrite(g_hExportCtxListLock);
	OSFreeMem(psContextExport);

	/* Unable to find exported context, return error */
	return PVRSRV_OK;
}

PVRSRV_ERROR
DevmemIntAcquireRemoteCtx(PMR *psPMR,
                          DEVMEMINT_CTX **ppsContext,
                          IMG_HANDLE *phPrivData)
{
	PDLLIST_NODE psListNode, psListNodeNext;
	DEVMEMINT_CTX_EXPORT *psCtxExport;

	OSWRLockAcquireRead(g_hExportCtxListLock);
	/* Find context from list using PMR as key */
	dllist_foreach_node(&g_sExportCtxList, psListNode, psListNodeNext)
	{
		psCtxExport = IMG_CONTAINER_OF(psListNode, DEVMEMINT_CTX_EXPORT, sNode);
		if (psCtxExport->psPMR == psPMR)
		{
			DevmemIntCtxAcquire(psCtxExport->psDevmemCtx);
			*ppsContext = psCtxExport->psDevmemCtx;
			*phPrivData = psCtxExport->psDevmemCtx->hPrivData;

			OSWRLockReleaseRead(g_hExportCtxListLock);

			/* PMR should have been already exported to import it
			 * If a PMR is exported, its immutable and the same is
			 * checked here */
			PVR_ASSERT(IMG_TRUE == PMR_IsMemLayoutFixed(psPMR));

			return PVRSRV_OK;
		}
	}
	OSWRLockReleaseRead(g_hExportCtxListLock);

	/* Unable to find exported context, return error */
	PVR_DPF((PVR_DBG_ERROR,
			"%s: Failed to acquire remote context. Could not retrieve context with given PMR",
			__func__));
	return PVRSRV_ERROR_INVALID_PARAMS;
}

/*************************************************************************/ /*!
@Function       DevmemIntRegisterPFNotify
@Description    Registers a PID to be notified when a page fault occurs on a
                specific device memory context.
@Input          psDevmemCtx    The context to be notified about.
@Input          ui32PID        The PID of the process that would like to be
                               notified.
@Input          bRegister      If true, register. If false, de-register.
@Return         PVRSRV_ERROR.
*/ /**************************************************************************/
PVRSRV_ERROR DevmemIntRegisterPFNotifyKM(DEVMEMINT_CTX *psDevmemCtx,
                                         IMG_INT32     ui32PID,
                                         IMG_BOOL      bRegister)
{
	PVRSRV_DEVICE_NODE *psDevNode;
	DLLIST_NODE         *psNode, *psNodeNext;
	DEVMEMINT_PF_NOTIFY *psNotifyNode;
	IMG_BOOL            bPresent = IMG_FALSE;
	PVRSRV_ERROR        eError;

	PVR_LOG_RETURN_IF_INVALID_PARAM(psDevmemCtx, "psDevmemCtx");

	/* Acquire write lock for the duration, to avoid resource free
	 * while trying to read (no need to then also acquire the read lock
	 * as we have exclusive access while holding the write lock)
	 */
	OSWRLockAcquireWrite(psDevmemCtx->hListLock);

	psDevNode = psDevmemCtx->psDevNode;

	if (bRegister)
	{
		/* If this is the first PID in the list, the device memory context
		 * needs to be registered for notification */
		if (dllist_is_empty(&psDevmemCtx->sProcessNotifyListHead))
		{
			OSWRLockAcquireWrite(psDevNode->hMemoryContextPageFaultNotifyListLock);
			dllist_add_to_tail(&psDevNode->sMemoryContextPageFaultNotifyListHead,
			                   &psDevmemCtx->sPageFaultNotifyListElem);
			OSWRLockReleaseWrite(psDevNode->hMemoryContextPageFaultNotifyListLock);
		}
	}

	/* Loop through the registered PIDs and check whether this one is
	 * present */
	dllist_foreach_node(&(psDevmemCtx->sProcessNotifyListHead), psNode, psNodeNext)
	{
		psNotifyNode = IMG_CONTAINER_OF(psNode, DEVMEMINT_PF_NOTIFY, sProcessNotifyListElem);

		if (psNotifyNode->ui32PID == ui32PID)
		{
			bPresent = IMG_TRUE;
			break;
		}
	}

	if (bRegister)
	{
		if (bPresent)
		{
			PVR_DPF((PVR_DBG_ERROR,
			         "%s: Trying to register a PID that is already registered",
			         __func__));
			eError = PVRSRV_ERROR_PID_ALREADY_REGISTERED;
			goto err_already_registered;
		}

		psNotifyNode = OSAllocMem(sizeof(*psNotifyNode));
		if (psNotifyNode == NULL)
		{
			PVR_DPF((PVR_DBG_ERROR,
			         "%s: Unable to allocate memory for the notify list",
			          __func__));
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto err_out_of_mem;
		}
		psNotifyNode->ui32PID = ui32PID;
		/* Write lock is already held */
		dllist_add_to_tail(&(psDevmemCtx->sProcessNotifyListHead), &(psNotifyNode->sProcessNotifyListElem));
	}
	else
	{
		if (!bPresent)
		{
			PVR_DPF((PVR_DBG_ERROR,
			         "%s: Trying to unregister a PID that is not registered",
			         __func__));
			eError = PVRSRV_ERROR_PID_NOT_REGISTERED;
			goto err_not_registered;
		}
		/* Write lock is already held */
		dllist_remove_node(psNode);
		psNotifyNode = IMG_CONTAINER_OF(psNode, DEVMEMINT_PF_NOTIFY, sProcessNotifyListElem);
		OSFreeMem(psNotifyNode);

		/* If the last process in the list is being unregistered, then also
		 * unregister the device memory context from the notify list. */
		if (dllist_is_empty(&psDevmemCtx->sProcessNotifyListHead))
		{
			OSWRLockAcquireWrite(psDevNode->hMemoryContextPageFaultNotifyListLock);
			dllist_remove_node(&psDevmemCtx->sPageFaultNotifyListElem);
			OSWRLockReleaseWrite(psDevNode->hMemoryContextPageFaultNotifyListLock);
		}
	}
	eError = PVRSRV_OK;

err_already_registered:
err_out_of_mem:
err_not_registered:

	OSWRLockReleaseWrite(psDevmemCtx->hListLock);
	return eError;
}

/*************************************************************************/ /*!
@Function       DevmemIntPFNotify
@Description    Notifies any processes that have registered themselves to be
                notified when a page fault happens on a specific device memory
                context.
@Input          *psDevNode           The device node.
@Input          ui64FaultedPCAddress The page catalogue address that faulted.
@Input          sFaultAddress        The address that triggered the fault.
@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR DevmemIntPFNotify(PVRSRV_DEVICE_NODE *psDevNode,
                               IMG_UINT64         ui64FaultedPCAddress,
                               IMG_DEV_VIRTADDR   sFaultAddress)
{
	DLLIST_NODE         *psNode, *psNodeNext;
	DEVMEMINT_PF_NOTIFY *psNotifyNode;
	PVRSRV_ERROR        eError;
	DEVMEMINT_CTX       *psDevmemCtx = NULL;
	IMG_BOOL            bFailed = IMG_FALSE;

	OSWRLockAcquireRead(psDevNode->hMemoryContextPageFaultNotifyListLock);
	if (dllist_is_empty(&(psDevNode->sMemoryContextPageFaultNotifyListHead)))
	{
		OSWRLockReleaseRead(psDevNode->hMemoryContextPageFaultNotifyListLock);
		return PVRSRV_OK;
	}

	dllist_foreach_node(&(psDevNode->sMemoryContextPageFaultNotifyListHead), psNode, psNodeNext)
	{
		DEVMEMINT_CTX *psThisContext =
			IMG_CONTAINER_OF(psNode, DEVMEMINT_CTX, sPageFaultNotifyListElem);
		IMG_DEV_PHYADDR sPCDevPAddr;

		eError = MMU_AcquireBaseAddr(psThisContext->psMMUContext, &sPCDevPAddr);
		if (eError != PVRSRV_OK)
		{
			PVR_LOG_ERROR(eError, "MMU_AcquireBaseAddr");
			OSWRLockReleaseRead(psDevNode->hMemoryContextPageFaultNotifyListLock);
			return eError;
		}

		if (sPCDevPAddr.uiAddr == ui64FaultedPCAddress)
		{
			psDevmemCtx = psThisContext;
			break;
		}
	}
	OSWRLockReleaseRead(psDevNode->hMemoryContextPageFaultNotifyListLock);

	if (psDevmemCtx == NULL)
	{
		/* Not found, just return */
		return PVRSRV_OK;
	}
	OSWRLockAcquireRead(psDevmemCtx->hListLock);

	/*
	 * Store the first occurrence of a page fault address,
	 * until that address is consumed by a client.
	 */
	if ((psDevmemCtx->ui32Flags & DEVMEMCTX_FLAGS_FAULT_ADDRESS_AVAILABLE) == 0)
	{
		psDevmemCtx->sFaultAddress = sFaultAddress;
		psDevmemCtx->ui32Flags |= DEVMEMCTX_FLAGS_FAULT_ADDRESS_AVAILABLE;
	}

	/* Loop through each registered PID and send a signal to the process */
	dllist_foreach_node(&(psDevmemCtx->sProcessNotifyListHead), psNode, psNodeNext)
	{
		psNotifyNode = IMG_CONTAINER_OF(psNode, DEVMEMINT_PF_NOTIFY, sProcessNotifyListElem);

		eError = OSDebugSignalPID(psNotifyNode->ui32PID);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,
			         "%s: Unable to signal process for PID: %u",
			         __func__,
			         psNotifyNode->ui32PID));

			PVR_ASSERT(!"Unable to signal process");

			bFailed = IMG_TRUE;
		}
	}
	OSWRLockReleaseRead(psDevmemCtx->hListLock);

	if (bFailed)
	{
		return PVRSRV_ERROR_SIGNAL_FAILED;
	}

	return PVRSRV_OK;
}

#if defined(PDUMP)
IMG_UINT32 DevmemIntMMUContextID(DEVMEMINT_CTX *psDevMemContext)
{
	IMG_UINT32 ui32MMUContextID;
	MMU_AcquirePDumpMMUContext(psDevMemContext->psMMUContext, &ui32MMUContextID, PDUMP_FLAGS_CONTINUOUS);
	return ui32MMUContextID;
}

PVRSRV_ERROR
DevmemIntPDumpSaveToFileVirtual(DEVMEMINT_CTX *psDevmemCtx,
                                IMG_DEV_VIRTADDR sDevAddrStart,
                                IMG_DEVMEM_SIZE_T uiSize,
                                IMG_UINT32 ui32ArraySize,
                                const IMG_CHAR *pszFilename,
                                IMG_UINT32 ui32FileOffset,
                                IMG_UINT32 ui32PDumpFlags)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 uiPDumpMMUCtx;

	PVR_UNREFERENCED_PARAMETER(ui32ArraySize);

	/* Confirm that the device node's ui32InternalID matches the bound
	 * PDump device stored in PVRSRV_DATA.
	 */
	if (psDevmemCtx->psDevNode->sDevId.ui32InternalID !=
	    (PVRSRVGetPVRSRVData())->ui32PDumpBoundDevice)
	{
		return PVRSRV_ERROR_PDUMP_CAPTURE_BOUND_TO_ANOTHER_DEVICE;
	}

	eError = MMU_AcquirePDumpMMUContext(psDevmemCtx->psMMUContext,
			&uiPDumpMMUCtx,
			ui32PDumpFlags);

	PVR_ASSERT(eError == PVRSRV_OK);

	/*
	   The following SYSMEM refers to the 'MMU Context', hence it
	   should be the MMU context, not the PMR, that says what the PDump
	   MemSpace tag is?
	   From a PDump P.O.V. it doesn't matter which name space we use as long
	   as that MemSpace is used on the 'MMU Context' we're dumping from
	   */
	eError = PDumpMMUSAB(psDevmemCtx->psDevNode,
	                     psDevmemCtx->psDevNode->sDevId.pszPDumpDevName,
	                     uiPDumpMMUCtx,
	                     sDevAddrStart,
	                     uiSize,
	                     pszFilename,
	                     ui32FileOffset,
	                     ui32PDumpFlags);
	PVR_ASSERT(eError == PVRSRV_OK);

	MMU_ReleasePDumpMMUContext(psDevmemCtx->psMMUContext, ui32PDumpFlags);
	return PVRSRV_OK;
}

PVRSRV_ERROR
DevmemIntPDumpImageDescriptor(CONNECTION_DATA * psConnection,
							  PVRSRV_DEVICE_NODE *psDeviceNode,
							  DEVMEMINT_CTX *psDevMemContext,
							  IMG_UINT32 ui32Size,
							  const IMG_CHAR *pszFileName,
							  IMG_DEV_VIRTADDR sData,
							  IMG_UINT32 ui32DataSize,
							  IMG_UINT32 ui32LogicalWidth,
							  IMG_UINT32 ui32LogicalHeight,
							  IMG_UINT32 ui32PhysicalWidth,
							  IMG_UINT32 ui32PhysicalHeight,
							  PDUMP_PIXEL_FORMAT ePixFmt,
							  IMG_MEMLAYOUT eMemLayout,
							  IMG_FB_COMPRESSION eFBCompression,
							  const IMG_UINT32 *paui32FBCClearColour,
							  PDUMP_FBC_SWIZZLE eFBCSwizzle,
							  IMG_DEV_VIRTADDR sHeader,
							  IMG_UINT32 ui32HeaderSize,
							  IMG_UINT32 ui32PDumpFlags)
{
	IMG_UINT32 ui32ContextID;
	PVRSRV_ERROR eError;

	PVR_UNREFERENCED_PARAMETER(psConnection);
	PVR_UNREFERENCED_PARAMETER(ui32Size);

	eError = MMU_AcquirePDumpMMUContext(psDevMemContext->psMMUContext, &ui32ContextID, ui32PDumpFlags);
	PVR_LOG_RETURN_IF_ERROR(eError, "MMU_AcquirePDumpMMUContext");

	eError = PDumpImageDescriptor(psDeviceNode,
									ui32ContextID,
									(IMG_CHAR *)pszFileName,
									sData,
									ui32DataSize,
									ui32LogicalWidth,
									ui32LogicalHeight,
									ui32PhysicalWidth,
									ui32PhysicalHeight,
									ePixFmt,
									eMemLayout,
									eFBCompression,
									paui32FBCClearColour,
									eFBCSwizzle,
									sHeader,
									ui32HeaderSize,
									ui32PDumpFlags);
	PVR_LOG_IF_ERROR(eError, "PDumpImageDescriptor");

	/* Don't care about return value */
	(void) MMU_ReleasePDumpMMUContext(psDevMemContext->psMMUContext, ui32PDumpFlags);

	return eError;
}

PVRSRV_ERROR
DevmemIntPDumpDataDescriptor(CONNECTION_DATA * psConnection,
							 PVRSRV_DEVICE_NODE *psDeviceNode,
							 DEVMEMINT_CTX *psDevMemContext,
							 IMG_UINT32 ui32Size,
							 const IMG_CHAR *pszFileName,
							 IMG_DEV_VIRTADDR sData,
							 IMG_UINT32 ui32DataSize,
							 IMG_UINT32 ui32HeaderType,
							 IMG_UINT32 ui32ElementType,
							 IMG_UINT32 ui32ElementCount,
							 IMG_UINT32 ui32PDumpFlags)
{
	IMG_UINT32 ui32ContextID;
	PVRSRV_ERROR eError;

	PVR_UNREFERENCED_PARAMETER(psConnection);
	PVR_UNREFERENCED_PARAMETER(ui32Size);

	if ((ui32HeaderType != IBIN_HEADER_TYPE) &&
		(ui32HeaderType != DATA_HEADER_TYPE))
	{
		PVR_DPF((PVR_DBG_ERROR,
		         "%s: Invalid header type (%u)",
		         __func__,
		         ui32HeaderType));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	eError = MMU_AcquirePDumpMMUContext(psDevMemContext->psMMUContext, &ui32ContextID, ui32PDumpFlags);
	PVR_LOG_RETURN_IF_ERROR(eError, "MMU_AcquirePDumpMMUContext");

	eError = PDumpDataDescriptor(psDeviceNode,
									ui32ContextID,
									(IMG_CHAR *)pszFileName,
									sData,
									ui32DataSize,
									ui32HeaderType,
									ui32ElementType,
									ui32ElementCount,
									ui32PDumpFlags);
	PVR_LOG_IF_ERROR(eError, "PDumpDataDescriptor");

	/* Don't care about return value */
	(void) MMU_ReleasePDumpMMUContext(psDevMemContext->psMMUContext, ui32PDumpFlags);

	return eError;
}

#endif
