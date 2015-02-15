/*===========================================================================
FILE:
   QMIDevice.c

DESCRIPTION:
   Functions related to the QMI interface device
   
FUNCTIONS:
   Generic functions
      IsDeviceValid
      PrintHex
      QSetDownReason
      QClearDownReason
      QTestDownReason

   Driver level asynchronous read functions
      ReadCallback
      IntCallback
      StartRead
      KillRead

   Internal read/write functions
      ReadAsync
      UpSem
      ReadSync
      WriteSyncCallback
      WriteSync

   Internal memory management functions
      GetClientID
      ReleaseClientID
      FindClientMem
      AddToReadMemList
      PopFromReadMemList
      AddToNotifyList
      NotifyAndPopNotifyList
      AddToURBList
      PopFromURBList

   Userspace wrappers
      UserspaceOpen
      UserspaceIOCTL
      UserspaceClose
      UserspaceRead
      UserspaceWrite

   Initializer and destructor
      RegisterQMIDevice
      DeregisterQMIDevice

   Driver level client management
      QMIReady
      QMIWDSCallback
      SetupQMIWDSCallback
      QMIDMSGetMEID

Copyright (c) 2010, The Linux Foundation. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 and
only version 2 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
02110-1301, USA.

===========================================================================*/

//---------------------------------------------------------------------------
// Include Files
//---------------------------------------------------------------------------
#include "QMIDevice.h"

//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------

extern int debug;

// Prototype to QCSuspend function
int QCSuspend( 
   struct usb_interface *     pIntf,
   pm_message_t               powerEvent );

// IOCTL to generate a client ID for this service type
#define IOCTL_QMI_GET_SERVICE_FILE 0x8BE0 + 1

// IOCTL to get the VIDPID of the device
#define IOCTL_QMI_GET_DEVICE_VIDPID 0x8BE0 + 2

// IOCTL to get the MEID of the device
#define IOCTL_QMI_GET_DEVICE_MEID 0x8BE0 + 3

// CDC GET_ENCAPSULATED_RESPONSE packet
#define CDC_GET_ENCAPSULATED_RESPONSE 0x01A1ll

// CDC CONNECTION_SPEED_CHANGE indication packet
#define CDC_CONNECTION_SPEED_CHANGE 0x08000000002AA1ll

/*=========================================================================*/
// UserspaceQMIFops
//    QMI device's userspace file operations
/*=========================================================================*/
struct file_operations UserspaceQMIFops = 
{
   .owner     = THIS_MODULE,
   .read      = UserspaceRead,
   .write     = UserspaceWrite,
   .ioctl     = UserspaceIOCTL,
   .open      = UserspaceOpen,
   .flush     = UserspaceClose,
};

/*=========================================================================*/
// Generic functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   IsDeviceValid (Public Method)

DESCRIPTION:
   Basic test to see if device memory is valid

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   bool
===========================================================================*/
bool IsDeviceValid( sQCUSBNet * pDev )
{
   if (pDev == NULL)
   {
      return false;
   }

   if (pDev->mbQMIValid == false)
   {
      return false;
   }
   
   return true;
} 

/*===========================================================================
METHOD:
   PrintHex (Public Method)

DESCRIPTION:
   Print Hex data, for debug purposes

PARAMETERS:
   pBuffer       [ I ] - Data buffer
   bufSize       [ I ] - Size of data buffer

RETURN VALUE:
   None
===========================================================================*/
void PrintHex(
   void *      pBuffer,
   u16         bufSize )
{
   char * pPrintBuf;
   u16 pos;
   int status;
   
   pPrintBuf = kmalloc( bufSize * 3 + 1, GFP_ATOMIC );
   if (pPrintBuf == NULL)
   {
      DBG( "Unable to allocate buffer\n" );
      return;
   }
   memset( pPrintBuf, 0 , bufSize * 3 + 1 );
   
   for (pos = 0; pos < bufSize; pos++)
   {
      status = snprintf( (pPrintBuf + (pos * 3)), 
                         4, 
                         "%02X ", 
                         *(u8 *)(pBuffer + pos) );
      if (status != 3)
      {
         DBG( "snprintf error %d\n", status );
         return;
      }
   }
   
   DBG( "   : %s\n", pPrintBuf );

   kfree( pPrintBuf );
   pPrintBuf = NULL;
   return;   
}

/*===========================================================================
METHOD:
   QSetDownReason (Public Method)

DESCRIPTION:
   Sets mDownReason and turns carrier off

PARAMETERS
   pDev     [ I ] - Device specific memory
   reason   [ I ] - Reason device is down

RETURN VALUE:
   None
===========================================================================*/
void QSetDownReason(
   sQCUSBNet *    pDev,
   u8             reason )
{
   set_bit( reason, &pDev->mDownReason );
   
   netif_carrier_off( pDev->mpNetDev->net );
}

/*===========================================================================
METHOD:
   QClearDownReason (Public Method)

DESCRIPTION:
   Clear mDownReason and may turn carrier on

PARAMETERS
   pDev     [ I ] - Device specific memory
   reason   [ I ] - Reason device is no longer down

RETURN VALUE:
   None
===========================================================================*/
void QClearDownReason(
   sQCUSBNet *    pDev,
   u8             reason )
{
   clear_bit( reason, &pDev->mDownReason );
   
   if (pDev->mDownReason == 0)
   {
      netif_carrier_on( pDev->mpNetDev->net );
   }
}

/*===========================================================================
METHOD:
   QTestDownReason (Public Method)

DESCRIPTION:
   Test mDownReason and returns whether reason is set

PARAMETERS
   pDev     [ I ] - Device specific memory
   reason   [ I ] - Reason device is down

RETURN VALUE:
   bool
===========================================================================*/
bool QTestDownReason(
   sQCUSBNet *    pDev,
   u8             reason )
{
   return test_bit( reason, &pDev->mDownReason );
}

/*=========================================================================*/
// Driver level asynchronous read functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   ReadCallback (Public Method)

DESCRIPTION:
   Put the data in storage and notify anyone waiting for data

PARAMETERS
   pReadURB       [ I ] - URB this callback is run for

RETURN VALUE:
   None
===========================================================================*/
void ReadCallback( struct urb * pReadURB )
{
   int result;
   u16 clientID;
   sClientMemList * pClientMem;
   void * pData;
   void * pDataCopy;
   u16 dataSize;
   sQCUSBNet * pDev;
   unsigned long flags;
   u16 transactionID;

   if (pReadURB == NULL)
   {
      DBG( "bad read URB\n" );
      return;
   }
   
   pDev = pReadURB->context;
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return;
   }   

   if (pReadURB->status != 0)
   {
      DBG( "Read status = %d\n", pReadURB->status );
      return;
   }
   DBG( "Read %d bytes\n", pReadURB->actual_length );
   
   pData = pReadURB->transfer_buffer;
   dataSize = pReadURB->actual_length;

   PrintHex( pData, dataSize );

   result = ParseQMUX( &clientID,
                       pData,
                       dataSize );
   if (result < 0)
   {
      DBG( "Read error parsing QMUX %d\n", result );
      return;
   }
   
   // Grab transaction ID

   // Data large enough?
   if (dataSize < result + 3)
   {
      DBG( "Data buffer too small to parse\n" );
      return;
   }
   
   // Transaction ID size is 1 for QMICTL, 2 for others
   if (clientID == QMICTL)
   {
      transactionID = *(u8*)(pData + result + 1);
   }
   else
   {
      transactionID = *(u16*)(pData + result + 1);
   }
   
   // Critical section
   spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );

   // Find memory storage for this service and Client ID
   // Not using FindClientMem because it can't handle broadcasts
   pClientMem = pDev->mQMIDev.mpClientMemList;
   while (pClientMem != NULL)
   {
      if (pClientMem->mClientID == clientID 
      ||  (pClientMem->mClientID | 0xff00) == clientID)
      {
         // Make copy of pData
         pDataCopy = kmalloc( dataSize, GFP_ATOMIC );
         memcpy( pDataCopy, pData, dataSize );

         if (AddToReadMemList( pDev,
                               pClientMem->mClientID,
                               transactionID,
                               pDataCopy,
                               dataSize ) == false)
         {
            DBG( "Error allocating pReadMemListEntry "
                 "read will be discarded\n" );
            kfree( pDataCopy );
            
            // End critical section
            spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
            return;
         }

         // Success
         DBG( "Creating new readListEntry for client 0x%04X, TID %x\n",
              clientID,
              transactionID );

         // Notify this client data exists
         NotifyAndPopNotifyList( pDev,
                                 pClientMem->mClientID,
                                 transactionID );

         // Not a broadcast
         if (clientID >> 8 != 0xff)
         {
            break;
         }
      }
      
      // Next element
      pClientMem = pClientMem->mpNext;
   }
   
   // End critical section
   spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
}

/*===========================================================================
METHOD:
   IntCallback (Public Method)

DESCRIPTION:
   Data is available, fire off a read URB

PARAMETERS
   pIntURB       [ I ] - URB this callback is run for

RETURN VALUE:
   None
===========================================================================*/
void IntCallback( struct urb * pIntURB )
{
   int status;
   int interval;
   
   sQCUSBNet * pDev = (sQCUSBNet *)pIntURB->context;
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return;
   }

   // Verify this was a normal interrupt
   if (pIntURB->status != 0)
   {
      DBG( "Int status = %d\n", pIntURB->status );
      
      // Ignore EOVERFLOW errors
      if (pIntURB->status != -EOVERFLOW)
      {
         // Read 'thread' dies here
         return;
      }
   }
   else
   {
      // CDC GET_ENCAPSULATED_RESPONSE
      if ((pIntURB->actual_length == 8)
      &&  (*(u64*)pIntURB->transfer_buffer == CDC_GET_ENCAPSULATED_RESPONSE))
      {
         // Time to read
         usb_fill_control_urb( pDev->mQMIDev.mpReadURB,
                               pDev->mpNetDev->udev,
                               usb_rcvctrlpipe( pDev->mpNetDev->udev, 0 ),
                               (unsigned char *)pDev->mQMIDev.mpReadSetupPacket,
                               pDev->mQMIDev.mpReadBuffer,
                               DEFAULT_READ_URB_LENGTH,
                               ReadCallback,
                               pDev );
         status = usb_submit_urb( pDev->mQMIDev.mpReadURB, GFP_ATOMIC );
         if (status != 0)
         {
            DBG( "Error submitting Read URB %d\n", status );
            return;
         }
      }
      // CDC CONNECTION_SPEED_CHANGE
      else if ((pIntURB->actual_length == 16)
      &&       (*(u64*)pIntURB->transfer_buffer == CDC_CONNECTION_SPEED_CHANGE))
      {
         // if upstream or downstream is 0, stop traffic.  Otherwise resume it
         if ((*(u32*)(pIntURB->transfer_buffer + 8) == 0)
         ||  (*(u32*)(pIntURB->transfer_buffer + 12) == 0))
         {
            QSetDownReason( pDev, CDC_CONNECTION_SPEED );
            DBG( "traffic stopping due to CONNECTION_SPEED_CHANGE\n" );
         }
         else
         {
            QClearDownReason( pDev, CDC_CONNECTION_SPEED );
            DBG( "resuming traffic due to CONNECTION_SPEED_CHANGE\n" );
         }
      }
      else
      {
         DBG( "ignoring invalid interrupt in packet\n" );
         PrintHex( pIntURB->transfer_buffer, pIntURB->actual_length );
      }
   }

   interval = (pDev->mpNetDev->udev->speed == USB_SPEED_HIGH) ? 7 : 3;

   // Reschedule interrupt URB
   usb_fill_int_urb( pIntURB,
                     pIntURB->dev,
                     pIntURB->pipe,
                     pIntURB->transfer_buffer,
                     pIntURB->transfer_buffer_length,
                     pIntURB->complete,
                     pIntURB->context,
                     interval );
   status = usb_submit_urb( pIntURB, GFP_ATOMIC );
   if (status != 0)
   {
      DBG( "Error re-submitting Int URB %d\n", status );
   }   
   return;
}

/*===========================================================================
METHOD:
   StartRead (Public Method)

DESCRIPTION:
   Start continuous read "thread" (callback driven)
   
PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int StartRead( sQCUSBNet * pDev )
{
   int interval;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }
   
   // Allocate URB buffers
   pDev->mQMIDev.mpReadURB = usb_alloc_urb( 0, GFP_KERNEL );
   if (pDev->mQMIDev.mpReadURB == NULL)
   {
      DBG( "Error allocating read urb\n" );
      return -ENOMEM;
   }
   
   pDev->mQMIDev.mpIntURB = usb_alloc_urb( 0, GFP_KERNEL );
   if (pDev->mQMIDev.mpIntURB == NULL)
   {
      DBG( "Error allocating int urb\n" );
      return -ENOMEM;
   }

   // Create data buffers
   pDev->mQMIDev.mpReadBuffer = kmalloc( DEFAULT_READ_URB_LENGTH, GFP_KERNEL );
   if (pDev->mQMIDev.mpReadBuffer == NULL)
   {
      DBG( "Error allocating read buffer\n" );
      return -ENOMEM;
   }
   
   pDev->mQMIDev.mpIntBuffer = kmalloc( DEFAULT_READ_URB_LENGTH, GFP_KERNEL );
   if (pDev->mQMIDev.mpIntBuffer == NULL)
   {
      DBG( "Error allocating int buffer\n" );
      return -ENOMEM;
   }      
   
   pDev->mQMIDev.mpReadSetupPacket = kmalloc( sizeof( sURBSetupPacket ), 
                                              GFP_KERNEL );
   if (pDev->mQMIDev.mpReadSetupPacket == NULL)
   {
      DBG( "Error allocating setup packet buffer\n" );
      return -ENOMEM;
   }

   // CDC Get Encapsulated Response packet
   pDev->mQMIDev.mpReadSetupPacket->mRequestType = 0xA1;
   pDev->mQMIDev.mpReadSetupPacket->mRequestCode = 1;
   pDev->mQMIDev.mpReadSetupPacket->mValue = 0;
   pDev->mQMIDev.mpReadSetupPacket->mIndex = 0;
   pDev->mQMIDev.mpReadSetupPacket->mLength = DEFAULT_READ_URB_LENGTH;

   interval = (pDev->mpNetDev->udev->speed == USB_SPEED_HIGH) ? 7 : 3;
   
   // Schedule interrupt URB
   usb_fill_int_urb( pDev->mQMIDev.mpIntURB,
                     pDev->mpNetDev->udev,
                     usb_rcvintpipe( pDev->mpNetDev->udev, 0x81 ),
                     pDev->mQMIDev.mpIntBuffer,
                     DEFAULT_READ_URB_LENGTH,
                     IntCallback,
                     pDev,
                     interval );
   return usb_submit_urb( pDev->mQMIDev.mpIntURB, GFP_KERNEL );
}

/*===========================================================================
METHOD:
   KillRead (Public Method)

DESCRIPTION:
   Kill continuous read "thread"
   
PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
void KillRead( sQCUSBNet * pDev )
{
   // Stop reading
   if (pDev->mQMIDev.mpReadURB != NULL)
   {
      DBG( "Killng read URB\n" );
      usb_kill_urb( pDev->mQMIDev.mpReadURB );
   }

   if (pDev->mQMIDev.mpIntURB != NULL)
   {
      DBG( "Killng int URB\n" );
      usb_kill_urb( pDev->mQMIDev.mpIntURB );
   }

   // Release buffers
   kfree( pDev->mQMIDev.mpReadSetupPacket );
   pDev->mQMIDev.mpReadSetupPacket = NULL;
   kfree( pDev->mQMIDev.mpReadBuffer );
   pDev->mQMIDev.mpReadBuffer = NULL;
   kfree( pDev->mQMIDev.mpIntBuffer );
   pDev->mQMIDev.mpIntBuffer = NULL;
   
   // Release URB's
   usb_free_urb( pDev->mQMIDev.mpReadURB );
   pDev->mQMIDev.mpReadURB = NULL;
   usb_free_urb( pDev->mQMIDev.mpIntURB );
   pDev->mQMIDev.mpIntURB = NULL;
}

/*=========================================================================*/
// Internal read/write functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   ReadAsync (Public Method)

DESCRIPTION:
   Start asynchronous read
   NOTE: Reading client's data store, not device

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any
   pCallback         [ I ] - Callback to be executed when data is available
   pData             [ I ] - Data buffer that willl be passed (unmodified) 
                             to callback

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int ReadAsync(
   sQCUSBNet *    pDev,
   u16            clientID,
   u16            transactionID,
   void           (*pCallback)(sQCUSBNet*, u16, void *),
   void *         pData )
{
   sClientMemList * pClientMem;
   sReadMemList ** ppReadMemList;
   
   unsigned long flags;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   // Critical section
   spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );

   // Find memory storage for this client ID
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find matching client ID 0x%04X\n",
           clientID );
           
      // End critical section
      spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
      return -ENXIO;
   }
   
   ppReadMemList = &(pClientMem->mpList);
   
   // Does data already exist?
   while (*ppReadMemList != NULL)
   {
      // Is this element our data?
      if (transactionID == 0 
      ||  transactionID == (*ppReadMemList)->mTransactionID)
      {
         // End critical section
         spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );

         // Run our own callback
         pCallback( pDev, clientID, pData );
         
         return 0;
      }
      
      // Next
      ppReadMemList = &(*ppReadMemList)->mpNext;
   }

   // Data not found, add ourself to list of waiters
   if (AddToNotifyList( pDev,
                        clientID,
                        transactionID, 
                        pCallback, 
                        pData ) == false)
   {
      DBG( "Unable to register for notification\n" );
   }

   // End critical section
   spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );

   // Success
   return 0;
}

/*===========================================================================
METHOD:
   UpSem (Public Method)

DESCRIPTION:
   Notification function for synchronous read

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   pData             [ I ] - Buffer that holds semaphore to be up()-ed

RETURN VALUE:
   None
===========================================================================*/
void UpSem( 
   sQCUSBNet * pDev,
   u16         clientID,
   void *      pData )
{
   DBG( "0x%04X\n", clientID );
        
   up( (struct semaphore *)pData );
   return;
}

/*===========================================================================
METHOD:
   ReadSync (Public Method)

DESCRIPTION:
   Start synchronous read
   NOTE: Reading client's data store, not device

PARAMETERS:
   pDev              [ I ] - Device specific memory
   ppOutBuffer       [I/O] - On success, will be filled with a 
                             pointer to read buffer
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any

RETURN VALUE:
   int - size of data read for success
         negative errno for failure
===========================================================================*/
int ReadSync(
   sQCUSBNet *    pDev,
   void **        ppOutBuffer,
   u16            clientID,
   u16            transactionID )
{
   int result;
   sClientMemList * pClientMem;
   sNotifyList ** ppNotifyList, * pDelNotifyListEntry;
   struct semaphore readSem;
   void * pData;
   unsigned long flags;
   u16 dataSize;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }
   
   // Critical section
   spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );

   // Find memory storage for this Client ID
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find matching client ID 0x%04X\n",
           clientID );
      
      // End critical section
      spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
      return -ENXIO;
   }
   
   // Note: in cases where read is interrupted, 
   //    this will verify client is still valid
   while (PopFromReadMemList( pDev,
                              clientID,
                              transactionID,
                              &pData,
                              &dataSize ) == false)
   {
      // Data does not yet exist, wait
      sema_init( &readSem, 0 );

      // Add ourself to list of waiters
      if (AddToNotifyList( pDev, 
                           clientID, 
                           transactionID, 
                           UpSem, 
                           &readSem ) == false)
      {
         DBG( "unable to register for notification\n" );
         spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
         return -EFAULT;
      }

      // End critical section while we block
      spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );

      // Wait for notification
      result = down_interruptible( &readSem );
      if (result != 0)
      {
         DBG( "Interrupted %d\n", result );

         // readSem will fall out of scope, 
         // remove from notify list so it's not referenced
         spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );
         ppNotifyList = &(pClientMem->mpReadNotifyList);
         pDelNotifyListEntry = NULL;

         // Find and delete matching entry
         while (*ppNotifyList != NULL)
         {
            if ((*ppNotifyList)->mpData == &readSem)
            {
               pDelNotifyListEntry = *ppNotifyList;
               *ppNotifyList = (*ppNotifyList)->mpNext;
               kfree( pDelNotifyListEntry );
               break;
            }

            // Next
            ppNotifyList = &(*ppNotifyList)->mpNext;
         }

         spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
         return -EINTR;
      }
      
      // Verify device is still valid
      if (IsDeviceValid( pDev ) == false)
      {
         DBG( "Invalid device!\n" );
         return -ENXIO;
      }
      
      // Restart critical section and continue loop
      spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );
   }
   
   // End Critical section
   spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );

   // Success
   *ppOutBuffer = pData;

   return dataSize;
}

/*===========================================================================
METHOD:
   WriteSyncCallback (Public Method)

DESCRIPTION:
   Write callback

PARAMETERS
   pWriteURB       [ I ] - URB this callback is run for

RETURN VALUE:
   None
===========================================================================*/
void WriteSyncCallback( struct urb * pWriteURB )
{
   if (pWriteURB == NULL)
   {
      DBG( "null urb\n" );
      return;
   }

   DBG( "Write status/size %d/%d\n", 
        pWriteURB->status, 
        pWriteURB->actual_length );

   // Notify that write has completed by up()-ing semeaphore
   up( (struct semaphore * )pWriteURB->context );
   
   return;
}

/*===========================================================================
METHOD:
   WriteSync (Public Method)

DESCRIPTION:
   Start synchronous write

PARAMETERS:
   pDev                 [ I ] - Device specific memory
   pWriteBuffer         [ I ] - Data to be written
   writeBufferSize      [ I ] - Size of data to be written
   clientID             [ I ] - Client ID of requester

RETURN VALUE:
   int - write size (includes QMUX)
         negative errno for failure
===========================================================================*/
int WriteSync(
   sQCUSBNet *        pDev,
   char *             pWriteBuffer,
   int                writeBufferSize,
   u16                clientID )
{
   int result;
   struct semaphore writeSem;
   struct urb * pWriteURB;
   sURBSetupPacket writeSetup;
   unsigned long flags;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   pWriteURB = usb_alloc_urb( 0, GFP_KERNEL );
   if (pWriteURB == NULL)
   {
      DBG( "URB mem error\n" );
      return -ENOMEM;
   }

   // Fill writeBuffer with QMUX
   result = FillQMUX( clientID, pWriteBuffer, writeBufferSize );
   if (result < 0)
   {
      usb_free_urb( pWriteURB );
      return result;
   }

   // CDC Send Encapsulated Request packet
   writeSetup.mRequestType = 0x21;
   writeSetup.mRequestCode = 0;
   writeSetup.mValue = 0;
   writeSetup.mIndex = 0;
   writeSetup.mLength = 0;
   writeSetup.mLength = writeBufferSize;

   // Create URB   
   usb_fill_control_urb( pWriteURB,
                         pDev->mpNetDev->udev,
                         usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
                         (unsigned char *)&writeSetup,
                         (void*)pWriteBuffer,
                         writeBufferSize,
                         NULL,
                         pDev );

   DBG( "Actual Write:\n" );
   PrintHex( pWriteBuffer, writeBufferSize );

   sema_init( &writeSem, 0 );
   
   pWriteURB->complete = WriteSyncCallback;
   pWriteURB->context = &writeSem;
   
   // Wake device
   result = usb_autopm_get_interface( pDev->mpIntf );
   if (result < 0)
   {
      DBG( "unable to resume interface: %d\n", result );
      
      // Likely caused by device going from autosuspend -> full suspend
      if (result == -EPERM)
      {
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,33 ))
         pDev->mpNetDev->udev->auto_pm = 0;
#endif
         QCSuspend( pDev->mpIntf, PMSG_SUSPEND );
      }

      return result;
   }

   // Critical section
   spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );

   if (AddToURBList( pDev, clientID, pWriteURB ) == false)
   {
      usb_free_urb( pWriteURB );

      // End critical section
      spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );   
      usb_autopm_put_interface( pDev->mpIntf );
      return -EINVAL;
   }

   result = usb_submit_urb( pWriteURB, GFP_KERNEL );
   if (result < 0)
   {
      DBG( "submit URB error %d\n", result );
      
      // Get URB back so we can destroy it
      if (PopFromURBList( pDev, clientID ) != pWriteURB)
      {
         // This shouldn't happen
         DBG( "Didn't get write URB back\n" );
      }

      usb_free_urb( pWriteURB );

      // End critical section
      spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
      usb_autopm_put_interface( pDev->mpIntf );
      return result;
   }
   
   // End critical section while we block
   spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );   

   // Wait for write to finish
   result = down_interruptible( &writeSem );

   // Verify device is still valid
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   // Write is done, release device
   usb_autopm_put_interface( pDev->mpIntf );

   // Restart critical section
   spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );

   // Get URB back so we can destroy it
   if (PopFromURBList( pDev, clientID ) != pWriteURB)
   {
      // This shouldn't happen
      DBG( "Didn't get write URB back\n" );
   
      // End critical section
      spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );   
      return -EINVAL;
   }

   // End critical section
   spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );   

   if (result == 0)
   {
      // Write is finished
      if (pWriteURB->status == 0)
      {
         // Return number of bytes that were supposed to have been written,
         //   not size of QMI request
         result = writeBufferSize;
      }
      else
      {
         DBG( "bad status = %d\n", pWriteURB->status );
         
         // Return error value
         result = pWriteURB->status;
      }
   }
   else
   {
      // We have been forcibly interrupted
      DBG( "Interrupted %d !!!\n", result );
      DBG( "Device may be in bad state and need reset !!!\n" );

      // URB has not finished
      usb_kill_urb( pWriteURB );
   }

   usb_free_urb( pWriteURB );

   return result;
}

/*=========================================================================*/
// Internal memory management functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   GetClientID (Public Method)

DESCRIPTION:
   Construct object/load file into memory

PARAMETERS:
   pDev           [ I ] - Device specific memory
   serviceType    [ I ] - Desired QMI service type

RETURN VALUE:
   int - Client ID for success (positive)
         Negative errno for error
===========================================================================*/
int GetClientID( 
   sQCUSBNet *    pDev,
   u8             serviceType )
{
   u16 clientID;
   sClientMemList ** ppClientMem;
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   unsigned long flags;
   u8 transactionID;
   
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   // Run QMI request to be asigned a Client ID
   if (serviceType != 0)
   {
      writeBufferSize = QMICTLGetClientIDReqSize();
      pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
      if (pWriteBuffer == NULL)
      {
         return -ENOMEM;
      }

      transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
      if (transactionID == 0)
      {
         atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
      }
      result = QMICTLGetClientIDReq( pWriteBuffer, 
                                     writeBufferSize,
                                     transactionID,
                                     serviceType );
      if (result < 0)
      {
         kfree( pWriteBuffer );
         return result;
      }
      
      result = WriteSync( pDev,
                          pWriteBuffer,
                          writeBufferSize,
                          QMICTL );
      kfree( pWriteBuffer );

      if (result < 0)
      {
         return result;
      }

      result = ReadSync( pDev,
                         &pReadBuffer,
                         QMICTL,
                         transactionID );
      if (result < 0)
      {
         DBG( "bad read data %d\n", result );
         return result;
      }
      readBufferSize = result;

      result = QMICTLGetClientIDResp( pReadBuffer,
                                      readBufferSize,
                                      &clientID );
      kfree( pReadBuffer );

      if (result < 0)
      {
         return result;
      }
   }
   else
   {
      // QMI CTL will always have client ID 0
      clientID = 0;
   }

   // Critical section
   spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );

   // Verify client is not already allocated
   if (FindClientMem( pDev, clientID ) != NULL)
   {
      DBG( "Client memory already exists\n" );

      // End Critical section
      spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
      return -ETOOMANYREFS;
   }

   // Go to last entry in client mem list
   ppClientMem = &pDev->mQMIDev.mpClientMemList;
   while (*ppClientMem != NULL)
   {
      ppClientMem = &(*ppClientMem)->mpNext;
   }
   
   // Create locations for read to place data into
   *ppClientMem = kmalloc( sizeof( sClientMemList ), GFP_ATOMIC );
   if (*ppClientMem == NULL)
   {
      DBG( "Error allocating read list\n" );

      // End critical section
      spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
      return -ENOMEM;
   }
      
   (*ppClientMem)->mClientID = clientID;
   (*ppClientMem)->mpList = NULL;
   (*ppClientMem)->mpReadNotifyList = NULL;
   (*ppClientMem)->mpURBList = NULL;
   (*ppClientMem)->mpNext = NULL;


   // End Critical section
   spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
   
   return clientID;
}

/*===========================================================================
METHOD:
   ReleaseClientID (Public Method)

DESCRIPTION:
   Release QMI client and free memory

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Requester's client ID

RETURN VALUE:
   None
===========================================================================*/
void ReleaseClientID(
   sQCUSBNet *    pDev,
   u16            clientID )
{
   int result;
   sClientMemList ** ppDelClientMem;
   sClientMemList * pNextClientMem;
   struct urb * pDelURB;
   void * pDelData;
   u16 dataSize;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   unsigned long flags;
   u8 transactionID;

   // Is device is still valid?
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "invalid device\n" );
      return;
   }
   
   DBG( "releasing 0x%04X\n", clientID );

   // Run QMI ReleaseClientID if this isn't QMICTL   
   if (clientID != QMICTL)
   {
      // Note: all errors are non fatal, as we always want to delete 
      //    client memory in latter part of function
      
      writeBufferSize = QMICTLReleaseClientIDReqSize();
      pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
      if (pWriteBuffer == NULL)
      {
         DBG( "memory error\n" );
      }
      else
      {
         transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
         if (transactionID == 0)
         {
            transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
         }
         result = QMICTLReleaseClientIDReq( pWriteBuffer, 
                                            writeBufferSize,
                                            transactionID,
                                            clientID );
         if (result < 0)
         {
            kfree( pWriteBuffer );
            DBG( "error %d filling req buffer\n", result );
         }
         else
         {
            result = WriteSync( pDev,
                                pWriteBuffer,
                                writeBufferSize,
                                QMICTL );
            kfree( pWriteBuffer );

            if (result < 0)
            {
               DBG( "bad write status %d\n", result );
            }
            else
            {
               result = ReadSync( pDev,
                                  &pReadBuffer,
                                  QMICTL,
                                  transactionID );
               if (result < 0)
               {
                  DBG( "bad read status %d\n", result );
               }
               else
               {
                  readBufferSize = result;

                  result = QMICTLReleaseClientIDResp( pReadBuffer,
                                                      readBufferSize );
                  kfree( pReadBuffer );

                  if (result < 0)
                  {
                     DBG( "error %d parsing response\n", result );
                  }
               }
            }
         }
      }
   }

   // Cleaning up client memory
   
   // Critical section
   spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );

   // Can't use FindClientMem, I need to keep pointer of previous
   ppDelClientMem = &pDev->mQMIDev.mpClientMemList;
   while (*ppDelClientMem != NULL)
   {
      if ((*ppDelClientMem)->mClientID == clientID)
      {
         pNextClientMem = (*ppDelClientMem)->mpNext;

         // Notify all clients
         while (NotifyAndPopNotifyList( pDev,
                                        clientID,
                                        0 ) == true );         

         // Kill and free all URB's
         pDelURB = PopFromURBList( pDev, clientID );
         while (pDelURB != NULL)
         {
            usb_kill_urb( pDelURB );
            usb_free_urb( pDelURB );
            pDelURB = PopFromURBList( pDev, clientID );
         }

         // Free any unread data
         while (PopFromReadMemList( pDev, 
                                    clientID,
                                    0,
                                    &pDelData,
                                    &dataSize ) == true )
         {
            kfree( pDelData );
         }

         // Delete client Mem
         kfree( *ppDelClientMem );

         // Overwrite the pointer that was to this client mem
         *ppDelClientMem = pNextClientMem;
      }
      else
      {
         // I now point to ( a pointer of ((the node I was at)'s mpNext))
         ppDelClientMem = &(*ppDelClientMem)->mpNext;
      }
   }
   
   // End Critical section
   spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );

   return;
}

/*===========================================================================
METHOD:
   FindClientMem (Public Method)

DESCRIPTION:
   Find this client's memory

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Requester's client ID

RETURN VALUE:
   sClientMemList - Pointer to requested sClientMemList for success
                    NULL for error
===========================================================================*/
sClientMemList * FindClientMem( 
   sQCUSBNet *      pDev,
   u16              clientID )
{
   sClientMemList * pClientMem;
   
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return NULL;
   }
   
#ifdef CONFIG_SMP
   // Verify Lock
   if (spin_is_locked( &pDev->mQMIDev.mClientMemLock ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif
   
   pClientMem = pDev->mQMIDev.mpClientMemList;
   while (pClientMem != NULL)
   {
      if (pClientMem->mClientID == clientID)
      {
         // Success
         //DBG( "Found client mem %p\n", pClientMem );
         return pClientMem;
      }
      
      pClientMem = pClientMem->mpNext;
   }

   DBG( "Could not find client mem 0x%04X\n", clientID );
   return NULL;
}

/*===========================================================================
METHOD:
   AddToReadMemList (Public Method)

DESCRIPTION:
   Add Data to this client's ReadMem list
   
   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Requester's client ID
   transactionID  [ I ] - Transaction ID or 0 for any
   pData          [ I ] - Data to add
   dataSize       [ I ] - Size of data to add

RETURN VALUE:
   bool
===========================================================================*/
bool AddToReadMemList( 
   sQCUSBNet *      pDev,
   u16              clientID,
   u16              transactionID,
   void *           pData,
   u16              dataSize )
{
   sClientMemList * pClientMem;
   sReadMemList ** ppThisReadMemList;

#ifdef CONFIG_SMP
   // Verify Lock
   if (spin_is_locked( &pDev->mQMIDev.mClientMemLock ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n",
           clientID );

      return false;
   }

   // Go to last ReadMemList entry
   ppThisReadMemList = &pClientMem->mpList;
   while (*ppThisReadMemList != NULL)
   {
      ppThisReadMemList = &(*ppThisReadMemList)->mpNext;
   }
   
   *ppThisReadMemList = kmalloc( sizeof( sReadMemList ), GFP_ATOMIC );
   if (*ppThisReadMemList == NULL)
   {
      DBG( "Mem error\n" );

      return false;
   }   
   
   (*ppThisReadMemList)->mpNext = NULL;
   (*ppThisReadMemList)->mpData = pData;
   (*ppThisReadMemList)->mDataSize = dataSize;
   (*ppThisReadMemList)->mTransactionID = transactionID;
   
   return true;
}

/*===========================================================================
METHOD:
   PopFromReadMemList (Public Method)

DESCRIPTION:
   Remove data from this client's ReadMem list if it matches 
   the specified transaction ID.
   
   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any
   ppData            [I/O] - On success, will be filled with a 
                             pointer to read buffer
   pDataSize         [I/O] - On succces, will be filled with the 
                             read buffer's size

RETURN VALUE:
   bool
===========================================================================*/
bool PopFromReadMemList( 
   sQCUSBNet *      pDev,
   u16              clientID,
   u16              transactionID,
   void **          ppData,
   u16 *            pDataSize )
{
   sClientMemList * pClientMem;
   sReadMemList * pDelReadMemList, ** ppReadMemList;

#ifdef CONFIG_SMP
   // Verify Lock
   if (spin_is_locked( &pDev->mQMIDev.mClientMemLock ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n",
           clientID );

      return false;
   }
   
   ppReadMemList = &(pClientMem->mpList);
   pDelReadMemList = NULL;
   
   // Find first message that matches this transaction ID
   while (*ppReadMemList != NULL)
   {
      // Do we care about transaction ID?
      if (transactionID == 0
      ||  transactionID == (*ppReadMemList)->mTransactionID )
      {
         pDelReadMemList = *ppReadMemList;
         break;
      }
      
      DBG( "skipping 0x%04X data TID = %x\n", clientID, (*ppReadMemList)->mTransactionID );
      
      // Next
      ppReadMemList = &(*ppReadMemList)->mpNext;
   }
   
   if (pDelReadMemList != NULL)
   {
      *ppReadMemList = (*ppReadMemList)->mpNext;
      
      // Copy to output
      *ppData = pDelReadMemList->mpData;
      *pDataSize = pDelReadMemList->mDataSize;
      
      // Free memory
      kfree( pDelReadMemList );
      
      return true;
   }
   else
   {
      DBG( "No read memory to pop, Client 0x%04X, TID = %x\n", 
           clientID, 
           transactionID );
      return false;
   }
}

/*===========================================================================
METHOD:
   AddToNotifyList (Public Method)

DESCRIPTION:
   Add Notify entry to this client's notify List
   
   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any
   pNotifyFunct      [ I ] - Callback function to be run when data is available
   pData             [ I ] - Data buffer that willl be passed (unmodified) 
                             to callback

RETURN VALUE:
   bool
===========================================================================*/
bool AddToNotifyList( 
   sQCUSBNet *      pDev,
   u16              clientID,
   u16              transactionID,
   void             (* pNotifyFunct)(sQCUSBNet *, u16, void *),
   void *           pData )
{
   sClientMemList * pClientMem;
   sNotifyList ** ppThisNotifyList;

#ifdef CONFIG_SMP
   // Verify Lock
   if (spin_is_locked( &pDev->mQMIDev.mClientMemLock ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n", clientID );
      return false;
   }

   // Go to last URBList entry
   ppThisNotifyList = &pClientMem->mpReadNotifyList;
   while (*ppThisNotifyList != NULL)
   {
      ppThisNotifyList = &(*ppThisNotifyList)->mpNext;
   }
   
   *ppThisNotifyList = kmalloc( sizeof( sNotifyList ), GFP_ATOMIC );
   if (*ppThisNotifyList == NULL)
   {
      DBG( "Mem error\n" );
      return false;
   }   
   
   (*ppThisNotifyList)->mpNext = NULL;
   (*ppThisNotifyList)->mpNotifyFunct = pNotifyFunct;
   (*ppThisNotifyList)->mpData = pData;
   (*ppThisNotifyList)->mTransactionID = transactionID;
   
   return true;
}

/*===========================================================================
METHOD:
   NotifyAndPopNotifyList (Public Method)

DESCRIPTION:
   Remove first Notify entry from this client's notify list 
   and Run function
   
   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any

RETURN VALUE:
   bool
===========================================================================*/
bool NotifyAndPopNotifyList( 
   sQCUSBNet *      pDev,
   u16              clientID,
   u16              transactionID )
{
   sClientMemList * pClientMem;
   sNotifyList * pDelNotifyList, ** ppNotifyList;

#ifdef CONFIG_SMP
   // Verify Lock
   if (spin_is_locked( &pDev->mQMIDev.mClientMemLock ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n", clientID );
      return false;
   }

   ppNotifyList = &(pClientMem->mpReadNotifyList);
   pDelNotifyList = NULL;

   // Remove from list
   while (*ppNotifyList != NULL)
   {
      // Do we care about transaction ID?
      if (transactionID == 0
      ||  (*ppNotifyList)->mTransactionID == 0
      ||  transactionID == (*ppNotifyList)->mTransactionID)
      {
         pDelNotifyList = *ppNotifyList;
         break;
      }
      
      DBG( "skipping data TID = %x\n", (*ppNotifyList)->mTransactionID );
      
      // next
      ppNotifyList = &(*ppNotifyList)->mpNext;
   }
   
   if (pDelNotifyList != NULL)
   {
      // Remove element
      *ppNotifyList = (*ppNotifyList)->mpNext;
      
      // Run notification function
      if (pDelNotifyList->mpNotifyFunct != NULL)
      {
         // Unlock for callback
         spin_unlock( &pDev->mQMIDev.mClientMemLock );
      
         pDelNotifyList->mpNotifyFunct( pDev,
                                        clientID,
                                        pDelNotifyList->mpData );
                                        
         // Restore lock
         spin_lock( &pDev->mQMIDev.mClientMemLock );
      }
      
      // Delete memory
      kfree( pDelNotifyList );

      return true;
   }
   else
   {
      DBG( "no one to notify for TID %x\n", transactionID );
      
      return false;
   }
}

/*===========================================================================
METHOD:
   AddToURBList (Public Method)

DESCRIPTION:
   Add URB to this client's URB list
   
   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   pURB              [ I ] - URB to be added

RETURN VALUE:
   bool
===========================================================================*/
bool AddToURBList( 
   sQCUSBNet *      pDev,
   u16              clientID,
   struct urb *     pURB )
{
   sClientMemList * pClientMem;
   sURBList ** ppThisURBList;

#ifdef CONFIG_SMP
   // Verify Lock
   if (spin_is_locked( &pDev->mQMIDev.mClientMemLock ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n", clientID );
      return false;
   }

   // Go to last URBList entry
   ppThisURBList = &pClientMem->mpURBList;
   while (*ppThisURBList != NULL)
   {
      ppThisURBList = &(*ppThisURBList)->mpNext;
   }
   
   *ppThisURBList = kmalloc( sizeof( sURBList ), GFP_ATOMIC );
   if (*ppThisURBList == NULL)
   {
      DBG( "Mem error\n" );
      return false;
   }   
   
   (*ppThisURBList)->mpNext = NULL;
   (*ppThisURBList)->mpURB = pURB;
   
   return true;
}

/*===========================================================================
METHOD:
   PopFromURBList (Public Method)

DESCRIPTION:
   Remove URB from this client's URB list
   
   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Requester's client ID

RETURN VALUE:
   struct urb - Pointer to requested client's URB
                NULL for error
===========================================================================*/
struct urb * PopFromURBList( 
   sQCUSBNet *      pDev,
   u16              clientID )
{
   sClientMemList * pClientMem;
   sURBList * pDelURBList;
   struct urb * pURB;

#ifdef CONFIG_SMP
   // Verify Lock
   if (spin_is_locked( &pDev->mQMIDev.mClientMemLock ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n", clientID );
      return NULL;
   }

   // Remove from list
   if (pClientMem->mpURBList != NULL)
   {
      pDelURBList = pClientMem->mpURBList;
      pClientMem->mpURBList = pClientMem->mpURBList->mpNext;
      
      // Copy to output
      pURB = pDelURBList->mpURB;
      
      // Delete memory
      kfree( pDelURBList );

      return pURB;
   }
   else
   {
      DBG( "No URB's to pop\n" );
      
      return NULL;
   }
}

/*=========================================================================*/
// Userspace wrappers
/*=========================================================================*/

/*===========================================================================
METHOD:
   UserspaceOpen (Public Method)

DESCRIPTION:
   Userspace open
      IOCTL must be called before reads or writes

PARAMETERS
   pInode       [ I ] - kernel file descriptor
   pFilp        [ I ] - userspace file descriptor

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int UserspaceOpen( 
   struct inode *         pInode, 
   struct file *          pFilp )
{
   sQMIFilpStorage * pFilpData;
   
   // Optain device pointer from pInode
   sQMIDev * pQMIDev = container_of( pInode->i_cdev,
                                     sQMIDev,
                                     mCdev );
   sQCUSBNet * pDev = container_of( pQMIDev,
                                    sQCUSBNet,
                                    mQMIDev );                                    

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return -ENXIO;
   }

   // Setup data in pFilp->private_data
   pFilp->private_data = kmalloc( sizeof( sQMIFilpStorage ), GFP_KERNEL );
   if (pFilp->private_data == NULL)
   {
      DBG( "Mem error\n" );
      return -ENOMEM;
   }
   
   pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   pFilpData->mClientID = (u16)-1;
   pFilpData->mpDev = pDev;
   
   return 0;
}

/*===========================================================================
METHOD:
   UserspaceIOCTL (Public Method)

DESCRIPTION:
   Userspace IOCTL functions

PARAMETERS
   pUnusedInode [ I ] - (unused) kernel file descriptor
   pFilp        [ I ] - userspace file descriptor
   cmd          [ I ] - IOCTL command
   arg          [ I ] - IOCTL argument

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int UserspaceIOCTL( 
   struct inode *    pUnusedInode, 
   struct file *     pFilp,
   unsigned int      cmd, 
   unsigned long     arg )
{
   int result;
   u32 devVIDPID;
   
   sQMIFilpStorage * pFilpData = (sQMIFilpStorage *)pFilp->private_data;

   if (pFilpData == NULL)
   {
      DBG( "Bad file data\n" );
      return -EBADF;
   }
   
   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      pFilp->f_op = pFilp->f_dentry->d_inode->i_fop;
      return -ENXIO;
   }

   switch (cmd)
   {
      case IOCTL_QMI_GET_SERVICE_FILE:
      
         DBG( "Setting up QMI for service %lu\n", arg );
         if ((u8)arg == 0)
         {
            DBG( "Cannot use QMICTL from userspace\n" );
            return -EINVAL;
         }

         // Connection is already setup
         if (pFilpData->mClientID != (u16)-1)
         {
            DBG( "Close the current connection before opening a new one\n" );
            return -EBADR;
         }
         
         result = GetClientID( pFilpData->mpDev, (u8)arg );
         if (result < 0)
         {
            return result;
         }
         pFilpData->mClientID = result;

         return 0;
         break;


      case IOCTL_QMI_GET_DEVICE_VIDPID:
         if (arg == 0)
         {
            DBG( "Bad VIDPID buffer\n" );
            return -EINVAL;
         }
         
         // Extra verification
         if (pFilpData->mpDev->mpNetDev == 0)
         {
            DBG( "Bad mpNetDev\n" );
            return -ENOMEM;
         }
         if (pFilpData->mpDev->mpNetDev->udev == 0)
         {
            DBG( "Bad udev\n" );
            return -ENOMEM;
         }

         devVIDPID = ((le16_to_cpu( pFilpData->mpDev->mpNetDev->udev->descriptor.idVendor ) << 16)
                     + le16_to_cpu( pFilpData->mpDev->mpNetDev->udev->descriptor.idProduct ) );

         result = copy_to_user( (unsigned int *)arg, &devVIDPID, 4 );
         if (result != 0)
         {
            DBG( "Copy to userspace failure\n" );
         }

         return result;
                 
         break;

      case IOCTL_QMI_GET_DEVICE_MEID:
         if (arg == 0)
         {
            DBG( "Bad MEID buffer\n" );
            return -EINVAL;
         }
         
         result = copy_to_user( (unsigned int *)arg, &pFilpData->mpDev->mMEID[0], 14 );
         if (result != 0)
         {
            DBG( "copy to userspace failure\n" );
         }

         return result;
                 
         break;
         
      default:
         return -EBADRQC;       
   }
}

/*===========================================================================
METHOD:
   UserspaceClose (Public Method)

DESCRIPTION:
   Userspace close
      Release client ID and free memory

PARAMETERS
   pFilp           [ I ] - userspace file descriptor
   unusedFileTable [ I ] - (unused) file table

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int UserspaceClose(
   struct file *       pFilp,
   fl_owner_t          unusedFileTable )
{
   sQMIFilpStorage * pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   struct list_head * pTasks;
   struct task_struct * pEachTask;
   struct fdtable * pFDT;
   int count = 0;
   int used = 0;
   unsigned long flags;

   if (pFilpData == NULL)
   {
      DBG( "bad file data\n" );
      return -EBADF;
   }

   // Fallthough.  If f_count == 1 no need to do more checks
   if (atomic_read( &pFilp->f_count ) != 1)
   {
      // "group_leader" points to the main process' task, which resides in
      // the global "tasks" list.
      list_for_each( pTasks, &current->group_leader->tasks )
      {
         pEachTask = container_of( pTasks, struct task_struct, tasks );
         if (pEachTask == NULL || pEachTask->files == NULL)
         {
            // Some tasks may not have files (e.g. Xsession)
            continue;
         }
         spin_lock_irqsave( &pEachTask->files->file_lock, flags );
         pFDT = files_fdtable( pEachTask->files );
         for (count = 0; count < pFDT->max_fds; count++)
         {
            // Before this function was called, this file was removed
            // from our task's file table so if we find it in a file
            // table then it is being used by another task
            if (pFDT->fd[count] == pFilp)
            {
               used++;
               break;
            }
         }
         spin_unlock_irqrestore( &pEachTask->files->file_lock, flags );
      }
      
      if (used > 0)
      {
         DBG( "not closing, as this FD is open by %d other process\n", used );
         return 0;
      }
   }

   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      pFilp->f_op = pFilp->f_dentry->d_inode->i_fop;
      return -ENXIO;
   }
   
   DBG( "0x%04X\n", pFilpData->mClientID );
   
   // Disable pFilpData so they can't keep sending read or write 
   //    should this function hang
   // Note: memory pointer is still saved in pFilpData to be deleted later
   pFilp->private_data = NULL;

   if (pFilpData->mClientID != (u16)-1)
   {
      ReleaseClientID( pFilpData->mpDev,
                       pFilpData->mClientID );
   }
      
   kfree( pFilpData );
   return 0;
}

/*===========================================================================
METHOD:
   UserspaceRead (Public Method)

DESCRIPTION:
   Userspace read (synchronous)

PARAMETERS
   pFilp           [ I ] - userspace file descriptor
   pBuf            [ I ] - read buffer
   size            [ I ] - size of read buffer
   pUnusedFpos     [ I ] - (unused) file position

RETURN VALUE:
   ssize_t - Number of bytes read for success
             Negative errno for failure
===========================================================================*/
ssize_t UserspaceRead( 
   struct file *          pFilp,
   char __user *          pBuf, 
   size_t                 size,
   loff_t *               pUnusedFpos )
{
   int result;
   void * pReadData = NULL;
   void * pSmallReadData;
   sQMIFilpStorage * pFilpData = (sQMIFilpStorage *)pFilp->private_data;

   if (pFilpData == NULL)
   {
      DBG( "Bad file data\n" );
      return -EBADF;
   }

   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      pFilp->f_op = pFilp->f_dentry->d_inode->i_fop;
      return -ENXIO;
   }
   
   if (pFilpData->mClientID == (u16)-1)
   {
      DBG( "Client ID must be set before reading 0x%04X\n",
           pFilpData->mClientID );
      return -EBADR;
   }
   
   // Perform synchronous read
   result = ReadSync( pFilpData->mpDev,
                      &pReadData,
                      pFilpData->mClientID,
                      0 );
   if (result <= 0)
   {
      return result;
   }
   
   // Discard QMUX header
   result -= QMUXHeaderSize();
   pSmallReadData = pReadData + QMUXHeaderSize();

   if (result > size)
   {
      DBG( "Read data is too large for amount user has requested\n" );
      kfree( pReadData );
      return -EOVERFLOW;
   }

   if (copy_to_user( pBuf, pSmallReadData, result ) != 0)
   {
      DBG( "Error copying read data to user\n" );
      result = -EFAULT;
   }
   
   // Reader is responsible for freeing read buffer
   kfree( pReadData );
   
   return result;
}

/*===========================================================================
METHOD:
   UserspaceWrite (Public Method)

DESCRIPTION:
   Userspace write (synchronous)

PARAMETERS
   pFilp           [ I ] - userspace file descriptor
   pBuf            [ I ] - write buffer
   size            [ I ] - size of write buffer
   pUnusedFpos     [ I ] - (unused) file position

RETURN VALUE:
   ssize_t - Number of bytes read for success
             Negative errno for failure
===========================================================================*/
ssize_t UserspaceWrite (
   struct file *        pFilp, 
   const char __user *  pBuf, 
   size_t               size,
   loff_t *             pUnusedFpos )
{
   int status;
   void * pWriteBuffer;
   sQMIFilpStorage * pFilpData = (sQMIFilpStorage *)pFilp->private_data;

   if (pFilpData == NULL)
   {
      DBG( "Bad file data\n" );
      return -EBADF;
   }

   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      pFilp->f_op = pFilp->f_dentry->d_inode->i_fop;
      return -ENXIO;
   }

   if (pFilpData->mClientID == (u16)-1)
   {
      DBG( "Client ID must be set before writing 0x%04X\n",
           pFilpData->mClientID );
      return -EBADR;
   }
   
   // Copy data from user to kernel space
   pWriteBuffer = kmalloc( size + QMUXHeaderSize(), GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }
   status = copy_from_user( pWriteBuffer + QMUXHeaderSize(), pBuf, size );
   if (status != 0)
   {
      DBG( "Unable to copy data from userspace %d\n", status );
      kfree( pWriteBuffer );
      return status;
   }

   status = WriteSync( pFilpData->mpDev,
                       pWriteBuffer, 
                       size + QMUXHeaderSize(),
                       pFilpData->mClientID );

   kfree( pWriteBuffer );
   
   // On success, return requested size, not full QMI reqest size
   if (status == size + QMUXHeaderSize())
   {
      return size;
   }
   else
   {
      return status;
   }
}

/*=========================================================================*/
// Initializer and destructor
/*=========================================================================*/

/*===========================================================================
METHOD:
   RegisterQMIDevice (Public Method)

DESCRIPTION:
   QMI Device initialization function

PARAMETERS:
   pDev     [ I ] - Device specific memory
   
RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int RegisterQMIDevice( sQCUSBNet * pDev )
{
   int result;
   int QCQMIIndex = 0;
   dev_t devno; 
   char * pDevName;
   
   pDev->mbQMIValid = true;

   // Set up for QMICTL
   //    (does not send QMI message, just sets up memory)
   result = GetClientID( pDev, QMICTL );
   if (result != 0)
   {
      pDev->mbQMIValid = false;
      return result;
   }
   atomic_set( &pDev->mQMIDev.mQMICTLTransactionID, 1 );

   // Start Async reading
   result = StartRead( pDev );
   if (result != 0)
   {
      pDev->mbQMIValid = false;
      return result;
   }
   
   // Device is not ready for QMI connections right away
   //   Wait up to 30 seconds before failing
   if (QMIReady( pDev, 30000 ) == false)
   {
      DBG( "Device unresponsive to QMI\n" );
      return -ETIMEDOUT;
   }

   // Setup WDS callback
   result = SetupQMIWDSCallback( pDev );
   if (result != 0)
   {
      pDev->mbQMIValid = false;
      return result;
   }

   // Fill MEID for device
   result = QMIDMSGetMEID( pDev );
   if (result != 0)
   {
      pDev->mbQMIValid = false;
      return result;
   }

   // allocate and fill devno with numbers
   result = alloc_chrdev_region( &devno, 0, 1, "qcqmi" );
   if (result < 0)
   {
      return result;
   }

   // Create cdev
   cdev_init( &pDev->mQMIDev.mCdev, &UserspaceQMIFops );
   pDev->mQMIDev.mCdev.owner = THIS_MODULE;
   pDev->mQMIDev.mCdev.ops = &UserspaceQMIFops;

   result = cdev_add( &pDev->mQMIDev.mCdev, devno, 1 );
   if (result != 0)
   {
      DBG( "error adding cdev\n" );
      return result;
   }

   // Match interface number (usb#)
   pDevName = strstr( pDev->mpNetDev->net->name, "usb" );
   if (pDevName == NULL)
   {
      DBG( "Bad net name: %s\n", pDev->mpNetDev->net->name );
      return -ENXIO;
   }
   pDevName += strlen("usb");
   QCQMIIndex = simple_strtoul( pDevName, NULL, 10 );
   if(QCQMIIndex < 0 )
   {
      DBG( "Bad minor number\n" );
      return -ENXIO;
   }

   // Always print this output
   printk( KERN_INFO "creating qcqmi%d\n",
           QCQMIIndex );
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,27 ))
   // kernel 2.6.27 added a new fourth parameter to device_create
   //    void * drvdata : the data to be added to the device for callbacks
   device_create( pDev->mQMIDev.mpDevClass,
                  NULL, 
                  devno,
                  NULL,
                  "qcqmi%d", 
                  QCQMIIndex );
#else
   device_create( pDev->mQMIDev.mpDevClass,
                  NULL, 
                  devno,
                  "qcqmi%d", 
                  QCQMIIndex );
#endif
   
   pDev->mQMIDev.mDevNum = devno;
   
   // Success
   return 0;
}

/*===========================================================================
METHOD:
   DeregisterQMIDevice (Public Method)

DESCRIPTION:
   QMI Device cleanup function
   
   NOTE: When this function is run the device is no longer valid

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
void DeregisterQMIDevice( sQCUSBNet * pDev )
{
   struct inode * pOpenInode;
   struct list_head * pInodeList;
   struct list_head * pTasks;
   struct task_struct * pEachTask;
   struct fdtable * pFDT;
   struct file * pFilp;
   unsigned long flags;
   int count = 0;

   // Should never happen, but check anyway
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "wrong device\n" );
      return;
   }

   // Release all clients
   while (pDev->mQMIDev.mpClientMemList != NULL)
   {
      DBG( "release 0x%04X\n", pDev->mQMIDev.mpClientMemList->mClientID );
   
      ReleaseClientID( pDev,
                       pDev->mQMIDev.mpClientMemList->mClientID );
      // NOTE: pDev->mQMIDev.mpClientMemList will 
      //       be updated in ReleaseClientID()
   }

   // Stop all reads
   KillRead( pDev );

   pDev->mbQMIValid = false;

   // Find each open file handle, and manually close it
   
   // Generally there will only be only one inode, but more are possible
   list_for_each( pInodeList, &pDev->mQMIDev.mCdev.list )
   {
      // Get the inode
      pOpenInode = container_of( pInodeList, struct inode, i_devices );
      if (pOpenInode != NULL && (IS_ERR( pOpenInode ) == false))
      {
         // Look for this inode in each task

         // "group_leader" points to the main process' task, which resides in
         // the global "tasks" list.
         list_for_each( pTasks, &current->group_leader->tasks )
         {
            pEachTask = container_of( pTasks, struct task_struct, tasks );
            if (pEachTask == NULL || pEachTask->files == NULL)
            {
               // Some tasks may not have files (e.g. Xsession)
               continue;
            }
            // For each file this task has open, check if it's referencing
            // our inode.
            spin_lock_irqsave( &pEachTask->files->file_lock, flags );
            pFDT = files_fdtable( pEachTask->files );
            for (count = 0; count < pFDT->max_fds; count++)
            {
               pFilp = pFDT->fd[count];
               if (pFilp != NULL &&  pFilp->f_dentry != NULL )
               {
                  if (pFilp->f_dentry->d_inode == pOpenInode)
                  {
                     // Close this file handle
                     rcu_assign_pointer( pFDT->fd[count], NULL );                     
                     spin_unlock_irqrestore( &pEachTask->files->file_lock, flags );
                     
                     DBG( "forcing close of open file handle\n" );
                     filp_close( pFilp, pEachTask->files );

                     spin_lock_irqsave( &pEachTask->files->file_lock, flags );
                  }
               }
            }
            spin_unlock_irqrestore( &pEachTask->files->file_lock, flags );
         }
      }
   }

   // Remove device (so no more calls can be made by users)
   if (IS_ERR(pDev->mQMIDev.mpDevClass) == false)
   {
      device_destroy( pDev->mQMIDev.mpDevClass, 
                      pDev->mQMIDev.mDevNum );   
   }
   cdev_del( &pDev->mQMIDev.mCdev );
   
   unregister_chrdev_region( pDev->mQMIDev.mDevNum, 1 );

   return;
}

/*=========================================================================*/
// Driver level client management
/*=========================================================================*/

/*===========================================================================
METHOD:
   QMIReady (Public Method)

DESCRIPTION:
   Send QMI CTL GET VERSION INFO REQ
   Wait for response or timeout

PARAMETERS:
   pDev     [ I ] - Device specific memory
   timeout  [ I ] - Milliseconds to wait for response

RETURN VALUE:
   bool
===========================================================================*/
bool QMIReady(
   sQCUSBNet *    pDev,
   u16            timeout )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   struct semaphore readSem;
   u16 curTime;
   unsigned long flags;
   u8 transactionID;
   
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return -EFAULT;
   }

   writeBufferSize = QMICTLReadyReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   // An implimentation of down_timeout has not been agreed on,
   //    so it's been added and removed from the kernel several times.
   //    We're just going to ignore it and poll the semaphore.

   // Send a write every 100 ms and see if we get a response
   for (curTime = 0; curTime < timeout; curTime += 100)
   {
      // Start read
      sema_init( &readSem, 0 );
   
      transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
      if (transactionID == 0)
      {
         transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
      }
      result = ReadAsync( pDev, QMICTL, transactionID, UpSem, &readSem );
      if (result != 0)
      {
         return false;
      }

      // Fill buffer
      result = QMICTLReadyReq( pWriteBuffer, 
                               writeBufferSize,
                               transactionID );
      if (result < 0)
      {
         kfree( pWriteBuffer );
         return false;
      }

      // Disregard status.  On errors, just try again
      WriteSync( pDev,
                 pWriteBuffer,
                 writeBufferSize,
                 QMICTL );

      msleep( 100 );
      if (down_trylock( &readSem ) == 0)
      {
         // Enter critical section
         spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );

         // Pop the read data
         if (PopFromReadMemList( pDev,
                                 QMICTL,
                                 transactionID,
                                 &pReadBuffer,
                                 &readBufferSize ) == true)
         {
            // Success

            // End critical section
            spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
         
            // We don't care about the result
            kfree( pReadBuffer );

            break;
         }
      }
      else
      {
         // Enter critical section
         spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );
         
         // Timeout, remove the async read
         NotifyAndPopNotifyList( pDev, QMICTL, transactionID );
         
         // End critical section
         spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
      }
   }

   kfree( pWriteBuffer );

   // Did we time out?   
   if (curTime >= timeout)
   {
      return false;
   }
   
   DBG( "QMI Ready after %u milliseconds\n", curTime );
   
   // TODO: 3580 and newer firmware does not require this delay
   msleep( 5000 );

   // Success
   return true;
}

/*===========================================================================
METHOD:
   QMIWDSCallback (Public Method)

DESCRIPTION:
   QMI WDS callback function
   Update net stats or link state

PARAMETERS:
   pDev     [ I ] - Device specific memory
   clientID [ I ] - Client ID
   pData    [ I ] - Callback data (unused)

RETURN VALUE:
   None
===========================================================================*/
void QMIWDSCallback(
   sQCUSBNet *    pDev,
   u16            clientID,
   void *         pData )
{
   bool bRet;
   int result;
   void * pReadBuffer;
   u16 readBufferSize;

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,31 ))
   struct net_device_stats * pStats = &(pDev->mpNetDev->stats);
#else
   struct net_device_stats * pStats = &(pDev->mpNetDev->net->stats);
#endif

   u32 TXOk = (u32)-1;
   u32 RXOk = (u32)-1;
   u32 TXErr = (u32)-1;
   u32 RXErr = (u32)-1;
   u32 TXOfl = (u32)-1;
   u32 RXOfl = (u32)-1;
   u64 TXBytesOk = (u64)-1;
   u64 RXBytesOk = (u64)-1;
   bool bLinkState;
   bool bReconfigure;
   unsigned long flags;
   
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return;
   }

   // Critical section
   spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags );
   
   bRet = PopFromReadMemList( pDev,
                              clientID,
                              0,
                              &pReadBuffer,
                              &readBufferSize );
   
   // End critical section
   spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags ); 
   
   if (bRet == false)
   {
      DBG( "WDS callback failed to get data\n" );
      return;
   }
   
   // Default values
   bLinkState = ! QTestDownReason( pDev, NO_NDIS_CONNECTION );
   bReconfigure = false;

   result = QMIWDSEventResp( pReadBuffer,
                             readBufferSize,
                             &TXOk,
                             &RXOk,
                             &TXErr,
                             &RXErr,
                             &TXOfl,
                             &RXOfl,
                             &TXBytesOk,
                             &RXBytesOk,
                             &bLinkState,
                             &bReconfigure );
   if (result < 0)
   {
      DBG( "bad WDS packet\n" );
   }
   else
   {

      // Fill in new values, ignore max values
      if (TXOfl != (u32)-1)
      {
         pStats->tx_fifo_errors = TXOfl;
      }
      
      if (RXOfl != (u32)-1)
      {
         pStats->rx_fifo_errors = RXOfl;
      }

      if (TXErr != (u32)-1)
      {
         pStats->tx_errors = TXErr;
      }
      
      if (RXErr != (u32)-1)
      {
         pStats->rx_errors = RXErr;
      }

      if (TXOk != (u32)-1)
      {
         pStats->tx_packets = TXOk + pStats->tx_errors;
      }
      
      if (RXOk != (u32)-1)
      {
         pStats->rx_packets = RXOk + pStats->rx_errors;
      }

      if (TXBytesOk != (u64)-1)
      {
         pStats->tx_bytes = TXBytesOk;
      }
      
      if (RXBytesOk != (u64)-1)
      {
         pStats->rx_bytes = RXBytesOk;
      }

      if (bReconfigure == true)
      {
         DBG( "Net device link reset\n" );
         QSetDownReason( pDev, NO_NDIS_CONNECTION );
         QClearDownReason( pDev, NO_NDIS_CONNECTION );
      }
      else 
      {
         if (bLinkState == true)
         {
            DBG( "Net device link is connected\n" );
            QClearDownReason( pDev, NO_NDIS_CONNECTION );
         }
         else
         {
            DBG( "Net device link is disconnected\n" );
            QSetDownReason( pDev, NO_NDIS_CONNECTION );
         }
      }
   }

   kfree( pReadBuffer );

   // Setup next read
   result = ReadAsync( pDev,
                       clientID,
                       0,
                       QMIWDSCallback,
                       pData );
   if (result != 0)
   {
      DBG( "unable to setup next async read\n" );
   }

   return;
}

/*===========================================================================
METHOD:
   SetupQMIWDSCallback (Public Method)

DESCRIPTION:
   Request client and fire off reqests and start async read for 
   QMI WDS callback

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int SetupQMIWDSCallback( sQCUSBNet * pDev )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   u16 WDSClientID;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return -EFAULT;
   }
   
   result = GetClientID( pDev, QMIWDS );
   if (result < 0)
   {
      return result;
   }
   WDSClientID = result;

   // QMI WDS Set Event Report
   writeBufferSize = QMIWDSSetEventReportReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }
   
   result = QMIWDSSetEventReportReq( pWriteBuffer, 
                                     writeBufferSize,
                                     1 );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   result = WriteSync( pDev,
                       pWriteBuffer,
                       writeBufferSize,
                       WDSClientID );
   kfree( pWriteBuffer );

   if (result < 0)
   {
      return result;
   }

   // QMI WDS Get PKG SRVC Status
   writeBufferSize = QMIWDSGetPKGSRVCStatusReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIWDSGetPKGSRVCStatusReq( pWriteBuffer, 
                                       writeBufferSize,
                                       2 );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }
   
   result = WriteSync( pDev,
                       pWriteBuffer,
                       writeBufferSize,
                       WDSClientID );
   kfree( pWriteBuffer );

   if (result < 0)
   {
      return result;
   }

   // Setup asnyc read callback
   result = ReadAsync( pDev,
                       WDSClientID,
                       0,
                       QMIWDSCallback,
                       NULL );
   if (result != 0)
   {
      DBG( "unable to setup async read\n" );
      return result;
   }

   // Send SetControlLineState request (USB_CDC)
   //   Required for Autoconnect
   result = usb_control_msg( pDev->mpNetDev->udev,
                             usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
                             0x22,
                             0x21,
                             1, // DTR present
                             0,
                             NULL,
                             0,
                             100 );
   if (result < 0)
   {
      DBG( "Bad SetControlLineState status %d\n", result );
      return result;
   }

   return 0;
}

/*===========================================================================
METHOD:
   QMIDMSGetMEID (Public Method)

DESCRIPTION:
   Register DMS client
   send MEID req and parse response
   Release DMS client

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
int QMIDMSGetMEID( sQCUSBNet * pDev )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   u16 DMSClientID;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return -EFAULT;
   }

   result = GetClientID( pDev, QMIDMS );
   if (result < 0)
   {
      return result;
   }
   DMSClientID = result;

   // QMI DMS Get Serial numbers Req
   writeBufferSize = QMIDMSGetMEIDReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIDMSGetMEIDReq( pWriteBuffer, 
                              writeBufferSize,
                              1 );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   result = WriteSync( pDev,
                       pWriteBuffer,
                       writeBufferSize,
                       DMSClientID );
   kfree( pWriteBuffer );

   if (result < 0)
   {
      return result;
   }

   // QMI DMS Get Serial numbers Resp
   result = ReadSync( pDev,
                      &pReadBuffer,
                      DMSClientID,
                      1 );
   if (result < 0)
   {
      return result;
   }
   readBufferSize = result;

   result = QMIDMSGetMEIDResp( pReadBuffer,
                               readBufferSize,
                               &pDev->mMEID[0],
                               14 );
   kfree( pReadBuffer );

   if (result < 0)
   {
      DBG( "bad get MEID resp\n" );
      
      // Non fatal error, device did not return any MEID
      //    Fill with 0's
      memset( &pDev->mMEID[0], '0', 14 );
   }

   ReleaseClientID( pDev, DMSClientID );

   // Success
   return 0;
}
