//  ____  ____  __  __ __  __  ____   ____ ___ ____  _  __
// / ___||  _ \|  \/  |  \/  |/ ___| |  _ \_ _/ ___|| |/ /
// \___ \| | | | |\/| | |\/| | |     | | | | |\___ \| ' /
//  ___) | |_| | |  | | |  | | |___  | |_| | | ___) | . \
// |____/|____/|_|  |_|_|  |_|\____| |____/___|____/|_|\_\

#include "macSDMMCDisk.hpp"

#include <IOKit/IOLib.h>
#include <IOKit/storage/IOBlockStorageDevice.h>

#include "macRTSX.hpp"
#include "macSDMMC.hpp"

/* - * - * - */

OSDefineMetaClassAndStructors(macSDMMCDisk, IOBlockStorageDevice);

/** Private variables **/

//===============================================

IOReturn macSDMMCDisk::doEjectMedia(void)
{
    return kIOReturnSuccess;
}

IOReturn macSDMMCDisk::doFormatMedia(UInt64 byteCapacity)
{
    return kIOReturnSuccess;
}

IOReturn macSDMMCDisk::doLockUnlockMedia(bool doLock)
{
    return kIOReturnUnsupported;
}

IOReturn macSDMMCDisk::doSynchronizeCache(void)
{
    return kIOReturnSuccess;
}

char *macSDMMCDisk::getVendorString(void)
{
    return NULL;
    return (char *)"macRTSX";
}

char *macSDMMCDisk::getProductString(void)
{
    return (char *)"SD Card";
}

char *macSDMMCDisk::getRevisionString(void)
{
    static char *revision = __DATE__ ":" __TIME__;

    return (char *)revision;
}

char *macSDMMCDisk::getAdditionalDeviceInfoString(void)
{
    return nullptr;
}

IOReturn macSDMMCDisk::reportBlockSize(UInt64 *blockSize)
{
    *blockSize = sdmmc_->sc_fn0->csd.sector_size;
    return kIOReturnSuccess;
}

IOReturn macSDMMCDisk::reportEjectability(bool *isEjectable)
{
    *isEjectable = true;
    return kIOReturnSuccess;
}

IOReturn macSDMMCDisk::reportLockability(bool *isLockable)
{
    *isLockable = false;
    return kIOReturnSuccess;
}

IOReturn macSDMMCDisk::reportMaxValidBlock(UInt64 *maxBlock)
{
    // blocks are zero indexed!
    *maxBlock = sdmmc_->sc_fn0->csd.capacity - 1;
    return kIOReturnSuccess;
}

IOReturn macSDMMCDisk::reportMediaState(bool *mediaPresent, bool *changedState)
{
    *mediaPresent = true;
    *changedState = false;

    return kIOReturnSuccess;
}

IOReturn macSDMMCDisk::reportPollRequirements(bool *pollRequired, bool *pollIsExpensive)
{
    *pollRequired = false;
    *pollIsExpensive = false;
    return kIOReturnSuccess;
}

IOReturn macSDMMCDisk::reportRemovability(bool *isRemovable)
{
    *isRemovable = true;
    return kIOReturnSuccess;
}

IOReturn macSDMMCDisk::reportWriteProtection(bool *isWriteProtected)
{
    *isWriteProtected = false;
    return kIOReturnSuccess;
}

IOReturn macSDMMCDisk::getWriteCacheState(bool *enabled)
{
    return kIOReturnUnsupported;
}

IOReturn macSDMMCDisk::setWriteCacheState(bool enabled)
{
    return kIOReturnUnsupported;
}

/**
 * Start an async read or write operation.
 * @param buffer
 * An IOMemoryDescriptor describing the data-transfer buffer. The data direction
 * is contained in the IOMemoryDescriptor. Responsibility for releasing the descriptor
 * rests with the caller.
 * @param block
 * The starting block number of the data transfer.
 * @param nblks
 * The integral number of blocks to be transferred.
 * @param attributes
 * Attributes of the data transfer. See IOStorageAttributes.
 * @param completion
 * The completion routine to call once the data transfer is complete.
 */
IOReturn macSDMMCDisk::doAsyncReadWrite(IOMemoryDescriptor *buffer,
                                        UInt64 block,
                                        UInt64 nblks,
                                        IOStorageAttributes *attributes,
                                        IOStorageCompletion *completion)
{
    // We'll just block here.

    int error = 0;
    IODirection direction;
    u_char *tmp_buffer = (u_char *)IOMalloc(nblks * blk_size_);

    direction = buffer->getDirection();
       if ((direction != kIODirectionIn) && (direction != kIODirectionOut))
           return kIOReturnBadArgument;

    if ((block + nblks) > num_blocks_)
        return kIOReturnBadArgument;

       if ((direction != kIODirectionIn) && (direction != kIODirectionOut))
           return kIOReturnBadArgument;

    // Careful here, rtsx can only process 128k at a time..
    uint64_t len = nblks * blk_size_;

    // XXX this could use some work. see the assert below.
    // assert(0);
    // we assume block size of 512.

    if (direction == kIODirectionIn)
    {

        uint64_t super_blocks = len / (256 * 512);
        uint64_t remainder = len - super_blocks * (256 * 512);

        for (int i = 0; i < super_blocks; ++i)
        {
            error = sdmmc_->mem_read_block(sdmmc_->sc_fn0, block + i * 256, tmp_buffer + (i * 256 * 512), 256 * 512);
            if (error)
            {
                goto bad;
            }
        }
        // complete with a normal transfer!
        if (remainder)
            error = sdmmc_->mem_read_block(sdmmc_->sc_fn0, block + super_blocks * 256, tmp_buffer + (super_blocks * 256 * 512), remainder);

        buffer->writeBytes(0, tmp_buffer, len);
    }

    if (direction == kIODirectionOut)
    {

        buffer->readBytes(0, tmp_buffer, len);

        uint64_t super_blocks = len / (256 * 512);
        uint64_t remainder = len - super_blocks * (256 * 512);

        for (int i = 0; i < super_blocks; ++i)
        {
            error = sdmmc_->mem_write_block(sdmmc_->sc_fn0, block + i * 256, tmp_buffer + (i * 256 * 512), 256 * 512);
            if (error)
            {
                goto bad;
            }
        }
        // complete with a normal transfer!
        if (remainder)
            error = sdmmc_->mem_write_block(sdmmc_->sc_fn0, block + super_blocks * 256, tmp_buffer + (super_blocks * 256 * 512), remainder);
    }

good:
    IOFree(tmp_buffer, len);

    completion->action(completion->target, completion->parameter, kIOReturnSuccess, len);

    return kIOReturnSuccess;

bad:
    printf("Error during IO!!!\n");
    IOFree(tmp_buffer, len);

    completion->action(completion->target, completion->parameter, kIOReturnIOError, 0);

    return kIOReturnIOError;
}
