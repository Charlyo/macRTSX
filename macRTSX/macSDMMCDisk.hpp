//  ____  ____  __  __ __  __  ____   ____ ___ ____  _  __
// / ___||  _ \|  \/  |  \/  |/ ___| |  _ \_ _/ ___|| |/ /
// \___ \| | | | |\/| | |\/| | |     | | | | |\___ \| ' /
//  ___) | |_| | |  | | |  | | |___  | |_| | | ___) | . \
// |____/|____/|_|  |_|_|  |_|\____| |____/___|____/|_|\_\

#pragma once

#include <IOKit/storage/IOBlockStorageDevice.h>
#include <IOKit/IOLocks.h>

#include "macSDMMC.hpp"

class macSDMMCDisk : public IOBlockStorageDevice
{
    OSDeclareDefaultStructors(macSDMMCDisk)

  private:
    macSDMMC * sdmmc_;
    IOLock *util_lock_;
    uint32_t num_blocks_;
    uint32_t blk_size_;
    struct sdmmc_task IO_task_;

    static void IO_task(void *arg)
    {
        printf("IO TASK\n");
    }

    struct IO_TASK_PAYLOAD
    {
        macSDMMCDisk * handle;
    };

  public:

    void setup(macSDMMC * sdmmc)
    {
        this->sdmmc_ = sdmmc;

        util_lock_ = IOLockAlloc();
        num_blocks_ = sdmmc_->sc_fn0->csd.capacity;
        blk_size_ = sdmmc_->sc_fn0->csd.sector_size;

        sdmmc_init_task(&this->IO_task_, &IO_task, this);
    }
    
    /**
	 * Subclass requirements.
	 */

    virtual IOReturn doEjectMedia(void) override;
    virtual IOReturn doFormatMedia(UInt64 byteCapacity) override;
    virtual UInt32 doGetFormatCapacities(UInt64 *capacities, UInt32 capacitiesMaxCount) const override;
    virtual IOReturn doLockUnlockMedia(bool doLock) override;
    virtual IOReturn doSynchronizeCache(void) override;
    virtual char *getVendorString(void) override;
    virtual char *getProductString(void) override;
    virtual char *getRevisionString(void) override;
    virtual char *getAdditionalDeviceInfoString(void) override;
    virtual IOReturn reportBlockSize(UInt64 *blockSize) override;
    virtual IOReturn reportEjectability(bool *isEjectable) override;
    virtual IOReturn reportLockability(bool *isLockable) override;
    virtual IOReturn reportMaxValidBlock(UInt64 *maxBlock) override;
    virtual IOReturn reportMediaState(bool *mediaPresent, bool *changedState) override;
    virtual IOReturn reportPollRequirements(bool *pollRequired, bool *pollIsExpensive) override;
    virtual IOReturn reportRemovability(bool *isRemovable) override;
    virtual IOReturn reportWriteProtection(bool *isWriteProtected) override;
    virtual IOReturn getWriteCacheState(bool *enabled) override;
    virtual IOReturn setWriteCacheState(bool enabled) override;
    virtual IOReturn doAsyncReadWrite(IOMemoryDescriptor *buffer, UInt64 block, UInt64 nblks, IOStorageAttributes *attributes, IOStorageCompletion *completion) override;    
};
