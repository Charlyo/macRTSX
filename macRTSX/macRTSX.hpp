//                       ____ _____ ______  __
//  _ __ ___   __ _  ___|  _ \_   _/ ___\ \/ /
// | '_ ` _ \ / _` |/ __| |_) || | \___ \\  /
// | | | | | | (_| | (__|  _ < | |  ___) /  \
// |_| |_| |_|\__,_|\___|_| \_\|_| |____/_/\_\

#pragma once

#include <IOKit/system.h>

#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOInterruptEventSource.h>

#include "rtsxvar.hpp"
/* - * - * - */

void rtsx_trampoline_intr(OSObject *ih, IOInterruptEventSource *, int);

class macSDMMCDisk;
class macRTSX : public IOService
{
    OSDeclareDefaultStructors(macRTSX);

public:
    virtual bool start(IOService *) override;
    virtual void stop(IOService *) override;

    virtual IOReturn setPowerState(unsigned long, IOService*) override;
    virtual IOReturn powerStateWillChangeTo(
                                            IOPMPowerFlags  capabilities,
                                            unsigned long   stateNumber,
                                            IOService *     whatDevice ) override;
    virtual IOReturn powerStateDidChangeTo(
                                           IOPMPowerFlags  capabilities,
                                           unsigned long   stateNumber,
                                           IOService *     whatDevice ) override;

    
    void DEVERR(const char * msg)
    {
        IOLog("macRTSX: (FATAL) %s", msg);
    }

    template<typename arg0_t>
    void DEVERR(const char * msg, arg0_t arg0)
    {
        IOLog("macRTSX: (FATAL) ");
        IOLog(msg, arg0);
    }

    void DEVVERBOSE(const char * msg)
    {
        IOLog("macRTSX: %s", msg);
    }

    template<typename arg0_t>
    void DEVVERBOSE(const char * msg, arg0_t arg0)
    {
        IOLog("macRTSX: ");
        IOLog(msg, arg0);
    }

    // make the intr a friend since it makes use of 'sc'.
    friend void ::rtsx_trampoline_intr(OSObject *ih, IOInterruptEventSource *, int);

  protected:
    // members
    IOPCIDevice * provider_;
    IOWorkLoop * workloop_;
    IOMemoryMap * memorymap_;
    IOMemoryDescriptor * memorydescriptor_;
    IOInterruptEventSource * interrupt_source_;

    rtsx_softc sc_ = {0,};

    bool registered_ = false;
};

#define PCI_PRODUCT_REALTEK_RTS5208     0x5208          /* RTS5209 PCI-E Card Reader */
#define PCI_PRODUCT_REALTEK_RTS5209     0x5209          /* RTS5209 PCI-E Card Reader */
#define PCI_PRODUCT_REALTEK_RTS5227     0x5227          /* RTS5227 PCI-E Card Reader */
#define PCI_PRODUCT_REALTEK_RTS5229     0x5229          /* RTS5229 PCI-E Card Reader */
#define PCI_PRODUCT_REALTEK_RTS5249     0x5249          /* RTS5249 PCI-E Card Reader */
#define PCI_PRODUCT_REALTEK_RTL8402     0x5286          /* RTL8402 PCI-E Card Reader */
#define PCI_PRODUCT_REALTEK_RTL8411B    0x5287          /* RTL8411B PCI-E Card Reader */
#define PCI_PRODUCT_REALTEK_RTL8411     0x5289          /* RTL8411 PCI-E Card Reader */
