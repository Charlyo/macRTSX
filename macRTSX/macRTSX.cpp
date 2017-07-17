//                       ____ _____ ______  __
//  _ __ ___   __ _  ___|  _ \_   _/ ___\ \/ /
// | '_ ` _ \ / _` |/ __| |_) || | \___ \\  /
// | | | | | | (_| | (__|  _ < | |  ___) /  \
// |_| |_| |_|\__,_|\___|_| \_\|_| |____/_/\_\

#include "macRTSX.hpp"

#include "rtsxreg.hpp"
#include "rtsxvar.hpp"
#include "rtsx_ops.hpp"

#include "macSDMMC.hpp"
#include "macSDMMCDisk.hpp"

/* - * - * - */

OSDefineMetaClassAndStructors(macRTSX, IOService);

/** Private variables **/
#define kIOPMPowerOff 0
#define POWER_STATE_OFF 0
#define POWER_STATE_ON 1

static IOPMPowerState sPowerStates[] = {

    // version, capabilityFlags, outputPowerCharacter, inputPowerRequirement, staticPower, unbudgetedPower, powerToAttain, timeToAttain, settleUpTime, timeToLower, settleDownTime, powerDomainBudget.

    // Device off;
    {kIOPMPowerStateVersion1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

    // Sleep mode;
    {kIOPMPowerStateVersion1, 0, kIOPMInitialDeviceState, kIOPMInitialDeviceState, 0, 0, 0, 0, 0, 0, 0, 0},

    // Device paused;
    {kIOPMPowerStateVersion1, kIOPMConfigRetained, kIOPMConfigRetained, kIOPMConfigRetained, 0, 0, 0, 0, 0, 0, 0, 0},

    // Device active;
    {kIOPMPowerStateVersion1, kIOPMPowerOn /*| kIOPMUsable ???? */, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0},

};

/*
 * Section for portability to macOS.
 */

#define SET(t, f) ((t) |= (f))
#define ISSET(t, f) ((t) & (f))
#define CLR(t, f) ((t) &= ~(f))

//===============================================

bool macRTSX::start(IOService *provider)
{
    uint16_t device_id;
    int flags;

    if (!IOService::start(provider))
    {
        return false;
    }

    assert(OSDynamicCast(IOPCIDevice, provider));
    provider_ = OSDynamicCast(IOPCIDevice, provider);
    workloop_ = getWorkLoop();

    PMinit();
    provider_->joinPMtree(this);
    registerPowerDriver(this, sPowerStates, sizeof(sPowerStates) / sizeof(IOPMPowerState));

    if ((provider_->extendedConfigRead16(RTSX_CFG_PCI) & RTSX_CFG_ASIC) != 0)
    {
        DEVERR("no asic\n");
        goto bad;
    }

    static const unsigned int RTSX_PCI_BAR = 0x10;
    memorymap_ = provider_->mapDeviceMemoryWithRegister(RTSX_PCI_BAR);
    if (!memorymap_)
    {
        DEVERR("couldn't map registers\n");
        goto bad;
    }
    memorydescriptor_ = memorymap_->getMemoryDescriptor();

    interrupt_source_ = IOInterruptEventSource::interruptEventSource(this, ::rtsx_trampoline_intr, provider_);
    if (!interrupt_source_)
    {
        DEVERR("couldn't map interrupt\n");
        goto bad;
    }
    workloop_->addEventSource(interrupt_source_);
    interrupt_source_->enable();

	/* Enable the device */
    provider_->setBusMasterEnable(true);

	/* Power up the device */
    if (this->requestPowerDomainState(kIOPMPowerOn,
                                      (IOPowerConnection *)this->getParentEntry(gIOPowerPlane),
                                      IOPMLowestState) != IOPMNoErr)
    {
        DEVERR("domain D0 not received\n");
        goto bad;
    }

    device_id = provider_->extendedConfigRead16(kIOPCIConfigDeviceID);
    switch (device_id)
    {
    case PCI_PRODUCT_REALTEK_RTS5209:
        flags = RTSX_F_5209;
        break;
    case PCI_PRODUCT_REALTEK_RTS5227:
        flags = RTSX_F_5227;
        break;
    case PCI_PRODUCT_REALTEK_RTS5229:
    case PCI_PRODUCT_REALTEK_RTS5249:
        flags = RTSX_F_5229;
        break;
    case PCI_PRODUCT_REALTEK_RTL8402:
        flags = RTSX_F_8402;
        break;
    case PCI_PRODUCT_REALTEK_RTL8411:
        flags = RTSX_F_8411;
        break;
    case PCI_PRODUCT_REALTEK_RTL8411B:
        flags = RTSX_F_8411B;
        break;
    default:
        flags = 0;
        break;
    }

    sc_.sdmmc = new macSDMMC();
    sc_.sdmmc->set_owner(this);

    if (rtsx_attach(&sc_, memorydescriptor_, flags) != 0)
    {
        DEVERR("can't initialize chip\n");
        goto bad;
    }

good:

    registerService();
    this->registered_ = true;

    return true;
bad:
    return false;
}

void macRTSX::stop(IOService *provider)
{
    rtsx_detach(&sc_);

    delete sc_.sdmmc;

    workloop_->removeEventSource(interrupt_source_);
    interrupt_source_->release();

    PMstop();
    
    IOService::stop(provider);
}

// forward declare for setPowerState.
int rtsx_activate(struct rtsx_softc *sc, bool sleeping);

IOReturn macRTSX::setPowerState(unsigned long which_state, IOService *which_service)
{
    DEVVERBOSE("macRTSX: in setPowerState(%lu, *);\n", which_state);

    switch (which_state)
    {
    case 0:
        // Going to sleep.
        printf("sleeping\n");

        if (registered_)
            rtsx_activate(&sc_, true);

        break;

    case 3:
        // Waking up.
        printf("waking up\n");

        provider_->setBusMasterEnable(true);

        if (registered_)
            rtsx_activate(&sc_, false);

        break;

    default:
        printf("Unknown state (%d).\n", which_state);
        break;
    }

    return kIOPMAckImplied;
}

IOReturn macRTSX::powerStateWillChangeTo(
    IOPMPowerFlags capabilities,
    unsigned long stateNumber,
    IOService *whatDevice)
{
    DEVVERBOSE("powerStateWillChangeTo  %d\n", stateNumber);

    // if (registered_)
    //     rtsx_activate(&sc_, true);
    
    return kIOPMAckImplied;
}

IOReturn macRTSX::powerStateDidChangeTo(
    IOPMPowerFlags capabilities,
    unsigned long stateNumber,
    IOService *whatDevice)
{
    DEVVERBOSE("powerStateDidChangeTo  %d\n", stateNumber);
    
    // if (registered_)
    //     rtsx_activate(&sc_, false);

    return kIOPMAckImplied;
}

void rtsx_trampoline_intr(OSObject *ih, IOInterruptEventSource *, int)
{
    // go to isr  handler
    macRTSX *that = OSDynamicCast(macRTSX, ih);
    // isr expects an 'sc' object and not this.
    rtsx_intr(&that->sc_);
}
