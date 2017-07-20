//                       ____ _____ ______  __
//  _ __ ___   __ _  ___|  _ \_   _/ ___\ \/ /
// | '_ ` _ \ / _` |/ __| |_) || | \___ \\  /
// | | | | | | (_| | (__|  _ < | |  ___) /  \
// |_| |_| |_|\__,_|\___|_| \_\|_| |____/_/\_\

#include "macRTSX.hpp"
#include "debug.hpp"
#include "sdmmcreg.h"
#define TRACE() logger_.development("%s:%d->%s\n", __FILE__, __LINE__, __FUNCTION__)

/* - * - * - */

OSDefineMetaClassAndStructors(macRTSX, IOService);

bool macRTSX::start(IOService* provider)
{
	if (!IOService::start(provider))
	{
		return false;
	}
    
    assert(OSDynamicCast(IOPCIDevice, provider));
	provider_ = static_cast<IOPCIDevice*>(provider);
	workloop_ = getWorkLoop();
	
	TRACE();
	
	hardware_bringup_ = IOCommandGate::commandGate(this, (IOCommandGate::Action)hardware_bringup_static);
	hardware_bringdown_ = IOCommandGate::commandGate(this, (IOCommandGate::Action)hardware_bringdown_static);
	detect_work_ = IOTimerEventSource::timerEventSource(this, (IOTimerEventSource::Action)detect_work_static);
	wait_for_interrupt_ = IOCommandGate::commandGate(this, (IOCommandGate::Action)wait_for_interrupt_static);
	
	workloop_->addEventSource(hardware_bringup_);
	workloop_->addEventSource(hardware_bringdown_);
	workloop_->addEventSource(wait_for_interrupt_);
	hardware_bringup_->enable();
	hardware_bringdown_->enable();
	wait_for_interrupt_->enable();
	
	workloop_->addEventSource(detect_work_);
	detect_work_->enable();
	
	const int RTSX_PCI_BAR = kIOPCIConfigBaseAddress0;
	memory_map_ = provider_->mapDeviceMemoryWithRegister(RTSX_PCI_BAR);
	if (!memory_map_)
	{
		logger_.error("couldn't map registers\n");
		return false;
	}
	memory_descriptor_ = memory_map_->getMemoryDescriptor();

	interrupt_source_ = IOInterruptEventSource::interruptEventSource(this, (IOInterruptEventSource::Action)interrupt_occurred_static, provider_);
	if (!interrupt_source_)
	{
		logger_.error("couldn't map interrupt\n");
		return false;
	}
	workloop_->addEventSource(interrupt_source_);
	interrupt_source_->enable();
	
	PMinit();
	provider_->joinPMtree(this);
	
	hardware_bringup_->runCommand();
	registerService();

	return true;
}

void macRTSX::stop(IOService* provider)
{
	TRACE();
	
	detect_work_->cancelTimeout();
	hardware_bringdown_->runCommand();
	
	PMstop();
	
	interrupt_source_->disable();
	workloop_->removeEventSource(interrupt_source_);
	interrupt_source_->release();

	memory_map_->release();

	detect_work_->disable();
	workloop_->removeEventSource(detect_work_);
	detect_work_->release();
	
	wait_for_interrupt_->disable();
	hardware_bringup_->disable();
	hardware_bringdown_->disable();
	workloop_->removeEventSource(wait_for_interrupt_);
	workloop_->removeEventSource(hardware_bringup_);
	workloop_->removeEventSource(hardware_bringdown_);
	hardware_bringup_->release();
	hardware_bringdown_->release();
	
	IOService::stop(provider);
}

/*
 * Return condition is indicated by the parameter.
 * Unlike the other command gates, we stay inside the static
 * method and perform the work there.
 *
 * We serve wait_for_interrupt and should not be called directly.
 */
IOReturn macRTSX::wait_for_interrupt_static(macRTSX *self, bool* success, int* ms_timeout)
{
	int reason;
	*success = false;

	/*
	 * Now we should be inside a gated context.
	 */
	if (!self->workloop_->inGate()) panic("GATED CONTEXT FAULT");

	/*
	 * Build the deadline, converting from milliseconds.
	 */
	AbsoluteTime deadline;
	clock_interval_to_deadline(*ms_timeout, kMillisecondScale, reinterpret_cast<uint64_t*> (&deadline));
	
	reason =
	self->wait_for_interrupt_->commandSleep(&self->intr_status_, deadline, THREAD_UNINT);
	
	*success = (reason == THREAD_AWAKENED);
	
	return kIOReturnSuccess;
}

IOReturn macRTSX::hardware_bringup_static(macRTSX *self)
{
	self->hardware_bringup();
	return kIOReturnSuccess;
}

IOReturn macRTSX::hardware_bringdown_static(macRTSX *self)
{
	self->hardware_bringdown();
	return kIOReturnSuccess;
}

void macRTSX::hardware_bringup()
{
	/*
	 * Now we should be inside a gated context.
	 */
	TRACE();
	
	if (!workloop_->inGate()) panic("GATED CONTEXT FAULT");
	
	/* Power up the device. */
	if (this->requestPowerDomainState(kIOPMPowerOn,
									  (IOPowerConnection *)this->getParentEntry(gIOPowerPlane),
									  IOPMLowestState) != IOPMNoErr)
	{
		logger_.error("domain d0 not received\n");
		return;
	}
	
	/* Enable the device. */
	provider_->setBusMasterEnable(true);
	
	/* Get the various probe bits. */
	device_id_ = provider_->extendedConfigRead16(kIOPCIConfigDeviceID);
	vendor_id_ = provider_->extendedConfigRead16(kIOPCIConfigVendorID);
	revision_id_ = provider_->extendedConfigRead16(kIOPCIConfigRevisionID);
	
	logger_.normal("Found device. [%04x:%04x] rev %04x\n", vendor_id_, device_id_, revision_id_);
	
	/* Read IC version from dummy register. */
	if (device_id_ == 0x5229)
	{
		uint8_t version;
		rtsx_read(RTSX_DUMMY_REG, &version);
	
		logger_.debug("IC version is %d\n", version);
		is5229_type_C_ = (version  == RTSX_IC_VERSION_C);
	}
	
	uint32_t status;
	
	/* Enable interrupt write-clear (default is read-clear). */
	RTSX_CLR(RTSX_NFTS_TX_CTRL, RTSX_INT_READ_CLR);
	
	/* Clear any pending interrupts. */
	status = READ4(RTSX_BIPR);
	WRITE4(RTSX_BIPR, status);
	
	/* Check for cards already inserted at attach time. */
	card_exists_ = status & RTSX_SD_EXIST;
	
	/* Enable interrupts. */
	WRITE4(RTSX_BIER,
	    RTSX_TRANS_OK_INT_EN | RTSX_TRANS_FAIL_INT_EN | RTSX_SD_INT_EN);

	/* Power on SSC clock. */
	RTSX_CLR(RTSX_FPDCTL, RTSX_SSC_POWER_DOWN);
	IODelay(200);
	
	/* XXX magic numbers from linux driver */
	if (device_id_ == 0x5209)
	{
		rtsx_write_phy(0x00, 0xB966);
	} else {
		rtsx_write_phy(0x00, 0xBA42);
	}
	
	RTSX_SET(RTSX_CLK_DIV, 0x07);

	/* Disable sleep mode. */
	RTSX_CLR(RTSX_HOST_SLEEP_STATE,
			 RTSX_HOST_ENTER_S1 | RTSX_HOST_ENTER_S3);
	
	/* Disable card clock. */
	RTSX_CLR(RTSX_CARD_CLK_EN, RTSX_CARD_CLK_EN_ALL);
	
	RTSX_CLR(RTSX_CHANGE_LINK_STATE,
			 RTSX_FORCE_RST_CORE_EN | RTSX_NON_STICKY_RST_N_DBG | 0x04);
	RTSX_WRITE(RTSX_SD30_DRIVE_SEL, RTSX_SD30_DRIVE_SEL_3V3);
	
	/* Enable SSC clock. */
	RTSX_WRITE(RTSX_SSC_CTL1, RTSX_SSC_8X_EN | RTSX_SSC_SEL_4M);
	RTSX_WRITE(RTSX_SSC_CTL2, 0x12);
	
	RTSX_SET(RTSX_CHANGE_LINK_STATE, RTSX_MAC_PHY_RST_N_DBG);
	RTSX_SET(RTSX_IRQSTAT0, RTSX_LINK_READY_INT);
	
	RTSX_WRITE(RTSX_PERST_GLITCH_WIDTH, 0x80);
	
	/* Set RC oscillator to 400K. */
	RTSX_CLR(RTSX_RCCTL, RTSX_RCCTL_F_2M);
	
	/* Request clock by driving CLKREQ pin to zero. */
	RTSX_SET(RTSX_PETXCFG, RTSX_PETXCFG_CLKREQ_PIN);
	
	/* Set up LED GPIO. */
	if (device_id_ == 0x5209) {
		RTSX_WRITE(RTSX_CARD_GPIO, 0x03);
		RTSX_WRITE(RTSX_CARD_GPIO_DIR, 0x03);
	} else {
		RTSX_SET(RTSX_GPIO_CTL, RTSX_GPIO_LED_ON);
		/* Switch LDO3318 source from DV33 to 3V3. */
		RTSX_CLR(RTSX_LDO_PWR_SEL, RTSX_LDO_PWR_SEL_DV33);
		RTSX_SET(RTSX_LDO_PWR_SEL, RTSX_LDO_PWR_SEL_3V3);
		/* Set default OLT blink period. */
		RTSX_SET(RTSX_OLT_LED_CTL, RTSX_OLT_LED_PERIOD);
	}
	
	TRACE();
	/** XXX **/
	
	uint32_t sdio_cfg;
	if (rtsx_read_cfg(0, RTSX_SDIOCFG_REG, &sdio_cfg))
	{
		if ((sdio_cfg & RTSX_SDIOCFG_SDIO_ONLY) ||
			(sdio_cfg & RTSX_SDIOCFG_HAVE_SDIO))
			supports_sdio_ = true;
	}
	
	/* Now handle cards discovered during attachment. */
	detect_work_->setTimeoutMS(200);
}

void macRTSX::hardware_bringdown()
{
	/*
	 * Now we should be inside a gated context.
	 */
	TRACE();
	
	if (!workloop_->inGate()) panic("GATED CONTEXT FAULT");
	
	/* Power down the device. */
	if (this->requestPowerDomainState(0,
									  (IOPowerConnection *)this->getParentEntry(gIOPowerPlane),
									  IOPMLowestState) != IOPMNoErr)
	{
		logger_.error("domain d3 not received\n");
		return;
	}
}

IOReturn macRTSX::detect_work_static(macRTSX *self, IOTimerEventSource *sender)
{
	self->detect_work();
	return kIOReturnSuccess;
}

void macRTSX::detect_work()
{
	TRACE();
	
	if (card_exists_ && card_inuse_) return;
	if (!card_exists_ && !card_inuse_) return;
	
	if (card_exists_ && !card_inuse_)
	{
		/* Insert event. */
		logger_.debug("card inserted.\n");
		card_inuse_ = true;
		
		card_attached_ = false;
		
		/*
		 * Power up the card (or card stack).
		 */
		if (!sdmmc_enable())
		{
			logger_.error("can't enable card\n");
			goto err;
		}
		
		// XXX

		card_attached_ = true;
	}
	
	if (!card_exists_ && card_inuse_)
	{
		/* Eject event. */
		logger_.debug("card ejected.\n");
		
		sdmmc_disable();

		// XXX

		card_inuse_ = false;
		card_attached_ = false;
	}
	
err:
done:
	;
}

void macRTSX::interrupt_occurred_static(macRTSX *self, IOInterruptEventSource *source, int)
{
	self->interrupt_occurred();
}

void macRTSX::interrupt_occurred()
{
	u_int32_t enabled, status;
	
	enabled = READ4(RTSX_BIER);
	status = READ4(RTSX_BIPR);
	
	/* Ack interrupts. */
	WRITE4(RTSX_BIPR, status);
	
	if (((enabled & status) == 0) || status == 0xffffffff)
		return;
	
	TRACE();
	
	if (status & RTSX_SD_INT)
	{
		card_exists_ = status & RTSX_SD_EXIST;
		detect_work_->setTimeoutMS(200);
	}
	
	if (status & (RTSX_TRANS_OK_INT | RTSX_TRANS_FAIL_INT)) {
		intr_status_ |= status;
		wait_for_interrupt_->commandWakeup(&intr_status_);
	}
}

bool macRTSX::wait_for_completion(int ms_timeout)
{
	bool success;
	
	if (wait_for_interrupt_->runCommand(&success, &ms_timeout) != kIOReturnSuccess)
	{
		success = false;
	}
	
	return success;
}

uint32_t macRTSX::READ4(uint32_t reg)
{
	uint32_t val;
	memory_descriptor_->readBytes(reg, &val, 4);
//	printf("READ4  %08x == %08x\n", reg, val);
	return val;
}

void macRTSX::WRITE4(uint32_t reg, uint32_t val)
{
//	printf("WRITE4  %08x := %08x\n", reg, val);
	memory_descriptor_->writeBytes(reg, &val, 4);
}

bool macRTSX::rtsx_read(uint16_t addr, uint8_t *val)
{
	int tries = 1024;
	uint32_t reg = 0;
	
	WRITE4(RTSX_HAIMR, RTSX_HAIMR_BUSY | (addr & 0x3fff) << 16);
	
	while (--tries)
	{
		IODelay(1);
		reg = READ4(RTSX_HAIMR);
		if (!(reg & RTSX_HAIMR_BUSY))
			break;
	}
	
	if (tries == 0)
	{
		logger_.error("rtsx_read(%04x) timeout\n", addr);
	}
	
	*val = (reg & 0xff);
	return tries != 0; // true == success
}

bool macRTSX::rtsx_write(uint16_t addr, uint8_t mask, uint8_t val)
{
	int tries = 1024;
	uint32_t reg = 0;
	
	WRITE4(RTSX_HAIMR, RTSX_HAIMR_BUSY | RTSX_HAIMR_WRITE |
		   (((addr & 0x3FFF) << 16) |
			(mask << 8) | val));
	
	while (--tries)
	{
		reg = READ4(RTSX_HAIMR);
		if (!(reg & RTSX_HAIMR_BUSY))
		{
			if (val != (reg & 0xff))
				return false;
			else
				return true;
		}
	}
	
	logger_.error("rtsx_write(%04x) timeout\n", addr);
	
	return false;
}

bool macRTSX::rtsx_write_phy(uint8_t addr, uint16_t val)
{
	int timeout = 100000;
	u_int8_t rwctl;
	
	RTSX_WRITE(RTSX_PHY_DATA0, val);
	RTSX_WRITE(RTSX_PHY_DATA1, val >> 8);
	RTSX_WRITE(RTSX_PHY_ADDR, addr);
	RTSX_WRITE(RTSX_PHY_RWCTL, RTSX_PHY_BUSY|RTSX_PHY_WRITE);
	
	while (--timeout) {
		RTSX_READ(RTSX_PHY_RWCTL, &rwctl);
		if (!(rwctl & RTSX_PHY_BUSY))
			break;
	}

	if (timeout == 0)
	{
		logger_.error("error writing to phy.\n");
		return false;
	}
	
	return true; // success
}

void macRTSX::RTSX_READ(uint16_t addr, uint8_t* val)
{
	rtsx_read(addr, val);
}

void macRTSX::RTSX_WRITE(uint16_t addr, uint8_t val)
{
	rtsx_write(addr, 0xff, val);
}

void macRTSX::RTSX_CLR(uint16_t addr, uint8_t bits)
{
	rtsx_write(addr, bits, 0);
}

void macRTSX::RTSX_SET(uint16_t addr, uint8_t bits)
{
	rtsx_write(addr, bits, 0xff);
}

bool macRTSX::rtsx_read_cfg(uint8_t func, uint16_t addr, uint32_t *val)
{
	int tries = 1024;
	u_int8_t data0, data1, data2, data3, rwctl;
	
	RTSX_WRITE(RTSX_CFGADDR0, addr);
	RTSX_WRITE(RTSX_CFGADDR1, addr >> 8);
	RTSX_WRITE(RTSX_CFGRWCTL, RTSX_CFG_BUSY | (func & 0x03 << 4));
	
	while (--tries) {
		RTSX_READ(RTSX_CFGRWCTL, &rwctl);
		if (!(rwctl & RTSX_CFG_BUSY))
			break;
	}
	
	if (tries == 0)
		return false;
	
	RTSX_READ(RTSX_CFGDATA0, &data0);
	RTSX_READ(RTSX_CFGDATA1, &data1);
	RTSX_READ(RTSX_CFGDATA2, &data2);
	RTSX_READ(RTSX_CFGDATA3, &data3);
	
	*val = (data3 << 24) | (data2 << 16) | (data1 << 8) | data0;

	return true;
}

/*
 * Set or change SD bus voltage and enable or disable SD bus power.
 * Return zero on success.
 */
bool macRTSX::rtsx_bus_power(uint32_t ocr)
{
	logger_.debug("voltage change ocr=0x%08x\n", ocr);
	
	/*
	 * Disable bus power before voltage change.
	 */
	if (!rtsx_bus_power_off())
		return false;
	
	IODelay(200);
	
	/* If power is disabled, reset the host and return now. */
	if (ocr == 0) {
		(void)rtsx_host_reset();
		return true;
	}

	// XXX check supported voltage.
	
	if (!rtsx_bus_power_on())
		return false;
	
	if (!rtsx_set_bus_width(1))
		return false;
	
	return true;
}

/*
 * Notice that the meaning of RTSX_PWR_GATE_CTRL changes between RTS5209 and
 * RTS5229. In RTS5209 it is a mask of disabled power gates, while in RTS5229
 * it is a mask of *enabled* gates.
 */

bool macRTSX::rtsx_bus_power_on()
{
	u_int8_t enable3;
	
	/* Select SD card. */
	RTSX_WRITE(RTSX_CARD_SELECT, RTSX_SD_MOD_SEL);
	RTSX_WRITE(RTSX_CARD_SHARE_MODE, RTSX_CARD_SHARE_48_SD);
	RTSX_SET(RTSX_CARD_CLK_EN, RTSX_SD_CLK_EN);
	
	/* Enable pull control. */
	RTSX_WRITE(RTSX_CARD_PULL_CTL1, RTSX_PULL_CTL_ENABLE12);
	RTSX_WRITE(RTSX_CARD_PULL_CTL2, RTSX_PULL_CTL_ENABLE12);
	if (device_id_ == 0x5229 && is5229_type_C_)
		enable3 = RTSX_PULL_CTL_ENABLE3_TYPE_C;
	else
		enable3 = RTSX_PULL_CTL_ENABLE3;
	RTSX_WRITE(RTSX_CARD_PULL_CTL3, enable3);
	
	/*
	 * To avoid a current peak, enable card power in two phases with a
	 * delay in between.
	 */
	
	/* Partial power. */
	RTSX_SET(RTSX_CARD_PWR_CTL, RTSX_SD_PARTIAL_PWR_ON);
	if (device_id_ == 0x5209)
		RTSX_SET(RTSX_PWR_GATE_CTRL, RTSX_LDO3318_SUSPEND);
	else
		RTSX_SET(RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC1);
	
	IODelay(200);
	
	/* Full power. */
	RTSX_CLR(RTSX_CARD_PWR_CTL, RTSX_SD_PWR_OFF);
	if (device_id_ == 0x5209)
		RTSX_CLR(RTSX_PWR_GATE_CTRL, RTSX_LDO3318_OFF);
	else
		RTSX_SET(RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC2);
	
	/* Enable SD card output. */
	RTSX_WRITE(RTSX_CARD_OE, RTSX_SD_OUTPUT_EN);
	
	return true;
}

bool macRTSX::rtsx_bus_power_off()
{
	u_int8_t disable3;
	
	if (!rtsx_stop_sd_clock())
		return false;
	
	/* Disable SD output. */
	RTSX_CLR(RTSX_CARD_OE, RTSX_CARD_OUTPUT_EN);
	
	/* Turn off power. */
	disable3 = RTSX_PULL_CTL_DISABLE3;
	if (device_id_ == 0x5209)
		RTSX_SET(RTSX_PWR_GATE_CTRL, RTSX_LDO3318_OFF);
	else {
		RTSX_CLR(RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC1 |
				 RTSX_LDO3318_VCC2);
		if (device_id_ == 0x5229 && is5229_type_C_)
			disable3 = RTSX_PULL_CTL_DISABLE3_TYPE_C;
	}
	
	RTSX_SET(RTSX_CARD_PWR_CTL, RTSX_SD_PWR_OFF);
	RTSX_CLR(RTSX_CARD_PWR_CTL, RTSX_PMOS_STRG_800mA);
	
	/* Disable pull control. */
	RTSX_WRITE(RTSX_CARD_PULL_CTL1, RTSX_PULL_CTL_DISABLE12);
	RTSX_WRITE(RTSX_CARD_PULL_CTL2, RTSX_PULL_CTL_DISABLE12);
	RTSX_WRITE(RTSX_CARD_PULL_CTL3, disable3);

	return true;
}

bool macRTSX::rtsx_stop_sd_clock()
{
	RTSX_CLR(RTSX_CARD_CLK_EN, RTSX_CARD_CLK_EN_ALL);
	RTSX_SET(RTSX_SD_BUS_STAT, RTSX_SD_CLK_FORCE_STOP);
	
	return true;
}

bool macRTSX::rtsx_set_bus_width(int w)
{
	uint32_t bus_width;
	
	switch (w) {
		case 8:
			bus_width = RTSX_BUS_WIDTH_8;
			break;
		case 4:
			bus_width = RTSX_BUS_WIDTH_4;
			break;
		case 1:
		default:
			bus_width = RTSX_BUS_WIDTH_1;
			break;
	}

	return rtsx_write(RTSX_SD_CFG1, RTSX_BUS_WIDTH_MASK, bus_width);
}

bool macRTSX::rtsx_bus_clock(int freq /* in hertz */)
{
	uint8_t n;
	int div, mcu;
	
	if (freq == 0)
	{
		return rtsx_stop_sd_clock();
	}
	
	/* Round down to a supported frequency. */
	if (freq >= 50000)
		freq = 50000;
	else if (freq >= 25000)
		freq = 25000;
	else
		freq = 400;

	/*
	 * Configure the clock frequency.
	 */
	switch (freq) {
		case 200:
			n = 80; /* minimum */
			div = RTSX_CLK_DIV_8;
			mcu = 7;
			RTSX_SET(RTSX_SD_CFG1, RTSX_CLK_DIVIDE_256);
			break;
		case 400:
			n = 80; /* minimum */
			div = RTSX_CLK_DIV_8;
			mcu = 7;
			RTSX_SET(RTSX_SD_CFG1, RTSX_CLK_DIVIDE_128);
			break;
		case 25000:
			n = 100;
			div = RTSX_CLK_DIV_4;
			mcu = 7;
			RTSX_CLR(RTSX_SD_CFG1, RTSX_CLK_DIVIDE_MASK);
			break;
		case 50000:
			n = 100;
			div = RTSX_CLK_DIV_2;
			mcu = 7;
			RTSX_CLR(RTSX_SD_CFG1, RTSX_CLK_DIVIDE_MASK);
			break;
		default:
			return false;
	}

	/*
	 * Enable SD clock.
	 */
	return rtsx_switch_sd_clock(n, div, mcu);
}

bool macRTSX::rtsx_switch_sd_clock(uint8_t n, int div, int mcu)
{
	/* Enable SD 2.0 mode. */
	RTSX_CLR(RTSX_SD_CFG1, RTSX_SD_MODE_MASK);
	
	RTSX_SET(RTSX_CLK_CTL, RTSX_CLK_LOW_FREQ);
	
	RTSX_WRITE(RTSX_CARD_CLK_SOURCE,
			   RTSX_CRC_FIX_CLK | RTSX_SD30_VAR_CLK0 | RTSX_SAMPLE_VAR_CLK1);
	RTSX_CLR(RTSX_SD_SAMPLE_POINT_CTL, RTSX_SD20_RX_SEL_MASK);
	RTSX_WRITE(RTSX_SD_PUSH_POINT_CTL, RTSX_SD20_TX_NEG_EDGE);
	RTSX_WRITE(RTSX_CLK_DIV, (div << 4) | mcu);
	RTSX_CLR(RTSX_SSC_CTL1, RTSX_RSTB);
	RTSX_CLR(RTSX_SSC_CTL2, RTSX_SSC_DEPTH_MASK);
	RTSX_WRITE(RTSX_SSC_DIV_N_0, n);
	RTSX_SET(RTSX_SSC_CTL1, RTSX_RSTB);
	IODelay(100);
	
	RTSX_CLR(RTSX_CLK_CTL, RTSX_CLK_LOW_FREQ);

	return true;
}

void macRTSX::rtsx_hostcmd(u_int32_t *cmdbuf, int *n, u_int8_t cmd, u_int16_t reg, u_int8_t mask, u_int8_t data)
{
	if (!(*n < RTSX_HOSTCMD_MAX)) panic();
	
	cmdbuf[(*n)++] = OSSwapHostToLittleInt32((u_int32_t)(cmd & 0x3) << 30) |
	((u_int32_t)(reg & 0x3fff) << 16) |
	((u_int32_t)(mask) << 8) |
	((u_int32_t)data);
}

bool macRTSX::rtsx_hostcmd_send(int ncmd)
{
	
	/* Tell the chip where the command buffer is and run the commands. */
	WRITE4(RTSX_HCBAR, command_buffer_->getPhysicalAddress());
	WRITE4(RTSX_HCBCTLR,
	    ((ncmd * 4) & 0x00ffffff) | RTSX_START_CMD | RTSX_HW_AUTO_RSP);
	
	return true;
}

bool macRTSX::rtsx_soft_reset()
{
	logger_.debug("soft reset\n");
	
	/* Stop command transfer. */
	WRITE4(RTSX_HCBCTLR, RTSX_STOP_CMD);
	
	(void)rtsx_write(RTSX_CARD_STOP, RTSX_SD_STOP|RTSX_SD_CLR_ERR,
		    RTSX_SD_STOP|RTSX_SD_CLR_ERR);
	
	/* Stop DMA transfer. */
	WRITE4(RTSX_HDBCTLR, RTSX_STOP_DMA);
	(void)rtsx_write(RTSX_DMACTL, RTSX_DMA_RST, RTSX_DMA_RST);
	
	(void)rtsx_write(RTSX_RBCTL, RTSX_RB_FLUSH, RTSX_RB_FLUSH);
	
	return true;
}

void macRTSX::rtsx_exec_command(SDMMC_COMMAND& command)
{
	uint32_t* cmdbuf;
	uint8_t rsp_type;
	uint16_t r;
	int ncmd;
	int error = 0;
	
	TRACE();
	
	logger_.debug("executing cmd %d\n", command.opcode);
	
	rsp_type = rtsx_response_type(command.flags & 0xff00);
	if (rsp_type == 0)
	{
		logger_.error("unknown response type 0x%x\n");
		error = kIOReturnUnsupported;
		goto ret;
	}
	
	/* Allocate and map the command buffer. */
	command_buffer_ = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
								kernel_task,
								kIOMemoryPhysicallyContiguous | kIOMapInhibitCache,
								RTSX_HOSTCMD_BUFSIZE,
								0x00000000ffffffffull);
	if (!command_buffer_)
	{
		logger_.error("error allocating memory for transfer.\n");
		error = kIOReturnNoMemory;
		goto ret;
	}
	
	/* The command buffer queues commands the host controller will
	 * run asynchronously. */
	cmdbuf = (uint32_t*)command_buffer_->getBytesNoCopy();
	ncmd = 0;
	
	/* Queue commands to set SD command index and argument. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD0, 0xff, 0x40 | command.opcode);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD1, 0xff, command.argument >> 24);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD2, 0xff, command.argument >> 16);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD3, 0xff, command.argument >> 8);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD4, 0xff, command.argument);

	/* Queue command to set response type. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CFG2, 0xff, rsp_type);

	/* Use the ping-pong buffer for commands which do not transfer data. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_CARD_DATA_SOURCE,
	    0x01, RTSX_PINGPONG_BUFFER);

	/* Queue commands to perform SD transfer. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_TRANSFER,
	    0xff, RTSX_TM_CMD_RSP | RTSX_SD_TRANSFER_START);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_CHECK_REG_CMD, RTSX_SD_TRANSFER,
	    RTSX_SD_TRANSFER_END|RTSX_SD_STAT_IDLE,
	    RTSX_SD_TRANSFER_END|RTSX_SD_STAT_IDLE);

	/* Queue commands to read back card status response. */
	if (rsp_type == RTSX_SD_RSP_TYPE_R2) {
		for (r = RTSX_PPBUF_BASE2 + 15; r > RTSX_PPBUF_BASE2; r--)
			rtsx_hostcmd(cmdbuf, &ncmd, RTSX_READ_REG_CMD, r, 0, 0);
		rtsx_hostcmd(cmdbuf, &ncmd, RTSX_READ_REG_CMD, RTSX_SD_CMD5,
		    0, 0);
	} else if (rsp_type != RTSX_SD_RSP_TYPE_R0) {
		for (r = RTSX_SD_CMD0; r <= RTSX_SD_CMD4; r++)
			rtsx_hostcmd(cmdbuf, &ncmd, RTSX_READ_REG_CMD, r, 0, 0);
	}

	/* Load and sync command DMA buffer. */
	/* NOP */;
	
	/* Run the command queue and wait for completion. */
	if (rtsx_hostcmd_send(ncmd) && wait_for_completion(1000))
	{
		/* Command success. */
		error = 0;
		
		/* Copy card response into sdmmc response buffer. */
		if (command.flags & SCF_RSP_PRESENT)
		{
			/* Copy bytes like sdhc(4), which on little-endian uses
			 * different byte order for short and long responses... */
			if (command.flags & SCF_RSP_136)
			{
				memcpy(command.response, ((uint8_t*)cmdbuf) + 1, sizeof(command.response));
			} else
			{
				/* First byte is CHECK_REG_CMD return value, second
				 * one is the command op code -- we skip those. */
				command.response[0] =
				((OSSwapBigToHostInt32(cmdbuf[0]) & 0x0000ffff) << 16) |
				((OSSwapBigToHostInt32(cmdbuf[1]) & 0xffff0000) >> 16);
			}
		}
	} else {
		error = kIOReturnTimeout;
	}

	if (command.data != nullptr)
	{
		error = rtsx_xfer(command, cmdbuf);
	}
	
	command_buffer_->release();
	
badalloc:
	;
ret:
	command.flags |= SCF_ITSDONE;
	command.error = error;
}

int macRTSX::rtsx_xfer(SDMMC_COMMAND &command, uint32_t *cmdbuf)
{
	int error = 0;
	int ncmd, read = (command.flags & SCF_CMD_READ), dma_dir, tmode;
	uint8_t cfg2;
	
	logger_.development("%s xfer: %d bytes with block size %d\n",
						command.flags & SCF_CMD_READ ? "read" : "write",
						command.datalen,
						command.blklen);
	
	if (command.datalen > 64 * 1024) // XXX this is a bit arbitrary
	{
		logger_.error("command.datalen too large: %d > %d\n",
					  command.datalen,
					  64 * 1024);
		return kIOReturnNoMemory;
	}

	/* Configure DMA transfer mode parameters. */
	cfg2 = RTSX_SD_NO_CHECK_WAIT_CRC_TO | RTSX_SD_CHECK_CRC16 |
	RTSX_SD_NO_WAIT_BUSY_END | RTSX_SD_RSP_LEN_0;
	if (read) {
		dma_dir = RTSX_DMA_DIR_FROM_CARD;
		/* Use transfer mode AUTO_READ3, which assumes we've already
		 * sent the read command and gotten the response, and will
		 * send CMD 12 manually after reading multiple blocks. */
		tmode = RTSX_TM_AUTO_READ3;
		cfg2 |= RTSX_SD_CALCULATE_CRC7 | RTSX_SD_CHECK_CRC7;
	} else {
		dma_dir = RTSX_DMA_DIR_TO_CARD;
		/* Use transfer mode AUTO_WRITE3, which assumes we've already
		 * sent the write command and gotten the response, and will
		 * send CMD 12 manually after writing multiple blocks. */
		tmode = RTSX_TM_AUTO_WRITE3;
		cfg2 |= RTSX_SD_NO_CALCULATE_CRC7 | RTSX_SD_NO_CHECK_CRC7;
	}

	ncmd = 0;
	
	rtsx_hostcmd(cmdbuf, &ncmd, RTSX_WRITE_REG_CMD, RTSX_SD_CFG2,
	    0xff, cfg2);
	
	/* Queue commands to configure data transfer size. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_BYTE_CNT_L, 0xff,
	    (command.blklen & 0xff));
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_BYTE_CNT_H, 0xff,
	    (command.blklen >> 8));
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_BLOCK_CNT_L, 0xff,
	    ((command.datalen / command.blklen) & 0xff));
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_BLOCK_CNT_H, 0xff,
	    ((command.datalen / command.blklen) >> 8));

	/* Use the DMA ring buffer for commands which transfer data. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_CARD_DATA_SOURCE, 0x01, RTSX_RING_BUFFER);

	/* Configure DMA controller. */
	rtsx_hostcmd(cmdbuf, &ncmd, RTSX_WRITE_REG_CMD, RTSX_IRQSTAT0,
	    RTSX_DMA_DONE_INT, RTSX_DMA_DONE_INT);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_DMATC3, 0xff, command.datalen >> 24);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_DMATC2, 0xff, command.datalen >> 16);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_DMATC1, 0xff, command.datalen >> 8);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_DMATC0, 0xff, command.datalen);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_DMACTL,
	    0x03 | RTSX_DMA_PACK_SIZE_MASK,
	    dma_dir | RTSX_DMA_EN | RTSX_DMA_512);

	/* Queue commands to perform SD transfer. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_TRANSFER,
	    0xff, tmode | RTSX_SD_TRANSFER_START);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_CHECK_REG_CMD, RTSX_SD_TRANSFER,
	    RTSX_SD_TRANSFER_END, RTSX_SD_TRANSFER_END);

	if (!rtsx_hostcmd_send(ncmd))
	{
		error = kIOReturnIOError;
		goto ret;
	}
	
	/* Allocate and map DMA memory for data transfer. */
	data_buffer_ = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
																	kernel_task,
																	kIOMemoryPhysicallyContiguous | kIOMapInhibitCache,
																	command.datalen,
																	0x00000000ffffffffull);
	if (!data_buffer_)
	{
		logger_.error("error allocating memory for transfer.\n");
		error = kIOReturnNoMemory;
		goto ret;
	}

	/* If this is a write, copy data from sdmmc-provided buffer. */
	if (!read)
		memcpy(data_buffer_->getBytesNoCopy(), command.data, command.datalen);

	/* Load the data buffer and sync it. */
	/* NOP */;

	/* Tell the chip where the data buffer is and run the transfer. */
	WRITE4(RTSX_HDBAR, data_buffer_->getPhysicalAddress());
	WRITE4(RTSX_HDBCTLR, RTSX_TRIG_DMA | (read ? RTSX_DMA_READ : 0) |
	    (command.datalen & 0x00ffffff));

	/* Wait for completion. */
	if (wait_for_completion(10000))
	{
		/* If this is a read, copy data into sdmmc-provided buffer. */
		if (read)
			memcpy(command.data, data_buffer_->getBytesNoCopy(), command.datalen);
	} else {
		error = kIOReturnTimeout;
	}
	
	data_buffer_->release();
	
ret:
	return error;
}

uint8_t macRTSX::rtsx_response_type(uint16_t sdmmc_rsp)
{
#define nitems(_a) (sizeof((_a)) / sizeof((_a)[0]))
	int i;
	struct rsp_type {
		u_int16_t sdmmc_rsp;
		u_int8_t rtsx_rsp;
	} rsp_types[] = {
		{ SCF_RSP_R0,	RTSX_SD_RSP_TYPE_R0 },
		{ SCF_RSP_R1,	RTSX_SD_RSP_TYPE_R1 },
		{ SCF_RSP_R1B,	RTSX_SD_RSP_TYPE_R1B },
		{ SCF_RSP_R2,	RTSX_SD_RSP_TYPE_R2 },
		{ SCF_RSP_R3,	RTSX_SD_RSP_TYPE_R3 },
		{ SCF_RSP_R4,	RTSX_SD_RSP_TYPE_R4 },
		{ SCF_RSP_R5,	RTSX_SD_RSP_TYPE_R5 },
		{ SCF_RSP_R6,	RTSX_SD_RSP_TYPE_R6 },
		{ SCF_RSP_R7,	RTSX_SD_RSP_TYPE_R7 }
	};

	for (i = 0; i < nitems(rsp_types); i++) {
		if (sdmmc_rsp == rsp_types[i].sdmmc_rsp)
			return rsp_types[i].rtsx_rsp;
	}
#undef nitems

	return 0;
}

bool macRTSX::rtsx_host_reset()
{
	uint32_t status;
	
	logger_.debug("host reset\n");
	
	if (card_exists_)
		rtsx_soft_reset();
	
	/* Enable interrupt write-clear (default is read-clear). */
	RTSX_CLR(RTSX_NFTS_TX_CTRL, RTSX_INT_READ_CLR);
	
	/* Clear any pending interrupts. */
	status = READ4(RTSX_BIPR);
	WRITE4(RTSX_BIPR, status);
	
	/* Enable interrupts. */
	WRITE4(RTSX_BIER,
	    RTSX_TRANS_OK_INT_EN | RTSX_TRANS_FAIL_INT_EN | RTSX_SD_INT_EN);
	
	/* Power on SSC clock. */
	RTSX_CLR(RTSX_FPDCTL, RTSX_SSC_POWER_DOWN);
	IODelay(200);
	
	/* XXX magic numbers from linux driver */
	if (device_id_ == 0x5209)
	{
		rtsx_write_phy(0x00, 0xB966);
	} else {
		rtsx_write_phy(0x00, 0xBA42);
	}
	
	RTSX_SET(RTSX_CLK_DIV, 0x07);
	
	/* Disable sleep mode. */
	RTSX_CLR(RTSX_HOST_SLEEP_STATE,
			 RTSX_HOST_ENTER_S1 | RTSX_HOST_ENTER_S3);
	
	/* Disable card clock. */
	RTSX_CLR(RTSX_CARD_CLK_EN, RTSX_CARD_CLK_EN_ALL);
	
	RTSX_CLR(RTSX_CHANGE_LINK_STATE,
			 RTSX_FORCE_RST_CORE_EN | RTSX_NON_STICKY_RST_N_DBG | 0x04);
	RTSX_WRITE(RTSX_SD30_DRIVE_SEL, RTSX_SD30_DRIVE_SEL_3V3);
	
	/* Enable SSC clock. */
	RTSX_WRITE(RTSX_SSC_CTL1, RTSX_SSC_8X_EN | RTSX_SSC_SEL_4M);
	RTSX_WRITE(RTSX_SSC_CTL2, 0x12);
	
	RTSX_SET(RTSX_CHANGE_LINK_STATE, RTSX_MAC_PHY_RST_N_DBG);
	RTSX_SET(RTSX_IRQSTAT0, RTSX_LINK_READY_INT);
	
	RTSX_WRITE(RTSX_PERST_GLITCH_WIDTH, 0x80);
	
	/* Set RC oscillator to 400K. */
	RTSX_CLR(RTSX_RCCTL, RTSX_RCCTL_F_2M);
	
	/* Request clock by driving CLKREQ pin to zero. */
	RTSX_SET(RTSX_PETXCFG, RTSX_PETXCFG_CLKREQ_PIN);
	
	/* Set up LED GPIO. */
	if (device_id_ == 0x5209) {
		RTSX_WRITE(RTSX_CARD_GPIO, 0x03);
		RTSX_WRITE(RTSX_CARD_GPIO_DIR, 0x03);
	} else {
		RTSX_SET(RTSX_GPIO_CTL, RTSX_GPIO_LED_ON);
		/* Switch LDO3318 source from DV33 to 3V3. */
		RTSX_CLR(RTSX_LDO_PWR_SEL, RTSX_LDO_PWR_SEL_DV33);
		RTSX_SET(RTSX_LDO_PWR_SEL, RTSX_LDO_PWR_SEL_3V3);
		/* Set default OLT blink period. */
		RTSX_SET(RTSX_OLT_LED_CTL, RTSX_OLT_LED_PERIOD);
	}
	
	return true;
}

/*********************************************************
 *
 * SDMMC
 *
 ******/

bool macRTSX::sdmmc_enable()
{
	u_int32_t host_ocr;

	/*
	 * Calculate the equivalent of the card OCR from the host
	 * capabilities and select the maximum supported bus voltage.
	 */
	host_ocr = (MMC_OCR_3_3V_3_4V|MMC_OCR_3_2V_3_3V|
				MMC_OCR_3_1V_3_2V|MMC_OCR_3_0V_3_1V|
				SD_OCR_SDHC_CAP);
	if (!rtsx_bus_power(host_ocr))
	{
		logger_.error("can't supply bus power\n");
		goto err;
	}
	
	/*
	 * Select the minimum clock frequency.
	 */
	if (!rtsx_bus_clock(200))
	{
		logger_.error("can't supply clock\n");
		goto err;
	}
	
	/* XXX wait for card to power up */
	IODelay(1000000);

	/* Initialize SD/MMC memory card(s). */
	if (!sdmmc_mem_enable())
	{
		goto err;
	}
	
	return true;
	
err:
	sdmmc_disable();
	return false;
}

bool macRTSX::sdmmc_disable()
{
	/* XXX complete commands if card is still present. */

	/* Turn off bus power and clock. */
	rtsx_bus_clock(0);
	rtsx_bus_power(0);
	
	return true;
}

/*
 * Initialize SD/MMC memory cards and memory in SDIO "combo" cards.
 */
bool macRTSX::sdmmc_mem_enable()
{
	uint32_t host_ocr, card_ocr;
	
	sdmmc_go_idle_state();
	
	/* CMD8 */
	{
		SDMMC_COMMAND cmd;

		cmd.opcode = 8;
		cmd.argument = 0x1AA;
		cmd.flags = SCF_CMD_BCR | SCF_RSP_R7;
	
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;
	
		if ((cmd.response[0] & 0xff) != 0xAA)
			return false;
		
		logger_.development("CMD8 response: %08x%08x%08x%08x\n",
							cmd.response[0],cmd.response[1],cmd.response[2],cmd.response[3]);
	}

	/* ACMD41 */
card_is_still_powering_up:
	{
		SDMMC_COMMAND cmd;
		
		cmd.opcode = 55;
		cmd.argument = 0; // Default RCA is 0x0000;
		cmd.flags = SCF_CMD_AC | SCF_RSP_R1;
		
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;
		
		logger_.development("CMD55 response: %08x\n",
							cmd.response[0]);
		
		cmd.opcode = 41;
		cmd.argument = (1<<23);
		cmd.flags = SCF_CMD_BCR | SCF_RSP_R3;
		
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;
		
		logger_.development("ACMD41 response: %08x%08x%08x%08x\n",
							cmd.response[0],cmd.response[1],cmd.response[2],cmd.response[3]);
		
		if (!(cmd.response[0] & (1<<31)))
		{
			goto card_is_still_powering_up;
		}
		
		IODelay(10000);
	}
	
	/* Card is now ready. */

	/* CMD2 */
	{
		SDMMC_COMMAND cmd;
		
		cmd.opcode = 2;
		cmd.argument = 0;
		cmd.flags = SCF_CMD_BCR | SCF_RSP_R2;
		
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;
		
		logger_.development("CMD2 response: %08x%08x%08x%08x\n",
							cmd.response[0],cmd.response[1],cmd.response[2],cmd.response[3]);
		
		/* Now we have the CID, go ahead and parse it. */
		__uint128_t cid;
		cid = *((__uint128_t*)cmd.response);
		
		logger_.debug("mdt: %03x\n", __bitfield((uint32_t*)&cid, 0, 12));
		logger_.debug("psn: %08x\n", __bitfield((uint32_t*)&cid, 16, 32));
		logger_.debug("prv: %02x\n", __bitfield((uint32_t*)&cid, 48, 8));
		logger_.debug("pnm: %08x%02x\n",
					  __bitfield((uint32_t*)&cid, 56, 32),
					  __bitfield((uint32_t*)&cid, 88, 8));
		logger_.debug("oid: %04x\n", __bitfield((uint32_t*)&cid, 96, 16));
		logger_.debug("mid: %02x\n", __bitfield((uint32_t*)&cid, 112, 8));
	}

	/* CMD3 */
	{
		SDMMC_COMMAND cmd;
		
		cmd.opcode = 3;
		cmd.argument = 0;
		cmd.flags = SCF_CMD_BCR | SCF_RSP_R6;
		
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;
		
		logger_.development("CMD3 response: %08x%08x%08x%08x\n",
							cmd.response[0],cmd.response[1],cmd.response[2],cmd.response[3]);
		
		/* Now we have the RCA. */
		RCA_ = (cmd.response[0] >> 16) & 0xffff;
		
		logger_.development("card RCA %08x\n", RCA_);
	}
	
	/* CMD7 */
	{
		SDMMC_COMMAND cmd;

		cmd.opcode = 7;
		cmd.argument = RCA_ << 16;
		cmd.flags = SCF_CMD_AC | SCF_RSP_R1B;
	
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;
	
		logger_.development("CMD7 response: %08x%08x%08x%08x\n",
							cmd.response[0],cmd.response[1],cmd.response[2],cmd.response[3]);
	}

	/* ACMD6 */
	{
		SDMMC_COMMAND cmd;
		
		cmd.opcode = 55;
		cmd.argument = RCA_ << 16;
		cmd.flags = SCF_CMD_AC | SCF_RSP_R1;
		
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;
	
		logger_.development("CMD55 response: %08x\n",
							cmd.response[0]);
		
		cmd.opcode = 6;
		cmd.argument = 2; // switch to 4-bit mode
		cmd.flags = SCF_CMD_AC | SCF_RSP_R1;
		
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;

		logger_.development("ACMD6 response: %08x%08x%08x%08x\n",
									cmd.response[0],cmd.response[1],cmd.response[2],cmd.response[3]);
		rtsx_set_bus_width(4);
	}
	
	/* ACMD6 */
	{
		SDMMC_COMMAND cmd;
		
		cmd.opcode = 55;
		cmd.argument = RCA_ << 16;
		cmd.flags = SCF_CMD_AC | SCF_RSP_R1;
		
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;
		
		logger_.development("CMD55 response: %08x\n",
							cmd.response[0]);
		
		cmd.opcode = 6;
		cmd.argument = 2; // switch to 4-bit mode
		cmd.flags = SCF_CMD_AC | SCF_RSP_R1;
		
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;
		
		logger_.development("ACMD6 response: %08x%08x%08x%08x\n",
							cmd.response[0],cmd.response[1],cmd.response[2],cmd.response[3]);
		rtsx_set_bus_width(4);
	}

	/* CMD9 */
//	{
//		SDMMC_COMMAND cmd;
//		
//		cmd.opcode = 9;
//		cmd.argument = RCA_ << 16;
//		cmd.flags = SCF_CMD_AC | SCF_RSP_R2;
//		
//		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
//			return false;
//		
//		logger_.development("CMD9 response: %08x%08x%08x%08x\n",
//							cmd.response[0],cmd.response[1],cmd.response[2],cmd.response[3]);
//		
//		/* Now we have the CSD. */
//	}

	/* CMD7 */
//	{
//		SDMMC_COMMAND cmd;
//	
//		cmd.opcode = 7;
//		cmd.argument = RCA_ << 16;
//		cmd.flags = SCF_CMD_AC | SCF_RSP_R1B;
//
//		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
//			return false;
//
//		logger_.development("CMD7 response: %08x%08x%08x%08x\n",
//							cmd.response[0],cmd.response[1],cmd.response[2],cmd.response[3]);
//	}

	/* ACMD51 */
	{
		SDMMC_COMMAND cmd;
		
		cmd.opcode = 55;
		cmd.argument = RCA_ << 16;
		cmd.flags = SCF_CMD_AC | SCF_RSP_R1;
		
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;
		
		logger_.development("CMD55 response: %08x\n",
							cmd.response[0]);
		
		cmd.opcode = 51;
		cmd.argument = 0;
		cmd.flags = SCF_CMD_ADTC | SCF_CMD_READ | SCF_RSP_R1;
		cmd.datalen = 8;
		cmd.blklen = 8;
		cmd.data = (uint8_t*)&SCR_;
		
		if (sdmmc_mmc_command(cmd) != kIOReturnSuccess)
			return false;
		
		logger_.development("ACMD51 response: %08x%08x%08x%08x\n",
							cmd.response[0],cmd.response[1],cmd.response[2],cmd.response[3]);
		
		logger_.debug("card SCR: %016llx\n", SCR_);
	}
	

	
	return true;
}

/*
 * Send the "GO IDLE STATE" command.
 */
void macRTSX::sdmmc_go_idle_state()
{
	SDMMC_COMMAND cmd;
	cmd.opcode = 0;
	cmd.flags = SCF_CMD_BC | SCF_RSP_R0;
	
	TRACE();
	
	rtsx_exec_command(cmd);
}

IOReturn macRTSX::sdmmc_app_command(SDMMC_COMMAND &cmd)
{
	SDMMC_COMMAND acmd;
	int error;
	
	acmd.opcode = MMC_APP_CMD;
	acmd.argument = 0;
	
	// XXX RCA
	
	acmd.flags = SCF_CMD_AC | SCF_RSP_R1;
	
	error = sdmmc_mmc_command(acmd);
	if (error != 0)
	{
		return error;
	}
	
	if (!(MMC_R1(acmd.response) & MMC_R1_APP_CMD))
	{
		/* Card does not support application commands. */
		return kIOReturnNoDevice;
	}
	
	error = sdmmc_mmc_command(cmd);
	return error;
}

IOReturn macRTSX::sdmmc_mmc_command(SDMMC_COMMAND &cmd)
{
	int error;
	
	cmd.response[0] = 0;
	cmd.response[1] = 0;
	cmd.response[2] = 0;
	cmd.response[3] = 0;
	
	rtsx_exec_command(cmd);
	
	error = cmd.error;
	
	return error;
}
