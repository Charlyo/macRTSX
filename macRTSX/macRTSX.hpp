//                       ____ _____ ______  __
//  _ __ ___   __ _  ___|  _ \_   _/ ___\ \/ /
// | '_ ` _ \ / _` |/ __| |_) || | \___ \\  /
// | | | | | | (_| | (__|  _ < | |  ___) /  \
// |_| |_| |_|\__,_|\___|_| \_\|_| |____/_/\_\

#pragma once

#include <IOKit/system.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IOTimerEventSource.h>

#include "debug.hpp"
#include "rtsxreg.hpp"

class SDMMC_COMMAND;

class macRTSX : public IOService
{
	OSDeclareDefaultStructors(macRTSX);
	
	////
	
	Logger logger_ = Logger("macRTSX");

	IOPCIDevice* provider_;
	IOWorkLoop* workloop_;
	
	IOMemoryMap* memory_map_;
	IOMemoryDescriptor* memory_descriptor_;
	
	IOInterruptEventSource* interrupt_source_;
	
	IOCommandGate* hardware_bringup_;
	IOCommandGate* hardware_bringdown_;
	IOTimerEventSource* detect_work_;
	IOCommandGate* wait_for_interrupt_;
	
	bool card_exists_, card_inuse_;
	bool card_attached_;
	bool supports_sdio_;
	
	uint32_t intr_status_;	/* Also used to sleep. */
	
	uint16_t RCA_;
	uint64_t SCR_;
	
	////
	
	const int RTSX_HOSTCMD_MAX = 256;
	
	int device_id_, vendor_id_, revision_id_;
	bool is5229_type_C_;
	
	/*** Command, Data DMA buffers.
	 ***/
	IOBufferMemoryDescriptor* command_buffer_;
	IOBufferMemoryDescriptor* data_buffer_;
    
public:
	virtual bool start(IOService*) override;
	virtual void stop(IOService*) override;
	
private:
	
	static IOReturn wait_for_interrupt_static(macRTSX* self, bool* success,
											  int* ms_timeout);
	static IOReturn hardware_bringup_static(macRTSX* self);
	static IOReturn hardware_bringdown_static(macRTSX* self);
	
	void hardware_bringup();
	void hardware_bringdown();
	
	static IOReturn detect_work_static(macRTSX* self, IOTimerEventSource *sender);
	void detect_work();
	
	static void interrupt_occurred_static(macRTSX* self, IOInterruptEventSource* source, int);
	void interrupt_occurred();

	bool wait_for_completion(int ms_timeout);
	
	uint32_t READ4(uint32_t reg);
	void WRITE4(uint32_t reg, uint32_t val);
	bool rtsx_read(uint16_t addr, uint8_t* val);
	bool rtsx_write(uint16_t addr, uint8_t mask, uint8_t val);
	bool rtsx_write_phy(uint8_t addr, uint16_t val);
	
	void RTSX_READ(uint16_t addr, uint8_t* val);
	void RTSX_WRITE(uint16_t addr, uint8_t val);
	void RTSX_CLR(uint16_t addr, uint8_t bits);
	void RTSX_SET(uint16_t addr, uint8_t bits);
	
	bool rtsx_read_cfg(uint8_t func, uint16_t addr, uint32_t* val);
	bool rtsx_bus_power(uint32_t ocr);
	bool rtsx_bus_power_on();
	bool rtsx_bus_power_off();
	bool rtsx_stop_sd_clock();
	bool rtsx_set_bus_width(int w);
	bool rtsx_bus_clock(int freq);
	bool rtsx_switch_sd_clock(uint8_t n, int div, int mcu);
	void rtsx_hostcmd(u_int32_t *cmdbuf, int *n, u_int8_t cmd, u_int16_t reg,
					  u_int8_t mask, u_int8_t data);
	bool rtsx_hostcmd_send(int ncmd);
	bool rtsx_soft_reset();
	unsigned int rtsx_host_maxblklen() { return 512; }
	void rtsx_exec_command(SDMMC_COMMAND& cmd);
	int rtsx_xfer(SDMMC_COMMAND& cmd, uint32_t* cmdbuf);
	uint8_t rtsx_response_type(uint16_t sdmmc_rsp);
	bool rtsx_host_reset();
	
	// --- //
	
	bool sdmmc_enable();
	bool sdmmc_disable();
	bool sdmmc_mem_enable();
	void sdmmc_go_idle_state();
	int sdmmc_app_command(SDMMC_COMMAND& cmd);
	int sdmmc_mmc_command(SDMMC_COMMAND& cmd);
	
	
private:
	const int RTSX_HOSTCMD_BUFSIZE = (sizeof(u_int32_t) * RTSX_HOSTCMD_MAX);
};


/*
 * This class represents an SD message to the RTSX chip.
 */
class SDMMC_COMMAND
{
public:
	uint16_t opcode = 0;
	uint32_t argument = 0;
	uint32_t response[4] = {};
	int flags = 0; // see below.
#define SCF_ITSDONE	 0x0001		/* command is complete */
#define SCF_CMD(flags)	 ((flags) & 0x00f0)
#define SCF_CMD_AC	 0x0000
#define SCF_CMD_ADTC	 0x0010
#define SCF_CMD_BC	 0x0020
#define SCF_CMD_BCR	 0x0030
#define SCF_CMD_READ	 0x0040		/* read command (data expected) */
#define SCF_RSP_BSY	 0x0100
#define SCF_RSP_136	 0x0200
#define SCF_RSP_CRC	 0x0400
#define SCF_RSP_IDX	 0x0800
#define SCF_RSP_PRESENT	 0x1000
	/* response types */
#define SCF_RSP_R0	 0 /* none */
#define SCF_RSP_R1	 (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX)
#define SCF_RSP_R1B	 (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX|SCF_RSP_BSY)
#define SCF_RSP_R2	 (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_136)
#define SCF_RSP_R3	 (SCF_RSP_PRESENT)
#define SCF_RSP_R4	 (SCF_RSP_PRESENT)
#define SCF_RSP_R5	 (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX)
#define SCF_RSP_R5B	 (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX|SCF_RSP_BSY)
#define SCF_RSP_R6	 (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX)
#define SCF_RSP_R7	 (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX)
	IOReturn error;	/* err value on completion */
	uint8_t* data = 0;
	size_t datalen = 0;
	size_t blklen = 0;
};

