//                       ____  ____  __  __ __  __  ____
//  _ __ ___   __ _  ___/ ___||  _ \|  \/  |  \/  |/ ___|
// | '_ ` _ \ / _` |/ __\___ \| | | | |\/| | |\/| | |
// | | | | | | (_| | (__ ___) | |_| | |  | | |  | | |___
// |_| |_| |_|\__,_|\___|____/|____/|_|  |_|_|  |_|\____|

#pragma once

#include <IOKit/system.h>
#include <IOKit/IOLib.h>

#include <sys/queue.h>

// For the task system.

class macSDMMC;
class macSDMMCDisk;

struct sdmmc_task
{
    void (*func)(void *arg);
    void *arg;
    int onqueue;
    macSDMMC *sc;
    TAILQ_ENTRY(sdmmc_task)
    next;
};

#define sdmmc_init_task(xtask, xfunc, xarg) \
    do                                      \
    {                                       \
        (xtask)->func = (xfunc);            \
        (xtask)->arg = (xarg);              \
        (xtask)->onqueue = 0;               \
        (xtask)->sc = NULL;                 \
    } while (0)

#define sdmmc_task_pending(xtask) ((xtask)->onqueue)

//===========================

struct sdmmc_csd
{
    int csdver;      /* CSD structure format */
    int mmcver;      /* MMC version (for CID format) */
    int capacity;    /* total number of sectors */
    int sector_size; /* sector size in bytes */
    int read_bl_len; /* block length for reads */
    int ccc;         /* Card Command Class for SD */
                     /* ... */
};

struct sdmmc_cid
{
    int mid;     /* manufacturer identification number */
    int oid;     /* OEM/product identification number */
    char pnm[8]; /* product name (MMC v1 has the longest) */
    int rev;     /* product revision */
    int psn;     /* product serial number */
    int mdt;     /* manufacturing date */
};

struct sdmmc_scr
{
    int sd_spec;
    int bus_width;
};

typedef u_int32_t sdmmc_response[4];

struct sdmmc_command
{
    struct sdmmc_task c_task; /* task queue entry */
    u_int16_t c_opcode;       /* SD or MMC command index */
    u_int32_t c_arg;          /* SD/MMC command argument */
    sdmmc_response c_resp;    /* response buffer */
    void *c_data;             /* buffer to send or read into */
    int c_datalen;            /* length of data buffer */
    int c_blklen;             /* block length */
    int c_flags;              /* see below */
#define SCF_ITSDONE 0x0001    /* command is complete */
#define SCF_CMD(flags) ((flags)&0x00f0)
#define SCF_CMD_AC 0x0000
#define SCF_CMD_ADTC 0x0010
#define SCF_CMD_BC 0x0020
#define SCF_CMD_BCR 0x0030
#define SCF_CMD_READ 0x0040 /* read command (data expected) */
#define SCF_RSP_BSY 0x0100
#define SCF_RSP_136 0x0200
#define SCF_RSP_CRC 0x0400
#define SCF_RSP_IDX 0x0800
#define SCF_RSP_PRESENT 0x1000
/* response types */
#define SCF_RSP_R0 0 /* none */
#define SCF_RSP_R1 (SCF_RSP_PRESENT | SCF_RSP_CRC | SCF_RSP_IDX)
#define SCF_RSP_R1B (SCF_RSP_PRESENT | SCF_RSP_CRC | SCF_RSP_IDX | SCF_RSP_BSY)
#define SCF_RSP_R2 (SCF_RSP_PRESENT | SCF_RSP_CRC | SCF_RSP_136)
#define SCF_RSP_R3 (SCF_RSP_PRESENT)
#define SCF_RSP_R4 (SCF_RSP_PRESENT)
#define SCF_RSP_R5 (SCF_RSP_PRESENT | SCF_RSP_CRC | SCF_RSP_IDX)
#define SCF_RSP_R5B (SCF_RSP_PRESENT | SCF_RSP_CRC | SCF_RSP_IDX | SCF_RSP_BSY)
#define SCF_RSP_R6 (SCF_RSP_PRESENT | SCF_RSP_CRC | SCF_RSP_IDX)
#define SCF_RSP_R7 (SCF_RSP_PRESENT | SCF_RSP_CRC | SCF_RSP_IDX)
    int c_error; /* errno value on completion */

    /* Host controller owned fields for data xfer in progress */
    int c_resid;   /* remaining I/O */
    u_char *c_buf; /* remaining data */
};

/*
 * Decoded PC Card 16 based Card Information Structure (CIS),
 * per card (function 0) and per function (1 and greater).
 */
struct sdmmc_cis {
	u_int16_t	 manufacturer;
#define SDMMC_VENDOR_INVALID	0xffff
	u_int16_t	 product;
#define SDMMC_PRODUCT_INVALID	0xffff
	u_int8_t	 function;
#define SDMMC_FUNCTION_INVALID	0xff
	u_char		 cis1_major;
	u_char		 cis1_minor;
	char		 cis1_info_buf[256];
	char		*cis1_info[4];
};

/*
 * Structure describing either an SD card I/O function or a SD/MMC
 * memory card from a "stack of cards" that responded to CMD2.  For a
 * combo card with one I/O function and one memory card, there will be
 * two of these structures allocated.  Each card slot has such a list
 * of sdmmc_function structures.
 */
struct sdmmc_function {
	/* common members */
	macSDMMC *sc;		/* card slot softc */
	u_int16_t rca;			/* relative card address */
	int flags;
#define SFF_ERROR		0x0001	/* function is poo; ignore it */
#define SFF_SDHC		0x0002	/* SD High Capacity card */
	STAILQ_ENTRY(sdmmc_function) sf_list;
	/* SD card I/O function members */
	int number;			/* I/O function number or -1 */
	struct device *child;		/* function driver */
	struct sdmmc_cis cis;		/* decoded CIS */
	/* SD/MMC memory card members */
	struct sdmmc_csd csd;		/* decoded CSD value */
	struct sdmmc_cid cid;		/* decoded CID value */
	sdmmc_response raw_cid;		/* temp. storage for decoding */
	struct sdmmc_scr scr;		/* decoded SCR value */
};

typedef struct __attribute__((packed, aligned(4))) { uint32_t _bits[512/32]; } sdmmc_bitfield512_t;

//===========================

void
sdmmc_task_thread(void *arg);

// Class describing a single SD/MMC/SDIO card slot.

class macSDMMC
{

  public:
    //============= SDMMC COMMON

    void attach(/* rtsx_functions, */ void *sch, int flags, int caps);
    int detach(int flags);

    void suspend();
    void resume();

    void add_task(struct sdmmc_task *task);
    void del_task(struct sdmmc_task *task);
    void needs_discover();
    void discover();
    void card_attach();
    void card_detach(bool force);
    int enable();
    void disable();
    int init();
    void delay(u_int usecs);
    int set_bus_power(u_int32_t host_ocr,
                      u_int32_t card_ocr);
    struct sdmmc_function * function_alloc();
    void function_free(struct sdmmc_function *);
    int scan();
    int app_command(struct sdmmc_command *cmd);
    int mmc_command(struct sdmmc_command *cmd);
    void go_idle_state();
    int send_if_cond(uint32_t card_ocr);
    int set_relative_addr(struct sdmmc_function *sf);
    int select_card(struct sdmmc_function *sf);
    void dump_command(struct sdmmc_command *cmd);

    //============= SDMMC MEM

    int mem_enable();
    void mem_scan();
    int decode_csd(sdmmc_response resp,
                   struct sdmmc_function *sf);
    int decode_cid(sdmmc_response resp,
                   struct sdmmc_function *sf);
    int mem_send_op_cond(u_int32_t ocr,
                         u_int32_t *ocrp);
    int mem_init(struct sdmmc_function *sf);
    int mem_set_blocklen(struct sdmmc_function *sf);
    int mem_sd_init(struct sdmmc_function *sf);
    int mem_mmc_init(struct sdmmc_function *sf);
    int mem_send_scr(uint32_t *);
    int mem_decode_scr(uint32_t *raw_scr,
                       struct sdmmc_function *sf);
    int set_bus_width(struct sdmmc_function *sf, int width);
    int mem_sd_switch(struct sdmmc_function *sf, int mode, int group,
                      int function, sdmmc_bitfield512_t *status);
    int mem_send_cxd_data(int opcode, void *data,
                          size_t datalen);
    int mem_mmc_switch(struct sdmmc_function *sf, uint8_t set, uint8_t index,
                             uint8_t value);

    int mem_read_block(struct sdmmc_function *sf, int blkno, u_char *data,
                       size_t datalen);
    int mem_single_read_block(struct sdmmc_function *sf, int blkno, u_char *data,
                              size_t datalen);
    int mem_read_block_subr(struct sdmmc_function *sf, int blkno, u_char *data, size_t datalen);

    int mem_write_block(struct sdmmc_function *sf, int blkno, u_char *data,
                       size_t datalen);
    int mem_single_write_block(struct sdmmc_function *sf, int blkno, u_char *data,
                              size_t datalen);
    int mem_write_block_subr(struct sdmmc_function *sf, int blkno, u_char *data, size_t datalen);

    //============= SDMMC SCSI

    void scsi_attach();
    void scsi_detach();

    //=============

    void DEVERR(const char * msg)
    {
        IOLog("macSDMMC: (FATAL) %s", msg);
    }

    template<typename arg0_t>
    void DEVERR(const char * msg, arg0_t arg0)
    {
        IOLog("macSDMMC: (FATAL) ");
        IOLog(msg, arg0);
    }

    template<typename arg0_t, typename arg1_t>
    void DEVERR(const char * msg, arg0_t arg0, arg1_t arg1)
    {
        IOLog("macSDMMC: (FATAL) ");
        IOLog(msg, arg0, arg1);
    }

    void DEVVERBOSE(const char * msg)
    {
        IOLog("macSDMMC: %s", msg);
    }

    template<typename arg0_t>
    void DEVVERBOSE(const char * msg, arg0_t arg0)
    {
        IOLog("macSDMMC: ");
        IOLog(msg, arg0);
    }

    template<typename arg0_t, typename arg1_t>
    void DEVVERBOSE(const char * msg, arg0_t arg0, arg1_t arg1)
    {
        IOLog("macSDMMC: ");
        IOLog(msg, arg0);
    }

    void set_owner(void * owner)
    {
        owner_ = owner;
    }

  protected:
    // handle to our top-level IOKit parent.
    void * owner_;
    
    // sch is an opaque void-handle to the chipset sc.
    void *sch;

    // handle to the scsi disk instance.
    macSDMMCDisk * disk_;
    friend class macSDMMCDisk;

    // ==================== SOFTC

    int sc_flags;
#define SMF_SD_MODE 0x0001             /* host in SD mode (MMC otherwise) */
#define SMF_IO_MODE 0x0002             /* host in I/O mode (SD mode only) */
#define SMF_MEM_MODE 0x0004            /* host in memory mode (SD or MMC) */
#define SMF_CARD_PRESENT 0x0010        /* card presence noticed */
#define SMF_CARD_ATTACHED 0x0020       /* card driver(s) attached */
#define SMF_STOP_AFTER_MULTIPLE 0x0040 /* send a stop after a multiple cmd */
#define SMF_CONFIG_PENDING 0x0080      /* config_pending_incr() called */

    uint32_t sc_caps;                 /* host capability */
#define SMC_CAPS_AUTO_STOP 0x0001     /* send CMD12 automagically by host */
#define SMC_CAPS_4BIT_MODE 0x0002     /* 4-bits data bus width */
#define SMC_CAPS_DMA 0x0004           /* DMA transfer */
#define SMC_CAPS_SPI_MODE 0x0008      /* SPI mode */
#define SMC_CAPS_POLL_CARD_DET 0x0010 /* Polling card detect */
#define SMC_CAPS_SINGLE_ONLY 0x0020   /* only single read/write */
#define SMC_CAPS_8BIT_MODE 0x0040     /* 8-bits data bus width */
#define SMC_CAPS_MULTI_SEG_DMA 0x0080 /* multiple segment DMA transfer */
#define SMC_CAPS_SD_HIGHSPEED 0x0100  /* SD high-speed timing */
#define SMC_CAPS_MMC_HIGHSPEED 0x0200 /* MMC high-speed timing */
#define SMC_CAPS_UHS_SDR50 0x0400     /* UHS SDR50 timing */
#define SMC_CAPS_UHS_SDR104 0x0800    /* UHS SDR104 timing */
#define SMC_CAPS_UHS_DDR50 0x1000     /* UHS DDR50 timing */
#define SMC_CAPS_UHS_MASK 0x1c00
#define SMC_CAPS_MMC_DDR52 0x2000 /* eMMC DDR52 timing */
#define SMC_CAPS_MMC_HS200 0x4000 /* eMMC HS200 timing */
#define SMC_CAPS_MMC_HS400 0x8000 /* eMMC HS400 timing */

	int sc_function_count;		/* number of I/O functions (SDIO) */
	struct sdmmc_function *sc_card;	/* selected card */
	struct sdmmc_function *sc_fn0;	/* function 0, the card itself */
    STAILQ_HEAD(, sdmmc_function)
    sf_head;                 /* list of card functions */
    int sc_dying;            /* bus driver is shutting down */
    IOThread sc_task_thread; /* asynchronous tasks */
    TAILQ_HEAD(, sdmmc_task)
    sc_tskq;                            /* task thread work queue */
    struct sdmmc_task sc_discover_task; /* card attach/detach task */

    IOLock * sc_lock;

    // =======================================================

    friend class macRTSX;
    friend void ::sdmmc_task_thread(void *arg);
    static void sdmmc_discover_task(void *arg);
};
