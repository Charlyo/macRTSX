//                       ____  ____  __  __ __  __  ____
//  _ __ ___   __ _  ___/ ___||  _ \|  \/  |  \/  |/ ___|
// | '_ ` _ \ / _` |/ __\___ \| | | | |\/| | |\/| | |
// | | | | | | (_| | (__ ___) | |_| | |  | | |  | | |___
// |_| |_| |_|\__,_|\___|____/|____/|_|  |_|_|  |_|\____|

#include "macSDMMC.hpp"
#include "macSDMMC_regs.hpp"

#include <IOKit/system.h>
#include <IOKit/IOLib.h>

#include <sys/kernel.h>
#include <sys/proc.h>

#include "xnu_private.h"
#include "rtsx_ops.hpp"

#include "macSDMMCDisk.hpp"

/*
 * Section for portability to macOS.
 */

#define SET(t, f) ((t) |= (f))
#define ISSET(t, f) ((t) & (f))
#define CLR(t, f) ((t) &= ~(f))

//===============================================

void macSDMMC::attach(/* rtsx_functions, */ void *sch, int flags, int caps)
{
    if (ISSET(caps, SMC_CAPS_8BIT_MODE))
        DEVVERBOSE(": 8-bit");
    else if (ISSET(caps, SMC_CAPS_4BIT_MODE))
        DEVVERBOSE(": 4-bit");
    else
        DEVVERBOSE(": 1-bit");
    if (ISSET(caps, SMC_CAPS_SD_HIGHSPEED))
        DEVVERBOSE(", sd high-speed");
    if (ISSET(caps, SMC_CAPS_MMC_HIGHSPEED))
        DEVVERBOSE(", mmc high-speed");
    if (ISSET(caps, SMC_CAPS_DMA))
        DEVVERBOSE(", dma");
    DEVVERBOSE("\n");

    this->sch = sch;
    this->sc_flags = flags;
    this->sc_caps = caps;
    // XXX max_xfer;

    if (ISSET(this->sc_caps, SMC_CAPS_DMA))
    {
        DEVERR("SMC_CAPS_DMA is not supported\n");
        return;
    }

    STAILQ_INIT(&this->sf_head);
    TAILQ_INIT(&this->sc_tskq);
    sdmmc_init_task(&this->sc_discover_task, &sdmmc_discover_task, this);
    sc_lock = IOLockAlloc();

    /*
	 * Create the event thread that will attach and detach cards
	 * and perform other lengthy operations.  Enter config_pending
	 * state until the discovery task has run for the first time.
	 */
    SET(this->sc_flags, SMF_CONFIG_PENDING);
    sc_task_thread = IOCreateThread(sdmmc_task_thread, this);
    if (!sc_task_thread)
    {
        DEVERR("can't create task thread\n");
    }
}

int macSDMMC::detach(int flags)
{
	this->sc_dying = 1;
	while (this->sc_task_thread != NULL) {
		wakeup(&this->sc_tskq);
		tsleep(this, PWAIT, "mmcdie", 0);
	}

	return 0;
}

// int
// sdmmc_activate(struct device *self, int act)
// {
// 	struct sdmmc_softc *sc = (struct sdmmc_softc *)self;
// 	int rv = 0;

// 	switch (act) {
// 	case DVACT_SUSPEND:
// 		rv = config_activate_children(self, act);
// 		/* If card in slot, cause a detach/re-attach */
// 		if (ISSET(sc->sc_flags, SMF_CARD_PRESENT))
// 			sc->sc_dying = -1;
// 		break;
// 	case DVACT_RESUME:
// 		rv = config_activate_children(self, act);
// 		wakeup(&sc->sc_tskq);
// 		break;
// 	default:
// 		rv = config_activate_children(self, act);
// 		break;
// 	}
// 	return (rv);
// }

void macSDMMC::suspend()
{
	/* If card in slot, cause a detach/re-attach */
	if (ISSET(sc_flags, SMF_CARD_PRESENT))
		sc_dying = -1;
}

void macSDMMC::resume()
{
	wakeup(&sc_tskq);
}

void
sdmmc_task_thread(void *arg)
{
    macSDMMC *that = static_cast<macSDMMC *>(arg);
	struct sdmmc_task *task;

    // XXX HACK HACK XXX
    IODelay(500);

restart:
    that->needs_discover();

	while (!that->sc_dying) {
		for (task = TAILQ_FIRST(&that->sc_tskq); task != NULL;
		     task = TAILQ_FIRST(&that->sc_tskq)) {
            that->del_task(task);
			task->func(task->arg);
		}
		tsleep(&that->sc_tskq, PWAIT, "mmctsk", 0);
	}

	if (ISSET(that->sc_flags, SMF_CARD_PRESENT)) {
        IOLockLock(that->sc_lock);
		that->card_detach(true); // force
        IOLockUnlock(that->sc_lock);
	}

	/*
	 * During a suspend, the card is detached since we do not know
	 * if it is the same upon wakeup.  Go re-discover the bus.
	 */
	if (that->sc_dying == -1) {
		CLR(that->sc_flags, SMF_CARD_PRESENT);
		that->sc_dying = 0;
		goto restart;
	}
	that->sc_task_thread = NULL;
	wakeup(that);
    IOExitThread();
}

void macSDMMC::add_task(struct sdmmc_task *task)
{
	TAILQ_INSERT_TAIL(&this->sc_tskq, task, next);
	task->onqueue = 1;
	task->sc = this;
	wakeup(&this->sc_tskq);
}

void macSDMMC::del_task(struct sdmmc_task *task)
{
    if (task->sc == NULL)
        return;

    task->sc = NULL;
	task->onqueue = 0;
	TAILQ_REMOVE(&this->sc_tskq, task, next);
}

void macSDMMC::needs_discover()
{
	if (!sdmmc_task_pending(&this->sc_discover_task))
	    add_task(&this->sc_discover_task);
}

void macSDMMC::sdmmc_discover_task(void *arg)
{
    macSDMMC * that = static_cast<macSDMMC *>(arg);

    that->discover();
}

void macSDMMC::discover()
{
    DEVVERBOSE("discover\n");

    	if (rtsx_card_detect((rtsx_softc*)sch)) {
		if (!ISSET(sc_flags, SMF_CARD_PRESENT)) {
			SET(sc_flags, SMF_CARD_PRESENT);
			card_attach();
		}
	} else {
		if (ISSET(sc_flags, SMF_CARD_PRESENT)) {
			CLR(sc_flags, SMF_CARD_PRESENT);
            IOLockLock(sc_lock);
			card_detach(true); // force
            IOLockUnlock(sc_lock);
		}
	}

	if (ISSET(sc_flags, SMF_CONFIG_PENDING)) {
		CLR(sc_flags, SMF_CONFIG_PENDING);
        DEVVERBOSE("config_pending_decr() here\n");
	}
}

// Called when a card is present.
void macSDMMC::card_attach()
{
    DEVVERBOSE("attach card\n");
    
    IOLockLock(sc_lock);
	CLR(sc_flags, SMF_CARD_ATTACHED);

	/*
	 * Power up the card (or card stack).
	 */
	if (enable() != 0) {
		DEVERR("can't enable card\n");
		goto err;
	}

	/*
	 * Scan for I/O functions and memory cards on the bus,
	 * allocating a sdmmc_function structure for each.
	 */
	if (scan() != 0) {
		DEVERR("no functions\n");
		goto err;
	}

	/*
	 * Initialize the I/O functions and memory cards.
	 */
	if (init() != 0) {
		DEVERR("init failed\n");
		goto err;
	}

    /* Attach SCSI emulation for memory cards. */
	if (ISSET(sc_flags, SMF_MEM_MODE))
		scsi_attach();

	SET(sc_flags, SMF_CARD_ATTACHED);
    IOLockUnlock(sc_lock);
	return;

err:
    card_detach(true); // force
    IOLockUnlock(sc_lock);
}

// Called when cards are gone.
void macSDMMC::card_detach(bool force)
{
    struct sdmmc_function *sf, *sfnext;

    DEVVERBOSE("detach card\n");

    if (ISSET(sc_flags, SMF_CARD_ATTACHED))
    {
        /* Detach I/O function drivers. */
        if (ISSET(sc_flags, SMF_IO_MODE))
        {
            DEVERR("sdmmc_io_detach not impl.\n");
            // sdmmc_io_detach(sc);
        }

        /* Detach the SCSI emulation for memory cards. */
        if (ISSET(sc_flags, SMF_MEM_MODE))
        {
            scsi_detach();
        }
        CLR(sc_flags, SMF_CARD_ATTACHED);
    }

    /* Power down. */
    disable();

    /* Free all sdmmc_function structures. */
    for (sf = STAILQ_FIRST(&sf_head); sf != NULL; sf = sfnext)
    {
        sfnext = STAILQ_NEXT(sf, sf_list);
        function_free(sf);
    }
    STAILQ_INIT(&sf_head);
    sc_function_count = 0;
    sc_fn0 = NULL;
}

int macSDMMC::enable()
{
    u_int32_t host_ocr;
    int error;

    /*
	 * Calculate the equivalent of the card OCR from the host
	 * capabilities and select the maximum supported bus voltage.
	 */
    host_ocr = rtsx_host_ocr();
    error = rtsx_bus_power((struct rtsx_softc *)sch, host_ocr);
    if (error != 0)
    {
        DEVERR("can't supply bus power\n");
        goto err;
    }

    /*
	 * Select the minimum clock frequency.
	 */
    error = rtsx_bus_clock((struct rtsx_softc *)sch, SDMMC_SDCLK_400KHZ, SDMMC_TIMING_LEGACY);
    if (error != 0)
    {
        DEVERR("can't supply clock\n");
        goto err;
    }

    /* XXX wait for card to power up */
    // sinetek: for some reason my cards take longer to power up. bumping up.
    delay(500000);

    // Skip looking for SDIO cards.
	SET(sc_flags, SMF_SD_MODE|SMF_IO_MODE|SMF_MEM_MODE);
    CLR(sc_flags, SMF_IO_MODE);

	/* Initialize SD/MMC memory card(s). */
	if (ISSET(sc_flags, SMF_MEM_MODE) &&
	    (error = mem_enable()) != 0)
		goto err;

err:
    if (error != 0)
        disable();

    return error;
}

void macSDMMC::disable()
{
    /* XXX complete commands if card is still present. */

    /* Make sure no card is still selected. */
    (void)select_card(NULL);

    /* Turn off bus power and clock. */
    (void)rtsx_bus_clock((struct rtsx_softc *)sch,
                         SDMMC_SDCLK_OFF, SDMMC_TIMING_LEGACY);
    (void)rtsx_bus_power((struct rtsx_softc *)sch, 0);
}

/*
 * Initialize all the distinguished functions of the card, be it I/O
 * or memory functions.
 */
int macSDMMC::init()
{
    struct sdmmc_function *sf;

    /* Initialize all identified card functions. */
    STAILQ_FOREACH(sf, &sf_head, sf_list)
    {
        if (ISSET(sc_flags, SMF_IO_MODE))
        {
            // No support.
            //     sdmmc_io_init(sc, sf) != 0)
            DEVERR("i/o init failed\n");
        }

        if (ISSET(sc_flags, SMF_MEM_MODE) &&
            mem_init(sf) != 0)
            DEVERR("mem init failed\n");
    }

    /* Any good functions left after initialization? */
    STAILQ_FOREACH(sf, &sf_head, sf_list)
    {
        if (!ISSET(sf->flags, SFF_ERROR))
            return 0;
    }
    /* No, we should probably power down the card. */
    return 1;
}

void macSDMMC::delay(u_int usecs)
{
    IODelay(usecs);
}

/*
 * Set the lowest bus voltage supported by the card and the host.
 */
int macSDMMC::set_bus_power(u_int32_t host_ocr, u_int32_t card_ocr)
{
	u_int32_t bit;

	/* Mask off unsupported voltage levels and select the lowest. */
    DEVVERBOSE("host_ocr=%x ", host_ocr);
	host_ocr &= card_ocr;
	for (bit = 4; bit < 23; bit++) {
		if (ISSET(host_ocr, 1<<bit)) {
			host_ocr &= 3<<bit;
			break;
		}
	}
    DEVVERBOSE("card_ocr=%x new_ocr=%x\n", card_ocr, host_ocr);

	if (host_ocr == 0 ||
	    rtsx_bus_power((struct rtsx_softc *)sch, host_ocr) != 0)
		return 1;
	return 0;
}

int macSDMMC::scan()
{
    /* Scan for memory cards on the bus. */
    if (ISSET(sc_flags, SMF_MEM_MODE))
        mem_scan();

    /* There should be at least one function now. */
    if (STAILQ_EMPTY(&sf_head))
    {
        printf("can't identify card\n");
        return 1;
    }
    return 0;
}

int macSDMMC::app_command(struct sdmmc_command *cmd)
{
    struct sdmmc_command acmd;
    int error;

    bzero(&acmd, sizeof acmd);
    acmd.c_opcode = MMC_APP_CMD;
    acmd.c_arg = 0;
    if (sc_card != NULL)
    {
        acmd.c_arg = sc_card->rca << 16;
    }
    acmd.c_flags = SCF_CMD_AC | SCF_RSP_R1;

    error = mmc_command(&acmd);
    if (error != 0)
    {
        return error;
    }

    if (!ISSET(MMC_R1(acmd.c_resp), MMC_R1_APP_CMD))
    {
        /* Card does not support application commands. */
        return ENODEV;
    }

    error = mmc_command(cmd);
    return error;
}

/*
 * Execute MMC command and data transfers.  All interactions with the
 * host controller to complete the command happen in the context of
 * the current process.
 */
int macSDMMC::mmc_command(struct sdmmc_command *cmd)
{
	int error;

	rtsx_exec_command((struct rtsx_softc *)sch, cmd);

    // Let's assume verbose output.
 	dump_command(cmd);

	error = cmd->c_error;
	wakeup(cmd);

	return error;
}

/*
 * Send the "GO IDLE STATE" command.
 */
void macSDMMC::go_idle_state()
{
	struct sdmmc_command cmd;

	bzero(&cmd, sizeof cmd);
	cmd.c_opcode = MMC_GO_IDLE_STATE;
	cmd.c_flags = SCF_CMD_BC | SCF_RSP_R0;

	(void)mmc_command(&cmd);
}

/*
 * Send the "SEND_IF_COND" command, to check operating condition
 */
int macSDMMC::send_if_cond(uint32_t card_ocr)
{
	struct sdmmc_command cmd;
	uint8_t pat = 0x23;	/* any pattern will do here */
	uint8_t res;

	bzero(&cmd, sizeof cmd);

	cmd.c_opcode = SD_SEND_IF_COND;
	cmd.c_arg = ((card_ocr & SD_OCR_VOL_MASK) != 0) << 8 | pat;
	cmd.c_flags = SCF_CMD_BCR | SCF_RSP_R7;

	if (mmc_command(&cmd) != 0)
		return 1;

	res = cmd.c_resp[0];
	if (res != pat)
		return 1;
	else
		return 0;
}

/*
 * Retrieve (SD) or set (MMC) the relative card address (RCA).
 */
int
macSDMMC::set_relative_addr(struct sdmmc_function *sf)
{
	struct sdmmc_command cmd;

	bzero(&cmd, sizeof cmd);

	if (ISSET(sc_flags, SMF_SD_MODE)) {
		cmd.c_opcode = SD_SEND_RELATIVE_ADDR;
		cmd.c_flags = SCF_CMD_BCR | SCF_RSP_R6;
	} else {
		cmd.c_opcode = MMC_SET_RELATIVE_ADDR;
		cmd.c_arg = MMC_ARG_RCA(sf->rca);
		cmd.c_flags = SCF_CMD_AC | SCF_RSP_R1;
	}

	if (mmc_command(&cmd) != 0)
		return 1;

	if (ISSET(sc_flags, SMF_SD_MODE))
		sf->rca = SD_R6_RCA(cmd.c_resp);
	return 0;
}

int macSDMMC::select_card(struct sdmmc_function *sf)
{
    struct sdmmc_command cmd;
    int error;
    
    if (sc_card == sf || (sf && sc_card &&
                              sc_card->rca == sf->rca)) {
        sc_card = sf;
        return 0;
    }
    
    bzero(&cmd, sizeof cmd);
    cmd.c_opcode = MMC_SELECT_CARD;
    cmd.c_arg = sf == NULL ? 0 : MMC_ARG_RCA(sf->rca);
    cmd.c_flags = SCF_CMD_AC | (sf == NULL ? SCF_RSP_R0 : SCF_RSP_R1);
    error = mmc_command(&cmd);
    if (error == 0 || sf == NULL)
        sc_card = sf;
    return error;
}

void macSDMMC::dump_command(struct sdmmc_command *cmd)
{
    int i;

    // DPRINTF(1,("%s: cmd %u arg=%#x data=%p dlen=%d flags=%#x "
    //     "proc=\"%s\" (error %d)\n", DEVNAME(sc), cmd->c_opcode,
    //     cmd->c_arg, cmd->c_data, cmd->c_datalen, cmd->c_flags,
    //     curproc ? curproc->p_p->ps_comm : "", cmd->c_error));

    // XXX this printf will require variadic DEVVERBOSE..

    printf("cmd %u arg=%#x data=%p dlen=%d flags=%#x "
           "proc=\"%s\" (error %d)\n",
           cmd->c_opcode,
           cmd->c_arg, cmd->c_data, cmd->c_datalen, cmd->c_flags,
           "kernel", cmd->c_error);

    if (cmd->c_error)
        return;

    printf("resp=");
    if (ISSET(cmd->c_flags, SCF_RSP_136))
        for (i = 0; i < sizeof cmd->c_resp; i++)
            printf("%02x ", ((u_char *)cmd->c_resp)[i]);
    else if (ISSET(cmd->c_flags, SCF_RSP_PRESENT))
        for (i = 0; i < 4; i++)
            printf("%02x ", ((u_char *)cmd->c_resp)[i]);
    printf("\n");
}

struct sdmmc_function *
macSDMMC::function_alloc()
{
    struct sdmmc_function *sf;

    sf = (struct sdmmc_function *)IOMalloc(sizeof *sf);
    sf->sc = this;
    sf->number = -1;
    sf->cis.manufacturer = SDMMC_VENDOR_INVALID;
    sf->cis.product = SDMMC_PRODUCT_INVALID;
    sf->cis.function = SDMMC_FUNCTION_INVALID;
    return sf;
}

void macSDMMC::function_free(struct sdmmc_function *sf)
{
    IOFree(sf, sizeof *sf);
}

//===================================================================
//  ____  ____  __  __ __  __  ____   __  __ _____ __  __
// / ___||  _ \|  \/  |  \/  |/ ___| |  \/  | ____|  \/  |
// \___ \| | | | |\/| | |\/| | |     | |\/| |  _| | |\/| |
//  ___) | |_| | |  | | |  | | |___  | |  | | |___| |  | |
// |____/|____/|_|  |_|_|  |_|\____| |_|  |_|_____|_|  |_|

/*
 * Initialize SD/MMC memory cards and memory in SDIO "combo" cards.
 */
int macSDMMC::mem_enable()
{
    u_int32_t host_ocr;
    u_int32_t card_ocr;

    /* Set host mode to SD "combo" card or SD memory-only. */
    SET(sc_flags, SMF_SD_MODE | SMF_MEM_MODE);

    /* Reset memory (*must* do that before CMD55 or CMD1). */
    go_idle_state();

    /*
	 * Read the SD/MMC memory OCR value by issuing CMD55 followed
	 * by ACMD41 to read the OCR value from memory-only SD cards.
	 * MMC cards will not respond to CMD55 or ACMD41 and this is
	 * how we distinguish them from SD cards.
	 */
mmc_mode:
    if (mem_send_op_cond(0, &card_ocr) != 0)
    {
        if (ISSET(sc_flags, SMF_SD_MODE) &&
            !ISSET(sc_flags, SMF_IO_MODE))
        {
            /* Not a SD card, switch to MMC mode. */
            CLR(sc_flags, SMF_SD_MODE);
            goto mmc_mode;
        }
        if (!ISSET(sc_flags, SMF_SD_MODE))
        {
            DEVERR("can't read memory OCR\n");
            return 1;
        }
        else
        {
            /* Not a "combo" card. */
            CLR(sc_flags, SMF_MEM_MODE);
            return 0;
        }
    }

    /* Set the lowest voltage supported by the card and host. */
    host_ocr = rtsx_host_ocr();
    if (set_bus_power(host_ocr, card_ocr) != 0)
    {
        DEVERR("can't supply voltage requested by card\n");
        return 1;
    }

    /* Tell the card(s) to enter the idle state (again). */
    go_idle_state();

    host_ocr &= card_ocr; /* only allow the common voltages */

    if (send_if_cond(card_ocr) == 0)
        host_ocr |= SD_OCR_SDHC_CAP;

    /* Send the new OCR value until all cards are ready. */
    if (mem_send_op_cond(host_ocr, NULL) != 0)
    {
        DEVERR("can't send memory OCR\n");
        return 1;
    }
    return 0;
}

void macSDMMC::mem_scan()
{
	struct sdmmc_command cmd;
	struct sdmmc_function *sf;
	u_int16_t next_rca;
	int error;
	int i;

	/*
	 * CMD2 is a broadcast command understood by SD cards and MMC
	 * cards.  All cards begin to respond to the command, but back
	 * off if another card drives the CMD line to a different level.
	 * Only one card will get its entire response through.  That
	 * card remains silent once it has been assigned a RCA.
	 */
	for (i = 0; i < 100; i++) {
		bzero(&cmd, sizeof cmd);
		cmd.c_opcode = MMC_ALL_SEND_CID;
		cmd.c_flags = SCF_CMD_BCR | SCF_RSP_R2;

		error = mmc_command(&cmd);
		if (error == ETIMEDOUT) {
			/* No more cards there. */
			break;
		} else if (error != 0) {
            DEVERR("can't read CID\n");
			break;
		}
        
		/* In MMC mode, find the next available RCA. */
		next_rca = 1;
		if (!ISSET(sc_flags, SMF_SD_MODE))
			STAILQ_FOREACH(sf, &sf_head, sf_list)
				next_rca++;

		/* Allocate a sdmmc_function structure. */
		sf = function_alloc();
		sf->rca = next_rca;

		/*
		 * Remember the CID returned in the CMD2 response for
		 * later decoding.
		 */
		bcopy(cmd.c_resp, sf->raw_cid, sizeof sf->raw_cid);

		/*
		 * Silence the card by assigning it a unique RCA, or
		 * querying it for its RCA in the case of SD.
		 */
		if (set_relative_addr(sf) != 0) {
			printf("can't set mem RCA\n");
			function_free(sf);
			break;
		}
        
#if 0
		/* Verify that the RCA has been set by selecting the card. */
		if (sdmmc_select_card(sc, sf) != 0) {
			printf("%s: can't select mem RCA %d\n",
			    DEVNAME(sc), sf->rca);
			sdmmc_function_free(sf);
			break;
		}

		/* Deselect. */
		(void)sdmmc_select_card(sc, NULL);
#endif

		/*
		 * If this is a memory-only card, the card responding
		 * first becomes an alias for SDIO function 0.
		 */
		if (sc_fn0 == NULL)
			sc_fn0 = sf;
        
		STAILQ_INSERT_TAIL(&sf_head, sf, sf_list);
	}

	/*
	 * All cards are either inactive or awaiting further commands.
	 * Read the CSDs and decode the raw CID for each card.
	 */
	STAILQ_FOREACH(sf, &sf_head, sf_list) {
		bzero(&cmd, sizeof cmd);
		cmd.c_opcode = MMC_SEND_CSD;
		cmd.c_arg = MMC_ARG_RCA(sf->rca);
		cmd.c_flags = SCF_CMD_AC | SCF_RSP_R2;

		if (mmc_command(&cmd) != 0) {
			SET(sf->flags, SFF_ERROR);
			continue;
		}

		if (decode_csd(cmd.c_resp, sf) != 0 ||
		    decode_cid(sf->raw_cid, sf) != 0) {
			SET(sf->flags, SFF_ERROR);
			continue;
		}

        // Assume verbosity. XXX will need variadic for DEVVERBOSE.
		printf("CID: ");
        printf("mid=0x%02x oid=0x%04x pnm=\"%s\" rev=0x%02x psn=0x%08x"
               " mdt=%03x\n", sf->cid.mid, sf->cid.oid, sf->cid.pnm, sf->cid.rev, sf->cid.psn,
               sf->cid.mdt);
	}
}

int macSDMMC::decode_csd(sdmmc_response resp,
                               struct sdmmc_function *sf)
{
    struct sdmmc_csd *csd = &sf->csd;

    if (ISSET(sc_flags, SMF_SD_MODE))
    {
        /*
		 * CSD version 1.0 corresponds to SD system
		 * specification version 1.0 - 1.10. (SanDisk, 3.5.3)
		 */
        csd->csdver = SD_CSD_CSDVER(resp);
        switch (csd->csdver)
        {
        case SD_CSD_CSDVER_2_0:
            sf->flags |= SFF_SDHC;
            csd->capacity = SD_CSD_V2_CAPACITY(resp);
            csd->read_bl_len = SD_CSD_V2_BL_LEN;
            break;
        case SD_CSD_CSDVER_1_0:
            csd->capacity = SD_CSD_CAPACITY(resp);
            csd->read_bl_len = SD_CSD_READ_BL_LEN(resp);
            break;
        default:
            DEVVERBOSE("unknown SD CSD structure version 0x%x\n",
                   csd->csdver);
            return 1;
            break;
        }
        csd->ccc = SD_CSD_CCC(resp);
    }
    else
    {
        csd->csdver = MMC_CSD_CSDVER(resp);
        if (csd->csdver == MMC_CSD_CSDVER_1_0 ||
            csd->csdver == MMC_CSD_CSDVER_2_0 ||
            csd->csdver == MMC_CSD_CSDVER_EXT_CSD)
        {
            csd->mmcver = MMC_CSD_MMCVER(resp);
            csd->capacity = MMC_CSD_CAPACITY(resp);
            csd->read_bl_len = MMC_CSD_READ_BL_LEN(resp);
        }
        else
        {
            DEVVERBOSE("unknown MMC CSD structure version 0x%x\n",
                   csd->csdver);
            return 1;
        }
    }
    csd->sector_size = MIN(1 << csd->read_bl_len,
                           rtsx_host_maxblklen());
    if (csd->sector_size < (1 << csd->read_bl_len))
        csd->capacity *= (1 << csd->read_bl_len) /
                         csd->sector_size;

    return 0;
}

int macSDMMC::decode_cid(sdmmc_response resp,
                         struct sdmmc_function *sf)
{
    struct sdmmc_cid *cid = &sf->cid;

    if (ISSET(sc_flags, SMF_SD_MODE))
    {
        cid->mid = SD_CID_MID(resp);
        cid->oid = SD_CID_OID(resp);
        SD_CID_PNM_CPY(resp, cid->pnm);
        cid->rev = SD_CID_REV(resp);
        cid->psn = SD_CID_PSN(resp);
        cid->mdt = SD_CID_MDT(resp);
    }
    else
    {
        switch (sf->csd.mmcver)
        {
        case MMC_CSD_MMCVER_1_0:
        case MMC_CSD_MMCVER_1_4:
            cid->mid = MMC_CID_MID_V1(resp);
            MMC_CID_PNM_V1_CPY(resp, cid->pnm);
            cid->rev = MMC_CID_REV_V1(resp);
            cid->psn = MMC_CID_PSN_V1(resp);
            cid->mdt = MMC_CID_MDT_V1(resp);
            break;
        case MMC_CSD_MMCVER_2_0:
        case MMC_CSD_MMCVER_3_1:
        case MMC_CSD_MMCVER_4_0:
            cid->mid = MMC_CID_MID_V2(resp);
            cid->oid = MMC_CID_OID_V2(resp);
            MMC_CID_PNM_V2_CPY(resp, cid->pnm);
            cid->psn = MMC_CID_PSN_V2(resp);
            break;
        default:
            DEVVERBOSE("%s: unknown MMC version %d\n",
                   sf->csd.mmcver);
            return 1;
        }
    }
    return 0;
}

/*
 * Get or set the card's memory OCR value (SD or MMC).
 */
int macSDMMC::mem_send_op_cond(u_int32_t ocr,
                               u_int32_t *ocrp)
{
    struct sdmmc_command cmd;
    int error;
    int i;

    /*
	 * If we change the OCR value, retry the command until the OCR
	 * we receive in response has the "CARD BUSY" bit set, meaning
	 * that all cards are ready for identification.
	 */
    for (i = 0; i < 100; i++)
    {
        bzero(&cmd, sizeof cmd);
        cmd.c_arg = ocr;
        cmd.c_flags = SCF_CMD_BCR | SCF_RSP_R3;

        if (ISSET(sc_flags, SMF_SD_MODE))
        {
            cmd.c_opcode = SD_APP_OP_COND;
            error = app_command(&cmd);
        }
        else
        {
            cmd.c_arg &= ~MMC_OCR_ACCESS_MODE_MASK;
            cmd.c_arg |= MMC_OCR_SECTOR_MODE;
            cmd.c_opcode = MMC_SEND_OP_COND;
            error = mmc_command(&cmd);
        }
        if (error != 0)
            break;
        if (ISSET(MMC_R3(cmd.c_resp), MMC_OCR_MEM_READY) ||
            ocr == 0)
            break;
        error = ETIMEDOUT;
        delay(10000);
    }
    if (error == 0 && ocrp != NULL)
        *ocrp = MMC_R3(cmd.c_resp);

    return error;
}

/*
 * Initialize a SD/MMC memory card.
 */
int macSDMMC::mem_init(struct sdmmc_function *sf)
{
    int error = 0;

    if (select_card(sf) != 0 ||
        mem_set_blocklen(sf) != 0)
        error = 1;

    if (ISSET(sc_flags, SMF_SD_MODE))
        error = mem_sd_init(sf);
    else
        error = mem_mmc_init(sf);

    return error;
}

/*
 * Set the read block length appropriately for this card, according to
 * the card CSD register value.
 */
int macSDMMC::mem_set_blocklen(struct sdmmc_function *sf)
{
	struct sdmmc_command cmd;

	bzero(&cmd, sizeof cmd);
	cmd.c_opcode = MMC_SET_BLOCKLEN;
	cmd.c_arg = sf->csd.sector_size;
	cmd.c_flags = SCF_CMD_AC | SCF_RSP_R1;
	DEVVERBOSE("read_bl_len=%d sector_size=%d\n",
	    1 << sf->csd.read_bl_len, sf->csd.sector_size);

	return mmc_command(&cmd);
}

int macSDMMC::mem_sd_init(struct sdmmc_function *sf)
{
	int support_func, best_func, error;
	sdmmc_bitfield512_t status; /* Switch Function Status */
	uint32_t raw_scr[2];

	/*
	 * All SD cards are supposed to support Default Speed mode
	 * with frequencies up to 25 MHz.  Bump up the clock frequency
	 * now as data transfers don't seem to work on the Realtek
	 * RTS5229 host controller if it is running at a low clock
	 * frequency.  Reading the SCR requires a data transfer.
	 */
	error = rtsx_bus_clock((struct rtsx_softc *)sch, SDMMC_SDCLK_25MHZ,
	    SDMMC_TIMING_LEGACY);
	if (error) {
		DEVERR("can't change bus clock\n");
		return error;
	}

	error = mem_send_scr(raw_scr);
	if (error) {
		DEVERR("%s: SD_SEND_SCR send failed\n");
		return error;
	}
	error = mem_decode_scr(raw_scr, sf);
	if (error)
		return error;

	if (ISSET(sc_caps, SMC_CAPS_4BIT_MODE) &&
	    ISSET(sf->scr.bus_width, SCR_SD_BUS_WIDTHS_4BIT)) {
		DEVVERBOSE("change bus width\n");
		error = set_bus_width(sf, 4);
		if (error) {
			DEVERR("can't change bus width\n");
			return error;
		}
	}

	best_func = 0;
	if (sf->scr.sd_spec >= SCR_SD_SPEC_VER_1_10 &&
	    ISSET(sf->csd.ccc, SD_CSD_CCC_SWITCH)) {
		DEVVERBOSE("switch func mode 0\n");
		error = mem_sd_switch(sf, 0, 1, 0, &status);
		if (error) {
			DEVERR("switch func mode 0 failed\n");
			return error;
		}

		support_func = SFUNC_STATUS_GROUP(&status, 1);

		if (support_func & (1 << SD_ACCESS_MODE_SDR25))
			best_func = 1;
	}

	if (best_func != 0) {
		DEVVERBOSE("switch func mode 1(func=%d)\n", best_func);
		error =
		    mem_sd_switch(sf, 1, 1, best_func, &status);
		if (error) {
			DEVERR("switch func mode 1 failed: group 1 function %d(0x%2x)\n",
			    best_func, support_func);
			return error;
		}

		/* Wait 400KHz x 8 clock (2.5us * 8 + slop) */
		delay(25);

		/* High Speed mode, Frequency up to 50MHz. */
		error = rtsx_bus_clock((struct rtsx_softc *)sch,
		    SDMMC_SDCLK_50MHZ, SDMMC_TIMING_HIGHSPEED);
		if (error) {
			DEVERR("can't change bus clock\n");
			return error;
		}
	}

	return 0;
}

int macSDMMC::mem_mmc_init(struct sdmmc_function *sf)
{
	int width, value;
	int card_type;
	int error = 0;
	u_int8_t ext_csd[512];
	int speed = 20000;
	int timing = SDMMC_TIMING_LEGACY;
	u_int32_t sectors = 0;

	if (sf->csd.mmcver >= MMC_CSD_MMCVER_4_0) {
		/* read EXT_CSD */
		error = mem_send_cxd_data(MMC_SEND_EXT_CSD, ext_csd, sizeof(ext_csd));
		if (error != 0) {
			SET(sf->flags, SFF_ERROR);
			DEVERR("can't read EXT_CSD\n");
			return error;
		}

		card_type = ext_csd[EXT_CSD_CARD_TYPE];

		if (card_type & EXT_CSD_CARD_TYPE_F_52M_1_8V &&
		    ISSET(sc_caps, SMC_CAPS_MMC_DDR52)) {
			speed = 52000;
			timing = SDMMC_TIMING_MMC_DDR52;
		} else if (card_type & EXT_CSD_CARD_TYPE_F_52M &&
		    ISSET(sc_caps, SMC_CAPS_MMC_HIGHSPEED)) {
			speed = 52000;
			timing = SDMMC_TIMING_HIGHSPEED;
		} else if (card_type & EXT_CSD_CARD_TYPE_F_26M) {
			speed = 26000;
		} else {
			DEVERR("unknown CARD_TYPE 0x%x\n", ext_csd[EXT_CSD_CARD_TYPE]);
		}

		if (timing != SDMMC_TIMING_LEGACY) {
			/* switch to high speed timing */
			error = mem_mmc_switch(sf, EXT_CSD_CMD_SET_NORMAL,
			    EXT_CSD_HS_TIMING, EXT_CSD_HS_TIMING_HS);
			if (error != 0) {
				DEVERR("can't change high speed\n");
				return error;
			}

			delay(10000);
		}

		error = rtsx_bus_clock((struct rtsx_softc *)sch, speed, SDMMC_TIMING_HIGHSPEED);
		if (error != 0) {
			DEVERR("can't change bus clock\n");
			return error;
		}

		if (timing != SDMMC_TIMING_LEGACY) {
			/* read EXT_CSD again */
			error = mem_send_cxd_data(MMC_SEND_EXT_CSD, ext_csd, sizeof(ext_csd));
			if (error != 0) {
				DEVERR("can't re-read EXT_CSD\n");
				return error;
			}
			if (ext_csd[EXT_CSD_HS_TIMING] != EXT_CSD_HS_TIMING_HS) {
				DEVERR("HS_TIMING set failed\n");
				return EINVAL;
			}
		}

		if (ISSET(sc_caps, SMC_CAPS_8BIT_MODE)) {
			width = 8;
			value = EXT_CSD_BUS_WIDTH_8;
		} else if (ISSET(sc_caps, SMC_CAPS_4BIT_MODE)) {
			width = 4;
			value = EXT_CSD_BUS_WIDTH_4;
		} else {
			width = 1;
			value = EXT_CSD_BUS_WIDTH_1;
		}

		if (width != 1) {
			error = mem_mmc_switch(sf, EXT_CSD_CMD_SET_NORMAL,
			    EXT_CSD_BUS_WIDTH, value);
			if (error == 0)
				error = rtsx_set_bus_width((struct rtsx_softc *)sch, width);
			else {
				DEVERR("can't change bus width"
				    " (%d bit)\n", width);
				return error;
			}

			/* XXXX: need bus test? (using by CMD14 & CMD19) */
			delay(10000);
		}

		if (timing == SDMMC_TIMING_MMC_DDR52) {
			switch (width) {
			case 4:
				value = EXT_CSD_BUS_WIDTH_4_DDR;
				break;
			case 8:
				value = EXT_CSD_BUS_WIDTH_8_DDR;
				break;
			}

			error = mem_mmc_switch(sf, EXT_CSD_CMD_SET_NORMAL,
			    EXT_CSD_BUS_WIDTH, value);
			if (error) {
				DEVERR("can't switch to DDR\n");
				return error;
			}

			delay(10000);

//			error = sdmmc_chip_signal_voltage((struct rtsx_softc *)sch,
//			    SDMMC_SIGNAL_VOLTAGE_180);
            error = EINVAL;
            // XXX not implemented.
			if (error) {
				DEVERR("can't switch signalling voltage\n");
				return error;
			}

			error = rtsx_bus_clock((struct rtsx_softc *)sch, speed, timing);
			if (error != 0) {
				DEVERR("can't change bus clock\n");
				return error;
			}

			delay(10000);
		}

		sectors = ext_csd[EXT_CSD_SEC_COUNT + 0] << 0 |
		    ext_csd[EXT_CSD_SEC_COUNT + 1] << 8  |
		    ext_csd[EXT_CSD_SEC_COUNT + 2] << 16 |
		    ext_csd[EXT_CSD_SEC_COUNT + 3] << 24;

		if (sectors > (2u * 1024 * 1024 * 1024) / 512) {
			sf->flags |= SFF_SDHC;
			sf->csd.capacity = sectors;
		}
	}

	return error;
}

int macSDMMC::mem_send_scr(uint32_t *scr)
{
	struct sdmmc_command cmd;
	void *ptr = NULL;
	int datalen = 8;
	int error = 0;

    ptr = IOMalloc(datalen);
    bzero(ptr, datalen);
	if (ptr == NULL)
		goto out;

	memset(&cmd, 0, sizeof(cmd));
	cmd.c_data = ptr;
	cmd.c_datalen = datalen;
	cmd.c_blklen = datalen;
	cmd.c_arg = 0;
	cmd.c_flags = SCF_CMD_ADTC | SCF_CMD_READ | SCF_RSP_R1;
	cmd.c_opcode = SD_APP_SEND_SCR;

	error = app_command(&cmd);
	if (error == 0)
		memcpy(scr, ptr, datalen);

out:
	if (ptr != NULL)
        IOFree(ptr, datalen);

	return error;
}

int macSDMMC::mem_decode_scr(uint32_t *raw_scr,
    struct sdmmc_function *sf)
{
	sdmmc_response resp;
	int ver;

	memset(resp, 0, sizeof(resp));
	/*
	 * Change the raw SCR to a response.
	 */
    resp[0] = OSSwapBigToHostInt32(raw_scr[1]) >> 8; // LSW
    resp[1] = OSSwapBigToHostInt32(raw_scr[0]);      // MSW
    resp[0] |= (resp[1] & 0xff) << 24;
	resp[1] >>= 8;

	ver = SCR_STRUCTURE(resp);
	sf->scr.sd_spec = SCR_SD_SPEC(resp);
	sf->scr.bus_width = SCR_SD_BUS_WIDTHS(resp);

    // XXX printf
	printf("%s: %08x%08x ver=%d, spec=%d, bus width=%d\n",
	    __func__, resp[1], resp[0],
	    ver, sf->scr.sd_spec, sf->scr.bus_width);

	if (ver != 0) {
		DEVERR("unknown SCR structure version: %d\n", ver);
		return EINVAL;
	}
	return 0;
}

int macSDMMC::set_bus_width(struct sdmmc_function *sf, int width)
{
	struct sdmmc_command cmd;
	int error;

	memset(&cmd, 0, sizeof(cmd));
	cmd.c_opcode = SD_APP_SET_BUS_WIDTH;
	cmd.c_flags = SCF_RSP_R1 | SCF_CMD_AC;

	switch (width) {
	case 1:
		cmd.c_arg = SD_ARG_BUS_WIDTH_1;
		break;

	case 4:
		cmd.c_arg = SD_ARG_BUS_WIDTH_4;
		break;

	default:
		return EINVAL;
	}

	error = app_command(&cmd);
	if (error == 0)
		error = rtsx_set_bus_width((struct rtsx_softc *)sch, width);
	return error;
}

/* make 512-bit BE quantity __bitfield()-compatible */
static void
sdmmc_be512_to_bitfield512(sdmmc_bitfield512_t *buf) {
#define nitems(_a)	(sizeof((_a)) / sizeof((_a)[0]))
	size_t i;
	uint32_t tmp0, tmp1;
	const size_t bitswords = nitems(buf->_bits);
	for (i = 0; i < bitswords/2; i++) {
		tmp0 = buf->_bits[i];
		tmp1 = buf->_bits[bitswords - 1 - i];
		buf->_bits[i] = OSSwapBigToHostInt32(tmp1);
		buf->_bits[bitswords - 1 - i] = OSSwapBigToHostInt32(tmp0);
	}
#undef nitems
}

int macSDMMC::mem_sd_switch(struct sdmmc_function *sf, int mode, int group,
                            int function, sdmmc_bitfield512_t *status)
{
    struct sdmmc_command cmd;
    void *ptr = NULL;
    int gsft, error = 0;
    const int statlen = 64;

    if (sf->scr.sd_spec >= SCR_SD_SPEC_VER_1_10 &&
        !ISSET(sf->csd.ccc, SD_CSD_CCC_SWITCH))
        return EINVAL;

    if (group <= 0 || group > 6 ||
        function < 0 || function > 15)
        return EINVAL;

    gsft = (group - 1) << 2;

    ptr = IOMalloc(statlen);
    if (ptr == NULL)
        goto out;
    bzero(ptr, statlen);

    memset(&cmd, 0, sizeof(cmd));
    cmd.c_data = ptr;
    cmd.c_datalen = statlen;
    cmd.c_blklen = statlen;
    cmd.c_opcode = SD_SEND_SWITCH_FUNC;
    cmd.c_arg =
        (!!mode << 31) | (function << gsft) | (0x00ffffff & ~(0xf << gsft));
    cmd.c_flags = SCF_CMD_ADTC | SCF_CMD_READ | SCF_RSP_R1;

    error = mmc_command(&cmd);
    if (error == 0)
        memcpy(status, ptr, statlen);

out:
    if (ptr != NULL)
        IOFree(ptr, statlen);

    if (error == 0)
        sdmmc_be512_to_bitfield512(status);

    return error;
}

int macSDMMC::mem_send_cxd_data(int opcode, void *data,
                                size_t datalen)
{
    struct sdmmc_command cmd;
    void *ptr = NULL;
    int error = 0;

    ptr = IOMalloc(datalen);
    if (ptr == NULL)
    {
        error = ENOMEM;
        goto out;
    }
    bzero(ptr, datalen);

    memset(&cmd, 0, sizeof(cmd));
    cmd.c_data = ptr;
    cmd.c_datalen = datalen;
    cmd.c_blklen = datalen;
    cmd.c_opcode = opcode;
    cmd.c_arg = 0;
    cmd.c_flags = SCF_CMD_ADTC | SCF_CMD_READ;
    if (opcode == MMC_SEND_EXT_CSD)
        SET(cmd.c_flags, SCF_RSP_R1);
    else
        SET(cmd.c_flags, SCF_RSP_R2);

    error = mmc_command(&cmd);
    if (error == 0)
        memcpy(data, ptr, datalen);

out:
    if (ptr != NULL)
        IOFree(ptr, datalen);

    return error;
}

int macSDMMC::mem_mmc_switch(struct sdmmc_function *sf, uint8_t set, uint8_t index,
                             uint8_t value)
{
    struct sdmmc_command cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.c_opcode = MMC_SWITCH;
    cmd.c_arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
                (index << 16) | (value << 8) | set;
    cmd.c_flags = SCF_RSP_R1B | SCF_CMD_AC;

    return mmc_command(&cmd);
}

int macSDMMC::mem_read_block(struct sdmmc_function *sf, int blkno, u_char *data,
                             size_t datalen)
{
	int error;

    IOLockLock(sc_lock);
    
	if (ISSET(sc_caps, SMC_CAPS_SINGLE_ONLY)) {
		error = mem_single_read_block(sf, blkno, data, datalen);
		goto out;
	}

	if (!ISSET(sc_caps, SMC_CAPS_DMA)) {
		error = mem_read_block_subr(sf, blkno,
		    data, datalen);
		goto out;
	}

	/* DMA transfer */
    // XXX no support.
unload:
    error = EINVAL;
    
out:
    IOLockUnlock(sc_lock);
	return (error);
}

int macSDMMC::mem_single_read_block(struct sdmmc_function *sf, int blkno, u_char *data,
                                          size_t datalen)
{
    int error = 0;
    int i;

    for (i = 0; i < datalen / sf->csd.sector_size; i++)
    {
        error = mem_read_block_subr(sf, blkno + i,
                                          data + i * sf->csd.sector_size, sf->csd.sector_size);
        if (error)
            break;
    }

    return (error);
}

int macSDMMC::mem_read_block_subr(struct sdmmc_function *sf, int blkno, u_char *data, size_t datalen)
{
	struct sdmmc_command cmd;
	int error;

	if ((error = select_card(sf)) != 0)
		goto err;

	bzero(&cmd, sizeof cmd);
	cmd.c_data = data;
	cmd.c_datalen = datalen;
	cmd.c_blklen = sf->csd.sector_size;
	cmd.c_opcode = (datalen / cmd.c_blklen) > 1 ?
	    MMC_READ_BLOCK_MULTIPLE : MMC_READ_BLOCK_SINGLE;
	if (sf->flags & SFF_SDHC)
		cmd.c_arg = blkno;
	else
		cmd.c_arg = blkno << 9;
	cmd.c_flags = SCF_CMD_ADTC | SCF_CMD_READ | SCF_RSP_R1;
//	cmd.c_dmamap = dmap;

	error = mmc_command(&cmd);
	if (error != 0)
		goto err;

	if (ISSET(sc_flags, SMF_STOP_AFTER_MULTIPLE) &&
	    cmd.c_opcode == MMC_READ_BLOCK_MULTIPLE) {
		bzero(&cmd, sizeof cmd);
		cmd.c_opcode = MMC_STOP_TRANSMISSION;
		cmd.c_arg = MMC_ARG_RCA(sf->rca);
		cmd.c_flags = SCF_CMD_AC | SCF_RSP_R1B;
		error = mmc_command(&cmd);
		if (error != 0)
			goto err;
	}

	do {
		bzero(&cmd, sizeof cmd);
		cmd.c_opcode = MMC_SEND_STATUS;
		cmd.c_arg = MMC_ARG_RCA(sf->rca);
		cmd.c_flags = SCF_CMD_AC | SCF_RSP_R1;
		error = mmc_command(&cmd);
		if (error != 0)
			break;
		/* XXX time out */
	} while (!ISSET(MMC_R1(cmd.c_resp), MMC_R1_READY_FOR_DATA));

err:
	return (error);
}

int macSDMMC::mem_write_block(struct sdmmc_function *sf, int blkno, u_char *data,
    size_t datalen)
{
	int error;

	IOLockLock(sc_lock);

	if (ISSET(sc_caps, SMC_CAPS_SINGLE_ONLY)) {
		error = mem_single_write_block(sf, blkno, data, datalen);
		goto out;
	}

	if (!ISSET(sc_caps, SMC_CAPS_DMA)) {
		error = mem_write_block_subr(sf, blkno,
		    data, datalen);
		goto out;
	}

	/* DMA transfer */
    // XXX no support.
unload:
    error = EINVAL;

out:
    IOLockUnlock(sc_lock);
	return (error);
}

int macSDMMC::mem_single_write_block(struct sdmmc_function *sf, int blkno, u_char *data,
                                     size_t datalen)
{
    int error = 0;
    int i;

    for (i = 0; i < datalen / sf->csd.sector_size; i++)
    {
        error = mem_write_block_subr(sf, blkno + i,
                                     data + i * sf->csd.sector_size, sf->csd.sector_size);
        if (error)
            break;
    }

    return (error);
}

int macSDMMC::mem_write_block_subr(struct sdmmc_function *sf, int blkno, u_char *data, size_t datalen)
{
	struct sdmmc_command cmd;
	int error;

	if ((error = select_card(sf)) != 0)
		goto err;

	bzero(&cmd, sizeof cmd);
	cmd.c_data = data;
	cmd.c_datalen = datalen;
	cmd.c_blklen = sf->csd.sector_size;
	cmd.c_opcode = (datalen / cmd.c_blklen) > 1 ?
	    MMC_WRITE_BLOCK_MULTIPLE : MMC_WRITE_BLOCK_SINGLE;
	if (sf->flags & SFF_SDHC)
		cmd.c_arg = blkno;
	else
		cmd.c_arg = blkno << 9;
	cmd.c_flags = SCF_CMD_ADTC | SCF_RSP_R1;
	// cmd.c_dmamap = dmap;

	error = mmc_command(&cmd);
	if (error != 0)
		goto err;

	if (ISSET(sc_flags, SMF_STOP_AFTER_MULTIPLE) &&
	    cmd.c_opcode == MMC_WRITE_BLOCK_MULTIPLE) {
		bzero(&cmd, sizeof cmd);
		cmd.c_opcode = MMC_STOP_TRANSMISSION;
		cmd.c_flags = SCF_CMD_AC | SCF_RSP_R1B;
		error = mmc_command(&cmd);
		if (error != 0)
			goto err;
	}

	do {
		bzero(&cmd, sizeof cmd);
		cmd.c_opcode = MMC_SEND_STATUS;
		cmd.c_arg = MMC_ARG_RCA(sf->rca);
		cmd.c_flags = SCF_CMD_AC | SCF_RSP_R1;
		error = mmc_command(&cmd);
		if (error != 0)
			break;
		/* XXX time out */
	} while (!ISSET(MMC_R1(cmd.c_resp), MMC_R1_READY_FOR_DATA));

err:
	return (error);
}

//  ____  ____  __  __ __  __  ____   ____   ____ ____ ___
// / ___||  _ \|  \/  |  \/  |/ ___| / ___| / ___/ ___|_ _|
// \___ \| | | | |\/| | |\/| | |     \___ \| |   \___ \| |
//  ___) | |_| | |  | | |  | | |___   ___) | |___ ___) | |
// |____/|____/|_|  |_|_|  |_|\____| |____/ \____|____/___|

void macSDMMC::scsi_attach()
{
    DEVVERBOSE("disk attach\n");

    disk_ = new macSDMMCDisk;
    disk_->init(0);
    disk_->attach( static_cast<IOService *>(owner_) );
    disk_->setup(this);
    disk_->release();
    disk_->registerService();
}

void macSDMMC::scsi_detach()
{
    DEVVERBOSE("disk detach\n");

    disk_->detach( static_cast<IOService *>(owner_) );
    disk_->terminate(kIOServiceSynchronous);
}
