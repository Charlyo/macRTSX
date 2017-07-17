/*	$OpenBSD: rtsxvar.h,v 1.4 2015/04/28 07:55:13 stsp Exp $	*/

/*
 * Copyright (c) 2006 Uwe Stuehler <uwe@openbsd.org>
 * Copyright (c) 2012 Stefan Sperling <stsp@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _RTSXVAR_H_
#define _RTSXVAR_H_

#include <IOKit/system.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>

/* Number of registers to save for suspend/resume in terms of their ranges. */
#define RTSX_NREG ((0XFDAE - 0XFDA0) + (0xFD69 - 0xFD32) + (0xFE34 - 0xFE20))

class macSDMMC;

struct rtsx_softc {
    IOMemoryDescriptor * memorydescriptor;
    macSDMMC *  sdmmc;
	int		    flags;
	uint32_t 	intr_status;	/* soft interrupt status */
	uint8_t	    regs[RTSX_NREG];/* host controller state */
	uint32_t	regs4[6];	/* host controller state */
    IOBufferMemoryDescriptor * dmap_cmd, *dmap_data;
};

/* Host controller functions called by the attachment driver. */
int rtsx_attach(struct rtsx_softc *, IOMemoryDescriptor *, int);
int rtsx_detach(struct rtsx_softc *);
// activate();
int rtsx_intr(void *);

/* flag values */
#define	RTSX_F_CARD_PRESENT	(1<<0)
#define	RTSX_F_SDIO_SUPPORT	(1<<1)
#define	RTSX_F_5209			(1<<2)
#define RTSX_F_5227			(1<<3)
#define	RTSX_F_5229			(1<<4)
#define	RTSX_F_5229_TYPE_C	(1<<5)
#define	RTSX_F_8402			(1<<6)
#define	RTSX_F_8411			(1<<7)
#define	RTSX_F_8411B		(1<<8)
#define	RTSX_F_8411B_QFN48	(1<<9)

#endif
