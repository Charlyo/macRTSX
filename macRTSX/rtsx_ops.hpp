#pragma once

#include <stdint.h>

struct rtsx_softc;

int rtsx_card_detect(struct rtsx_softc *sc);
uint32_t rtsx_host_ocr();
int rtsx_bus_power(struct rtsx_softc *sc, u_int32_t ocr);
int rtsx_set_bus_width(struct rtsx_softc *sc, int w);
int rtsx_bus_clock(struct rtsx_softc *sc, int freq, int timing);

/* clock frequencies for sdmmc_chip_bus_clock() */
#define SDMMC_SDCLK_OFF		0
#define SDMMC_SDCLK_400KHZ	400
#define SDMMC_SDCLK_25MHZ	25000
#define SDMMC_SDCLK_50MHZ	50000

/* voltage levels for sdmmc_chip_signal_voltage() */
#define SDMMC_SIGNAL_VOLTAGE_330	0
#define SDMMC_SIGNAL_VOLTAGE_180	1

#define SDMMC_TIMING_LEGACY	0
#define SDMMC_TIMING_HIGHSPEED	1
#define SDMMC_TIMING_MMC_DDR52	2

void rtsx_exec_command(struct rtsx_softc *sc, struct sdmmc_command *cmd);
int rtsx_host_maxblklen();

void rtsx_save_regs(struct rtsx_softc *);
void rtsx_restore_regs(struct rtsx_softc *);
