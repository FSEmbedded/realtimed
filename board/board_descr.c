/*
 * Copyright (c) 2024 F&S Elektronik Systeme GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"

#include <board/board.h>
#include <fdt/libfdt.h>

#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_reset.h"

static struct board_descr *gd_board_descr = NULL;

void print_board(enum board_types btype)
{
    PRINTF("BOARD: ");

    switch(btype) {
        case BT_PICOCOREMX8ULP:
            PRINTF("PicoCoreMX8ULP\r\n");
            break;
        case BT_OSMSFMX8ULP:
            PRINTF("FS-OSM-SF-MX8ULP\r\n");
            break;
        case BT_ARMSTONEMX8ULP:
            PRINTF("armStoneMX8ULP\r\n");
            break;
        case BT_SOLDERCOREMX8ULP:
            PRINTF("SolderCoreMX8ULP\r\n");
            break;
        default:
            break;
    }
}

static void *BOARD_get_fdt_config(void *addr){
    return addr + 0x40; // Skip F&S Header
}

int BOARD_get_type(void)
{
    static bool is_fdt_read = false;
    static enum board_types btype = BT_UNKNOWN;
    void *fdt = BOARD_get_fdt_config((void *)CFG_FUS_BOARDCFG_ADDR);
    int offs, len, ret;
    const void *prop;
    
    if(is_fdt_read)
        return btype;

    is_fdt_read = true;
    offs = fdt_path_offset(fdt, "/board-cfg");
    if(offs <= 0){
        PRINTF("ERROR: %s\n", fdt_strerror(offs));
        return btype;
    }

    prop = fdt_getprop(fdt, offs, "board-name", &len);

    ret = fdt_stringlist_contains(prop, len, "PicoCoreMX8ULP");
    if(ret == 1){
        btype = BT_PICOCOREMX8ULP;
        return btype;
    }

    ret = fdt_stringlist_contains(prop, len, "FS-OSM-SF-MX8ULP");
    if(ret == 1){
        btype = BT_OSMSFMX8ULP;
        return btype;
    }

    ret = fdt_stringlist_contains(prop, len, "armStoneMX8ULP");
    if(ret == 1){
        btype = BT_ARMSTONEMX8ULP;
        return btype;
    }

    ret = fdt_stringlist_contains(prop, len, "SolderCoreMX8ULP");
    if(ret == 1){
        btype = BT_SOLDERCOREMX8ULP;
        return btype;
    }
    
    return btype;    
}

struct board_descr *get_board_description(void)
{
    return gd_board_descr;
}

int BOARD_InitBoardDescr(enum board_types btype)
{
    struct dev tpm_devs[] = {
        {
            .base_addr  = TPM0_BASE,
            .ip_name    = kCLOCK_Tpm0,
            .ip_src     = kCLOCK_Pcc1BusIpSrcCm33Bus, 
            .instance   = 0,
            .irq        = TPM0_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Tpm0,
        },
        {
            .base_addr  = TPM1_BASE,
            .ip_name    = kCLOCK_Tpm1,
            .ip_src     = kCLOCK_Pcc1BusIpSrcCm33Bus, 
            .instance   = 1,
            .irq        = TPM1_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Tpm1,
        },
        {
            .base_addr  = TPM2_BASE,
            .ip_name    = kCLOCK_Tpm2,
            .ip_src     = kCLOCK_Pcc2BusIpSrcFusionDspBus, 
            .instance   = 2,
            .irq        = TPM2_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Tpm2,
        },
        {
            .base_addr  = TPM3_BASE,
            .ip_name    = kCLOCK_Tpm3,
            .ip_src     = kCLOCK_Pcc2BusIpSrcFusionDspBus, 
            .instance   = 3,
            .irq        = TPM3_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Tpm3,
        }
    };

    struct dev i2c_devs[] = {
        {
            .base_addr  = LPI2C0_BASE,
            .ip_name    = kCLOCK_Lpi2c0,
            .ip_src     = kCLOCK_Pcc1BusIpSrcCm33Bus, 
            .instance   = 0,
            .irq        = LPI2C0_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Lpi2c0,
        },
        {
            .base_addr  = LPI2C1_BASE,
            .ip_name    = kCLOCK_Lpi2c1,
            .ip_src     = kCLOCK_Pcc1BusIpSrcCm33Bus, 
            .instance   = 1,
            .irq        = LPI2C1_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Lpi2c1,
        },
        {
            .base_addr  = LPI2C2_BASE,
            .ip_name    = kCLOCK_Lpi2c2,
            .ip_src     = kCLOCK_Pcc1BusIpSrcCm33Bus, 
            .instance   = 2,
            .irq        = LPI2C2_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Lpi2c2,
        },
        {
            .base_addr  = LPI2C3_BASE,
            .ip_name    = kCLOCK_Lpi2c3,
            .ip_src     = kCLOCK_Pcc1BusIpSrcCm33Bus, 
            .instance   = 3,
            .irq        = LPI2C3_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Lpi2c3,
        },
        {
            .base_addr  = I3C0_BASE,
            .ip_name    = kCLOCK_I3c0,
            .ip_src     = kCLOCK_Pcc0BusIpSrcSysOscDiv2,
            .instance   = 0,
            .irq        = I3C0_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_I3c0,
        },
        {
            .base_addr  = I3C1_BASE,
            .ip_name    = kCLOCK_I3c1,
            .ip_src     = kCLOCK_Pcc0BusIpSrcSysOscDiv2,
            .instance   = 1,
            .irq        = I3C1_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_I3c1,
        }
    };

    struct dev sai_devs[] = {
        {
            .base_addr  = SAI0_BASE,
            .ip_name    = kCLOCK_Sai0,
            .ip_src     = kCLOCK_Pcc1BusIpSrcCm33Bus, 
            .instance   = 0,
            .irq        = SAI0_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Sai0,
        },
        {
            .base_addr  = SAI1_BASE,
            .ip_name    = kCLOCK_Sai1,
            .ip_src     = kCLOCK_Pcc1BusIpSrcCm33Bus, 
            .instance   = 1,
            .irq        = SAI1_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Sai1,
        },
        {
            .base_addr  = SAI2_BASE,
            .ip_name    = kCLOCK_Sai2,
            .ip_src     = kCLOCK_Pcc1BusIpSrcCm33Bus, 
            .instance   = 2,
            .irq        = SAI2_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Sai2,
        },
        {
            .base_addr  = SAI3_BASE,
            .ip_name    = kCLOCK_Sai3,
            .ip_src     = kCLOCK_Pcc1BusIpSrcCm33Bus, 
            .instance   = 3,
            .irq        = SAI3_IRQn,
            .irqHandler = NULL,
            .reset      = kRESET_Sai3,
        }
    };

    struct dev uart_devs[] = {
        {
            .base_addr  = LPUART0_BASE,
            .ip_name    = kCLOCK_Lpuart0,
            .ip_src     = kCLOCK_Pcc1BusIpSrcSysOscDiv2, 
            .instance   = 0,
            .irq        = 0,
            .irqHandler = NULL,
            .reset      = kRESET_Lpuart0,
        },
        {
            .base_addr  = LPUART1_BASE,
            .ip_name    = kCLOCK_Lpuart1,
            .ip_src     = kCLOCK_Pcc1BusIpSrcSysOscDiv2, 
            .instance   = 1,
            .irq        = 0,
            .irqHandler = NULL,
            .reset      = kRESET_Lpuart1,
        },
        {
            .base_addr  = LPUART2_BASE,
            .ip_name    = kCLOCK_Lpuart2,
            .ip_src     = kCLOCK_Pcc1BusIpSrcSysOscDiv2, 
            .instance   = 2,
            .irq        = 0,
            .irqHandler = NULL,
            .reset      = kRESET_Lpuart2,
        },
        {
            .base_addr  = LPUART3_BASE,
            .ip_name    = kCLOCK_Lpuart3,
            .ip_src     = kCLOCK_Pcc1BusIpSrcSysOscDiv2, 
            .instance   = 3,
            .irq        = 0,
            .irqHandler = NULL,
            .reset      = kRESET_Lpuart3,
        },
        {
            .base_addr  = LPUART4_BASE,
            .ip_name    = kCLOCK_Lpuart4,
            .ip_src     = kCLOCK_Pcc1BusIpSrcSysOscDiv2, 
            .instance   = 4,
            .irq        = 0,
            .irqHandler = NULL,
            .reset      = kRESET_Lpuart4,
        },
    };

    struct dev io_devs[] = {
        {
            .base_addr  = GPIOA_BASE,
            .ip_name    = kCLOCK_RgpioA,
            .ip_src     = 0, 
            .instance   = 0,
            .irq        = 0,
            .irqHandler = NULL,
            .reset      = 0,
        },
        {
            .base_addr  = GPIOB_BASE,
            .ip_name    = kCLOCK_RgpioB,
            .ip_src     = 0, 
            .instance   = 1,
            .irq        = 0,
            .irqHandler = NULL,
            .reset      = 0,
        },
        {
            .base_addr  = GPIOC_BASE,
            .ip_name    = kCLOCK_RgpioC,
            .ip_src     = 0, 
            .instance   = 2,
            .irq        = 0,
            .irqHandler = NULL,
            .reset      = 0,
        },
    };

    struct board_descr *bdescr = NULL;
    int ret;

    bdescr = pvPortMalloc(sizeof(struct board_descr));
    if(!bdescr)
        return -ENOMEM;

    memset(bdescr, 0, sizeof(struct board_descr));

    bdescr->btype = btype;

    ret = init_dbg_info(&bdescr->dbg_info, uart_devs, btype);
    if(ret)
        return ret;

    ret = init_uart_adapter(&bdescr->uart_adapter, uart_devs, btype);
    if(ret)
        return ret;
    
    gd_board_descr = bdescr;
    return 0;
}

void BOARD_InitPeripherie(struct board_descr *bdescr)
{    
    struct dbg_info *dbg_info = &bdescr->dbg_info;

    CLOCK_EnableClock(kCLOCK_Dma0Ch0);
    CLOCK_EnableClock(kCLOCK_Dma0Ch16);
    CLOCK_EnableClock(kCLOCK_Dma0Ch17);
    CLOCK_EnableClock(kCLOCK_Wuu0);
    CLOCK_EnableClock(kCLOCK_Bbnsm);

    BOARD_InitDebugConsole(dbg_info);

    BOARD_carrier_en(bdescr);
}


#if defined(CONFIG_BOARD_ARMSTONEMX8ULP) || defined(CONFIG_BOARD_OSMSFMX8ULP)
void BOARD_carrier_en(struct board_descr *bdescr)
{
/* TODO: */    
}
#endif /* CONFIG_BOARD_ARMSTONEMX8ULP || CONFIG_BOARD_OSMSFMX8ULP */