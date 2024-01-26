// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014-2015 MediaTek Inc.
 * Author: Chaotian.Jing <chaotian.jing@mediatek.com>
 */

#define ICOM_MTK_MMC_NEW_TIMEOUT
// #define ICOM_MTK_MMC_NEW_TIMEOUT_DEBUG
// #define ICOM_MTK_MMC_NEW_TIMEOUT_VDEBUG
// #define ICOM_MTK_MMC_NEW_TIMEOUT_VVDEBUG
#define ICOM_MTK_MMC_DVFSRC_VCORE
#define ICOM_MTK_MMC_CPU_LATENCY_REQUEST
#define ICOM_MTK_MMC_NEW_BEST_DELAY

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_qos.h>
#include <linux/cpu.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/ratelimit_types.h>
#include <linux/ktime.h>

#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/slot-gpio.h>

#include "cqhci.h"

#define MAX_BD_NUM          1024
#define MSDC_NR_CLOCKS      3

/*--------------------------------------------------------------------------*/
/* Common Definition                                                        */
/*--------------------------------------------------------------------------*/
#define MSDC_BUS_1BITS          0x0
#define MSDC_BUS_4BITS          0x1
#define MSDC_BUS_8BITS          0x2

#define MSDC_BURST_64B          0x6

/*--------------------------------------------------------------------------*/
/* Register Offset                                                          */
/*--------------------------------------------------------------------------*/
#define MSDC_CFG         0x0
#define MSDC_IOCON       0x04
#define MSDC_PS          0x08
#define MSDC_INT         0x0c
#define MSDC_INTEN       0x10
#define MSDC_FIFOCS      0x14
#define SDC_CFG          0x30
#define SDC_CMD          0x34
#define SDC_ARG          0x38
#define SDC_STS          0x3c
#define SDC_RESP0        0x40
#define SDC_RESP1        0x44
#define SDC_RESP2        0x48
#define SDC_RESP3        0x4c
#define SDC_BLK_NUM      0x50
#define SDC_ADV_CFG0     0x64
#define EMMC_IOCON       0x7c
#define SDC_ACMD_RESP    0x80
#define DMA_SA_H4BIT     0x8c
#define MSDC_DMA_SA      0x90
#define MSDC_DMA_CTRL    0x98
#define MSDC_DMA_CFG     0x9c
#define MSDC_PATCH_BIT   0xb0
#define MSDC_PATCH_BIT1  0xb4
#define MSDC_PATCH_BIT2  0xb8
#define MSDC_PAD_TUNE    0xec
#define MSDC_PAD_TUNE0   0xf0
#define PAD_DS_TUNE      0x188
#define PAD_CMD_TUNE     0x18c
#define EMMC50_CFG0      0x208
#define EMMC50_CFG3      0x220
#define SDC_FIFO_CFG     0x228

/*--------------------------------------------------------------------------*/
/* Top Pad Register Offset                                                  */
/*--------------------------------------------------------------------------*/
#define EMMC_TOP_CONTROL	0x00
#define EMMC_TOP_CMD		0x04
#define EMMC50_PAD_DS_TUNE	0x0c

/*--------------------------------------------------------------------------*/
/* Register Mask                                                            */
/*--------------------------------------------------------------------------*/

/* MSDC_CFG mask */
#define MSDC_CFG_MODE           (0x1 << 0)	/* RW */
#define MSDC_CFG_CKPDN          (0x1 << 1)	/* RW */
#define MSDC_CFG_RST            (0x1 << 2)	/* RW */
#define MSDC_CFG_PIO            (0x1 << 3)	/* RW */
#define MSDC_CFG_CKDRVEN        (0x1 << 4)	/* RW */
#define MSDC_CFG_BV18SDT        (0x1 << 5)	/* RW */
#define MSDC_CFG_BV18PSS        (0x1 << 6)	/* R  */
#define MSDC_CFG_CKSTB          (0x1 << 7)	/* R  */
#define MSDC_CFG_CKDIV          (0xff << 8)	/* RW */
#define MSDC_CFG_CKMOD          (0x3 << 16)	/* RW */
#define MSDC_CFG_HS400_CK_MODE  (0x1 << 18)	/* RW */
#define MSDC_CFG_HS400_CK_MODE_EXTRA  (0x1 << 22)	/* RW */
#define MSDC_CFG_CKDIV_EXTRA    (0xfff << 8)	/* RW */
#define MSDC_CFG_CKMOD_EXTRA    (0x3 << 20)	/* RW */

/* MSDC_IOCON mask */
#define MSDC_IOCON_SDR104CKS    (0x1 << 0)	/* RW */
#define MSDC_IOCON_RSPL         (0x1 << 1)	/* RW */
#define MSDC_IOCON_DSPL         (0x1 << 2)	/* RW */
#define MSDC_IOCON_DDLSEL       (0x1 << 3)	/* RW */
#define MSDC_IOCON_DDR50CKD     (0x1 << 4)	/* RW */
#define MSDC_IOCON_DSPLSEL      (0x1 << 5)	/* RW */
#define MSDC_IOCON_W_DSPL       (0x1 << 8)	/* RW */
#define MSDC_IOCON_D0SPL        (0x1 << 16)	/* RW */
#define MSDC_IOCON_D1SPL        (0x1 << 17)	/* RW */
#define MSDC_IOCON_D2SPL        (0x1 << 18)	/* RW */
#define MSDC_IOCON_D3SPL        (0x1 << 19)	/* RW */
#define MSDC_IOCON_D4SPL        (0x1 << 20)	/* RW */
#define MSDC_IOCON_D5SPL        (0x1 << 21)	/* RW */
#define MSDC_IOCON_D6SPL        (0x1 << 22)	/* RW */
#define MSDC_IOCON_D7SPL        (0x1 << 23)	/* RW */
#define MSDC_IOCON_RISCSZ       (0x3 << 24)	/* RW */

/* MSDC_PS mask */
#define MSDC_PS_CDEN            (0x1 << 0)	/* RW */
#define MSDC_PS_CDSTS           (0x1 << 1)	/* R  */
#define MSDC_PS_CDDEBOUNCE      (0xf << 12)	/* RW */
#define MSDC_PS_DAT             (0xff << 16)	/* R  */
#define MSDC_PS_DATA1           (0x1 << 17)	/* R  */
#define MSDC_PS_CMD             (0x1 << 24)	/* R  */
#define MSDC_PS_WP              (0x1 << 31)	/* R  */

/* MSDC_INT mask */
#define MSDC_INT_MMCIRQ         (0x1 << 0)	/* W1C */
#define MSDC_INT_CDSC           (0x1 << 1)	/* W1C */
#define MSDC_INT_ACMDRDY        (0x1 << 3)	/* W1C */
#define MSDC_INT_ACMDTMO        (0x1 << 4)	/* W1C */
#define MSDC_INT_ACMDCRCERR     (0x1 << 5)	/* W1C */
#define MSDC_INT_DMAQ_EMPTY     (0x1 << 6)	/* W1C */
#define MSDC_INT_SDIOIRQ        (0x1 << 7)	/* W1C */
#define MSDC_INT_CMDRDY         (0x1 << 8)	/* W1C */
#define MSDC_INT_CMDTMO         (0x1 << 9)	/* W1C */
#define MSDC_INT_RSPCRCERR      (0x1 << 10)	/* W1C */
#define MSDC_INT_CSTA           (0x1 << 11)	/* R */
#define MSDC_INT_XFER_COMPL     (0x1 << 12)	/* W1C */
#define MSDC_INT_DXFER_DONE     (0x1 << 13)	/* W1C */
#define MSDC_INT_DATTMO         (0x1 << 14)	/* W1C */
#define MSDC_INT_DATCRCERR      (0x1 << 15)	/* W1C */
#define MSDC_INT_ACMD19_DONE    (0x1 << 16)	/* W1C */
#define MSDC_INT_DMA_BDCSERR    (0x1 << 17)	/* W1C */
#define MSDC_INT_DMA_GPDCSERR   (0x1 << 18)	/* W1C */
#define MSDC_INT_DMA_PROTECT    (0x1 << 19)	/* W1C */
#define MSDC_INT_CMDQ           (0x1 << 28)	/* W1C */

/* MSDC_INTEN mask */
#define MSDC_INTEN_MMCIRQ       (0x1 << 0)	/* RW */
#define MSDC_INTEN_CDSC         (0x1 << 1)	/* RW */
#define MSDC_INTEN_ACMDRDY      (0x1 << 3)	/* RW */
#define MSDC_INTEN_ACMDTMO      (0x1 << 4)	/* RW */
#define MSDC_INTEN_ACMDCRCERR   (0x1 << 5)	/* RW */
#define MSDC_INTEN_DMAQ_EMPTY   (0x1 << 6)	/* RW */
#define MSDC_INTEN_SDIOIRQ      (0x1 << 7)	/* RW */
#define MSDC_INTEN_CMDRDY       (0x1 << 8)	/* RW */
#define MSDC_INTEN_CMDTMO       (0x1 << 9)	/* RW */
#define MSDC_INTEN_RSPCRCERR    (0x1 << 10)	/* RW */
#define MSDC_INTEN_CSTA         (0x1 << 11)	/* RW */
#define MSDC_INTEN_XFER_COMPL   (0x1 << 12)	/* RW */
#define MSDC_INTEN_DXFER_DONE   (0x1 << 13)	/* RW */
#define MSDC_INTEN_DATTMO       (0x1 << 14)	/* RW */
#define MSDC_INTEN_DATCRCERR    (0x1 << 15)	/* RW */
#define MSDC_INTEN_ACMD19_DONE  (0x1 << 16)	/* RW */
#define MSDC_INTEN_DMA_BDCSERR  (0x1 << 17)	/* RW */
#define MSDC_INTEN_DMA_GPDCSERR (0x1 << 18)	/* RW */
#define MSDC_INTEN_DMA_PROTECT  (0x1 << 19)	/* RW */

/* MSDC_FIFOCS mask */
#define MSDC_FIFOCS_RXCNT       (0xff << 0)	/* R */
#define MSDC_FIFOCS_TXCNT       (0xff << 16)	/* R */
#define MSDC_FIFOCS_CLR         (0x1 << 31)	/* RW */

/* SDC_CFG mask */
#define SDC_CFG_SDIOINTWKUP     (0x1 << 0)	/* RW */
#define SDC_CFG_INSWKUP         (0x1 << 1)	/* RW */
#define SDC_CFG_WRDTOC          (0x1fff  << 2)  /* RW */
#define SDC_CFG_BUSWIDTH        (0x3 << 16)	/* RW */
#define SDC_CFG_SDIO            (0x1 << 19)	/* RW */
#define SDC_CFG_SDIOIDE         (0x1 << 20)	/* RW */
#define SDC_CFG_INTATGAP        (0x1 << 21)	/* RW */
#define SDC_CFG_DTOC            (0xff << 24)	/* RW */

/* SDC_STS mask */
#define SDC_STS_SDCBUSY         (0x1 << 0)	/* RW */
#define SDC_STS_CMDBUSY         (0x1 << 1)	/* RW */
#define SDC_STS_SWR_COMPL       (0x1 << 31)	/* RW */

#define SDC_DAT1_IRQ_TRIGGER	(0x1 << 19)	/* RW */
/* SDC_ADV_CFG0 mask */
#define SDC_RX_ENHANCE_EN	(0x1 << 20)	/* RW */

/* DMA_SA_H4BIT mask */
#define DMA_ADDR_HIGH_4BIT      (0xf << 0)      /* RW */

/* MSDC_DMA_CTRL mask */
#define MSDC_DMA_CTRL_START     (0x1 << 0)	/* W */
#define MSDC_DMA_CTRL_STOP      (0x1 << 1)	/* W */
#define MSDC_DMA_CTRL_RESUME    (0x1 << 2)	/* W */
#define MSDC_DMA_CTRL_MODE      (0x1 << 8)	/* RW */
#define MSDC_DMA_CTRL_LASTBUF   (0x1 << 10)	/* RW */
#define MSDC_DMA_CTRL_BRUSTSZ   (0x7 << 12)	/* RW */

/* MSDC_DMA_CFG mask */
#define MSDC_DMA_CFG_STS        (0x1 << 0)	/* R */
#define MSDC_DMA_CFG_DECSEN     (0x1 << 1)	/* RW */
#define MSDC_DMA_CFG_AHBHPROT2  (0x2 << 8)	/* RW */
#define MSDC_DMA_CFG_ACTIVEEN   (0x2 << 12)	/* RW */
#define MSDC_DMA_CFG_CS12B16B   (0x1 << 16)	/* RW */

/* MSDC_PATCH_BIT mask */
#define MSDC_PATCH_BIT_ODDSUPP    (0x1 <<  1)	/* RW */
#define MSDC_INT_DAT_LATCH_CK_SEL (0x7 <<  7)
#define MSDC_CKGEN_MSDC_DLY_SEL   (0x1f << 10)
#define MSDC_PATCH_BIT_IODSSEL    (0x1 << 16)	/* RW */
#define MSDC_PATCH_BIT_IOINTSEL   (0x1 << 17)	/* RW */
#define MSDC_PATCH_BIT_BUSYDLY    (0xf << 18)	/* RW */
#define MSDC_PATCH_BIT_WDOD       (0xf << 22)	/* RW */
#define MSDC_PATCH_BIT_IDRTSEL    (0x1 << 26)	/* RW */
#define MSDC_PATCH_BIT_CMDFSEL    (0x1 << 27)	/* RW */
#define MSDC_PATCH_BIT_INTDLSEL   (0x1 << 28)	/* RW */
#define MSDC_PATCH_BIT_SPCPUSH    (0x1 << 29)	/* RW */
#define MSDC_PATCH_BIT_DECRCTMO   (0x1 << 30)	/* RW */

#define MSDC_PATCH_BIT1_CMDTA     (0x7 << 3)    /* RW */
#define MSDC_PB1_BUSY_CHECK_SEL   (0x1 << 7)    /* RW */
#define MSDC_PATCH_BIT1_STOP_DLY  (0xf << 8)    /* RW */
#define MSDC_PATCH_BIT1_SINGLE_BURST    (0x1 << 16)

#define MSDC_PATCH_BIT2_CFGRESP   (0x1 << 15)   /* RW */
#define MSDC_PATCH_BIT2_CFGCRCSTS (0x1 << 28)   /* RW */
#define MSDC_PB2_SUPPORT_64G      (0x1 << 1)    /* RW */
#define MSDC_PB2_RESPWAIT         (0x3 << 2)    /* RW */
#define MSDC_PB2_RESPSTSENSEL     (0x7 << 16)   /* RW */
#define MSDC_PB2_CRCSTSENSEL      (0x7 << 29)   /* RW */

#define MSDC_PAD_TUNE_DATWRDLY	  (0x1f <<  0)	/* RW */
#define MSDC_PAD_TUNE_DATRRDLY	  (0x1f <<  8)	/* RW */
#define MSDC_PAD_TUNE_DATRRDLY2	  (0x1f << 8)	/* RW */
#define MSDC_PAD_TUNE_CMDRDLY2	  (0x1f << 16)  /* RW */
#define MSDC_PAD_TUNE_CMDRDLY	  (0x1f << 16)  /* RW */
#define MSDC_PAD_TUNE_CMDRRDLY	  (0x1f << 22)	/* RW */
#define MSDC_PAD_TUNE_CLKTDLY	  (0x1f << 27)  /* RW */
#define MSDC_PAD_TUNE_RXDLYSEL	  (0x1 << 15)   /* RW */
#define MSDC_PAD_TUNE_RD_SEL	  (0x1 << 13)   /* RW */
#define MSDC_PAD_TUNE_CMD_SEL	  (0x1 << 21)   /* RW */
#define MSDC_PAD_TUNE_RD2_SEL	  (0x1 << 13)	/* RW */
#define MSDC_PAD_TUNE_CMD2_SEL	  (0x1 << 21)	/* RW */

#define PAD_DS_TUNE_DLY1	  (0x1f << 2)   /* RW */
#define PAD_DS_TUNE_DLY2	  (0x1f << 7)   /* RW */
#define PAD_DS_TUNE_DLY3	  (0x1f << 12)  /* RW */

#define PAD_CMD_TUNE_RX_DLY3	  (0x1f << 1)  /* RW */

#define EMMC50_CFG_PADCMD_LATCHCK (0x1 << 0)   /* RW */
#define EMMC50_CFG_CRCSTS_EDGE    (0x1 << 3)   /* RW */
#define EMMC50_CFG_CFCSTS_SEL     (0x1 << 4)   /* RW */

#define EMMC50_CFG3_OUTS_WR       (0x1f << 0)  /* RW */

#define SDC_FIFO_CFG_WRVALIDSEL   (0x1 << 24)  /* RW */
#define SDC_FIFO_CFG_RDVALIDSEL   (0x1 << 25)  /* RW */

/* EMMC_TOP_CONTROL mask */
#define PAD_RXDLY_SEL           (0x1 << 0)      /* RW */
#define DELAY_EN                (0x1 << 1)      /* RW */
#define PAD_DAT_RD_RXDLY2       (0x1f << 2)     /* RW */
#define PAD_DAT_RD_RXDLY        (0x1f << 7)     /* RW */
#define PAD_DAT_RD_RXDLY2_SEL   (0x1 << 12)     /* RW */
#define PAD_DAT_RD_RXDLY_SEL    (0x1 << 13)     /* RW */
#define DATA_K_VALUE_SEL        (0x1 << 14)     /* RW */
#define SDC_RX_ENH_EN           (0x1 << 15)     /* TW */

/* EMMC_TOP_CMD mask */
#define PAD_CMD_RXDLY2          (0x1f << 0)     /* RW */
#define PAD_CMD_RXDLY           (0x1f << 5)     /* RW */
#define PAD_CMD_RD_RXDLY2_SEL   (0x1 << 10)     /* RW */
#define PAD_CMD_RD_RXDLY_SEL    (0x1 << 11)     /* RW */
#define PAD_CMD_TX_DLY          (0x1f << 12)    /* RW */

#define REQ_CMD_EIO  (0x1 << 0)
#define REQ_CMD_TMO  (0x1 << 1)
#define REQ_DAT_ERR  (0x1 << 2)
#define REQ_STOP_EIO (0x1 << 3)
#define REQ_STOP_TMO (0x1 << 4)
#define REQ_CMD_BUSY (0x1 << 5)

#define MSDC_PREPARE_FLAG (0x1 << 0)
#define MSDC_ASYNC_FLAG (0x1 << 1)
#define MSDC_MMAP_FLAG (0x1 << 2)

#define MTK_MMC_AUTOSUSPEND_DELAY	50
#define CMD_TIMEOUT         (HZ/10 * 5)	/* 100ms x5 */
#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
#define DAT_TIMEOUT         (HZ   * 10)	/* 1000ms x10 */
#else
#define DAT_TIMEOUT         (HZ    * 5)	/* 1000ms x5 */
#endif

#define DEFAULT_DEBOUNCE	(8)	/* 8 cycles CD debounce */

#define PAD_DELAY_MAX	32 /* PAD delay cells */
/* Enable PAD_DELAY_64 to support 64-bit phase delay: DTS change also required */
//#define PAD_DELAY_64	64
#if defined(PAD_DELAY_64) && (PAD_DELAY_64 == 64)
  #define PAD_DELAY_USE_64BITS	1
  #define PAD_DELAY_TYPE	u64
  #define PAD_DELAY_BIT_MAX	(64)
  #define PAD_DELAY_BIT_TYPE	(1ULL)
  #define PAD_DELAY_PRIx	"llx"
  #define PAD_DELAY_PRIx0	"016llx"
#else
  #define PAD_DELAY_TYPE	u32
  #define PAD_DELAY_BIT_MAX	(32)
  #define PAD_DELAY_BIT_TYPE	(1)
  #define PAD_DELAY_PRIx	"x"
  #define PAD_DELAY_PRIx0	"08x"
#endif
#define PAD_DELAY_TUNING_LOOP	(5)
#define PAD_DELAY_RISING_MIN_LENGTH	(12)

#define SDIO_CCCR_INTERRUPT_EXT	0x16
#define SDIO_INTERRUPT_EXT_SAI	(1 << 0)
#define SDIO_INTERRUPT_EXT_EAI	(1 << 1)
/*--------------------------------------------------------------------------*/
/* Descriptor Structure                                                     */
/*--------------------------------------------------------------------------*/
struct mt_gpdma_desc {
	u32 gpd_info;
#define GPDMA_DESC_HWO		(0x1 << 0)
#define GPDMA_DESC_BDP		(0x1 << 1)
#define GPDMA_DESC_CHECKSUM	(0xff << 8) /* bit8 ~ bit15 */
#define GPDMA_DESC_INT		(0x1 << 16)
#define GPDMA_DESC_NEXT_H4	(0xf << 24)
#define GPDMA_DESC_PTR_H4	(0xf << 28)
	u32 next;
	u32 ptr;
	u32 gpd_data_len;
#define GPDMA_DESC_BUFLEN	(0xffff) /* bit0 ~ bit15 */
#define GPDMA_DESC_EXTLEN	(0xff << 16) /* bit16 ~ bit23 */
	u32 arg;
	u32 blknum;
	u32 cmd;
};

struct mt_bdma_desc {
	u32 bd_info;
#define BDMA_DESC_EOL		(0x1 << 0)
#define BDMA_DESC_CHECKSUM	(0xff << 8) /* bit8 ~ bit15 */
#define BDMA_DESC_BLKPAD	(0x1 << 17)
#define BDMA_DESC_DWPAD		(0x1 << 18)
#define BDMA_DESC_NEXT_H4	(0xf << 24)
#define BDMA_DESC_PTR_H4	(0xf << 28)
	u32 next;
	u32 ptr;
	u32 bd_data_len;
#define BDMA_DESC_BUFLEN	(0xffff) /* bit0 ~ bit15 */
#define BDMA_DESC_BUFLEN_EXT	(0xffffff) /* bit0 ~ bit23 */
};

struct msdc_dma {
	struct scatterlist *sg;	/* I/O scatter list */
	struct mt_gpdma_desc *gpd;		/* pointer to gpd array */
	struct mt_bdma_desc *bd;		/* pointer to bd array */
	dma_addr_t gpd_addr;	/* the physical address of gpd array */
	dma_addr_t bd_addr;	/* the physical address of bd array */
};

struct msdc_save_para {
	u32 msdc_cfg;
	u32 iocon;
	u32 sdc_cfg;
	u32 pad_tune;
	u32 patch_bit0;
	u32 patch_bit1;
	u32 patch_bit2;
	u32 pad_ds_tune;
	u32 pad_cmd_tune;
	u32 emmc50_cfg0;
	u32 emmc50_cfg3;
	u32 sdc_fifo_cfg;
	u32 emmc_top_control;
	u32 emmc_top_cmd;
	u32 emmc50_pad_ds_tune;
};

struct mtk_mmc_compatible {
	u8 clk_div_bits;
	bool recheck_sdio_irq;
	bool hs400_tune; /* only used for MT8173 */
	u32 pad_tune_reg;
	bool async_fifo;
	bool data_tune;
	bool busy_check;
	bool stop_clk_fix;
	bool enhance_rx;
	bool support_64g;
	bool use_internal_cd;
};

struct msdc_tune_para {
	u32 iocon;
	u32 pad_tune;
	u32 pad_cmd_tune;
	u32 emmc_top_control;
	u32 emmc_top_cmd;
};

struct msdc_delay_phase {
	u8 maxlen;
	u8 start;
	u8 final_phase;
};

struct msdc_host {
	struct device *dev;
	const struct mtk_mmc_compatible *dev_comp;
	int cmd_rsp;

	spinlock_t lock;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;
	int error;

	void __iomem *base;		/* host base address */
	void __iomem *top_base;		/* host top register base address */

	struct msdc_dma dma;	/* dma channel */
	u64 dma_mask;

	u32 timeout_ns;		/* data timeout ns */
	u32 timeout_clks;	/* data timeout clks */

#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
	u32 max_busy_timeout_us;	/* max. busy detect timeout in us */
	u64 data_timeout_ns;	/* data timeout (DTO) in ns */
#endif

	int signal_voltage; /* signalling voltage (1.8V or 3.3V) */
	unsigned short ios_vdd; /* stored ios vdd value */
	bool sd_poweroff_reset_signal_volt; /* sd-poweroff-reset-signal-volt */

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_normal;
	struct pinctrl_state *pins_poweron;
	struct pinctrl_state *pins_poweroff;
	struct pinctrl_state *pins_uhs;
	struct pinctrl_state *pins_eint;
	struct pinctrl_state *pins_dat1;
	struct delayed_work req_timeout;
	int irq;		/* host interrupt */
	int eint_irq;           /* device interrupt */
	int sdio_irq_cnt;       /* irq enable cnt */
	struct reset_control *reset;

	struct clk *src_clk;	/* msdc source clock */
	struct clk *h_clk;      /* msdc h_clk */
	struct clk *bus_clk;	/* bus clock which used to access register */
	struct clk *src_clk_cg; /* msdc source clock control gate */
	struct clk *sys_clk_cg;	/* msdc subsys clock control gate */
	struct clk *crypto_clk;    /* msdc crypto clock */
	struct clk *crypto_cg;     /* msdc crypto clock control gate */
	struct clk_bulk_data bulk_clks[MSDC_NR_CLOCKS];

	/* clk_auto_powerdown == true => Clear MSDC_CFG_CKPDN bit always:
	 *   1'b0: Clock will be gated to 0 if no command or data is transmitted.
	 *   1'b1: Clock will be running freely even if no command or data is
	 *         transmitted.  (The clock may still be stopped when MSDC write data are
	 *         not enough or there is no space for the next read data.)
	 */
	bool clk_auto_powerdown; /* MSDC_CFG_CKPDN */
	u32 mclk;		/* mmc subsystem clock frequency */
	u32 src_clk_freq;	/* source clock frequency */
	unsigned char timing;
	bool vqmmc_enabled;
	struct regulator *vioa;	/* Optional vio-a supply */
	struct regulator *viob;	/* Optional vio-b supply */
	bool vioa_enabled;
	bool viob_enabled;
	bool tuning_both_edges;
	struct regulator *vcore_power;
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	u32 *vcore_lookup_table;
	int n_vcore_lookup_table;
	PAD_DELAY_TYPE *rising_expected_delay_maps;

	PAD_DELAY_TYPE rising_delay_map_mask;
	PAD_DELAY_TYPE falling_delay_map_mask;
	int rising_delay_min_length;
#endif
	int tuning_loop;
	u32 ckgen_delay;
	u32 latch_ck;
	u32 hs400_ds_delay;
	u32 hs200_cmd_int_delay; /* cmd internal delay for HS200/SDR104 */
	u32 hs400_cmd_int_delay; /* cmd internal delay for HS400 */
	bool hs400_cmd_resp_sel_rising;
				 /* cmd response sample selection for HS400 */
	bool hs400_mode;	/* current eMMC will run at hs400 mode */
	bool internal_cd;	/* Use internal card-detect logic */
	bool cqhci;		/* support eMMC hw cmdq */
	bool sdio_eint_ready;   /* Ready to support SDIO eint interrupt */
	bool enable_async_irq;
	bool ahb_bus_no_incr1; /* AHB bus no incr1 burst type */
	struct msdc_save_para save_para; /* used when gate HCLK */
	struct msdc_tune_para def_tune_para; /* default tune setting */
	struct msdc_tune_para saved_tune_para; /* tune result of CMD21/CMD19 */
#if IS_ENABLED(CONFIG_MMC_CQHCI)
	struct cqhci_host *cq_host;
#endif

	u32 autosuspend_delay;

#ifdef ICOM_MTK_MMC_DVFSRC_VCORE
	struct regulator *dvfsrc_vcore_power;
	int req_vcore;
#endif
	u32 affinity_hint_cpu;
#ifdef ICOM_MTK_MMC_CPU_LATENCY_REQUEST
	bool use_affinity_hint_cpu_latency;
	struct device *cpu_dev;
	struct pm_qos_request pm_qos_req;
	int cpu_latency_req;
#endif

	struct ratelimit_state cmd_err_ratelimit;
	struct ratelimit_state cmd_err_crc_ratelimit;
	struct ratelimit_state data_err_ratelimit;
	struct ratelimit_state data_err_crc_ratelimit;
};

static const struct mtk_mmc_compatible mt8135_compat = {
	.clk_div_bits = 8,
	.recheck_sdio_irq = true,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE,
	.async_fifo = false,
	.data_tune = false,
	.busy_check = false,
	.stop_clk_fix = false,
	.enhance_rx = false,
	.support_64g = false,
};

static const struct mtk_mmc_compatible mt8173_compat = {
	.clk_div_bits = 8,
	.recheck_sdio_irq = true,
	.hs400_tune = true,
	.pad_tune_reg = MSDC_PAD_TUNE,
	.async_fifo = false,
	.data_tune = false,
	.busy_check = false,
	.stop_clk_fix = false,
	.enhance_rx = false,
	.support_64g = false,
};

static const struct mtk_mmc_compatible mt8183_compat = {
	.clk_div_bits = 12,
	.recheck_sdio_irq = false,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt2701_compat = {
	.clk_div_bits = 12,
	.recheck_sdio_irq = true,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = false,
	.stop_clk_fix = false,
	.enhance_rx = false,
	.support_64g = false,
};

static const struct mtk_mmc_compatible mt2712_compat = {
	.clk_div_bits = 12,
	.recheck_sdio_irq = false,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt7622_compat = {
	.clk_div_bits = 12,
	.recheck_sdio_irq = true,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = false,
};

static const struct mtk_mmc_compatible mt8516_compat = {
	.clk_div_bits = 12,
	.recheck_sdio_irq = true,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
};

static const struct mtk_mmc_compatible mt7620_compat = {
	.clk_div_bits = 8,
	.recheck_sdio_irq = true,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE,
	.async_fifo = false,
	.data_tune = false,
	.busy_check = false,
	.stop_clk_fix = false,
	.enhance_rx = false,
	.use_internal_cd = true,
};

static const struct mtk_mmc_compatible mt6779_compat = {
	.clk_div_bits = 12,
	.recheck_sdio_irq = false,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt6765_compat = {
	.clk_div_bits = 12,
	.recheck_sdio_irq = false,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt6768_compat = {
	.clk_div_bits = 12,
	.recheck_sdio_irq = false,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

/* Refer to mt6789_compat */
static const struct mtk_mmc_compatible mt8781_compat = {
	.clk_div_bits = 12,
	.recheck_sdio_irq = false,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct of_device_id msdc_of_ids[] = {
	{ .compatible = "mediatek,mt8135-mmc", .data = &mt8135_compat},
	{ .compatible = "mediatek,mt8173-mmc", .data = &mt8173_compat},
	{ .compatible = "mediatek,mt8183-mmc", .data = &mt8183_compat},
	{ .compatible = "mediatek,mt2701-mmc", .data = &mt2701_compat},
	{ .compatible = "mediatek,mt2712-mmc", .data = &mt2712_compat},
	{ .compatible = "mediatek,mt7622-mmc", .data = &mt7622_compat},
	{ .compatible = "mediatek,mt8516-mmc", .data = &mt8516_compat},
	{ .compatible = "mediatek,mt7620-mmc", .data = &mt7620_compat},
	{ .compatible = "mediatek,mt6779-mmc", .data = &mt6779_compat},
	{ .compatible = "mediatek,mt6765-mmc", .data = &mt6765_compat},
	{ .compatible = "mediatek,mt6768-mmc", .data = &mt6768_compat},
	{ .compatible = "mediatek,mt8781-sd", .data = &mt8781_compat},
	{}
};
MODULE_DEVICE_TABLE(of, msdc_of_ids);

/* true: allowed */
static bool msdc_ratelimit(struct ratelimit_state *rs)
{
	/* Paired with WRITE_ONCE() in .proc_handler().
	 * Changing two values seperately could be inconsistent
	 * and some message could be lost.  (See: net_ratelimit_state).
	 */
	int interval = READ_ONCE(rs->interval);
	int burst = READ_ONCE(rs->burst);
#if 0
	unsigned long flags;
#endif
	bool ret;

	if (!interval)
		return true;

#if 0
	/*
	 * If we contend on this state's lock then almost
	 * by definition we are too busy to print a message,
	 * in addition to the one that will be printed by
	 * the entity that is holding the lock already:
	 */
	if (!raw_spin_trylock_irqsave(&rs->lock, flags))
		return false;
#endif

	if (!rs->begin)
		rs->begin = jiffies;

	if (time_is_before_jiffies(rs->begin + interval)) {
#if 0
		if (rs->missed) {
			if (!(rs->flags & RATELIMIT_MSG_ON_RELEASE))
				rs->missed = 0;
		}
#else
		rs->missed  = 0;
#endif
		rs->begin   = jiffies;
		rs->printed = 0;
	}
	if (burst && burst > rs->printed) {
		rs->printed++;
		ret = true;
	} else {
		rs->missed++;
		ret = false;
	}
#if 0
	raw_spin_unlock_irqrestore(&rs->lock, flags);
#endif

	return ret;
}

static void sdr_set_bits(void __iomem *reg, u32 bs)
{
	u32 val = readl(reg);

	val |= bs;
	writel(val, reg);
}

static void sdr_clr_bits(void __iomem *reg, u32 bs)
{
	u32 val = readl(reg);

	val &= ~bs;
	writel(val, reg);
}

static void sdr_set_field(void __iomem *reg, u32 field, u32 val)
{
	unsigned int tv = readl(reg);

	tv &= ~field;
#if 0
	tv |= ((val) << (ffs((unsigned int)field) - 1));
#else
	tv |= (((val) << (ffs((unsigned int)field) - 1)) & field);
#endif
	writel(tv, reg);
}

static void sdr_get_field(void __iomem *reg, u32 field, u32 *val)
{
	unsigned int tv = readl(reg);

	*val = ((tv & field) >> (ffs((unsigned int)field) - 1));
}

static void msdc_reset_hw(struct msdc_host *host)
{
	u32 val;

	sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_RST);
	readl_poll_timeout_atomic(host->base + MSDC_CFG, val, !(val & MSDC_CFG_RST), 0, 0);

	sdr_set_bits(host->base + MSDC_FIFOCS, MSDC_FIFOCS_CLR);
	readl_poll_timeout_atomic(host->base + MSDC_FIFOCS, val,
				  !(val & MSDC_FIFOCS_CLR), 0, 0);

	val = readl(host->base + MSDC_INT);
	writel(val, host->base + MSDC_INT);
}

static void msdc_cmd_next(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd);
static void __msdc_enable_sdio_irq(struct msdc_host *host, int enb);

static const u32 cmd_ints_mask = MSDC_INTEN_CMDRDY | MSDC_INTEN_RSPCRCERR |
			MSDC_INTEN_CMDTMO | MSDC_INTEN_ACMDRDY |
			MSDC_INTEN_ACMDCRCERR | MSDC_INTEN_ACMDTMO;
static const u32 data_ints_mask = MSDC_INTEN_XFER_COMPL | MSDC_INTEN_DATTMO |
			MSDC_INTEN_DATCRCERR | MSDC_INTEN_DMA_BDCSERR |
			MSDC_INTEN_DMA_GPDCSERR | MSDC_INTEN_DMA_PROTECT;

static u8 msdc_dma_calcs(u8 *buf, u32 len)
{
	u32 i, sum = 0;

	for (i = 0; i < len; i++)
		sum += buf[i];
	return 0xff - (u8) sum;
}

static inline void msdc_dma_setup(struct msdc_host *host, struct msdc_dma *dma,
		struct mmc_data *data)
{
	unsigned int j, dma_len;
	dma_addr_t dma_address;
	u32 dma_ctrl;
	struct scatterlist *sg;
	struct mt_gpdma_desc *gpd;
	struct mt_bdma_desc *bd;

	sg = data->sg;

	gpd = dma->gpd;
	bd = dma->bd;

	/* modify gpd */
	gpd->gpd_info |= GPDMA_DESC_HWO;
	gpd->gpd_info |= GPDMA_DESC_BDP;
	/* need to clear first. use these bits to calc checksum */
	gpd->gpd_info &= ~GPDMA_DESC_CHECKSUM;
	gpd->gpd_info |= msdc_dma_calcs((u8 *) gpd, 16) << 8;

	/* modify bd */
	for_each_sg(data->sg, sg, data->sg_count, j) {
		dma_address = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);

		/* init bd */
		bd[j].bd_info &= ~BDMA_DESC_BLKPAD;
		bd[j].bd_info &= ~BDMA_DESC_DWPAD;
		bd[j].ptr = lower_32_bits(dma_address);
		if (host->dev_comp->support_64g) {
			bd[j].bd_info &= ~BDMA_DESC_PTR_H4;
			bd[j].bd_info |= (upper_32_bits(dma_address) & 0xf)
					 << 28;
		}

		if (host->dev_comp->support_64g) {
			bd[j].bd_data_len &= ~BDMA_DESC_BUFLEN_EXT;
			bd[j].bd_data_len |= (dma_len & BDMA_DESC_BUFLEN_EXT);
		} else {
			bd[j].bd_data_len &= ~BDMA_DESC_BUFLEN;
			bd[j].bd_data_len |= (dma_len & BDMA_DESC_BUFLEN);
		}

		if (j == data->sg_count - 1) /* the last bd */
			bd[j].bd_info |= BDMA_DESC_EOL;
		else
			bd[j].bd_info &= ~BDMA_DESC_EOL;

		/* checksume need to clear first */
		bd[j].bd_info &= ~BDMA_DESC_CHECKSUM;
		bd[j].bd_info |= msdc_dma_calcs((u8 *)(&bd[j]), 16) << 8;
	}

	sdr_set_field(host->base + MSDC_DMA_CFG, MSDC_DMA_CFG_DECSEN, 1);
	dma_ctrl = readl_relaxed(host->base + MSDC_DMA_CTRL);
	dma_ctrl &= ~(MSDC_DMA_CTRL_BRUSTSZ | MSDC_DMA_CTRL_MODE);
	dma_ctrl |= (MSDC_BURST_64B << 12 | 1 << 8);
	writel_relaxed(dma_ctrl, host->base + MSDC_DMA_CTRL);
	if (host->dev_comp->support_64g)
		sdr_set_field(host->base + DMA_SA_H4BIT, DMA_ADDR_HIGH_4BIT,
			      upper_32_bits(dma->gpd_addr) & 0xf);
	writel(lower_32_bits(dma->gpd_addr), host->base + MSDC_DMA_SA);
}

static void msdc_prepare_data(struct msdc_host *host, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;

	if (!(data->host_cookie & MSDC_PREPARE_FLAG)) {
		data->host_cookie |= MSDC_PREPARE_FLAG;
		data->sg_count = dma_map_sg(host->dev, data->sg, data->sg_len,
					    mmc_get_dma_dir(data));
	}
}

static void msdc_unprepare_data(struct msdc_host *host, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;

	if (data->host_cookie & MSDC_ASYNC_FLAG)
		return;

	if (data->host_cookie & MSDC_PREPARE_FLAG) {
		dma_unmap_sg(host->dev, data->sg, data->sg_len,
			     mmc_get_dma_dir(data));
		data->host_cookie &= ~MSDC_PREPARE_FLAG;
	}
}

static u64 msdc_timeout_cal(struct msdc_host *host, u64 ns, u64 clks)
{
	struct mmc_host *mmc = mmc_from_priv(host);
	u64 timeout, clk_ns;
	u32 mode = 0;

	if (mmc->actual_clock == 0) {
		timeout = 0;
	} else {
		clk_ns  = 1000000000ULL;
		do_div(clk_ns, mmc->actual_clock);
		timeout = ns + clk_ns - 1;
		do_div(timeout, clk_ns);
		timeout += clks;
		/* in 1048576 sclk cycle unit */
		timeout = DIV_ROUND_UP(timeout, (0x1 << 20));
		if (host->dev_comp->clk_div_bits == 8)
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD, &mode);
		else
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD_EXTRA, &mode);
		/*DDR mode will double the clk cycles for data timeout */
		timeout = mode >= 2 ? timeout * 2 : timeout;
		timeout = timeout > 1 ? timeout - 1 : 0;
	}
	return timeout;
}

/* clock control primitives */
static void msdc_set_timeout(struct msdc_host *host, u64 ns, u64 clks)
{
	u64 timeout;

	host->timeout_ns = ns;
	host->timeout_clks = clks;

	timeout = msdc_timeout_cal(host, ns, clks);
	sdr_set_field(host->base + SDC_CFG, SDC_CFG_DTOC,
		      (u32)(timeout > 255 ? 255 : timeout));
}

#if IS_ENABLED(CONFIG_MMC_CQHCI)
static void msdc_set_busy_timeout(struct msdc_host *host, u64 ns, u64 clks)
{
	u64 timeout;

	timeout = msdc_timeout_cal(host, ns, clks);
	sdr_set_field(host->base + SDC_CFG, SDC_CFG_WRDTOC,
		      (u32)(timeout > 8191 ? 8191 : timeout));
}
#endif

#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
static u32 msdc_max_busy_timeout_us(struct msdc_host *host)
{
	struct mmc_host *mmc = mmc_from_priv(host);
	u64 timeout;
	u32 mode = 0;

	if (mmc->actual_clock == 0) {
		timeout = 0;
	} else {
		timeout = 1000000ULL * 8192 * (1ULL << 20);
		do_div(timeout, mmc->actual_clock); /* us */

		if (host->dev_comp->clk_div_bits == 8)
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD, &mode);
		else
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD_EXTRA, &mode);
		/* DDR mode timeout will be in half */
		if (mode >= 2)
			do_div(timeout, 2);
	}

#ifdef ICOM_MTK_MMC_NEW_TIMEOUT_DEBUG
	dev_info(host->dev, "max busy timeout: %llu us (mode=%u, actual_clock=%u)\n",
			timeout, mode, mmc->actual_clock);
#endif

	return (u32)timeout;
}

static void msdc_set_data_timeout(struct msdc_host *host, u64 timeout_ns)
{
	struct mmc_host *mmc = mmc_from_priv(host);
	u64 count;

	host->data_timeout_ns = timeout_ns;
	host->timeout_ns = timeout_ns;
	host->timeout_clks = 0;

	if (mmc->actual_clock == 0) {
		count = 0;
	} else {
		/* Refer to msdc_timeout_cal() */
		u32 mode = 0;
		u64 clk_ns = 1000000000ULL;

		do_div(clk_ns, mmc->actual_clock);
		count = timeout_ns + clk_ns - 1;
		do_div(count, clk_ns);

		/* in 1048576 sclk cycle unit */
		count = DIV_ROUND_UP(count, (0x1 << 20));

		if (host->dev_comp->clk_div_bits == 8)
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD, &mode);
		else
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD_EXTRA, &mode);
		/* DDR mode will double the clk cycles for data timeout */
		count = mode >= 2 ? count * 2 : count;
		count = count > 1 ? count - 1 : 0;
	}

#ifdef ICOM_MTK_MMC_NEW_TIMEOUT_DEBUG
	dev_info(host->dev, "set data timeout: %llu us (%u)\n", timeout_ns / NSEC_PER_USEC, (u32)count);
#endif

	/* read data timeout */
	sdr_set_field(host->base + SDC_CFG, SDC_CFG_DTOC,
		      (u32)(count > 255 ? 255 : count));
	/* write data / busy timeout */
	sdr_set_field(host->base + SDC_CFG, SDC_CFG_WRDTOC,
		      (u32)(count > 8191 ? 8191 : count));
}
#endif

static void msdc_gate_clock(struct msdc_host *host)
{
	clk_bulk_disable_unprepare(MSDC_NR_CLOCKS, host->bulk_clks);
	clk_disable_unprepare(host->src_clk_cg);
	clk_disable_unprepare(host->crypto_cg);
	clk_disable_unprepare(host->src_clk);
	clk_disable_unprepare(host->bus_clk);
	clk_disable_unprepare(host->h_clk);
	clk_disable_unprepare(host->crypto_clk);
}

static int msdc_ungate_clock(struct msdc_host *host)
{
	u32 val;
	int ret;

	clk_prepare_enable(host->crypto_clk);
	clk_prepare_enable(host->h_clk);
	clk_prepare_enable(host->bus_clk);
	clk_prepare_enable(host->src_clk);
	clk_prepare_enable(host->crypto_cg);
	clk_prepare_enable(host->src_clk_cg);
	ret = clk_bulk_prepare_enable(MSDC_NR_CLOCKS, host->bulk_clks);
	if (ret) {
		dev_err(host->dev, "Cannot enable pclk/axi/ahb clock gates\n");
		return ret;
	}

	return readl_poll_timeout(host->base + MSDC_CFG, val,
				  (val & MSDC_CFG_CKSTB), 1, 20000);
}

static void msdc_set_mclk(struct msdc_host *host, unsigned char timing, u32 hz)
{
	struct mmc_host *mmc = mmc_from_priv(host);
	u32 mode;
	u32 flags;
	u32 div;
	u32 sclk;
	u32 tune_reg = host->dev_comp->pad_tune_reg;
	u32 val;

	if (!hz) {
		dev_dbg(host->dev, "set mclk to 0\n");
		host->mclk = 0;
		mmc->actual_clock = 0;
		sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
		return;
	}

	flags = readl(host->base + MSDC_INTEN);
	sdr_clr_bits(host->base + MSDC_INTEN, flags);
	if (host->dev_comp->clk_div_bits == 8)
		sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_HS400_CK_MODE);
	else
		sdr_clr_bits(host->base + MSDC_CFG,
			     MSDC_CFG_HS400_CK_MODE_EXTRA);
	if (timing == MMC_TIMING_UHS_DDR50 ||
	    timing == MMC_TIMING_MMC_DDR52 ||
	    timing == MMC_TIMING_MMC_HS400) {
		if (timing == MMC_TIMING_MMC_HS400)
			mode = 0x3;
		else
			mode = 0x2; /* ddr mode and use divisor */

		if (hz >= (host->src_clk_freq >> 2)) {
			div = 0; /* mean div = 1/4 */
			sclk = host->src_clk_freq >> 2; /* sclk = clk / 4 */
		} else {
			div = (host->src_clk_freq + ((hz << 2) - 1)) / (hz << 2);
			sclk = (host->src_clk_freq >> 2) / div;
			div = (div >> 1);
		}

		if (timing == MMC_TIMING_MMC_HS400 &&
		    hz >= (host->src_clk_freq >> 1)) {
			if (host->dev_comp->clk_div_bits == 8)
				sdr_set_bits(host->base + MSDC_CFG,
					     MSDC_CFG_HS400_CK_MODE);
			else
				sdr_set_bits(host->base + MSDC_CFG,
					     MSDC_CFG_HS400_CK_MODE_EXTRA);
			sclk = host->src_clk_freq >> 1;
			div = 0; /* div is ignore when bit18 is set */
		}
	} else if (hz >= host->src_clk_freq) {
		mode = 0x1; /* no divisor */
		div = 0;
		sclk = host->src_clk_freq;
	} else {
		mode = 0x0; /* use divisor */
		if (hz >= (host->src_clk_freq >> 1)) {
			div = 0; /* mean div = 1/2 */
			sclk = host->src_clk_freq >> 1; /* sclk = clk / 2 */
		} else {
			div = (host->src_clk_freq + ((hz << 2) - 1)) / (hz << 2);
			sclk = (host->src_clk_freq >> 2) / div;
		}
	}
	sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);

	clk_disable_unprepare(host->src_clk_cg);
	if (host->dev_comp->clk_div_bits == 8)
		sdr_set_field(host->base + MSDC_CFG,
			      MSDC_CFG_CKMOD | MSDC_CFG_CKDIV,
			      (mode << 8) | div);
	else
		sdr_set_field(host->base + MSDC_CFG,
			      MSDC_CFG_CKMOD_EXTRA | MSDC_CFG_CKDIV_EXTRA,
			      (mode << 12) | div);

	clk_prepare_enable(host->src_clk_cg);
	readl_poll_timeout(host->base + MSDC_CFG, val, (val & MSDC_CFG_CKSTB), 0, 0);
	if (host->clk_auto_powerdown) {
		if (host->mclk == 0 && (mmc->caps2 & MMC_CAP2_NO_MMC)
			&& mmc->ios.signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
			/* Refer to mmc_host_set_uhs_voltage() for SD/SDIO */
			dev_alert(host->dev, "[%s]: enable clk free run 1ms+ for 1.8v switch\n",
				__func__);
			/* Clock will be running freely even if no command or data is transmitted */
			sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
			usleep_range(1000, 1500);
			/* Clock will be gated to 0 if no command or data is transmitted. */
			sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
		}
	} else
		/* Clock will be running freely even if no command or data is transmitted */
		sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
	mmc->actual_clock = sclk;
	host->mclk = hz;
	host->timing = timing;
	/* need because clk changed. */
#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
	host->max_busy_timeout_us = msdc_max_busy_timeout_us(host);
	// msdc_set_data_timeout(host, host->data_timeout_ns);
#endif
	msdc_set_timeout(host, host->timeout_ns, host->timeout_clks);
	sdr_set_bits(host->base + MSDC_INTEN, flags);

	/*
	 * mmc_select_hs400() will drop to 50Mhz and High speed mode,
	 * tune result of hs200/200Mhz is not suitable for 50Mhz
	 */
	if (mmc->actual_clock <= 52000000) {
		writel(host->def_tune_para.iocon, host->base + MSDC_IOCON);
		if (host->top_base) {
			writel(host->def_tune_para.emmc_top_control,
			       host->top_base + EMMC_TOP_CONTROL);
			writel(host->def_tune_para.emmc_top_cmd,
			       host->top_base + EMMC_TOP_CMD);
		} else {
			writel(host->def_tune_para.pad_tune,
			       host->base + tune_reg);
		}
	} else {
		writel(host->saved_tune_para.iocon, host->base + MSDC_IOCON);
		writel(host->saved_tune_para.pad_cmd_tune,
		       host->base + PAD_CMD_TUNE);
		if (host->top_base) {
			writel(host->saved_tune_para.emmc_top_control,
			       host->top_base + EMMC_TOP_CONTROL);
			writel(host->saved_tune_para.emmc_top_cmd,
			       host->top_base + EMMC_TOP_CMD);
		} else {
			writel(host->saved_tune_para.pad_tune,
			       host->base + tune_reg);
		}
	}

	if (timing == MMC_TIMING_MMC_HS400 &&
	    host->dev_comp->hs400_tune)
		sdr_set_field(host->base + tune_reg,
			      MSDC_PAD_TUNE_CMDRRDLY,
			      host->hs400_cmd_int_delay);
	dev_dbg(host->dev, "sclk: %d, timing: %d\n", mmc->actual_clock,
		timing);
}

static inline u32 msdc_cmd_find_resp(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	u32 resp;

	switch (mmc_resp_type(cmd)) {
		/* Actually, R1, R5, R6, R7 are the same */
	case MMC_RSP_R1:
		resp = 0x1;
		break;
	case MMC_RSP_R1B:
		resp = 0x7;
		break;
	case MMC_RSP_R2:
		resp = 0x2;
		break;
	case MMC_RSP_R3:
		resp = 0x3;
		break;
	case MMC_RSP_NONE:
	default:
		resp = 0x0;
		break;
	}

	return resp;
}

static inline u32 msdc_cmd_prepare_raw_cmd(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	struct mmc_host *mmc = mmc_from_priv(host);
	/* rawcmd :
	 * vol_swt << 30 | auto_cmd << 28 | blklen << 16 | go_irq << 15 |
	 * stop << 14 | rw << 13 | dtype << 11 | rsptyp << 7 | brk << 6 | opcode
	 */
	u32 opcode = cmd->opcode;
	u32 resp = msdc_cmd_find_resp(host, mrq, cmd);
	u32 rawcmd = (opcode & 0x3f) | ((resp & 0x7) << 7);

	host->cmd_rsp = resp;

#if 0
	if ((opcode == SD_IO_RW_DIRECT && cmd->flags == (unsigned int) -1) ||
	    opcode == MMC_STOP_TRANSMISSION)
#else
	if ((opcode == SD_IO_RW_DIRECT &&
			(((cmd->arg >> 9) & 0x1FFFF) == SDIO_CCCR_ABORT || cmd->flags == ((unsigned int) -1))) ||
	    opcode == MMC_STOP_TRANSMISSION)
#endif
		rawcmd |= (0x1 << 14);
	else if (opcode == SD_SWITCH_VOLTAGE)
		rawcmd |= (0x1 << 30);
	else if (opcode == SD_APP_SEND_SCR ||
		 opcode == SD_APP_SEND_NUM_WR_BLKS ||
		 (opcode == SD_SWITCH && mmc_cmd_type(cmd) == MMC_CMD_ADTC) ||
		 (opcode == SD_APP_SD_STATUS && mmc_cmd_type(cmd) == MMC_CMD_ADTC) ||
		 (opcode == MMC_SEND_EXT_CSD && mmc_cmd_type(cmd) == MMC_CMD_ADTC))
		rawcmd |= (0x1 << 11);

	if (cmd->data) {
		struct mmc_data *data = cmd->data;

		if (mmc_op_multi(opcode)) {
			if (mmc_card_mmc(mmc->card) && mrq->sbc &&
			    !(mrq->sbc->arg & 0xFFFF0000))
				rawcmd |= 0x2 << 28; /* AutoCMD23 */
		}

		rawcmd |= ((data->blksz & 0xFFF) << 16);
		if (data->flags & MMC_DATA_WRITE)
			rawcmd |= (0x1 << 13);
		if (data->blocks > 1)
			rawcmd |= (0x2 << 11);
		else
			rawcmd |= (0x1 << 11);
		/* Always use dma mode */
		sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_PIO);

#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
		/* Use msdc_set_data_timeout() instead */
#else
		if (host->timeout_ns != data->timeout_ns ||
		    host->timeout_clks != data->timeout_clks)
			msdc_set_timeout(host, data->timeout_ns,
					data->timeout_clks);
#endif

		writel(data->blocks, host->base + SDC_BLK_NUM);
	}
	return rawcmd;
}

static void msdc_start_data(struct msdc_host *host, struct mmc_request *mrq,
			    struct mmc_command *cmd, struct mmc_data *data)
{
	bool read;

	WARN_ON(host->data);
	host->data = data;
	read = data->flags & MMC_DATA_READ;

#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
	/* msdc_start_command() */
#else
	mod_delayed_work(system_wq, &host->req_timeout, DAT_TIMEOUT);
#endif
	msdc_dma_setup(host, &host->dma, data);
	sdr_set_bits(host->base + MSDC_INTEN, data_ints_mask);
	sdr_set_field(host->base + MSDC_DMA_CTRL, MSDC_DMA_CTRL_START, 1);
	dev_dbg(host->dev, "DMA start\n");
	dev_dbg(host->dev, "%s: cmd=%d DMA data: %d blocks; read=%d\n",
			__func__, cmd->opcode, data->blocks, read);
}

static int msdc_auto_cmd_done(struct msdc_host *host, int events,
		struct mmc_command *cmd)
{
	u32 *rsp = cmd->resp;

	rsp[0] = readl(host->base + SDC_ACMD_RESP);

	if (events & MSDC_INT_ACMDRDY) {
		cmd->error = 0;
	} else {
		msdc_reset_hw(host);
		if (events & MSDC_INT_ACMDCRCERR) {
			cmd->error = -EILSEQ;
			host->error |= REQ_STOP_EIO;
		} else if (events & MSDC_INT_ACMDTMO) {
			cmd->error = -ETIMEDOUT;
			host->error |= REQ_STOP_TMO;
		}
		dev_err(host->dev,
			"%s: AUTO_CMD%d arg=%08X; rsp %08X; cmd_error=%d\n",
			__func__, cmd->opcode, cmd->arg, rsp[0], cmd->error);
	}
	return cmd->error;
}

/*
 * msdc_recheck_sdio_irq - recheck whether the SDIO irq is lost
 *
 * Host controller may lost interrupt in some special case.
 * Add SDIO irq recheck mechanism to make sure all interrupts
 * can be processed immediately
 */
static void msdc_recheck_sdio_irq(struct msdc_host *host)
{
	struct mmc_host *mmc = mmc_from_priv(host);
	u32 reg_int, reg_inten, reg_ps;

	if (mmc->caps & MMC_CAP_SDIO_IRQ) {
		reg_inten = readl(host->base + MSDC_INTEN);
		if (reg_inten & MSDC_INTEN_SDIOIRQ) {
			reg_int = readl(host->base + MSDC_INT);
			reg_ps = readl(host->base + MSDC_PS);
			if (!(reg_int & MSDC_INT_SDIOIRQ ||
			      reg_ps & MSDC_PS_DATA1)) {
				__msdc_enable_sdio_irq(host, 0);
				sdio_signal_irq(mmc);
			}
		}
	}
}

static void msdc_track_cmd_data(struct msdc_host *host,
				struct mmc_command *cmd, struct mmc_data *data)
{
	if (host->error &&
		cmd->opcode != MMC_SEND_TUNING_BLOCK &&
		cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200)
		dev_dbg(host->dev, "%s: cmd=%d arg=%08X; host->error=0x%08X\n",
			__func__, cmd->opcode, cmd->arg, host->error);
}

static void msdc_request_done(struct msdc_host *host, struct mmc_request *mrq)
{
	unsigned long flags;

	/*
	 * No need check the return value of cancel_delayed_work, as only ONE
	 * path will go here!
	 */
	cancel_delayed_work(&host->req_timeout);

	spin_lock_irqsave(&host->lock, flags);
	host->mrq = NULL;
	spin_unlock_irqrestore(&host->lock, flags);

	msdc_track_cmd_data(host, mrq->cmd, mrq->data);
	if (mrq->data)
		msdc_unprepare_data(host, mrq);
	if (host->error)
		msdc_reset_hw(host);
	mmc_request_done(mmc_from_priv(host), mrq);
	if (host->dev_comp->recheck_sdio_irq)
		msdc_recheck_sdio_irq(host);
}

/* returns true if command is fully handled; returns false otherwise */
static bool msdc_cmd_done(struct msdc_host *host, int events,
			  struct mmc_request *mrq, struct mmc_command *cmd)
{
	bool done = false;
	bool sbc_error;
	unsigned long flags;
	u32 *rsp;

	if (mrq->sbc && cmd == mrq->cmd &&
	    (events & (MSDC_INT_ACMDRDY | MSDC_INT_ACMDCRCERR
				   | MSDC_INT_ACMDTMO)))
		msdc_auto_cmd_done(host, events, mrq->sbc);

	sbc_error = mrq->sbc && mrq->sbc->error;

	if (!sbc_error && !(events & (MSDC_INT_CMDRDY
					| MSDC_INT_RSPCRCERR
					| MSDC_INT_CMDTMO)))
		return done;

	spin_lock_irqsave(&host->lock, flags);
	done = !host->cmd;
	host->cmd = NULL;
	spin_unlock_irqrestore(&host->lock, flags);

	if (done)
		return true;
	rsp = cmd->resp;

	sdr_clr_bits(host->base + MSDC_INTEN, cmd_ints_mask);
#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
	sdr_clr_bits(host->base + MSDC_PATCH_BIT1, MSDC_PB1_BUSY_CHECK_SEL);
#endif

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			rsp[0] = readl(host->base + SDC_RESP3);
			rsp[1] = readl(host->base + SDC_RESP2);
			rsp[2] = readl(host->base + SDC_RESP1);
			rsp[3] = readl(host->base + SDC_RESP0);
		} else {
			rsp[0] = readl(host->base + SDC_RESP0);
		}
	}

	if (!sbc_error && !(events & MSDC_INT_CMDRDY)) {
		if (events & MSDC_INT_CMDTMO ||
		    (cmd->opcode != MMC_SEND_TUNING_BLOCK &&
		     cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200))
			/*
			 * should not clear fifo/interrupt as the tune data
			 * may have alreay come when cmd19/cmd21 gets response
			 * CRC error.
			 */
			msdc_reset_hw(host);
		if (events & MSDC_INT_RSPCRCERR) {
			struct mmc_host *mmc = mmc_from_priv(host);

			cmd->error = -EILSEQ;
			host->error |= REQ_CMD_EIO;

			if (!(mmc->retune_crc_disable)
					&& cmd->opcode != MMC_SEND_TUNING_BLOCK
					&& cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200
					&& cmd->opcode != MMC_SEND_STATUS) {
				if (mmc->caps & MMC_CAP_NONREMOVABLE) {
					dev_alert(host->dev, "need both-edge retune since cmd%d crc error\n", cmd->opcode);
					/**
					 * The current tuned phase delay might be not good enough,
					 * Let the system tune the both edges and get the new best delay from rising or falling.
					 */
					host->tuning_both_edges = true;
				} else
					dev_alert(host->dev, "need retune since cmd%d crc error\n", cmd->opcode);

				/* call mmc_retune_needed() here to "reduce" the phase tuning error.
				 *
				 * [  140.760392] mtk-sd 11240000.mmc: [name:mtk_sd&]Rising phase error map: 0x0
				 * [  140.761372] mtk-sd 11240000.mmc: [name:mtk_sd&]Falling phase error map: 0x0
				 * [  140.761382] mtk-sd 11240000.mmc: [name:mtk_sd&]Final pad rising delay: 0xff [map:0]
				 * [  140.761385] mtk-sd 11240000.mmc: [name:mtk_sd&]Tune cmd/data fail!
				 */
				mmc_retune_needed(mmc);
			}
		} else if (events & MSDC_INT_CMDTMO) {
			cmd->error = -ETIMEDOUT;
			host->error |= REQ_CMD_TMO;
		}
	}
	if (cmd->error &&
		cmd->opcode != MMC_SEND_TUNING_BLOCK &&
		cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200) {
		if ((cmd->error == -EILSEQ && msdc_ratelimit(&host->cmd_err_crc_ratelimit)) ||
			(cmd->error != -EILSEQ && msdc_ratelimit(&host->cmd_err_ratelimit))) {
#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
			dev_err(host->dev,
				"%s: cmd=%d arg=0x%x flag=0x%x; rsp=0x%x; cmd_error=%d; events=0x%x; dto_ms=%llu\n",
				__func__, cmd->opcode, cmd->arg, cmd->flags, rsp[0],
				cmd->error, events, host->data_timeout_ns / NSEC_PER_MSEC);
#else
			dev_err(host->dev,
				"%s: cmd=%d arg=0x%x flag=0x%x; rsp=0x%x; cmd_error=%d; events=0x%x\n",
				__func__, cmd->opcode, cmd->arg, cmd->flags, rsp[0],
				cmd->error, events);
#endif
		}
	}

	msdc_cmd_next(host, mrq, cmd);
	return true;
}

/* It is the core layer's responsibility to ensure card status
 * is correct before issue a request. but host design do below
 * checks recommended.
 */
static inline bool msdc_cmd_is_ready(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	u32 val;
	int ret;

	/* The max busy time we can endure is 20ms */
	ret = readl_poll_timeout_atomic(host->base + SDC_STS, val,
					!(val & SDC_STS_CMDBUSY), 1, 20000);
	if (ret) {
		dev_err(host->dev, "CMD bus busy detected\n");
		host->error |= REQ_CMD_BUSY;
		msdc_cmd_done(host, MSDC_INT_CMDTMO, mrq, cmd);
		return false;
	}

	if (mmc_resp_type(cmd) == MMC_RSP_R1B || cmd->data) {
		/* R1B or with data, should check SDCBUSY */
		ret = readl_poll_timeout_atomic(host->base + SDC_STS, val,
						!(val & SDC_STS_SDCBUSY), 1, 20000);
		if (ret) {
			dev_err(host->dev, "Controller busy detected\n");
			host->error |= REQ_CMD_BUSY;
			msdc_cmd_done(host, MSDC_INT_CMDTMO, mrq, cmd);
			return false;
		}
	}
	return true;
}

#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
/* Refer to sdhci_target_timeout() */
static u64 msdc_target_timeout(struct msdc_host *host,
		 struct mmc_command *cmd, struct mmc_data *data)
{
	//struct mmc_host *mmc = mmc_from_priv(host);
	u64 target_timeout; /* us */

	/* timeout in us */
	if (!data) {
		if (!cmd)
			target_timeout = host->max_busy_timeout_us;
		else if (cmd->busy_timeout)
			target_timeout = 1000ULL * cmd->busy_timeout; /* ms -> us */
		else if (cmd->flags & MMC_RSP_BUSY)
			target_timeout = host->max_busy_timeout_us;
		else
			target_timeout = 500000ULL; /* 500ms -> us */
#ifdef ICOM_MTK_MMC_NEW_TIMEOUT_VDEBUG
		dev_info(host->dev, "target cmd timeout: %llu us\n", target_timeout);
#endif
	} else {
		target_timeout = DIV_ROUND_UP(data->timeout_ns, 1000); /* ns -> us */
		/* Refer to mmc_set_data_timeout() */
		if (host->mclk && data->timeout_clks) {
			u64 val;

			/*
			 * data->timeout_clks is in units of clock cycles.
			 * host->clock is in Hz.  target_timeout is in us.
			 * Hence, us = 1000000 * cycles / Hz.  Round up.
			 */
			val = 1000000ULL * data->timeout_clks + host->mclk - 1;
			do_div(val, host->mclk);
			target_timeout += val;
		}
#ifdef ICOM_MTK_MMC_NEW_TIMEOUT_VDEBUG
		dev_info(host->dev, "target data timeout: %llu us (nr=%u)\n", target_timeout, data->blocks);
#endif
	}

	return target_timeout;
}

/* Refer to sdhci_calc_timeout() & sdhci_calc_sw_timeout() */
static u64 msdc_calc_timeout(struct msdc_host *host,
		struct mmc_command *cmd)
{
	struct mmc_data *data = cmd->data;
	u64 target_timeout;
	u64 data_timeout_ns;

	target_timeout = msdc_target_timeout(host, cmd, data); /* us */
	target_timeout *= NSEC_PER_USEC; /* ns */

	if (data) {
#if 0
		struct mmc_host *mmc = mmc_from_priv(host);
		struct mmc_ios *ios = &mmc->ios;
		unsigned char bus_width = 1 << ios->bus_width;
		unsigned int blksz = data->blksz;
		unsigned int freq = mmc->actual_clock ? : host->mclk;
		u64 transfer_time = (u64)blksz * NSEC_PER_SEC * (8 / bus_width);

		do_div(transfer_time, freq);
		/* multiply by '2' to account for any unknowns */
		transfer_time = transfer_time * 2;

		/* calculate timeout for the entire data */
		data_timeout_ns = target_timeout * data->blocks +
				     transfer_time;

#ifdef ICOM_MTK_MMC_NEW_TIMEOUT_VVDEBUG
		dev_info(host->dev, "calc data timeout: %u us (nr=%u) transfer: %u ns (blksz=%u)\n",
				data_timeout_ns / NSEC_PER_USEC, data->blocks, transfer_time, blksz);
#endif
#else
		/* add more time below to skip transfer_time calculation */
		/* calculate timeout for the entire data */
		data_timeout_ns = target_timeout * data->blocks + (20 * NSEC_PER_MSEC)/*20ms*/;

#ifdef ICOM_MTK_MMC_NEW_TIMEOUT_VVDEBUG
		dev_info(host->dev, "calc data timeout: %u us (nr=%u)\n",
				data_timeout_ns / NSEC_PER_USEC, data->blocks);
#endif
#endif
	} else {
		data_timeout_ns = target_timeout;
#ifdef ICOM_MTK_MMC_NEW_TIMEOUT_VVDEBUG
		dev_info(host->dev, "calc cmd data timeout: %u us\n",
				data_timeout_ns / NSEC_PER_USEC);
#endif
	}

	if (data_timeout_ns) {
		data_timeout_ns += (100ULL * NSEC_PER_MSEC); /* more time: >=MMC_CMD_TRANSFER_TIME */
		if (data_timeout_ns < (500ULL * NSEC_PER_MSEC))
			data_timeout_ns = 500ULL * NSEC_PER_MSEC;
	}

	return data_timeout_ns;
}
#endif

static void msdc_start_command(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	u32 rawcmd;
	unsigned long flags;
#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
	u64 data_timeout_ns;
	unsigned long sw_timeout_jiffies;
#endif

	WARN_ON(host->cmd);
	host->cmd = cmd;

	mod_delayed_work(system_wq, &host->req_timeout, DAT_TIMEOUT);
	if (!msdc_cmd_is_ready(host, mrq, cmd))
		return;

	if ((readl(host->base + MSDC_FIFOCS) & MSDC_FIFOCS_TXCNT) >> 16 ||
	    readl(host->base + MSDC_FIFOCS) & MSDC_FIFOCS_RXCNT) {
		dev_err(host->dev, "TX/RX FIFO non-empty before start of IO. Reset\n");
		msdc_reset_hw(host);
	}

	cmd->error = 0;
	rawcmd = msdc_cmd_prepare_raw_cmd(host, mrq, cmd);

	spin_lock_irqsave(&host->lock, flags);
#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
	data_timeout_ns = msdc_calc_timeout(host, cmd);
	if (data_timeout_ns != host->data_timeout_ns)
		msdc_set_data_timeout(host, data_timeout_ns);

	if (host->data_timeout_ns)
		sw_timeout_jiffies = nsecs_to_jiffies(host->data_timeout_ns) + HZ;
	else if (!cmd->data)
		sw_timeout_jiffies = CMD_TIMEOUT + (HZ >> 1)/*~500ms*/;
	else
		sw_timeout_jiffies = DAT_TIMEOUT;
	mod_delayed_work(system_wq, &host->req_timeout, sw_timeout_jiffies);

	if (cmd->flags & MMC_RSP_BUSY)
		sdr_set_bits(host->base + MSDC_PATCH_BIT1, MSDC_PB1_BUSY_CHECK_SEL);
#endif
	sdr_set_bits(host->base + MSDC_INTEN, cmd_ints_mask);
	spin_unlock_irqrestore(&host->lock, flags);

	writel(cmd->arg, host->base + SDC_ARG);
	writel(rawcmd, host->base + SDC_CMD);
}

static void msdc_cmd_next(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	if ((cmd->error &&
	    !(cmd->error == -EILSEQ &&
	      (cmd->opcode == MMC_SEND_TUNING_BLOCK ||
	       cmd->opcode == MMC_SEND_TUNING_BLOCK_HS200))) ||
	    (mrq->sbc && mrq->sbc->error))
		msdc_request_done(host, mrq);
	else if (cmd == mrq->sbc)
		msdc_start_command(host, mrq, mrq->cmd);
	else if (!cmd->data)
		msdc_request_done(host, mrq);
	else
		msdc_start_data(host, mrq, cmd, cmd->data);
}

static void msdc_ops_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);

	host->error = 0;
	WARN_ON(host->mrq);
	host->mrq = mrq;

	if (mrq->data)
		msdc_prepare_data(host, mrq);

	/* if SBC is required, we have HW option and SW option.
	 * if HW option is enabled, and SBC does not have "special" flags,
	 * use HW option,  otherwise use SW option
	 */
	if (mrq->sbc && (!mmc_card_mmc(mmc->card) ||
	    (mrq->sbc->arg & 0xFFFF0000)))
		msdc_start_command(host, mrq, mrq->sbc);
	else
		msdc_start_command(host, mrq, mrq->cmd);
}

static void msdc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	msdc_prepare_data(host, mrq);
	data->host_cookie |= MSDC_ASYNC_FLAG;
}

static void msdc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
		int err)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_data *data;

	data = mrq->data;
	if (!data)
		return;
	if (data->host_cookie) {
		data->host_cookie &= ~MSDC_ASYNC_FLAG;
		msdc_unprepare_data(host, mrq);
	}
}

static void msdc_data_xfer_next(struct msdc_host *host,
				struct mmc_request *mrq, struct mmc_data *data)
{
	if (mmc_op_multi(mrq->cmd->opcode) && mrq->stop && !mrq->stop->error &&
	    !mrq->sbc)
		msdc_start_command(host, mrq, mrq->stop);
	else
		msdc_request_done(host, mrq);
}

static void msdc_data_xfer_done(struct msdc_host *host, u32 events,
				struct mmc_request *mrq, struct mmc_data *data)
{
	struct mmc_command *stop;
	unsigned long flags;
	bool done;
	unsigned int check_data = events &
	    (MSDC_INT_XFER_COMPL | MSDC_INT_DATCRCERR | MSDC_INT_DATTMO
	     | MSDC_INT_DMA_BDCSERR | MSDC_INT_DMA_GPDCSERR
	     | MSDC_INT_DMA_PROTECT);

	spin_lock_irqsave(&host->lock, flags);
	done = !host->data;
	if (check_data)
		host->data = NULL;
	spin_unlock_irqrestore(&host->lock, flags);

	if (done)
		return;
	stop = data->stop;

	if (check_data || (stop && stop->error)) {
		u32 val;
		int ret;

		dev_dbg(host->dev, "DMA status: 0x%8X\n",
				readl(host->base + MSDC_DMA_CFG));
		sdr_set_field(host->base + MSDC_DMA_CTRL, MSDC_DMA_CTRL_STOP,
				1);

		ret = readl_poll_timeout_atomic(host->base + MSDC_DMA_CTRL, val,
						!(val & MSDC_DMA_CTRL_STOP), 1, 20000);
		if (ret)
			dev_err(host->dev, "DMA stop timed out\n");

		ret = readl_poll_timeout_atomic(host->base + MSDC_DMA_CFG, val,
						!(val & MSDC_DMA_CFG_STS), 1, 20000);
		if (ret)
			dev_err(host->dev, "DMA inactive timed out\n");

		sdr_clr_bits(host->base + MSDC_INTEN, data_ints_mask);
		dev_dbg(host->dev, "DMA stop\n");

		if ((events & MSDC_INT_XFER_COMPL) && (!stop || !stop->error)) {
			data->bytes_xfered = data->blocks * data->blksz;
		} else {
			dev_dbg(host->dev, "interrupt events: %x\n", events);
			msdc_reset_hw(host);
			host->error |= REQ_DAT_ERR;
			data->bytes_xfered = 0;

			if (events & MSDC_INT_DATTMO)
				data->error = -ETIMEDOUT;
			else if (events & MSDC_INT_DATCRCERR) {
				struct mmc_host *mmc = mmc_from_priv(host);

				data->error = -EILSEQ;

				if (!(mmc->retune_crc_disable) && mrq && mrq->cmd
						&& mrq->cmd->opcode != MMC_SEND_TUNING_BLOCK
						&& mrq->cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200
						&& mrq->cmd->opcode != MMC_SEND_STATUS) {
					if (mmc->caps & MMC_CAP_NONREMOVABLE) {
						dev_alert(host->dev, "need both-edge retune since cmd%d data crc error\n",
								mrq->cmd->opcode);
						/**
						 * The current tuned phase delay might be not good enough,
						 * Let the system tune the both edges and get the new best delay from rising or falling.
						 */
						host->tuning_both_edges = true;
					} else
						dev_alert(host->dev, "need retune since cmd%d data crc error\n",
								mrq->cmd->opcode);

					mmc_retune_needed(mmc);
				}
			}

			if (mrq && mrq->cmd &&
					mrq->cmd->opcode != MMC_SEND_TUNING_BLOCK &&
					mrq->cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200) {
				if ((data->error == -EILSEQ && msdc_ratelimit(&host->data_err_crc_ratelimit)) ||
						(data->error != -EILSEQ && msdc_ratelimit(&host->data_err_ratelimit))) {
#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
					dev_err(host->dev, "%s: cmd=%d data_flags=0x%x blocks=%u data_error=%d xfer_size=%d dto_ms=%llu",
						__func__, mrq->cmd->opcode, data->flags, data->blocks,
						data->error, data->bytes_xfered, host->data_timeout_ns / NSEC_PER_MSEC);
#else
					dev_err(host->dev, "%s: cmd=%d data_flags=0x%x blocks=%u data_error=%d xfer_size=%d",
						__func__, mrq->cmd->opcode, data->flags, data->blocks,
						data->error, data->bytes_xfered);
#endif
				}
			} else {
				if ((data->error == -EILSEQ && msdc_ratelimit(&host->data_err_crc_ratelimit)) ||
						(data->error != -EILSEQ && msdc_ratelimit(&host->data_err_ratelimit))) {
#ifdef ICOM_MTK_MMC_NEW_TIMEOUT
					dev_err(host->dev, "%s: data_flags=0x%x blocks=%u data_error=%d xfer_size=%d dto_ms=%llu",
						__func__, data->flags, data->blocks,
						data->error, data->bytes_xfered, host->data_timeout_ns / NSEC_PER_MSEC);
#else
					dev_err(host->dev, "%s: data_flags=0x%x blocks=%u data_error=%d xfer_size=%d",
						__func__, data->flags, data->blocks,
						data->error, data->bytes_xfered);
#endif
				}
			}
		}

		msdc_data_xfer_next(host, mrq, data);
	}
}

static void msdc_set_buswidth(struct msdc_host *host, u32 width)
{
	u32 val = readl(host->base + SDC_CFG);

	val &= ~SDC_CFG_BUSWIDTH;

	switch (width) {
	default:
	case MMC_BUS_WIDTH_1:
		val |= (MSDC_BUS_1BITS << 16);
		break;
	case MMC_BUS_WIDTH_4:
		val |= (MSDC_BUS_4BITS << 16);
		break;
	case MMC_BUS_WIDTH_8:
		val |= (MSDC_BUS_8BITS << 16);
		break;
	}

	writel(val, host->base + SDC_CFG);
	dev_dbg(host->dev, "Bus Width = %d", width);
}

static int msdc_ops_switch_volt(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret;

	if (!IS_ERR(mmc->supply.vqmmc)) {
		if (ios->signal_voltage != MMC_SIGNAL_VOLTAGE_330 &&
		    ios->signal_voltage != MMC_SIGNAL_VOLTAGE_180) {
			dev_err(host->dev, "Unsupported signal voltage!\n");
			return -EINVAL;
		}

		ret = mmc_regulator_set_vqmmc(mmc, ios);
		if (ret < 0) {
			dev_dbg(host->dev, "Regulator set error %d (%d)\n",
				ret, ios->signal_voltage);
			return ret;
		}

		if (ios->power_mode == MMC_POWER_ON) {
			if (host->signal_voltage != ios->signal_voltage) {
				host->signal_voltage = ios->signal_voltage;
				/* Apply different pinctrl settings for different signal voltage */
				if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
					dev_dbg(host->dev, "%s: switch-volt: MMC_SIGNAL_VOLTAGE_180 & pins-uhs\n", mmc_hostname(mmc));
					pinctrl_select_state(host->pinctrl, host->pins_uhs);
				} else if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
					dev_dbg(host->dev, "%s: switch-volt: MMC_SIGNAL_VOLTAGE_330 & pins-normal\n", mmc_hostname(mmc));
					pinctrl_select_state(host->pinctrl, host->pins_normal);
				}
			}
		}
	}
	return 0;
}

static int msdc_card_busy(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 status = readl(host->base + MSDC_PS);

	/* only check if data0 is low */
	return !(status & BIT(16));
}

static void msdc_request_timeout(struct work_struct *work)
{
	struct msdc_host *host = container_of(work, struct msdc_host,
			req_timeout.work);

	/* simulate HW timeout status */
	dev_err(host->dev, "%s: aborting cmd/data/mrq\n", __func__);
	if (host->mrq) {
		dev_err(host->dev, "%s: aborting mrq=%p cmd=%d\n", __func__,
				host->mrq, host->mrq->cmd->opcode);
		if (host->cmd) {
			dev_err(host->dev, "%s: aborting cmd=%d\n",
					__func__, host->cmd->opcode);
			msdc_cmd_done(host, MSDC_INT_CMDTMO, host->mrq,
					host->cmd);
		} else if (host->data) {
			dev_err(host->dev, "%s: abort data: cmd%d; %d blocks\n",
					__func__, host->mrq->cmd->opcode,
					host->data->blocks);
			msdc_data_xfer_done(host, MSDC_INT_DATTMO, host->mrq,
					host->data);
		}
	}
}

static void __msdc_enable_sdio_irq(struct msdc_host *host, int enb)
{
	if (enb) {
		sdr_set_bits(host->base + MSDC_INTEN, MSDC_INTEN_SDIOIRQ);
		sdr_set_bits(host->base + SDC_CFG, SDC_CFG_SDIOIDE);
		if (host->dev_comp->recheck_sdio_irq)
			msdc_recheck_sdio_irq(host);
	} else {
		sdr_clr_bits(host->base + MSDC_INTEN, MSDC_INTEN_SDIOIRQ);
		sdr_clr_bits(host->base + SDC_CFG, SDC_CFG_SDIOIDE);
	}
}

static void msdc_enable_sdio_irq(struct mmc_host *mmc, int enb)
{
	unsigned long flags;
	struct msdc_host *host = mmc_priv(mmc);
	int ret;
	unsigned char data;
	struct sdio_func *func;

	spin_lock_irqsave(&host->lock, flags);
	__msdc_enable_sdio_irq(host, enb);
	spin_unlock_irqrestore(&host->lock, flags);

	if (host->enable_async_irq && enb) {
		/* Refer to mtk_sdio_async_irq_enable() */

		/* Enable capability to write CCCR */
		func = mmc->card->sdio_func[0];

		/* Read CCCR 0x16 (interrupt extension)*/
		data = sdio_f0_readb(func, SDIO_CCCR_INTERRUPT_EXT, &ret);
		if (ret) {
			dev_err(host->dev, "%s: failed to read SDIO_CCCR_INTERRUPT_EXT: %d\n",
					mmc_hostname(mmc), ret);
			host->enable_async_irq = false;
			host->sdio_eint_ready = false;
			goto out;
		}

		if (!(data & SDIO_INTERRUPT_EXT_SAI)) {
			/* SAI = 0 */
			dev_warn(host->dev, "%s: no SDIO CCCR Async-IRQ capability\n",
					mmc_hostname(mmc));
			host->enable_async_irq = false;
			host->sdio_eint_ready = false;
		} else if (data & SDIO_INTERRUPT_EXT_EAI) {
			/* EAI = 1 */
			dev_info(host->dev, "%s: SDIO CCCR Async-IRQ was enabled\n",
					mmc_hostname(mmc));
		} else {
			/* EAI = 0 */
			unsigned int quirks_bak;

			/* Set EAI bit */
			data |= SDIO_INTERRUPT_EXT_EAI;

			/* Enable capability to write CCCR */
			quirks_bak = func->card->quirks;
			func->card->quirks |= MMC_QUIRK_LENIENT_FN0;

			/* Write CCCR into card */
			sdio_f0_writeb(func, data, SDIO_CCCR_INTERRUPT_EXT, &ret);
			func->card->quirks = quirks_bak;
			if (ret) {
				dev_err(host->dev, "%s: failed to write SDIO_CCCR_INTERRUPT_EXT: %d\n",
						mmc_hostname(mmc), ret);
				host->enable_async_irq = false;
				host->sdio_eint_ready = false;
				goto out;
			} else {
				data = sdio_f0_readb(func, SDIO_CCCR_INTERRUPT_EXT, &ret);
				if (ret) {
					dev_err(host->dev, "%s: failed to re-read SDIO_CCCR_INTERRUPT_EXT: %d\n",
							mmc_hostname(mmc), ret);
					host->enable_async_irq = false;
					host->sdio_eint_ready = false;
					goto out;
				}
				if (!(data & SDIO_INTERRUPT_EXT_EAI)) {
					/* EAI = 0 */
					dev_err(host->dev, "%s: failed to write SDIO_CCCR_INTERRUPT_EXT: 0x%x\n",
							mmc_hostname(mmc), data);
					host->enable_async_irq = false;
					host->sdio_eint_ready = false;
					goto out;
				}
				dev_alert(host->dev, "%s: SDIO CCCR Async-IRQ enabled\n",
						mmc_hostname(mmc));
			}
		}
	}

out:
	if (mmc->card && !host->enable_async_irq) {
		if (enb)
			pm_runtime_get_noresume(host->dev);
		else
			pm_runtime_put_noidle(host->dev);
	}
}

#if IS_ENABLED(CONFIG_MMC_CQHCI)
static irqreturn_t msdc_cmdq_irq(struct msdc_host *host, u32 intsts)
{
	struct mmc_host *mmc = mmc_from_priv(host);
	int cmd_err = 0, dat_err = 0;

	if (intsts & MSDC_INT_RSPCRCERR) {
		cmd_err = -EILSEQ;
		dev_err(host->dev, "%s: CMD CRC ERR", __func__);
	} else if (intsts & MSDC_INT_CMDTMO) {
		cmd_err = -ETIMEDOUT;
		dev_err(host->dev, "%s: CMD TIMEOUT ERR", __func__);
	}

	if (intsts & MSDC_INT_DATCRCERR) {
		dat_err = -EILSEQ;
		dev_err(host->dev, "%s: DATA CRC ERR", __func__);
	} else if (intsts & MSDC_INT_DATTMO) {
		dat_err = -ETIMEDOUT;
		dev_err(host->dev, "%s: DATA TIMEOUT ERR", __func__);
	}

	if (cmd_err || dat_err) {
		dev_err(host->dev, "cmd_err = %d, dat_err =%d, intsts = 0x%x",
			cmd_err, dat_err, intsts);
	}

	return cqhci_irq(mmc, 0, cmd_err, dat_err);
}
#endif

static irqreturn_t msdc_irq(int irq, void *dev_id)
{
	struct msdc_host *host = (struct msdc_host *) dev_id;
	struct mmc_host *mmc = mmc_from_priv(host);

	while (true) {
		struct mmc_request *mrq;
		struct mmc_command *cmd;
		struct mmc_data *data;
		u32 events, event_mask;

		spin_lock(&host->lock);
		events = readl(host->base + MSDC_INT);
		event_mask = readl(host->base + MSDC_INTEN);
		if ((events & event_mask) & MSDC_INT_SDIOIRQ)
			__msdc_enable_sdio_irq(host, 0);
		/* clear interrupts */
		writel(events & event_mask, host->base + MSDC_INT);

		mrq = host->mrq;
		cmd = host->cmd;
		data = host->data;
		spin_unlock(&host->lock);

		if ((events & event_mask) & MSDC_INT_SDIOIRQ)
			sdio_signal_irq(mmc);

		if ((events & event_mask) & MSDC_INT_CDSC) {
			if (host->internal_cd)
				mmc_detect_change(mmc, msecs_to_jiffies(20));
			events &= ~MSDC_INT_CDSC;
		}

		if (!(events & (event_mask & ~MSDC_INT_SDIOIRQ)))
			break;

		if ((mmc->caps2 & MMC_CAP2_CQE) &&
		    (events & MSDC_INT_CMDQ)) {
#if IS_ENABLED(CONFIG_MMC_CQHCI)
			msdc_cmdq_irq(host, events);
#endif
			/* clear interrupts */
			writel(events, host->base + MSDC_INT);
			return IRQ_HANDLED;
		}

		if (!mrq) {
			dev_err(host->dev,
				"%s: MRQ=NULL; events=%08X; event_mask=%08X\n",
				__func__, events, event_mask);
			WARN_ON(1);
			break;
		}

		dev_dbg(host->dev, "%s: events=%08X\n", __func__, events);

		if (cmd)
			msdc_cmd_done(host, events, mrq, cmd);
		else if (data)
			msdc_data_xfer_done(host, events, mrq, data);
	}

	return IRQ_HANDLED;
}

static void msdc_init_hw(struct msdc_host *host)
{
	u32 val;
	u32 tune_reg = host->dev_comp->pad_tune_reg;
	struct mmc_host *mmc = mmc_from_priv(host);

	if (host->reset) {
		reset_control_assert(host->reset);
		usleep_range(10, 50);
		reset_control_deassert(host->reset);
	}

	/*
	 * Configure to MMC/SD mode, modify clock free running
	 * respectively because some wifi devices need no clock
	 * output in init stage.
	 */
	if (host->clk_auto_powerdown) {
		/* Clear MSDC_CFG_CKPDN bit always.
		 *   1'b0: Clock will be gated to 0 if no command or data is transmitted.
		 *   1'b1: Clock will be running freely even if no command or data is
		 *         transmitted.  (The clock may still be stopped when MSDC write data are
		 *         not enough or there is no space for the next read data.)
		 */
		sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_MODE);
		/* MTK default RESET value: Clock will be gated to 0 if no command or data is transmitted. */
		sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
	} else if (mmc->caps2 & MMC_CAP2_NO_SDIO) /* not SDIO */
		/* Clock will be running freely even if no command or data is transmitted */
		sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_MODE | MSDC_CFG_CKPDN);
	else /* SDIO */
		/* Clock will be gated to 0 if no command or data is transmitted. */
		sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_MODE);

	/* Reset */
	msdc_reset_hw(host);

	/* Disable and clear all interrupts */
	writel(0, host->base + MSDC_INTEN);
	val = readl(host->base + MSDC_INT);
	writel(val, host->base + MSDC_INT);

	/* Configure card detection */
	if (host->internal_cd) {
		sdr_set_field(host->base + MSDC_PS, MSDC_PS_CDDEBOUNCE,
			      DEFAULT_DEBOUNCE);
		sdr_set_bits(host->base + MSDC_PS, MSDC_PS_CDEN);
		sdr_set_bits(host->base + MSDC_INTEN, MSDC_INTEN_CDSC);
		sdr_set_bits(host->base + SDC_CFG, SDC_CFG_INSWKUP);
	} else {
		sdr_clr_bits(host->base + SDC_CFG, SDC_CFG_INSWKUP);
		sdr_clr_bits(host->base + MSDC_PS, MSDC_PS_CDEN);
		sdr_clr_bits(host->base + MSDC_INTEN, MSDC_INTEN_CDSC);
	}

	if (host->top_base) {
		writel(0, host->top_base + EMMC_TOP_CONTROL);
		writel(0, host->top_base + EMMC_TOP_CMD);
	} else {
		writel(0, host->base + tune_reg);
	}
	writel(0, host->base + MSDC_IOCON);
	sdr_set_field(host->base + MSDC_IOCON, MSDC_IOCON_DDLSEL, 0);
	writel(0x403c0046, host->base + MSDC_PATCH_BIT);
#if 0 /* MSDC_CKGEN_MSDC_DLY_SEL/MSDC_PB0_CKGEN_MSDC_DLY_SEL in MSDC_PATCH_BIT should be 0 */
	sdr_set_field(host->base + MSDC_PATCH_BIT, MSDC_CKGEN_MSDC_DLY_SEL, 1);
#else
	sdr_set_field(host->base + MSDC_PATCH_BIT, MSDC_CKGEN_MSDC_DLY_SEL, 
			host->ckgen_delay);
	dev_dbg(host->dev, "msdc_init_hw: ckgen-delay=%u\n", host->ckgen_delay);
#endif
	writel(0xffff4089, host->base + MSDC_PATCH_BIT1);
	/* AHB bus or AXI bus */
	if (host->ahb_bus_no_incr1) {
		/* MSDC_PATCH_BIT1=0xffff4089 */
		/**
		 * AHB bus will not support incr1 burst type: Set ENABLE_SINGLE_BURST bit.
		 *     1'b1: HW will send single burst type instead of incr1 type.
		 */
		sdr_set_bits(host->base + MSDC_PATCH_BIT1,
				MSDC_PATCH_BIT1_SINGLE_BURST);
		dev_dbg(host->dev, "%s: set ENABLE_SINGLE_BURST (MSDC_PATCH_BIT1=0x%08x)",
				__func__, readl(host->base + MSDC_PATCH_BIT1));
	} else {
		/* MSDC_PATCH_BIT1=0xfffe4089 */
		/**
		 * AXI bus: Clear ENABLE_SINGLE_BURST bit:
		 *     1'b0: HW will send incr1 burst type.
		 */
		sdr_clr_bits(host->base + MSDC_PATCH_BIT1,
				MSDC_PATCH_BIT1_SINGLE_BURST);
		dev_dbg(host->dev, "%s: clear ENABLE_SINGLE_BURST (MSDC_PATCH_BIT1=0x%08x)",
				__func__, readl(host->base + MSDC_PATCH_BIT1));
	}
	sdr_set_bits(host->base + EMMC50_CFG0, EMMC50_CFG_CFCSTS_SEL);

	if (host->dev_comp->stop_clk_fix) {
		sdr_set_field(host->base + MSDC_PATCH_BIT1,
			      MSDC_PATCH_BIT1_STOP_DLY, 3);
		sdr_clr_bits(host->base + SDC_FIFO_CFG,
			     SDC_FIFO_CFG_WRVALIDSEL);
		sdr_clr_bits(host->base + SDC_FIFO_CFG,
			     SDC_FIFO_CFG_RDVALIDSEL);
	}

	if (host->dev_comp->busy_check)
		sdr_clr_bits(host->base + MSDC_PATCH_BIT1, (1 << 7));

	if (host->dev_comp->async_fifo) {
		sdr_set_field(host->base + MSDC_PATCH_BIT2,
			      MSDC_PB2_RESPWAIT, 3);
		if (host->dev_comp->enhance_rx) {
			if (host->top_base)
				sdr_set_bits(host->top_base + EMMC_TOP_CONTROL,
					     SDC_RX_ENH_EN);
			else
				sdr_set_bits(host->base + SDC_ADV_CFG0,
					     SDC_RX_ENHANCE_EN);
		} else {
			sdr_set_field(host->base + MSDC_PATCH_BIT2,
				      MSDC_PB2_RESPSTSENSEL, 2);
			sdr_set_field(host->base + MSDC_PATCH_BIT2,
				      MSDC_PB2_CRCSTSENSEL, 2);
		}
		/* use async fifo, then no need tune internal delay */
		sdr_clr_bits(host->base + MSDC_PATCH_BIT2,
			     MSDC_PATCH_BIT2_CFGRESP);
		sdr_set_bits(host->base + MSDC_PATCH_BIT2,
			     MSDC_PATCH_BIT2_CFGCRCSTS);
	}

	if (host->dev_comp->support_64g)
		sdr_set_bits(host->base + MSDC_PATCH_BIT2,
			     MSDC_PB2_SUPPORT_64G);
	if (host->dev_comp->data_tune) {
		if (host->top_base) {
			sdr_set_bits(host->top_base + EMMC_TOP_CONTROL,
				     PAD_DAT_RD_RXDLY_SEL);
#ifdef PAD_DELAY_USE_64BITS
			sdr_set_bits(host->top_base + EMMC_TOP_CONTROL,
				     PAD_DAT_RD_RXDLY2_SEL);
#endif
			sdr_clr_bits(host->top_base + EMMC_TOP_CONTROL,
				     DATA_K_VALUE_SEL);
			sdr_set_bits(host->top_base + EMMC_TOP_CMD,
				     PAD_CMD_RD_RXDLY_SEL);
#ifdef PAD_DELAY_USE_64BITS
			sdr_set_bits(host->top_base + EMMC_TOP_CMD,
				     PAD_CMD_RD_RXDLY2_SEL);
#endif
		} else {
			sdr_set_bits(host->base + tune_reg,
				     MSDC_PAD_TUNE_RD_SEL |
				     MSDC_PAD_TUNE_CMD_SEL);
#ifdef PAD_DELAY_USE_64BITS
			sdr_set_bits(host->base + tune_reg + 4,
				     MSDC_PAD_TUNE_RD2_SEL |
				     MSDC_PAD_TUNE_CMD2_SEL);
#endif
		}
	} else {
		/* choose clock tune */
		if (host->top_base)
			sdr_set_bits(host->top_base + EMMC_TOP_CONTROL,
				     PAD_RXDLY_SEL);
		else
			sdr_set_bits(host->base + tune_reg,
				     MSDC_PAD_TUNE_RXDLYSEL);
	}

	if (mmc->caps2 & MMC_CAP2_NO_SDIO) { /* not SDIO */
		sdr_clr_bits(host->base + SDC_CFG, SDC_CFG_SDIO);
		sdr_clr_bits(host->base + MSDC_INTEN, MSDC_INTEN_SDIOIRQ);
		sdr_clr_bits(host->base + SDC_ADV_CFG0, SDC_DAT1_IRQ_TRIGGER);
	} else { /* SDIO */
		/* Configure to enable SDIO mode, otherwise SDIO CMD5 fails */
		sdr_set_bits(host->base + SDC_CFG, SDC_CFG_SDIO);

		/* Config SDIO device detect interrupt function */
		sdr_clr_bits(host->base + SDC_CFG, SDC_CFG_SDIOIDE);
		sdr_set_bits(host->base + SDC_ADV_CFG0, SDC_DAT1_IRQ_TRIGGER);
	}

	/* Configure to default data timeout */
	sdr_set_field(host->base + SDC_CFG, SDC_CFG_DTOC, 3);

	host->def_tune_para.iocon = readl(host->base + MSDC_IOCON);
	host->saved_tune_para.iocon = readl(host->base + MSDC_IOCON);
	if (host->top_base) {
		host->def_tune_para.emmc_top_control =
			readl(host->top_base + EMMC_TOP_CONTROL);
		host->def_tune_para.emmc_top_cmd =
			readl(host->top_base + EMMC_TOP_CMD);
		host->saved_tune_para.emmc_top_control =
			readl(host->top_base + EMMC_TOP_CONTROL);
		host->saved_tune_para.emmc_top_cmd =
			readl(host->top_base + EMMC_TOP_CMD);
	} else {
		host->def_tune_para.pad_tune = readl(host->base + tune_reg);
		host->saved_tune_para.pad_tune = readl(host->base + tune_reg);
	}
	dev_dbg(host->dev, "init hardware done!");
}

static void msdc_deinit_hw(struct msdc_host *host)
{
	u32 val;

	if (host->internal_cd) {
		/* Disabled card-detect */
		sdr_clr_bits(host->base + MSDC_PS, MSDC_PS_CDEN);
		sdr_clr_bits(host->base + SDC_CFG, SDC_CFG_INSWKUP);
	}

	/* Disable and clear all interrupts */
	writel(0, host->base + MSDC_INTEN);

	val = readl(host->base + MSDC_INT);
	writel(val, host->base + MSDC_INT);
}

/* init gpd and bd list in msdc_drv_probe */
static void msdc_init_gpd_bd(struct msdc_host *host, struct msdc_dma *dma)
{
	struct mt_gpdma_desc *gpd = dma->gpd;
	struct mt_bdma_desc *bd = dma->bd;
	dma_addr_t dma_addr;
	int i;

	memset(gpd, 0, sizeof(struct mt_gpdma_desc) * 2);

	dma_addr = dma->gpd_addr + sizeof(struct mt_gpdma_desc);
	gpd->gpd_info = GPDMA_DESC_BDP; /* hwo, cs, bd pointer */
	/* gpd->next is must set for desc DMA
	 * That's why must alloc 2 gpd structure.
	 */
	gpd->next = lower_32_bits(dma_addr);
	if (host->dev_comp->support_64g)
		gpd->gpd_info |= (upper_32_bits(dma_addr) & 0xf) << 24;

	dma_addr = dma->bd_addr;
	gpd->ptr = lower_32_bits(dma->bd_addr); /* physical address */
	if (host->dev_comp->support_64g)
		gpd->gpd_info |= (upper_32_bits(dma_addr) & 0xf) << 28;

	memset(bd, 0, sizeof(struct mt_bdma_desc) * MAX_BD_NUM);
	for (i = 0; i < (MAX_BD_NUM - 1); i++) {
		dma_addr = dma->bd_addr + sizeof(*bd) * (i + 1);
		bd[i].next = lower_32_bits(dma_addr);
		if (host->dev_comp->support_64g)
			bd[i].bd_info |= (upper_32_bits(dma_addr) & 0xf) << 24;
	}
}

/* Support to rollback to the default signal voltage. Refer to mmc_set_signal_voltage() */
static int msdc_set_signal_voltage(struct mmc_host *mmc, int signal_voltage)
{
	int err = 0;
	int old_signal_voltage = mmc->ios.signal_voltage;

	mmc->ios.signal_voltage = signal_voltage;
	if (mmc->ops->start_signal_voltage_switch)
		err = mmc->ops->start_signal_voltage_switch(mmc, &mmc->ios);

	if (err)
		mmc->ios.signal_voltage = old_signal_voltage;

	return err;
}

/* Support to rollback to the default signal voltage. Refer to mmc_set_initial_signal_voltage() */
static void msdc_set_initial_signal_voltage(struct msdc_host *host)
{
	struct mmc_host *mmc = mmc_from_priv(host);
	unsigned short old_ios_vdd;

	/* mmc_regulator_set_vqmmc() will check "mmc->ios.vdd" */
	old_ios_vdd = mmc->ios.vdd;
	mmc->ios.vdd = host->ios_vdd;

	/* Try to set signal voltage to 3.3V but fall back to 1.8v or 1.2v */
	if (!msdc_set_signal_voltage(mmc, MMC_SIGNAL_VOLTAGE_330))
		dev_dbg(mmc_dev(mmc), "set initial signal voltage of 3.3v\n");
	else if (!msdc_set_signal_voltage(mmc, MMC_SIGNAL_VOLTAGE_180))
		dev_dbg(mmc_dev(mmc), "set initial signal voltage of 1.8v\n");
	else if (!msdc_set_signal_voltage(mmc, MMC_SIGNAL_VOLTAGE_120))
		dev_dbg(mmc_dev(mmc), "set initial signal voltage of 1.2v\n");

	mmc->ios.vdd = old_ios_vdd;
}

static void msdc_ops_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret;

	msdc_set_buswidth(host, ios->bus_width);

	/* Suspend/Resume will do power off/on */
	switch (ios->power_mode) {
	case MMC_POWER_UP:
		host->signal_voltage = -1;
		if (!IS_ERR(mmc->supply.vmmc)) {
			msdc_init_hw(host);
			ret = mmc_regulator_set_ocr(mmc, mmc->supply.vmmc,
					ios->vdd);
			if (ret) {
				dev_err(host->dev, "Failed to set vmmc power!\n");
				return;
			}

			/* In order to rollback to the default signal voltage */
			host->ios_vdd = ios->vdd; /* store ios->vdd value */
		}
		if (host->vioa && !host->vioa_enabled) {
			ret = regulator_enable(host->vioa);
			if (ret)
				dev_err(host->dev, "Failed to set vio-a power!\n");
			else
				host->vioa_enabled = true;
		}
		if (host->viob && !host->viob_enabled) {
			ret = regulator_enable(host->viob);
			if (ret)
				dev_err(host->dev, "Failed to set vio-b power!\n");
			else
				host->viob_enabled = true;
		}
		break;
	case MMC_POWER_ON:
		if (!IS_ERR(mmc->supply.vqmmc) && !host->vqmmc_enabled) {
			ret = regulator_enable(mmc->supply.vqmmc);
			if (ret)
				dev_err(host->dev, "Failed to set vqmmc power!\n");
			else
				host->vqmmc_enabled = true;
		}
		if (host->pins_poweron) {
			dev_dbg(host->dev, "%s: POWER_ON: pins-power-on\n", mmc_hostname(mmc));
			pinctrl_select_state(host->pinctrl, host->pins_poweron);
		}
		if (host->signal_voltage != ios->signal_voltage) {
			host->signal_voltage = ios->signal_voltage;

			/* Apply different pinctrl settings for different signal voltage */
			if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
				dev_dbg(host->dev, "%s: POWER_ON: MMC_SIGNAL_VOLTAGE_180 & pins-uhs\n", mmc_hostname(mmc));
				pinctrl_select_state(host->pinctrl, host->pins_uhs);
			} else if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
				dev_dbg(host->dev, "%s: POWER_ON: MMC_SIGNAL_VOLTAGE_330 & pins-normal\n", mmc_hostname(mmc));
				pinctrl_select_state(host->pinctrl, host->pins_normal);
			} else
				dev_err(host->dev, "%s: unsupported signal voltage!\n", __func__);
		}
		break;
	case MMC_POWER_OFF:
		if (host->pins_poweroff) {
			dev_dbg(host->dev, "%s: POWER_OFF: pins-power-off\n", mmc_hostname(mmc));
			pinctrl_select_state(host->pinctrl, host->pins_poweroff);
		}
		if (!IS_ERR(mmc->supply.vqmmc) && host->vqmmc_enabled) {
			regulator_disable(mmc->supply.vqmmc);
			host->vqmmc_enabled = false;

			/* rollback to the default signal voltage */
			if (host->sd_poweroff_reset_signal_volt && !(mmc->caps2 & MMC_CAP2_NO_SD)) /* SD Card */
				msdc_set_initial_signal_voltage(host);
		}
		if (host->vioa && host->vioa_enabled) {
			regulator_disable(host->vioa);
			host->vioa_enabled = false;
		}
		if (host->viob && host->viob_enabled) {
			regulator_disable(host->viob);
			host->viob_enabled = false;
		}
		/* turn off vmmc finally to prevent to power leakage */
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		break;
	default:
		break;
	}

	if (host->mclk != ios->clock || host->timing != ios->timing)
		msdc_set_mclk(host, ios->timing, ios->clock);
}

#ifdef PAD_DELAY_USE_64BITS
static u64 test_delay_bit(u64 delay, u32 bit)
{
	bit %= PAD_DELAY_64;
	return delay & (1ULL << bit);
}

static int get_delay_len(u64 delay, u32 start_bit)
{
	int i;
	u32 loop_cnt = PAD_DELAY_64 - start_bit;

	for (i = 0; i < loop_cnt; i++) {
		if (test_delay_bit(delay, start_bit + i) == 0)
			return i;
	}
	return PAD_DELAY_64 - start_bit;
}

static struct msdc_delay_phase get_best_delay(struct msdc_host *host, u64 delay, const char* log_prefix)
{
	int start = 0, len = 0;
	int start_final = 0, len_final = 0;
	u8 final_phase = 0xff;
	struct msdc_delay_phase delay_phase = { 0, };

	if (delay == 0) {
		dev_err(host->dev, "%s phase error map: 0x0\n",
				log_prefix ? log_prefix : "");
		delay_phase.final_phase = final_phase;
		return delay_phase;
	}

	while (start < PAD_DELAY_64) {
		len = get_delay_len(delay, start);
		if (len_final < len) {
			start_final = start;
			len_final = len;
		}
		start += len ? len : 1;
	}

	/* The rule is that to find the smallest delay cell */
	if (start_final == 0)
		final_phase = (start_final + len_final / 3) % PAD_DELAY_64;
	else
		final_phase = (start_final + len_final / 2) % PAD_DELAY_64;

	dev_alert(host->dev, "%s phase: [map:0x%" PAD_DELAY_PRIx0 "] [start:%d] [maxlen:%d] [final:%d]\n",
		log_prefix ? log_prefix : "", delay, start_final, len_final, final_phase);

	delay_phase.maxlen = len_final;
	delay_phase.start = start_final;
	delay_phase.final_phase = final_phase;
	return delay_phase;
}

static inline void msdc_set_cmd_delay(struct msdc_host *host, u32 value)
{
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	if (host->top_base) {
		if (value < PAD_DELAY_MAX) {
			sdr_set_field(host->top_base + EMMC_TOP_CMD, PAD_CMD_RXDLY,
				      value);
			sdr_set_field(host->top_base + EMMC_TOP_CMD, PAD_CMD_RXDLY2,
				      0);
		} else {
			sdr_set_field(host->top_base + EMMC_TOP_CMD, PAD_CMD_RXDLY,
				      PAD_DELAY_MAX - 1);
			sdr_set_field(host->top_base + EMMC_TOP_CMD, PAD_CMD_RXDLY2,
				      value - PAD_DELAY_MAX);
		}
	} else {
		if (value < PAD_DELAY_MAX) {
			sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_CMDRDLY,
				      value);
			sdr_set_field(host->base + tune_reg + 4, MSDC_PAD_TUNE_CMDRDLY2,
				      0);
		} else {
			sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_CMDRDLY,
				      PAD_DELAY_MAX - 1);
			sdr_set_field(host->base + tune_reg + 4, MSDC_PAD_TUNE_CMDRDLY2,
				      value - PAD_DELAY_MAX);
		}
	}
}

static inline void msdc_set_data_delay(struct msdc_host *host, u32 value)
{
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	if (host->top_base) {
		if (value < PAD_DELAY_MAX) {
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL, PAD_DAT_RD_RXDLY,
				      value);
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL, PAD_DAT_RD_RXDLY2,
				      0);
		} else {
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL, PAD_DAT_RD_RXDLY,
				      PAD_DELAY_MAX - 1);
			sdr_set_field(host->top_base + EMMC_TOP_CONTROL, PAD_DAT_RD_RXDLY2,
				      value - PAD_DELAY_MAX);
		}
	} else {
		if (value < PAD_DELAY_MAX) {
			sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_DATRRDLY, value);
			sdr_set_field(host->base + tune_reg + 4, MSDC_PAD_TUNE_DATRRDLY2,
				      0);
		} else {
			sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_DATRRDLY,
				      PAD_DELAY_MAX - 1);
			sdr_set_field(host->base + tune_reg + 4, MSDC_PAD_TUNE_DATRRDLY2,
				      value - PAD_DELAY_MAX);
		}
	}
}
#else
static u32 test_delay_bit(u32 delay, u32 bit)
{
	bit %= PAD_DELAY_MAX;
	return delay & (1 << bit);
}

static int get_delay_len(u32 delay, u32 start_bit)
{
	int i;

	for (i = 0; i < (PAD_DELAY_MAX - start_bit); i++) {
		if (test_delay_bit(delay, start_bit + i) == 0)
			return i;
	}
	return PAD_DELAY_MAX - start_bit;
}

static struct msdc_delay_phase get_best_delay(struct msdc_host *host, u32 delay, const char* log_prefix)
{
	int start = 0, len = 0;
	int start_final = 0, len_final = 0;
	u8 final_phase = 0xff;
	struct msdc_delay_phase delay_phase = { 0, };

	if (delay == 0) {
		dev_err(host->dev, "%s phase error map: 0x0\n",
				log_prefix ? log_prefix : "");
		delay_phase.final_phase = final_phase;
		return delay_phase;
	}

	while (start < PAD_DELAY_MAX) {
		len = get_delay_len(delay, start);
		if (len_final < len) {
			start_final = start;
			len_final = len;
		}
		start += len ? len : 1;
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
#else
		if (len >= 12 && start_final < 4)
			break;
#endif
	}

	/* The rule is that to find the smallest delay cell */
	if (start_final == 0)
		final_phase = (start_final + len_final / 3) % PAD_DELAY_MAX;
	else
		final_phase = (start_final + len_final / 2) % PAD_DELAY_MAX;

	dev_alert(host->dev, "%s phase: [map:0x%" PAD_DELAY_PRIx0 "] [start:%d] [maxlen:%d] [final:%d]\n",
		log_prefix ? log_prefix : "", delay, start_final, len_final, final_phase);

	delay_phase.maxlen = len_final;
	delay_phase.start = start_final;
	delay_phase.final_phase = final_phase;
	return delay_phase;
}

static inline void msdc_set_cmd_delay(struct msdc_host *host, u32 value)
{
	u32 tune_reg = host->dev_comp->pad_tune_reg;
	if (host->top_base)
		sdr_set_field(host->top_base + EMMC_TOP_CMD, PAD_CMD_RXDLY,
			      value);
	else
		sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_CMDRDLY,
			      value);
}

static inline void msdc_set_data_delay(struct msdc_host *host, u32 value)
{
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	if (host->top_base)
		sdr_set_field(host->top_base + EMMC_TOP_CONTROL,
			      PAD_DAT_RD_RXDLY, value);
	else
		sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_DATRRDLY,
			      value);
}
#endif /* PAD_DELAY_USE_64BITS */

#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
/**
 * If the rising delay map is not good as expected,
 * force to do the falling edge delay tuning for the best delay comparison.
 * The system will tune the both edges and get the best delay from rising or falling.
 */
static bool msdc_check_tune_both_edges(struct msdc_host *host, PAD_DELAY_TYPE rise_delay, int vcore)
{
	if (vcore > 0 && host->n_vcore_lookup_table > 0) {
		int i;
		for (i = 0; i < host->n_vcore_lookup_table; i++) {
			if (vcore == host->vcore_lookup_table[i]) {
				if ((rise_delay & host->rising_expected_delay_maps[i])
						== host->rising_expected_delay_maps[i])
					return false;
				else {
					dev_alert(host->dev,
						"[expected:0x%" PAD_DELAY_PRIx0 "] [rising:0x%" PAD_DELAY_PRIx0 "] @vcore:%d\n",
						host->rising_expected_delay_maps[i], rise_delay, vcore);
					return true;
				}
			}
		}
	}

	return false;
}
#endif

static int msdc_tune_response(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	PAD_DELAY_TYPE rise_delay = 0, fall_delay = 0;
	struct msdc_delay_phase final_rise_delay, final_fall_delay = { 0,};
	struct msdc_delay_phase internal_delay_phase;
	u8 final_delay, final_maxlen;
	PAD_DELAY_TYPE internal_delay = 0;
	u32 tune_reg = host->dev_comp->pad_tune_reg;
	int cmd_err;
	int i, j;
	bool use_rise = false;

	if (mmc->ios.timing == MMC_TIMING_MMC_HS200 ||
	    mmc->ios.timing == MMC_TIMING_UHS_SDR104)
		sdr_set_field(host->base + tune_reg,
			      MSDC_PAD_TUNE_CMDRRDLY,
			      host->hs200_cmd_int_delay);

	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	for (i = 0 ; i < PAD_DELAY_BIT_MAX; i++) {
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
		/**
		 * If tuning_both_edges is false, we would like to know whether
		 * the rising delay map is good as expected or not.
		 *
		 * So we must check all delays here.
		 */
		if (host->tuning_both_edges && !(host->rising_delay_map_mask & (PAD_DELAY_BIT_TYPE << i)))
			continue;
#endif
		msdc_set_cmd_delay(host, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters host->tuning_loop times.
		 */
		cmd_err = 0;
		for (j = 0; j < host->tuning_loop; j++) {
			mmc_send_tuning(mmc, opcode, &cmd_err);
			if (!cmd_err) {
				rise_delay |= (PAD_DELAY_BIT_TYPE << i);
			} else {
				rise_delay &= ~(PAD_DELAY_BIT_TYPE << i);
				break;
			}
		}
	}
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	if (!host->tuning_both_edges) {
		int vcore = host->vcore_power ? regulator_get_voltage(host->vcore_power) : -1;

		/**
		 * If the rising delay map is not good as expected,
		 * force to do the falling edge delay tuning for the best delay comparison.
		 * The system will tune the both edges and get the best delay from rising or falling.
		 */
		if (msdc_check_tune_both_edges(host, rise_delay, vcore))
			/* The system will tune the both edges and get the best delay from rising or falling. */
			host->tuning_both_edges = true;

		if (rise_delay && (rise_delay & host->rising_delay_map_mask) != rise_delay)
			dev_alert(host->dev, "[rising:0x%" PAD_DELAY_PRIx0 "->0x%" PAD_DELAY_PRIx "] @vcore:%d\n",
				rise_delay, (rise_delay & host->rising_delay_map_mask), vcore);
#if 0
		else
			dev_alert(host->dev, "[rising:0x%" PAD_DELAY_PRIx "] @vcore:%d\n", rise_delay, vcore);
#endif
	}

	rise_delay &= host->rising_delay_map_mask;
#endif
	final_rise_delay = get_best_delay(host, rise_delay, "Rising");
	/* if rising edge has enough margin, then do not scan falling edge */
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	if (!host->tuning_both_edges &&
		((int)final_rise_delay.maxlen >= host->rising_delay_min_length))
		goto skip_fall;
#else
	if (!host->tuning_both_edges && (final_rise_delay.maxlen >= 12 ||
	    (final_rise_delay.start == 0 && final_rise_delay.maxlen >= 4)))
		goto skip_fall;
#endif

	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	for (i = 0; i < PAD_DELAY_BIT_MAX; i++) {
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
		/* Here is the falling edge tuning so only tune for falling_delay_map_mask. */
		if (!(host->falling_delay_map_mask & (PAD_DELAY_BIT_TYPE << i)))
			continue;
#endif
		msdc_set_cmd_delay(host, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters host->tuning_loop times.
		 */
		cmd_err = 0;
		for (j = 0; j < host->tuning_loop; j++) {
			mmc_send_tuning(mmc, opcode, &cmd_err);
			if (!cmd_err) {
				fall_delay |= (PAD_DELAY_BIT_TYPE << i);
			} else {
				fall_delay &= ~(PAD_DELAY_BIT_TYPE << i);
				break;
			}
		}
	}
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	fall_delay &= host->falling_delay_map_mask;
#endif
	final_fall_delay = get_best_delay(host, fall_delay, "Falling");

skip_fall:
	final_maxlen = max(final_rise_delay.maxlen, final_fall_delay.maxlen);
	if (final_fall_delay.maxlen >= 12 && final_fall_delay.start < 4)
		final_maxlen = final_fall_delay.maxlen;
	if (final_maxlen == final_rise_delay.maxlen) {
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		final_delay = final_rise_delay.final_phase;
		use_rise = true;
	} else {
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		final_delay = final_fall_delay.final_phase;
	}
	msdc_set_cmd_delay(host, final_delay);

	if (host->dev_comp->async_fifo || host->hs200_cmd_int_delay)
		goto skip_internal;

	for (i = 0; i < PAD_DELAY_MAX; i++) {
		sdr_set_field(host->base + tune_reg,
			      MSDC_PAD_TUNE_CMDRRDLY, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters host->tuning_loop times.
		 */
		cmd_err = 0;
		for (j = 0; j < host->tuning_loop; j++) {
			mmc_send_tuning(mmc, opcode, &cmd_err);
			if (!cmd_err)
				internal_delay |= (PAD_DELAY_BIT_TYPE << i);
			else {
				internal_delay &= ~(PAD_DELAY_BIT_TYPE << i);
				break;
			}
		}
	}
	dev_alert(host->dev, "Final internal delay: 0x%x\n", internal_delay);
	internal_delay_phase = get_best_delay(host, internal_delay, "Internal-CMD");
	sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_CMDRRDLY,
		      internal_delay_phase.final_phase);
skip_internal:
	dev_alert(host->dev, "Final cmd pad %s delay: 0x%x [map:%" PAD_DELAY_PRIx "]\n",
			use_rise ? "rising" : "falling", final_delay,
			use_rise ? rise_delay : fall_delay);
	return final_delay == 0xff ? -EIO : 0;
}

static int hs400_tune_response(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	PAD_DELAY_TYPE cmd_delay = 0;
	struct msdc_delay_phase final_cmd_delay = { 0,};
	u8 final_delay;
	int cmd_err;
	int i, j;

	/* select EMMC50 PAD CMD tune */
	sdr_set_bits(host->base + PAD_CMD_TUNE, BIT(0));
	sdr_set_field(host->base + MSDC_PATCH_BIT1, MSDC_PATCH_BIT1_CMDTA, 2);

	if (mmc->ios.timing == MMC_TIMING_MMC_HS200 ||
	    mmc->ios.timing == MMC_TIMING_UHS_SDR104)
		sdr_set_field(host->base + MSDC_PAD_TUNE,
			      MSDC_PAD_TUNE_CMDRRDLY,
			      host->hs200_cmd_int_delay);

	if (host->hs400_cmd_resp_sel_rising)
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	else
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	for (i = 0 ; i < PAD_DELAY_MAX; i++) {
		sdr_set_field(host->base + PAD_CMD_TUNE,
			      PAD_CMD_TUNE_RX_DLY3, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters host->tuning_loop times.
		 */
		cmd_err = 0;
		for (j = 0; j < host->tuning_loop; j++) {
			mmc_send_tuning(mmc, opcode, &cmd_err);
			if (!cmd_err) {
				cmd_delay |= (PAD_DELAY_BIT_TYPE << i);
			} else {
				cmd_delay &= ~(PAD_DELAY_BIT_TYPE << i);
				break;
			}
		}
	}
	final_cmd_delay = get_best_delay(host, cmd_delay, "CMD");
	sdr_set_field(host->base + PAD_CMD_TUNE, PAD_CMD_TUNE_RX_DLY3,
		      final_cmd_delay.final_phase);
	final_delay = final_cmd_delay.final_phase;

	dev_alert(host->dev, "Final cmd pad delay: 0x%x\n", final_delay);
	return final_delay == 0xff ? -EIO : 0;
}

static int msdc_tune_data(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	PAD_DELAY_TYPE rise_delay = 0, fall_delay = 0;
	struct msdc_delay_phase final_rise_delay, final_fall_delay = { 0,};
	u8 final_delay, final_maxlen;
	int i, ret, j;
	bool use_rise = false;

	sdr_set_field(host->base + MSDC_PATCH_BIT, MSDC_INT_DAT_LATCH_CK_SEL,
		      host->latch_ck);
	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
	for (i = 0 ; i < PAD_DELAY_BIT_MAX; i++) {
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
		/**
		 * If tuning_both_edges is false, we would like to know whether
		 * the rising delay map is good as expected or not.
		 *
		 * So we must check all delays here.
		 */
		if (host->tuning_both_edges && !(host->rising_delay_map_mask & (PAD_DELAY_BIT_TYPE << i)))
			continue;
#endif
		msdc_set_data_delay(host, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters host->tuning_loop times.
		 */
		for (j = 0; j < host->tuning_loop; j++) {
			ret = mmc_send_tuning(mmc, opcode, NULL);
			if (!ret)
				rise_delay |= (PAD_DELAY_BIT_TYPE << i);
			else {
				rise_delay &= ~(PAD_DELAY_BIT_TYPE << i);
				break;
			}
		}
	}
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	if (!host->tuning_both_edges) {
		int vcore = host->vcore_power ? regulator_get_voltage(host->vcore_power) : -1;

		/**
		 * If the rising delay map is not good as expected,
		 * force to do the falling edge delay tuning for the best delay comparison.
		 * The system will tune the both edges and get the best delay from rising or falling.
		 */
		if (msdc_check_tune_both_edges(host, rise_delay, vcore))
			/* The system will tune the both edges and get the best delay from rising or falling. */
			host->tuning_both_edges = true;

		if (rise_delay && (rise_delay & host->rising_delay_map_mask) != rise_delay)
			dev_alert(host->dev, "[rising:0x%" PAD_DELAY_PRIx0 "->0x%" PAD_DELAY_PRIx "] @vcore:%d\n",
				rise_delay, (rise_delay & host->rising_delay_map_mask), vcore);
#if 0
		else
			dev_alert(host->dev, "[rising:0x%" PAD_DELAY_PRIx "] @vcore:%d\n", rise_delay, vcore);
#endif
	}

	rise_delay &= host->rising_delay_map_mask;
#endif
	final_rise_delay = get_best_delay(host, rise_delay, "Rising");
	/* if rising edge has enough margin, then do not scan falling edge */
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	if (!host->tuning_both_edges &&
		((int)final_rise_delay.maxlen >= host->rising_delay_min_length))
		goto skip_fall;
#else
	if (!host->tuning_both_edges && (final_rise_delay.maxlen >= 12 ||
	    (final_rise_delay.start == 0 && final_rise_delay.maxlen >= 4)))
		goto skip_fall;
#endif

	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
	for (i = 0; i < PAD_DELAY_BIT_MAX; i++) {
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
		/* Here is the falling edge tuning so only tune for falling_delay_map_mask. */
		if (!(host->falling_delay_map_mask & (PAD_DELAY_BIT_TYPE << i)))
			continue;
#endif
		msdc_set_data_delay(host, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters host->tuning_loop times.
		 */
		for (j = 0; j < host->tuning_loop; j++) {
			ret = mmc_send_tuning(mmc, opcode, NULL);
			if (!ret)
				fall_delay |= (PAD_DELAY_BIT_TYPE << i);
			else {
				fall_delay &= ~(PAD_DELAY_BIT_TYPE << i);
				break;
			}
		}
	}
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	fall_delay &= host->falling_delay_map_mask;
#endif
	final_fall_delay = get_best_delay(host, fall_delay, "Falling");

skip_fall:
	final_maxlen = max(final_rise_delay.maxlen, final_fall_delay.maxlen);
	if (final_maxlen == final_rise_delay.maxlen) {
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
		final_delay = final_rise_delay.final_phase;
		use_rise = true;
	} else {
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
		final_delay = final_fall_delay.final_phase;
	}
	msdc_set_data_delay(host, final_delay);

	dev_alert(host->dev, "Final data pad %s delay: 0x%x [map:%" PAD_DELAY_PRIx "]\n",
			use_rise ? "rising" : "falling", final_delay,
			use_rise ? rise_delay : fall_delay);
	return final_delay == 0xff ? -EIO : 0;
}

/*
 * MSDC IP which supports data tune + async fifo can do CMD/DAT tune
 * together, which can save the tuning time.
 */
static int msdc_tune_together(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	PAD_DELAY_TYPE rise_delay = 0, fall_delay = 0;
	struct msdc_delay_phase final_rise_delay, final_fall_delay = { 0,};
	u8 final_delay, final_maxlen;
	int i, ret, j;
	bool use_rise = false;

	sdr_set_field(host->base + MSDC_PATCH_BIT, MSDC_INT_DAT_LATCH_CK_SEL,
		      host->latch_ck);

	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	sdr_clr_bits(host->base + MSDC_IOCON,
		     MSDC_IOCON_DSPL | MSDC_IOCON_W_DSPL);
	for (i = 0 ; i < PAD_DELAY_BIT_MAX; i++) {
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
		/**
		 * If tuning_both_edges is false, we would like to know whether
		 * the rising delay map is good as expected or not.
		 *
		 * So we must check all delays here.
		 */
		if (host->tuning_both_edges && !(host->rising_delay_map_mask & (PAD_DELAY_BIT_TYPE << i)))
			continue;
#endif
		msdc_set_cmd_delay(host, i);
		msdc_set_data_delay(host, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters host->tuning_loop times.
		 */
		for (j = 0; j < host->tuning_loop; j++) {
			ret = mmc_send_tuning(mmc, opcode, NULL);
			if (!ret)
				rise_delay |= (PAD_DELAY_BIT_TYPE << i);
			else {
				rise_delay &= ~(PAD_DELAY_BIT_TYPE << i);
				break;
			}
		}
	}
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	if (!host->tuning_both_edges) {
		int vcore = host->vcore_power ? regulator_get_voltage(host->vcore_power) : -1;

		/**
		 * If the rising delay map is not good as expected,
		 * force to do the falling edge delay tuning for the best delay comparison.
		 * The system will tune the both edges and get the best delay from rising or falling.
		 */
		if (msdc_check_tune_both_edges(host, rise_delay, vcore))
			/* The system will tune the both edges and get the best delay from rising or falling. */
			host->tuning_both_edges = true;

		if (rise_delay && (rise_delay & host->rising_delay_map_mask) != rise_delay)
			dev_alert(host->dev, "[rising:0x%" PAD_DELAY_PRIx0 "->0x%" PAD_DELAY_PRIx "] @vcore:%d\n",
				rise_delay, (rise_delay & host->rising_delay_map_mask), vcore);
#if 0
		else
			dev_alert(host->dev, "[rising:0x%" PAD_DELAY_PRIx "] @vcore:%d\n", rise_delay, vcore);
#endif
	}

	rise_delay &= host->rising_delay_map_mask;
#endif
	final_rise_delay = get_best_delay(host, rise_delay, "Rising");
	/* if rising edge has enough margin, then do not scan falling edge */
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	if (!host->tuning_both_edges &&
		((int)final_rise_delay.maxlen >= host->rising_delay_min_length))
		goto skip_fall;
#else
	if (!host->tuning_both_edges && (final_rise_delay.maxlen >= 12 ||
	    (final_rise_delay.start == 0 && final_rise_delay.maxlen >= 4)))
		goto skip_fall;
#endif

	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	sdr_set_bits(host->base + MSDC_IOCON,
		     MSDC_IOCON_DSPL | MSDC_IOCON_W_DSPL);
	for (i = 0; i < PAD_DELAY_BIT_MAX; i++) {
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
		/* Here is the falling edge tuning so only tune for falling_delay_map_mask. */
		if (!(host->falling_delay_map_mask & (PAD_DELAY_BIT_TYPE << i)))
			continue;
#endif
		msdc_set_cmd_delay(host, i);
		msdc_set_data_delay(host, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters host->tuning_loop times.
		 */
		for (j = 0; j < host->tuning_loop; j++) {
			ret = mmc_send_tuning(mmc, opcode, NULL);
			if (!ret)
				fall_delay |= (PAD_DELAY_BIT_TYPE << i);
			else {
				fall_delay &= ~(PAD_DELAY_BIT_TYPE << i);
				break;
			}
		}
	}
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	fall_delay &= host->falling_delay_map_mask;
#endif
	final_fall_delay = get_best_delay(host, fall_delay, "Falling");

skip_fall:
	final_maxlen = max(final_rise_delay.maxlen, final_fall_delay.maxlen);
	if (final_maxlen == final_rise_delay.maxlen) {
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		sdr_clr_bits(host->base + MSDC_IOCON,
			     MSDC_IOCON_DSPL | MSDC_IOCON_W_DSPL);
		final_delay = final_rise_delay.final_phase;
		use_rise = true;
	} else {
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		sdr_set_bits(host->base + MSDC_IOCON,
			     MSDC_IOCON_DSPL | MSDC_IOCON_W_DSPL);
		final_delay = final_fall_delay.final_phase;
	}

	msdc_set_cmd_delay(host, final_delay);
	msdc_set_data_delay(host, final_delay);

	dev_alert(host->dev, "Final pad %s delay: 0x%x [map:%" PAD_DELAY_PRIx "]\n",
			use_rise ? "rising" : "falling", final_delay,
			use_rise ? rise_delay : fall_delay);
	return final_delay == 0xff ? -EIO : 0;
}

static int msdc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret;
	u32 tune_reg = host->dev_comp->pad_tune_reg;
	ktime_t time_start = ktime_get();

	if (host->dev_comp->data_tune && host->dev_comp->async_fifo) {
		ret = msdc_tune_together(mmc, opcode);
		if (ret == -EIO)
			dev_err(host->dev, "Tune cmd/data fail!\n");
		if (host->hs400_mode) {
			sdr_clr_bits(host->base + MSDC_IOCON,
				     MSDC_IOCON_DSPL | MSDC_IOCON_W_DSPL);
			msdc_set_data_delay(host, 0);
		}
		goto tune_done;
	}
	if (host->hs400_mode &&
	    host->dev_comp->hs400_tune)
		ret = hs400_tune_response(mmc, opcode);
	else
		ret = msdc_tune_response(mmc, opcode);
	if (ret == -EIO) {
		dev_err(host->dev, "Tune response fail!\n");
		//return ret;
		goto tune_done;
	}
	if (host->hs400_mode == false) {
		ret = msdc_tune_data(mmc, opcode);
		if (ret == -EIO)
			dev_err(host->dev, "Tune data fail!\n");
	}

tune_done:
	host->saved_tune_para.iocon = readl(host->base + MSDC_IOCON);
	host->saved_tune_para.pad_tune = readl(host->base + tune_reg);
	host->saved_tune_para.pad_cmd_tune = readl(host->base + PAD_CMD_TUNE);
	if (host->top_base) {
		host->saved_tune_para.emmc_top_control = readl(host->top_base +
				EMMC_TOP_CONTROL);
		host->saved_tune_para.emmc_top_cmd = readl(host->top_base +
				EMMC_TOP_CMD);
	}

	if (host->vcore_power)
		dev_alert(host->dev, "tuning cost %lld us @vcore:%d\n",
			ktime_us_delta(ktime_get(), time_start), regulator_get_voltage(host->vcore_power));
	else
		dev_alert(host->dev, "tuning cost %lld us\n",
			ktime_us_delta(ktime_get(), time_start));

	return ret;
}

static int msdc_prepare_hs400_tuning(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	host->hs400_mode = true;

	if (host->top_base)
		writel(host->hs400_ds_delay,
		       host->top_base + EMMC50_PAD_DS_TUNE);
	else
		writel(host->hs400_ds_delay, host->base + PAD_DS_TUNE);
	/* hs400 mode must set it to 0 */
	sdr_clr_bits(host->base + MSDC_PATCH_BIT2, MSDC_PATCH_BIT2_CFGCRCSTS);
	/* to improve read performance, set outstanding to 2 */
	sdr_set_field(host->base + EMMC50_CFG3, EMMC50_CFG3_OUTS_WR, 2);

	return 0;
}

static void msdc_hw_reset(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	sdr_set_bits(host->base + EMMC_IOCON, 1);
	udelay(10); /* 10us is enough */
	sdr_clr_bits(host->base + EMMC_IOCON, 1);
}

static void msdc_ack_sdio_irq(struct mmc_host *mmc)
{
	unsigned long flags;
	struct msdc_host *host = mmc_priv(mmc);

	spin_lock_irqsave(&host->lock, flags);
	__msdc_enable_sdio_irq(host, 1);
	spin_unlock_irqrestore(&host->lock, flags);
}

static int msdc_get_cd(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	int val;

	if (mmc->caps & MMC_CAP_NONREMOVABLE)
		return 1;

	if (!host->internal_cd)
		return mmc_gpio_get_cd(mmc);

	val = readl(host->base + MSDC_PS) & MSDC_PS_CDSTS;
	if (mmc->caps2 & MMC_CAP2_CD_ACTIVE_HIGH)
		return !!val;
	else
		return !val;
}

#if IS_ENABLED(CONFIG_MMC_CQHCI)
static void msdc_cqe_enable(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	/* enable cmdq irq */
	writel(MSDC_INT_CMDQ, host->base + MSDC_INTEN);
	/* enable busy check */
	sdr_set_bits(host->base + MSDC_PATCH_BIT1, MSDC_PB1_BUSY_CHECK_SEL);
	/* default write data / busy timeout 20s */
	msdc_set_busy_timeout(host, 20 * 1000000000ULL, 0);
	/* default read data timeout 1s */
	msdc_set_timeout(host, 1000000000ULL, 0);
}

static void msdc_cqe_disable(struct mmc_host *mmc, bool recovery)
{
	struct msdc_host *host = mmc_priv(mmc);
	unsigned int val = 0;

	/* disable cmdq irq */
	sdr_clr_bits(host->base + MSDC_INTEN, MSDC_INT_CMDQ);
	/* disable busy check */
	sdr_clr_bits(host->base + MSDC_PATCH_BIT1, MSDC_PB1_BUSY_CHECK_SEL);

	val = readl(host->base + MSDC_INT);
	writel(val, host->base + MSDC_INT);

	if (recovery) {
		sdr_set_field(host->base + MSDC_DMA_CTRL,
			      MSDC_DMA_CTRL_STOP, 1);
		if (WARN_ON(readl_poll_timeout(host->base + MSDC_DMA_CTRL, val,
			!(val & MSDC_DMA_CTRL_STOP), 1, 3000))) {
			dev_err(host->dev, "CQE recovery: DMA stop timed out\n");
			return;
		}
		if (WARN_ON(readl_poll_timeout(host->base + MSDC_DMA_CFG, val,
			!(val & MSDC_DMA_CFG_STS), 1, 3000))) {
			dev_err(host->dev, "CQE recovery: DMA inactive timed out\n");
			return;
		}
		msdc_reset_hw(host);
	}
}

static void msdc_cqe_pre_enable(struct mmc_host *mmc)
{
	struct cqhci_host *cq_host = mmc->cqe_private;
	u32 reg;

	reg = cqhci_readl(cq_host, CQHCI_CFG);
	reg |= CQHCI_ENABLE;
	cqhci_writel(cq_host, reg, CQHCI_CFG);
}

static void msdc_cqe_post_disable(struct mmc_host *mmc)
{
	struct cqhci_host *cq_host = mmc->cqe_private;
	u32 reg;

	reg = cqhci_readl(cq_host, CQHCI_CFG);
	reg &= ~CQHCI_ENABLE;
	cqhci_writel(cq_host, reg, CQHCI_CFG);
}
#endif

static const struct mmc_host_ops mt_msdc_ops = {
	.post_req = msdc_post_req,
	.pre_req = msdc_pre_req,
	.request = msdc_ops_request,
	.set_ios = msdc_ops_set_ios,
	.get_ro = mmc_gpio_get_ro,
	.get_cd = msdc_get_cd,
	.enable_sdio_irq = msdc_enable_sdio_irq,
	.ack_sdio_irq = msdc_ack_sdio_irq,
	.start_signal_voltage_switch = msdc_ops_switch_volt,
	.card_busy = msdc_card_busy,
	.execute_tuning = msdc_execute_tuning,
	.prepare_hs400_tuning = msdc_prepare_hs400_tuning,
	.hw_reset = msdc_hw_reset,
};

static irqreturn_t msdc_sdio_eint_irq(int irq, void *dev_id)
{
	struct msdc_host *host = (struct msdc_host *)dev_id;
	struct mmc_host *mmc = mmc_from_priv(host);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	//dev_alert(host->dev, "msdc_sdio_eint_irq: sdio_irq_cnt=%d\n", host->sdio_irq_cnt);
	if (likely(host->sdio_irq_cnt > 0)) {
		disable_irq_nosync(host->eint_irq);
		disable_irq_wake(host->eint_irq);
		host->sdio_irq_cnt--;
	}
	spin_unlock_irqrestore(&host->lock, flags);

	sdio_signal_irq(mmc);

	return IRQ_HANDLED;
}

static int msdc_request_dat1_eint_irq(struct msdc_host *host)
{
	struct gpio_desc *desc;
	int irq, ret;

	desc = devm_gpiod_get(host->dev, "eint", GPIOD_IN);
	if (IS_ERR(desc))
		return PTR_ERR(desc);

	irq = gpiod_to_irq(desc);
	if (irq < 0)
		return irq;

	irq_set_status_flags(irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(host->dev, irq, NULL, msdc_sdio_eint_irq,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"sdio-eint", host);
	if (ret)
		return ret;

	host->eint_irq = irq;

	return 0;
}

#if IS_ENABLED(CONFIG_MMC_CQHCI)
static const struct cqhci_host_ops msdc_cmdq_ops = {
	.enable         = msdc_cqe_enable,
	.disable        = msdc_cqe_disable,
	.pre_enable = msdc_cqe_pre_enable,
	.post_disable = msdc_cqe_post_disable,
};
#endif

static void msdc_of_property_parse(struct platform_device *pdev,
				   struct msdc_host *host)
{
	u32 value;

	/**
	 * If 'tuning-both-edges' is defined,
	 * the system will tune the both edges and get the best delay from rising or falling.
	 */
	if (of_property_read_bool(pdev->dev.of_node, "tuning-both-edges"))
		host->tuning_both_edges = true;
	else
		host->tuning_both_edges = false;
	dev_dbg(&pdev->dev, "tuning-both-edges: %u\n", host->tuning_both_edges);

#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	/**
	 * If 'rising-delay-map-mask' is defined,
	 * the system will mask the the current tuned rising phase delay map with 'rising-delay-map-mask'.
	 *
	 * If 'falling-delay-map-mask' is defined,
	 * the system will mask the the current tuned falling phase delay map with 'falling-delay-map-mask'.
	 *
	 * If the vcore voltage is not locked,
	 * both 'rising-delay-map-mask' and 'falling-delay-map-mask' can be used to limit the result of the delay map.
	 * The delay map mask value is the expected delay map that is suitable for all vcore voltage levels.
	 *
	 * So please perform the stress retune at all vcore voltage levels and find out a reasonable delay map mask.
	 */
#ifdef PAD_DELAY_USE_64BITS
	if (of_property_read_u64(pdev->dev.of_node, "rising-delay-map-mask",
			&host->rising_delay_map_mask) || host->rising_delay_map_mask == 0)
		host->rising_delay_map_mask = 0xffffffffffffffff;
	else
		dev_alert(&pdev->dev, "rising-delay-map-mask: 0x%" PAD_DELAY_PRIx0 "\n",
			host->rising_delay_map_mask);

	if (of_property_read_u64(pdev->dev.of_node, "falling-delay-map-mask",
			&host->falling_delay_map_mask) || host->falling_delay_map_mask == 0)
		host->falling_delay_map_mask = 0xffffffffffffffff;
	else
		dev_alert(&pdev->dev, "falling-delay-map-mask: 0x%" PAD_DELAY_PRIx0 "\n",
			host->falling_delay_map_mask);
#else
	if (of_property_read_u32(pdev->dev.of_node, "rising-delay-map-mask",
			&host->rising_delay_map_mask) || host->rising_delay_map_mask == 0)
		host->rising_delay_map_mask = 0xffffffff;
	else
		dev_alert(&pdev->dev, "rising-delay-map-mask: 0x%" PAD_DELAY_PRIx0 "\n",
			host->rising_delay_map_mask);

	if (of_property_read_u32(pdev->dev.of_node, "falling-delay-map-mask",
			&host->falling_delay_map_mask) || host->falling_delay_map_mask == 0)
		host->falling_delay_map_mask = 0xffffffff;
	else
		dev_alert(&pdev->dev, "falling-delay-map-mask: 0x%" PAD_DELAY_PRIx0 "\n",
			host->falling_delay_map_mask);
#endif

	/**
	 * The default 'rising-delay-min-length' is PAD_DELAY_RISING_MIN_LENGTH.
	 *
	 * For the false of 'tuning-both-edges' case,
	 * if the max length of the tuned rising delay map is equal to or greater than 'rising-delay-min-length',
	 * the system will pick the rising phase tuning result and won't run the falling tuning case.
	 */
	value = 0;
	if (of_property_read_u32(pdev->dev.of_node, "rising-delay-min-length", &value) ||
			value == 0 || value > PAD_DELAY_BIT_MAX)
		value = PAD_DELAY_RISING_MIN_LENGTH;
	host->rising_delay_min_length = (int)value;
	dev_dbg(&pdev->dev, "rising-delay-min-length: %d\n", host->rising_delay_min_length);
#endif /* ICOM_MTK_MMC_NEW_BEST_DELAY */

	/* Define the tuning loop number for each delay at tuning */
	value = 0;
	if (of_property_read_u32(pdev->dev.of_node, "each-delay-tuning-loop", &value) ||
			value < 3 || value > 40)
		value = PAD_DELAY_TUNING_LOOP;
	host->tuning_loop = (int)value;
	dev_dbg(&pdev->dev, "each-delay-tuning-loop: %d\n", host->tuning_loop);

	of_property_read_u32(pdev->dev.of_node, "mediatek,ckgen-delay",
			     &host->ckgen_delay);
	dev_dbg(&pdev->dev, "ckgen-delay: %u\n", host->ckgen_delay);

	of_property_read_u32(pdev->dev.of_node, "mediatek,latch-ck",
			     &host->latch_ck);
	dev_dbg(&pdev->dev, "latch-ck: %u\n", host->latch_ck);

	of_property_read_u32(pdev->dev.of_node, "hs400-ds-delay",
			     &host->hs400_ds_delay);
	dev_dbg(&pdev->dev, "hs400-ds-delay: %u\n", host->hs400_ds_delay);

	of_property_read_u32(pdev->dev.of_node, "mediatek,hs200-cmd-int-delay",
			     &host->hs200_cmd_int_delay);

	of_property_read_u32(pdev->dev.of_node, "mediatek,hs400-cmd-int-delay",
			     &host->hs400_cmd_int_delay);

	if (of_property_read_bool(pdev->dev.of_node,
				  "mediatek,hs400-cmd-resp-sel-rising"))
		host->hs400_cmd_resp_sel_rising = true;
	else
		host->hs400_cmd_resp_sel_rising = false;

#if IS_ENABLED(CONFIG_MMC_CQHCI)
	if (of_property_read_bool(pdev->dev.of_node,
				  "supports-cqe"))
		host->cqhci = true;
	else
		host->cqhci = false;
#else
	host->cqhci = false;
#endif

	/* 'vcore-supply' is the PMIC regulator that is used to get the current vcore voltage. */
	if (of_property_read_bool(pdev->dev.of_node, "vcore-supply")) {
		host->vcore_power = devm_regulator_get_optional(&pdev->dev, "vcore");
		if (IS_ERR(host->vcore_power)) {
			dev_warn(&pdev->dev, "failed to get vcore regulator: %d\n",
				PTR_ERR(host->vcore_power));
			host->vcore_power = NULL;
		}
	} else
		host->vcore_power = NULL;

#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	if (!host->tuning_both_edges && host->vcore_power) {
		int n_vcore_table, n_rising_map_table;

		/**
		 * If the rising delay map is not good as expected,
		 * force to do the falling edge delay tuning for the best delay comparison.
		 * The system will tune the both edges and get the best delay from rising or falling.
		 *
		 * Define the expected rising delay maps for all vcore voltage levels by 'rising-expected-delay-maps'.
		 */
		n_vcore_table = of_property_count_elems_of_size(pdev->dev.of_node,
						"vcore-levels", sizeof(u32));
		n_rising_map_table = of_property_count_elems_of_size(pdev->dev.of_node,
						"rising-expected-delay-maps", sizeof(PAD_DELAY_TYPE));
		if (n_vcore_table > 0 && n_vcore_table == n_rising_map_table) {
			host->vcore_lookup_table = devm_kcalloc(&pdev->dev,
					n_vcore_table, sizeof(*host->vcore_lookup_table),
					GFP_KERNEL);
			host->rising_expected_delay_maps = devm_kcalloc(&pdev->dev,
					n_vcore_table, sizeof(*host->rising_expected_delay_maps),
					GFP_KERNEL);
			if (host->vcore_lookup_table && host->rising_expected_delay_maps) {
				int err = of_property_read_u32_array(pdev->dev.of_node, "vcore-levels",
					 		(u32 *)host->vcore_lookup_table, n_vcore_table);
				if (!err) {
#ifdef PAD_DELAY_USE_64BITS
					err = of_property_read_u64_array(pdev->dev.of_node, "rising-expected-delay-maps",
					 	(u64 *)host->rising_expected_delay_maps, n_rising_map_table);
#else
					err = of_property_read_u32_array(pdev->dev.of_node, "rising-expected-delay-maps",
					 	(u32 *)host->rising_expected_delay_maps, n_rising_map_table);
#endif
					if (err) {
						dev_err(&pdev->dev, "failed to get rising-expected-delay-maps: %d\n", err);
						devm_kfree(&pdev->dev, host->vcore_lookup_table);
						devm_kfree(&pdev->dev, host->rising_expected_delay_maps);
						host->vcore_lookup_table = NULL;
						host->rising_expected_delay_maps = NULL;
					} else {
						host->n_vcore_lookup_table = n_vcore_table;
#if 0
						for (err = 0 ; err < n_vcore_table ; err++)
							dev_alert(&pdev->dev,
								"[%d] [vcore:%u] [rising-expected-delay:0x%" PAD_DELAY_PRIx0 "]\n",
								err,
								host->vcore_lookup_table[err],
								host->rising_expected_delay_maps[err]);
#endif
					}
				} else {
					dev_err(&pdev->dev, "failed to get vcore-levels: %d\n", err);
					devm_kfree(&pdev->dev, host->vcore_lookup_table);
					devm_kfree(&pdev->dev, host->rising_expected_delay_maps);
					host->vcore_lookup_table = NULL;
					host->rising_expected_delay_maps = NULL;
				}
			} else {
				dev_err(&pdev->dev, "out of memeory (vcore_lookup_table/rising_expected_delay_maps)\n");
				if (host->vcore_lookup_table)
					devm_kfree(&pdev->dev, host->vcore_lookup_table);
				if (host->rising_expected_delay_maps)
					devm_kfree(&pdev->dev, host->rising_expected_delay_maps);
				host->vcore_lookup_table = NULL;
				host->rising_expected_delay_maps = NULL;
			}
		} else if (n_vcore_table > 0 && n_vcore_table != n_rising_map_table)
			dev_warn(&pdev->dev, "invalid vcore-levels (%d) and rising-expected-delay-maps (%d)\n",
					n_vcore_table, n_rising_map_table);
	}
#endif

#ifdef ICOM_MTK_MMC_DVFSRC_VCORE
	/**
	 * MSDC phase tuning delay map is highly related to the vcore voltage level.
	 * The vcore voltage impacts on the phase tuning delay result.
	 *
	 * If you would like to reduce the impact of the vcore voltage,
	 * please lock the vcore voltage level ('req-vcore') and
	 * turn on the runtime PM to reduce the impact on the power consumption.
	 */
	value = 0;
	of_property_read_u32(pdev->dev.of_node, "req-vcore", &value);
	host->req_vcore = (int)value;
	dev_dbg(&pdev->dev, "dvfsrc-vcore req-vcore: %d\n", host->req_vcore);
	if (host->req_vcore > 0 && of_property_read_bool(pdev->dev.of_node, "dvfsrc-vcore-supply")) {
		host->dvfsrc_vcore_power = devm_regulator_get_optional(&pdev->dev, "dvfsrc-vcore");
		if (IS_ERR(host->dvfsrc_vcore_power)) {
			dev_warn(&pdev->dev, "failed to get dvfsrc-vcore regulator: %d\n",
				PTR_ERR(host->dvfsrc_vcore_power));
			host->dvfsrc_vcore_power = NULL;
			host->req_vcore = 0;
		}
	} else
		host->dvfsrc_vcore_power = NULL;
#endif

#ifdef ICOM_MTK_MMC_CPU_LATENCY_REQUEST
	/**
	 * There are throughput performance issues if the CPU latency is not locked.
	 * Please check the exit_latency of each CPU idle state to define the CPU latency request.
	 *
	 * If 'affinity-hint-cpu-latency-req' is defined,
	 * the system will use the specific affinity_hint_cpu to request its CPU latency.
	 * Please note that the CPU latency request of the specific affinity_hint_cpu is not thread safe.
	 *
	 * If you don't know what 'affinity-hint-cpu-latency-req' is, please use 'cpu-latency-req' instead.
	 * In short, It's "device request" vs. "global request".
	 * Please refer to cpuidle_governor_latency_req().
	 */
	if (of_property_read_u32(pdev->dev.of_node, "affinity-hint-cpu-latency-req", &value)) {
		host->use_affinity_hint_cpu_latency = false;
		if (of_property_read_u32(pdev->dev.of_node, "cpu-latency-req", &value))
			host->cpu_latency_req = -EINVAL;
		else
			host->cpu_latency_req = (int)value;
	} else {
		host->use_affinity_hint_cpu_latency = true;
		host->cpu_latency_req = (int)value;
	}
	if (host->use_affinity_hint_cpu_latency)
		dev_dbg(&pdev->dev, "affinity-hint-cpu-latency-req: %d\n", host->cpu_latency_req);
	else
		dev_dbg(&pdev->dev, "cpu-latency-req: %d\n", host->cpu_latency_req);
#endif

	/* clk_auto_powerdown == true => Clear MSDC_CFG_CKPDN bit always:
	 *   1'b0: Clock will be gated to 0 if no command or data is transmitted.
	 *   1'b1: Clock will be running freely even if no command or data is
	 *         transmitted.  (The clock may still be stopped when MSDC write data are
	 *         not enough or there is no space for the next read data.)
	 */
	if (of_property_read_bool(pdev->dev.of_node, "clk-auto-power-down"))
		host->clk_auto_powerdown = true;
	else
		host->clk_auto_powerdown = false;
	dev_dbg(&pdev->dev, "clk-auto-power-down: %d\n", host->clk_auto_powerdown);

	/** ahb_bus_no_incr1 == true => Set ENABLE_SINGLE_BURST bit:
	 *   AHB bus will not support incr1 burst type in future. This will only affect AHB bus MSDC design, but not AXI bus design.
	 *     1'b0: HW will send incr1 burst type.
	 *     1'b1: HW will send single burst type instead of incr1 type.
	 */
	if (of_property_read_bool(pdev->dev.of_node, "ahb-bus-no-incr1"))
		host->ahb_bus_no_incr1 = true;
	else
		host->ahb_bus_no_incr1 = false;
	dev_dbg(&pdev->dev, "ahb-bus-no-incr1: %d\n", host->ahb_bus_no_incr1);

	if (of_property_read_u32(pdev->dev.of_node, "autosuspend-delay",
			&host->autosuspend_delay))
		host->autosuspend_delay = MTK_MMC_AUTOSUSPEND_DELAY;
	else if (host->autosuspend_delay < MTK_MMC_AUTOSUSPEND_DELAY)
		host->autosuspend_delay = MTK_MMC_AUTOSUSPEND_DELAY;
	else
		dev_dbg(&pdev->dev, "autosuspend-delay: %u\n", host->autosuspend_delay);

	/* Support to rollback to the default signal voltage. */
	if (of_property_read_bool(pdev->dev.of_node, "sd-poweroff-reset-signal-volt"))
		host->sd_poweroff_reset_signal_volt = true;
	else
		host->sd_poweroff_reset_signal_volt = false;
	dev_dbg(&pdev->dev, "sd-poweroff-reset-signal-volt: %d\n", host->sd_poweroff_reset_signal_volt);
}

static int msdc_of_clock_parse(struct platform_device *pdev,
			       struct msdc_host *host)
{
	int ret;

	host->src_clk = devm_clk_get(&pdev->dev, "source");
	if (IS_ERR(host->src_clk))
		return PTR_ERR(host->src_clk);

	host->h_clk = devm_clk_get(&pdev->dev, "hclk");
	if (IS_ERR(host->h_clk))
		return PTR_ERR(host->h_clk);

	host->bus_clk = devm_clk_get_optional(&pdev->dev, "bus_clk");
	if (IS_ERR(host->bus_clk))
		host->bus_clk = NULL;

	host->crypto_clk = devm_clk_get_optional(&pdev->dev, "crypto_clk");
	if (IS_ERR(host->crypto_clk)) {
		host->crypto_clk = NULL;
		dev_info(&pdev->dev, "Cannot get crypto clk\n");
	}

	/*source clock control gate is optional clock*/
	host->src_clk_cg = devm_clk_get_optional(&pdev->dev, "source_cg");
	if (IS_ERR(host->src_clk_cg))
		return PTR_ERR(host->src_clk_cg);

	/*
	 * Fallback for legacy device-trees: src_clk and HCLK use the same
	 * bit to control gating but they are parented to a different mux,
	 * hence if our intention is to gate only the source, required
	 * during a clk mode switch to avoid hw hangs, we need to gate
	 * its parent (specified as a different clock only on new DTs).
	 */
	if (!host->src_clk_cg) {
		host->src_clk_cg = clk_get_parent(host->src_clk);
		if (IS_ERR(host->src_clk_cg))
			return PTR_ERR(host->src_clk_cg);
	}

	host->sys_clk_cg = devm_clk_get_optional(&pdev->dev, "sys_cg");
	if (IS_ERR(host->sys_clk_cg))
		host->sys_clk_cg = NULL;

	host->crypto_cg = devm_clk_get_optional(&pdev->dev, "crypto_cg");
	if (IS_ERR(host->crypto_cg))
		host->crypto_cg = NULL;

	/* If present, always enable for this clock gate */
	ret = clk_prepare_enable(host->sys_clk_cg);
	if (ret) {
		dev_err(&pdev->dev, "Cannot get prepare sys_cg clk\n");
		return ret;
	}
	host->bulk_clks[0].id = "pclk_cg";
	host->bulk_clks[1].id = "axi_cg";
	host->bulk_clks[2].id = "ahb_cg";
	ret = devm_clk_bulk_get_optional(&pdev->dev, MSDC_NR_CLOCKS,
					 host->bulk_clks);
	if (ret) {
		dev_err(&pdev->dev, "Cannot get pclk/axi/ahb clock gates\n");
		return ret;
	}

	return 0;
}

#ifdef ICOM_MTK_MMC_CPU_LATENCY_REQUEST
static ssize_t cpu_latency_req_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);

	return sysfs_emit(buf, "%d\n", host->cpu_latency_req);
}

static ssize_t cpu_latency_req_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t n)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);
	s32 value = -EINVAL;

	if (kstrtos32(buf, 0, &value))
		return -EINVAL;

	if (host->cpu_latency_req < 0 && value >= 0) {
		/* NEW */
		host->cpu_latency_req = value; /* update in next request */
		if (host->cpu_dev)
			dev_pm_qos_update_request(host->cpu_dev->power.qos->resume_latency_req,
				PM_QOS_RESUME_LATENCY_NO_CONSTRAINT);
		else
			cpu_latency_qos_add_request(&host->pm_qos_req, PM_QOS_DEFAULT_VALUE);
		dev_alert(dev, "ADDED cpu-latency-req: %d\n", host->cpu_latency_req);
	} else if (host->cpu_latency_req >= 0 && value < 0) {
		/* REMOVE */
		host->cpu_latency_req = value;
		if (host->cpu_dev)
			dev_pm_qos_update_request(host->cpu_dev->power.qos->resume_latency_req,
				PM_QOS_RESUME_LATENCY_NO_CONSTRAINT);
		else
			cpu_latency_qos_remove_request(&host->pm_qos_req);
		dev_alert(dev, "REMOVED cpu-latency-req: %d\n", host->cpu_latency_req);
	} else if (host->cpu_latency_req >= 0 && value >= 0) {
		/* UPDATE */
		host->cpu_latency_req = value; /* update in next request */
#if 0
		if (host->cpu_dev)
			dev_pm_qos_update_request(host->cpu_dev->power.qos->resume_latency_req,
				host->cpu_latency_req);
		else
			cpu_latency_qos_update_request(&host->pm_qos_req, host->cpu_latency_req);
#endif
		dev_alert(dev, "UPDATE cpu-latency-req: %d at NEXT runtime resume\n", host->cpu_latency_req);
	}

	return n;
}

static DEVICE_ATTR_RW(cpu_latency_req);
#endif

static ssize_t tuning_both_edges_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);

	return sysfs_emit(buf, "%d\n", host->tuning_both_edges);
}

static ssize_t tuning_both_edges_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t n)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);
	u32 value = 0;

	if (kstrtou32(buf, 0, &value))
		return -EINVAL;

	host->tuning_both_edges = value ? true : false;

	dev_alert(dev, "new tuning-both-edges: %d\n", host->tuning_both_edges);

	return n;
}

static DEVICE_ATTR_RW(tuning_both_edges);

#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
static ssize_t rising_delay_map_mask_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);

	return sysfs_emit(buf, "0x%" PAD_DELAY_PRIx0 "\n", host->rising_delay_map_mask);
}

static ssize_t rising_delay_map_mask_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t n)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);
	PAD_DELAY_TYPE value = 0;
	int ret;

#ifdef PAD_DELAY_USE_64BITS
	ret = kstrtou64(buf, 16, &value);
#else
	ret = kstrtou32(buf, 16, &value);
#endif
	if (ret) {
		dev_err(dev, "%s: invalid value ('%s') err %d\n", __func__, buf, ret);
		return ret;
	}
	if (value == 0) {
		dev_err(dev, "%s: invalid value ('%s') 0\n", __func__, buf);
		return -EINVAL;
	}

	dev_alert(dev, "new rising-delay-map-mask: 0x%" PAD_DELAY_PRIx0 "\n", value);

	host->rising_delay_map_mask = value;

	return n;
}

static DEVICE_ATTR_RW(rising_delay_map_mask);

static ssize_t falling_delay_map_mask_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);

	return sysfs_emit(buf, "0x%" PAD_DELAY_PRIx0 "\n", host->falling_delay_map_mask);
}

static ssize_t falling_delay_map_mask_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t n)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);
	PAD_DELAY_TYPE value = 0;
	int ret;

#ifdef PAD_DELAY_USE_64BITS
	ret = kstrtou64(buf, 16, &value);
#else
	ret = kstrtou32(buf, 16, &value);
#endif
	if (ret) {
		dev_err(dev, "%s: invalid value ('%s') err %d\n", __func__, buf, ret);
		return ret;
	}
	if (value == 0) {
		dev_err(dev, "%s: invalid value ('%s') 0\n", __func__, buf);
		return -EINVAL;
	}

	dev_alert(dev, "new falling-delay-map-mask: 0x%" PAD_DELAY_PRIx0 "\n", value);

	host->falling_delay_map_mask = value;

	return n;
}

static DEVICE_ATTR_RW(falling_delay_map_mask);
#endif

static ssize_t clk_auto_powerdown_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);

	return sysfs_emit(buf, "%d\n", host->clk_auto_powerdown);
}

static ssize_t clk_auto_powerdown_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t n)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);
	s32 value = 0;

	if (kstrtos32(buf, 0, &value) || value < 0)
		return -EINVAL;

	dev_alert(dev, "new clk-auto-power-down: %u (for next rescan)\n", value);

	host->clk_auto_powerdown = value ? true : false;

	return n;
}

static DEVICE_ATTR_RW(clk_auto_powerdown);

static ssize_t ckgen_delay_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);

	return sysfs_emit(buf, "%u\n", host->ckgen_delay);
}

static ssize_t ckgen_delay_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t n)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);
	s32 value = 0;

	if (kstrtos32(buf, 0, &value) || value < 0)
		return -EINVAL;

	dev_alert(dev, "new ckgen-delay: %u (for next rescan)\n", value);

	host->ckgen_delay = value;

	return n;
}

static DEVICE_ATTR_RW(ckgen_delay);

static ssize_t latch_ck_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);

	return sysfs_emit(buf, "%u\n", host->latch_ck);
}

static ssize_t latch_ck_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t n)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);
	s32 value = 0;

	if (kstrtos32(buf, 0, &value) || value < 0)
		return -EINVAL;

	dev_alert(dev, "new latch-ck: %u (for next rescan)\n", value);

	host->latch_ck = value;

	return n;
}

static DEVICE_ATTR_RW(latch_ck);

static ssize_t retune_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	//struct msdc_host *host = mmc_priv(mmc);

	//mmc_retune_recheck(mmc);
	mmc_retune_needed(mmc);

	return 0;
}

static DEVICE_ATTR_RO(retune);

static ssize_t force_rescan_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);

	dev_alert(dev, "force rescan\n");

	pm_runtime_get_sync(host->dev);

	mmc_remove_host(mmc);

	msleep(500);

	mmc->rescan_entered = 0;
	mmc_add_host(mmc);

	pm_runtime_put_noidle(host->dev);
	//pm_runtime_mark_last_busy(host->dev);
	//pm_runtime_put_autosuspend(host->dev);

	return 0;
}

static DEVICE_ATTR_RO(force_rescan);

static ssize_t host_remove_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);

	mmc_remove_host(mmc);
	mmc->rescan_entered = 0;

	return 0;
}

static DEVICE_ATTR_RO(host_remove);

static ssize_t host_add_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);

	mmc->rescan_entered = 0;
	mmc_add_host(mmc);

	return 0;
}

static DEVICE_ATTR_RO(host_add);

static struct attribute *_attributes[] = {
#ifdef ICOM_MTK_MMC_CPU_LATENCY_REQUEST
	&dev_attr_cpu_latency_req.attr,
#endif
	&dev_attr_tuning_both_edges.attr,
#ifdef ICOM_MTK_MMC_NEW_BEST_DELAY
	&dev_attr_rising_delay_map_mask.attr,
	&dev_attr_falling_delay_map_mask.attr,
#endif
	&dev_attr_clk_auto_powerdown.attr,
	&dev_attr_ckgen_delay.attr,
	&dev_attr_latch_ck.attr,
	&dev_attr_retune.attr,
	&dev_attr_force_rescan.attr,
	&dev_attr_host_remove.attr,
	&dev_attr_host_add.attr,
	NULL,
};

static const struct attribute_group _attr_group = {
	.attrs = _attributes,
};

static int msdc_drv_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct msdc_host *host;
	struct resource *res;
	int ret;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}

#if 0
	if (of_property_read_bool(pdev->dev.of_node, "resets") &&
		of_property_read_bool(pdev->dev.of_node, "reset-names")) {
		/* Refer to ufs_mtk_probe() */
		struct device_node *reset_node;
		struct platform_device *reset_pdev;
		struct device_link *link;

		ret = -ENODEV;
		reset_node = of_find_compatible_node(NULL, NULL,
					"ti,syscon-reset");
		if (reset_node) {
			reset_pdev = of_find_device_by_node(reset_node);
			if (reset_pdev) {
				link = device_link_add(&pdev->dev, &reset_pdev->dev,
					DL_FLAG_AUTOPROBE_CONSUMER);
				if (link) {
					/* supplier is not probed */
					if (link->status == DL_STATE_DORMANT) {
						dev_err(&pdev->dev, "wait for ti,syscon-reset probed\n");
						ret = -EPROBE_DEFER;
					} else
						ret = 0;
				} else
					dev_notice(&pdev->dev, "add reset device_link fail\n");
			} else
				dev_alert(&pdev->dev, "find reset_pdev fail\n");

			of_node_put(reset_node);
		} else
			dev_alert(&pdev->dev, "find ti,syscon-reset fail\n");

		if (ret)
			return ret;
	}
#endif

	/* Allocate MMC host for this device */
	mmc = mmc_alloc_host(sizeof(struct msdc_host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->dev = &pdev->dev;
	ret = mmc_of_parse(mmc);
	if (ret)
		goto host_free;

	ratelimit_state_init(&host->cmd_err_ratelimit, 5 * HZ, 1); /* 5secs */
	ratelimit_state_init(&host->cmd_err_crc_ratelimit, HZ, 1); /* 1sec */
	ratelimit_state_init(&host->data_err_ratelimit, 5 * HZ, 1); /* 5secs */
	ratelimit_state_init(&host->data_err_crc_ratelimit, HZ, 1); /* 1secs */

	if (device_property_read_bool(mmc->parent, "cap-sdio-async-int"))
		host->enable_async_irq = true;

	host->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto host_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		host->top_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(host->top_base))
			host->top_base = NULL;
	}

	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		goto host_free;

	host->vioa = devm_regulator_get_optional(&pdev->dev, "vioa");
	if (IS_ERR(host->vioa)) {
		if (PTR_ERR(host->vioa) == -EPROBE_DEFER) {
			ret = PTR_ERR(host->vioa);
			goto host_free;
		}
		host->vioa = NULL;
		dev_dbg(&pdev->dev, "No vio-a regulator found\n");
	}

	host->viob = devm_regulator_get_optional(&pdev->dev, "viob");
	if (IS_ERR(host->viob)) {
		if (PTR_ERR(host->viob) == -EPROBE_DEFER) {
			ret = PTR_ERR(host->viob);
			goto host_free;
		}
		host->viob = NULL;
		dev_dbg(&pdev->dev, "No vio-b regulator found\n");
	}

	ret = msdc_of_clock_parse(pdev, host);
	if (ret)
		goto host_free;

	host->reset = devm_reset_control_get_optional_exclusive(&pdev->dev,
								"hrst");
	if (IS_ERR(host->reset)) {
		ret = PTR_ERR(host->reset);
		goto host_free;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		ret = -EINVAL;
		goto host_free;
	}

	host->signal_voltage = -1;

	host->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(host->pinctrl)) {
		ret = PTR_ERR(host->pinctrl);
		dev_err(&pdev->dev, "Cannot find pinctrl: %d\n", ret);
		goto host_free;
	}

	host->pins_normal = pinctrl_lookup_state(host->pinctrl, "state_normal");
	if (IS_ERR(host->pins_normal)) {
		ret = PTR_ERR(host->pins_normal);
		dev_err(&pdev->dev, "Cannot find pinctrl normal: %d\n", ret);
		goto host_free;
	}

	host->pins_poweroff = pinctrl_lookup_state(host->pinctrl, "power-off");
	if (IS_ERR(host->pins_poweroff)) {
		ret = PTR_ERR(host->pins_poweroff);
		dev_dbg(&pdev->dev, "Cannot find pinctrl poweroff: %d\n", ret);
		host->pins_poweroff = NULL;
	}

	host->pins_poweron = pinctrl_lookup_state(host->pinctrl, "power-on");
	if (IS_ERR(host->pins_poweron)) {
		ret = PTR_ERR(host->pins_poweron);
		dev_dbg(&pdev->dev, "Cannot find pinctrl poweron: %d\n", ret);
		host->pins_poweron = NULL;
	}

	host->pins_uhs = pinctrl_lookup_state(host->pinctrl, "state_uhs");
	if (IS_ERR(host->pins_uhs)) {
		ret = PTR_ERR(host->pins_uhs);
		dev_err(&pdev->dev, "Cannot find pinctrl uhs: %d\n", ret);
		goto host_free;
	}

	if (!(mmc->caps2 & MMC_CAP2_NO_SDIO) && host->enable_async_irq) { /* SDIO */
		/* Support for SDIO eint irq */
		host->pins_eint = pinctrl_lookup_state(host->pinctrl, "state_eint");
		if (IS_ERR(host->pins_eint)) {
			dev_err(&pdev->dev, "Cannot find pinctrl eint: %d\n", PTR_ERR(host->pins_eint));
			host->pins_eint = NULL;
			host->enable_async_irq = false;
		} else if (host->pins_eint) {
#if 0
			host->pins_dat1 = pinctrl_lookup_state(host->pinctrl, "state_dat1");
			if (IS_ERR(host->pins_dat1)) {
				dev_info(&pdev->dev, "Cannot find pinctrl dat1!\n");
				goto host_free;
			}
#endif
			host->sdio_eint_ready = true;
		}
	}

	if (!(mmc->caps2 & MMC_CAP2_NO_SDIO) && !host->sdio_eint_ready) /* SDIO */
		dev_info(&pdev->dev, "%s: skip SDIO CCCR Async-IRQ\n",
				mmc_hostname(mmc));

	msdc_of_property_parse(pdev, host);

	//host->dev = &pdev->dev;
	host->dev_comp = of_device_get_match_data(&pdev->dev);
	host->src_clk_freq = clk_get_rate(host->src_clk);
	/* Set host parameters to mmc */
	mmc->ops = &mt_msdc_ops;
	if (host->dev_comp->clk_div_bits == 8)
		mmc->f_min = DIV_ROUND_UP(host->src_clk_freq, 4 * 255);
	else
		mmc->f_min = DIV_ROUND_UP(host->src_clk_freq, 4 * 4095);

	if (!(mmc->caps & MMC_CAP_NONREMOVABLE) &&
	    !mmc_can_gpio_cd(mmc) &&
	    host->dev_comp->use_internal_cd) {
		/*
		 * Is removable but no GPIO declared, so
		 * use internal functionality.
		 */
		host->internal_cd = true;
	}

	if (mmc->caps & MMC_CAP_SDIO_IRQ)
		mmc->caps2 |= MMC_CAP2_SDIO_IRQ_NOTHREAD;

	mmc->caps |= MMC_CAP_CMD23;
#if IS_ENABLED(CONFIG_MMC_CQHCI)
	if (host->cqhci)
		mmc->caps2 |= MMC_CAP2_CQE | MMC_CAP2_CQE_DCMD;
#endif
	/* MMC core transfer sizes tunable parameters */
	mmc->max_segs = MAX_BD_NUM;
	if (host->dev_comp->support_64g)
		mmc->max_seg_size = BDMA_DESC_BUFLEN_EXT;
	else
		mmc->max_seg_size = BDMA_DESC_BUFLEN;
	mmc->max_blk_size = 2048;
	mmc->max_req_size = 512 * 1024;
	mmc->max_blk_count = mmc->max_req_size / 512;
	if (host->dev_comp->support_64g)
		host->dma_mask = DMA_BIT_MASK(36);
	else
		host->dma_mask = DMA_BIT_MASK(32);
	mmc_dev(mmc)->dma_mask = &host->dma_mask;

	host->timeout_clks = 3 * 1048576;
	host->dma.gpd = dma_alloc_coherent(&pdev->dev,
				2 * sizeof(struct mt_gpdma_desc),
				&host->dma.gpd_addr, GFP_KERNEL);
	host->dma.bd = dma_alloc_coherent(&pdev->dev,
				MAX_BD_NUM * sizeof(struct mt_bdma_desc),
				&host->dma.bd_addr, GFP_KERNEL);
	if (!host->dma.gpd || !host->dma.bd) {
		ret = -ENOMEM;
		goto release_mem;
	}
	msdc_init_gpd_bd(host, &host->dma);
	INIT_DELAYED_WORK(&host->req_timeout, msdc_request_timeout);
	spin_lock_init(&host->lock);

	platform_set_drvdata(pdev, mmc);
	ret = msdc_ungate_clock(host);
	if (ret) {
		dev_err(&pdev->dev, "Cannot ungate clocks: %d\n", ret);
		goto release_mem;
	}
	msdc_init_hw(host);

#if IS_ENABLED(CONFIG_MMC_CQHCI)
	if (mmc->caps2 & MMC_CAP2_CQE) {
		host->cq_host = devm_kzalloc(mmc->parent,
					     sizeof(*host->cq_host),
					     GFP_KERNEL);
		if (!host->cq_host) {
			ret = -ENOMEM;
			goto host_free;
		}
		host->cq_host->caps |= CQHCI_TASK_DESC_SZ_128;
		host->cq_host->mmio = host->base + 0x800;
		host->cq_host->ops = &msdc_cmdq_ops;
		ret = cqhci_init(host->cq_host, mmc, true);
		if (ret)
			goto host_free;
		mmc->max_segs = 128;
		/* cqhci 16bit length */
		/* 0 size, means 65536 so we don't have to -1 here */
		mmc->max_seg_size = 64 * 1024;
	}
#endif

{
	u32 cpu = 0;
	if (of_property_read_u32(pdev->dev.of_node, "affinity-hint-cpu", &cpu) != 0) {
		if (!(mmc->caps2 & MMC_CAP2_NO_SDIO)) /* SDIO */
			cpu = 2;
		else /* not SDIO */
			cpu = 3;
	}
	ret = irq_set_affinity_hint(host->irq, get_cpu_mask(cpu));
	if (ret)
		dev_warn(&pdev->dev, "set irq affinity hint cpu#%u err %d\n", cpu, ret);
	else {
		dev_dbg(&pdev->dev, "set irq affinity hint: cpu#%u\n", cpu);
		host->affinity_hint_cpu = cpu;
	}
}
	ret = devm_request_irq(&pdev->dev, host->irq, msdc_irq,
			       IRQF_TRIGGER_NONE, pdev->name, host);
	if (ret)
		goto release;

#if 0 /* Moved before devm_request_irq() */
	if (!(mmc->caps2 & MMC_CAP2_NO_SDIO))
		irq_set_affinity_hint(host->irq, get_cpu_mask(2));
	else
		irq_set_affinity_hint(host->irq, get_cpu_mask(3));
#endif

	/* Support for SDIO eint irq ? */
	if (host->sdio_eint_ready) {
		ret = msdc_request_dat1_eint_irq(host);
		if (ret) {
			dev_err(&pdev->dev, "Failed to register data1 eint irq: %d\n", ret);
			goto release;
		}

#if 0
		pinctrl_select_state(host->pinctrl, host->pins_dat1);
#endif

		ret = device_init_wakeup(&pdev->dev, true);
		if (ret)
			dev_err(&pdev->dev, "Failed to init device wakeup: %d\n", ret);
#if 0 /* Don't set MMC_CAP_CD_WAKE */
	} else if (!(mmc->caps2 & MMC_CAP2_NO_SD) && !host->internal_cd &&
			mmc_card_is_removable(mmc) && mmc_can_gpio_cd(mmc)) { /* SD Card */
		ret = device_init_wakeup(&pdev->dev, true);
		if (ret)
			dev_err(&pdev->dev, "Failed to init device wakeup: %d\n", ret);
		else
			mmc->caps |= MMC_CAP_CD_WAKE;
#endif
	}

#ifdef ICOM_MTK_MMC_DVFSRC_VCORE
	if (host->dvfsrc_vcore_power) {
		ret = regulator_set_voltage(host->dvfsrc_vcore_power,
				host->req_vcore, INT_MAX);
		if (ret)
			dev_err(&pdev->dev, "%s: set vcore to %d err %d\n", __func__,
				host->req_vcore, ret);
		else
			dev_alert(&pdev->dev, "%s: requested vcore %d\n",
				mmc_hostname(mmc), host->req_vcore);
	}
#endif

#ifdef ICOM_MTK_MMC_CPU_LATENCY_REQUEST
	if (host->cpu_latency_req >= 0) {
		if (host->use_affinity_hint_cpu_latency && host->affinity_hint_cpu < CONFIG_NR_CPUS)
			host->cpu_dev = get_cpu_device(host->affinity_hint_cpu);
		else
			host->cpu_dev = NULL;
		if (host->cpu_dev) {
			/* it's okay to apply PM_QOS_RESUME_LATENCY_NO_CONSTRAINT here */
			dev_pm_qos_update_request(host->cpu_dev->power.qos->resume_latency_req,
				PM_QOS_RESUME_LATENCY_NO_CONSTRAINT);
			dev_alert(&pdev->dev, "%s: affinity-hint cpu#%u latency request: %d\n",
					mmc_hostname(mmc), host->affinity_hint_cpu, host->cpu_latency_req);
		} else {
			/* it's okay to apply PM_QOS_DEFAULT_VALUE here */
			cpu_latency_qos_add_request(&host->pm_qos_req, PM_QOS_DEFAULT_VALUE);
			//cpu_latency_qos_add_request(&host->pm_qos_req, host->cpu_latency_req);
			dev_alert(&pdev->dev, "%s: global cpu latency request: %d\n",
					mmc_hostname(mmc), host->cpu_latency_req);
		}
	}
#endif

	pm_runtime_set_active(host->dev);
	pm_runtime_set_autosuspend_delay(host->dev, host->autosuspend_delay);
	pm_runtime_use_autosuspend(host->dev);
	pm_runtime_enable(host->dev);
	ret = mmc_add_host(mmc);

	if (ret)
		goto end;

	ret = sysfs_create_group(&pdev->dev.kobj, &_attr_group);
	if (ret)
		dev_warn(&pdev->dev, "create sysfs err %d\n", ret);

	dev_alert(&pdev->dev, "probed\n");

	return 0;
end:
#ifdef ICOM_MTK_MMC_DVFSRC_VCORE
	if (host->dvfsrc_vcore_power) {
		ret = regulator_set_voltage(host->dvfsrc_vcore_power, 0, INT_MAX);
		if (ret)
			dev_err(&pdev->dev, "%s: release vcore err %d\n", __func__, ret);
	}
#endif
	pm_runtime_disable(host->dev);
#ifdef ICOM_MTK_MMC_CPU_LATENCY_REQUEST
	if (host->cpu_latency_req >= 0 && !host->cpu_dev)
		cpu_latency_qos_remove_request(&host->pm_qos_req);
#endif
release:
	irq_set_affinity_hint(host->irq, NULL);
	platform_set_drvdata(pdev, NULL);
	msdc_deinit_hw(host);
	msdc_gate_clock(host);
release_mem:
	if (host->dma.gpd)
		dma_free_coherent(&pdev->dev,
			2 * sizeof(struct mt_gpdma_desc),
			host->dma.gpd, host->dma.gpd_addr);
	if (host->dma.bd)
		dma_free_coherent(&pdev->dev,
			MAX_BD_NUM * sizeof(struct mt_bdma_desc),
			host->dma.bd, host->dma.bd_addr);
host_free:
	mmc_free_host(mmc);
	dev_err(&pdev->dev, "probe err: %d\n", ret);

	return ret;
}

static int msdc_drv_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct msdc_host *host;

	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);

	pm_runtime_get_sync(host->dev);

	sysfs_remove_group(&pdev->dev.kobj, &_attr_group);

	irq_set_affinity_hint(host->irq, NULL);
	platform_set_drvdata(pdev, NULL);
	mmc_remove_host(mmc);
	msdc_deinit_hw(host);
	msdc_gate_clock(host);

	pm_runtime_disable(host->dev);
	pm_runtime_put_noidle(host->dev);
#ifdef ICOM_MTK_MMC_DVFSRC_VCORE
	if (host->dvfsrc_vcore_power)
		regulator_set_voltage(host->dvfsrc_vcore_power, 0, INT_MAX);
#endif
#ifdef ICOM_MTK_MMC_CPU_LATENCY_REQUEST
	if (host->cpu_latency_req >= 0 && !host->cpu_dev)
		cpu_latency_qos_remove_request(&host->pm_qos_req);
#endif
	dma_free_coherent(&pdev->dev,
			2 * sizeof(struct mt_gpdma_desc),
			host->dma.gpd, host->dma.gpd_addr);
	dma_free_coherent(&pdev->dev, MAX_BD_NUM * sizeof(struct mt_bdma_desc),
			host->dma.bd, host->dma.bd_addr);

	mmc_free_host(mmc);

	return 0;
}

static void msdc_save_reg(struct msdc_host *host)
{
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	host->save_para.msdc_cfg = readl(host->base + MSDC_CFG);
	host->save_para.iocon = readl(host->base + MSDC_IOCON);
	host->save_para.sdc_cfg = readl(host->base + SDC_CFG);
	host->save_para.patch_bit0 = readl(host->base + MSDC_PATCH_BIT);
	host->save_para.patch_bit1 = readl(host->base + MSDC_PATCH_BIT1);
	host->save_para.patch_bit2 = readl(host->base + MSDC_PATCH_BIT2);
	host->save_para.pad_ds_tune = readl(host->base + PAD_DS_TUNE);
	host->save_para.pad_cmd_tune = readl(host->base + PAD_CMD_TUNE);
	host->save_para.emmc50_cfg0 = readl(host->base + EMMC50_CFG0);
	host->save_para.emmc50_cfg3 = readl(host->base + EMMC50_CFG3);
	host->save_para.sdc_fifo_cfg = readl(host->base + SDC_FIFO_CFG);
	if (host->top_base) {
		host->save_para.emmc_top_control =
			readl(host->top_base + EMMC_TOP_CONTROL);
		host->save_para.emmc_top_cmd =
			readl(host->top_base + EMMC_TOP_CMD);
		host->save_para.emmc50_pad_ds_tune =
			readl(host->top_base + EMMC50_PAD_DS_TUNE);
	} else {
		host->save_para.pad_tune = readl(host->base + tune_reg);
	}
}

static void msdc_restore_reg(struct msdc_host *host)
{
	struct mmc_host *mmc = mmc_from_priv(host);
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	writel(host->save_para.msdc_cfg, host->base + MSDC_CFG);
	writel(host->save_para.iocon, host->base + MSDC_IOCON);
	writel(host->save_para.sdc_cfg, host->base + SDC_CFG);
	writel(host->save_para.patch_bit0, host->base + MSDC_PATCH_BIT);
	writel(host->save_para.patch_bit1, host->base + MSDC_PATCH_BIT1);
	writel(host->save_para.patch_bit2, host->base + MSDC_PATCH_BIT2);
	writel(host->save_para.pad_ds_tune, host->base + PAD_DS_TUNE);
	writel(host->save_para.pad_cmd_tune, host->base + PAD_CMD_TUNE);
	writel(host->save_para.emmc50_cfg0, host->base + EMMC50_CFG0);
	writel(host->save_para.emmc50_cfg3, host->base + EMMC50_CFG3);
	writel(host->save_para.sdc_fifo_cfg, host->base + SDC_FIFO_CFG);
	if (host->top_base) {
		writel(host->save_para.emmc_top_control,
		       host->top_base + EMMC_TOP_CONTROL);
		writel(host->save_para.emmc_top_cmd,
		       host->top_base + EMMC_TOP_CMD);
		writel(host->save_para.emmc50_pad_ds_tune,
		       host->top_base + EMMC50_PAD_DS_TUNE);
	} else {
		writel(host->save_para.pad_tune, host->base + tune_reg);
	}

	if (!host->sdio_eint_ready && sdio_irq_claimed(mmc))
		__msdc_enable_sdio_irq(host, 1);
}

static int __maybe_unused msdc_runtime_suspend(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);
	unsigned long flags;

	dev_dbg(dev, "%s: %s\n", mmc_hostname(mmc), __func__);

#if IS_ENABLED(CONFIG_MMC_CQHCI)
	if (mmc->caps2 & MMC_CAP2_CQE) {
		int ret;
		u32 val;

		if (mmc->cqe_on)
			mmc->cqe_ops->cqe_off(mmc);

		ret = cqhci_suspend(mmc);
		if (ret)
			return ret;
		val = readl(host->base + MSDC_INT);
		writel(val, host->base + MSDC_INT);
	}
#endif

	msdc_save_reg(host);

	if (host->sdio_eint_ready && sdio_irq_claimed(mmc)) {
		disable_irq(host->irq);
		pinctrl_select_state(host->pinctrl, host->pins_eint);
		spin_lock_irqsave(&host->lock, flags);
		//dev_alert(dev, "msdc_runtime_suspend: sdio_irq_cnt=%d\n", host->sdio_irq_cnt);
		if (host->sdio_irq_cnt == 0) {
			enable_irq(host->eint_irq);
			enable_irq_wake(host->eint_irq);
			host->sdio_irq_cnt++;
		}
		//sdr_clr_bits(host->base + SDC_CFG, SDC_CFG_SDIOIDE);
		__msdc_enable_sdio_irq(host, 0);
		spin_unlock_irqrestore(&host->lock, flags);
	}

	msdc_gate_clock(host);

#ifdef ICOM_MTK_MMC_DVFSRC_VCORE
	if (host->dvfsrc_vcore_power) {
		int ret = regulator_set_voltage(host->dvfsrc_vcore_power, 0, INT_MAX);
		if (ret)
			dev_err(dev, "%s: release vcore err %d\n", __func__, ret);
	}
#endif
#ifdef ICOM_MTK_MMC_CPU_LATENCY_REQUEST
	if (host->cpu_latency_req >= 0) {
		if (host->cpu_dev)
			dev_pm_qos_update_request(host->cpu_dev->power.qos->resume_latency_req,
				PM_QOS_RESUME_LATENCY_NO_CONSTRAINT);
		else
			cpu_latency_qos_update_request(&host->pm_qos_req, PM_QOS_DEFAULT_VALUE);
	}
#endif

	if (!(mmc->caps2 & MMC_CAP2_NO_SD)/* && mmc->ios.power_mode == MMC_POWER_ON*/) /* SD Card */
		if (host->pins_poweroff) {
			dev_dbg(dev, "%s: pins-power-off\n", mmc_hostname(mmc));
			pinctrl_select_state(host->pinctrl, host->pins_poweroff);
		}

	return 0;
}

static int __maybe_unused msdc_runtime_resume(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);
	unsigned long flags;
	int ret;

	dev_dbg(dev, "%s: %s\n", mmc_hostname(mmc), __func__);

	if (!(mmc->caps2 & MMC_CAP2_NO_SD) && mmc->ios.power_mode == MMC_POWER_ON) { /* SD Card */
		if (host->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
			dev_dbg(dev, "%s: MMC_SIGNAL_VOLTAGE_180 & pins-uhs\n", mmc_hostname(mmc));
			pinctrl_select_state(host->pinctrl, host->pins_uhs);
		} else if (host->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
			dev_dbg(dev, "%s: MMC_SIGNAL_VOLTAGE_330 & pins-normal\n", mmc_hostname(mmc));
			pinctrl_select_state(host->pinctrl, host->pins_normal);
		}
	}

#ifdef ICOM_MTK_MMC_CPU_LATENCY_REQUEST
	if (host->cpu_latency_req >= 0) {
		if (host->cpu_dev)
			dev_pm_qos_update_request(host->cpu_dev->power.qos->resume_latency_req,
				host->cpu_latency_req);
		else
			cpu_latency_qos_update_request(&host->pm_qos_req, host->cpu_latency_req);
	}
#endif
#ifdef ICOM_MTK_MMC_DVFSRC_VCORE
	if (host->dvfsrc_vcore_power) {
		ret = regulator_set_voltage(host->dvfsrc_vcore_power,
				host->req_vcore, INT_MAX);
		if (ret)
			dev_err(dev, "%s: set vcore to %d err %d\n", __func__,
				host->req_vcore, ret);
	}
#endif

	ret = msdc_ungate_clock(host);
	if (ret) {
		dev_err(dev, "cannot ungate clocks: %d\n", ret);
		return ret;
	}

	msdc_restore_reg(host);

	if (host->sdio_eint_ready && sdio_irq_claimed(mmc)) {
		spin_lock_irqsave(&host->lock, flags);
		//dev_alert(dev, "msdc_runtime_resume: sdio_irq_cnt=%d\n", host->sdio_irq_cnt);
		if (host->sdio_irq_cnt > 0) {
			disable_irq_nosync(host->eint_irq);
			disable_irq_wake(host->eint_irq);
			host->sdio_irq_cnt--;
			//sdr_set_bits(host->base + SDC_CFG, SDC_CFG_SDIOIDE);
		} else {
			//sdr_clr_bits(host->base + MSDC_INTEN, MSDC_INTEN_SDIOIRQ);
		}
		__msdc_enable_sdio_irq(host, 1);
		spin_unlock_irqrestore(&host->lock, flags);
		if (host->signal_voltage == MMC_SIGNAL_VOLTAGE_180)
			pinctrl_select_state(host->pinctrl, host->pins_uhs);
		else if (host->signal_voltage == MMC_SIGNAL_VOLTAGE_330)
			pinctrl_select_state(host->pinctrl, host->pins_normal);
		enable_irq(host->irq);
	}

#if IS_ENABLED(CONFIG_MMC_CQHCI)
	if (mmc->caps2 & MMC_CAP2_CQE)
		cqhci_resume(mmc);
#endif

	return 0;
}

static int __maybe_unused msdc_suspend(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	//struct msdc_host *host = mmc_priv(mmc);

	dev_dbg(dev, "%s: %s\n", mmc_hostname(mmc), __func__);

#if IS_ENABLED(CONFIG_MMC_CQHCI)
	if (mmc->caps2 & MMC_CAP2_CQE) {
		struct msdc_host *host = mmc_priv(mmc);
		int ret;
		u32 val;

		if (mmc->cqe_on)
			mmc->cqe_ops->cqe_off(mmc);

		ret = cqhci_suspend(mmc);
		if (ret)
			return ret;
		val = readl(host->base + MSDC_INT);
		writel(val, host->base + MSDC_INT);
	}
#endif

	if (!(mmc->caps2 & MMC_CAP2_NO_SDIO)) /* SDIO */
		return 0;

	return pm_runtime_force_suspend(dev);
}

static int __maybe_unused msdc_suspend_noirq(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	//struct msdc_host *host = mmc_priv(mmc);

	dev_dbg(dev, "%s: %s\n", mmc_hostname(mmc), __func__);

	if (!(mmc->caps2 & MMC_CAP2_NO_SDIO)) /* SDIO */
		return pm_runtime_force_suspend(dev);

	if (!(mmc->caps2 & MMC_CAP2_NO_SD) && (mmc->caps & MMC_CAP_CD_WAKE)) { /* SD Card */
		dev_dbg(dev, "%s: %s: GPIO set cd wake enable\n",
				__func__, mmc_hostname(mmc));
		mmc_gpio_set_cd_wake(mmc, true);
	}

	return 0;
}

static int __maybe_unused msdc_resume_noirq(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	//struct msdc_host *host = mmc_priv(mmc);

	dev_dbg(dev, "%s: %s\n", mmc_hostname(mmc), __func__);

	if (!(mmc->caps2 & MMC_CAP2_NO_SDIO)) /* SDIO */
		return pm_runtime_force_resume(dev);

	if (!(mmc->caps2 & MMC_CAP2_NO_SD) && (mmc->caps & MMC_CAP_CD_WAKE)) { /* SD Card */
		dev_dbg(dev, "%s: %s: GPIO set cd wake disable\n",
				__func__, mmc_hostname(mmc));
		mmc_gpio_set_cd_wake(mmc, false);
	}

	return 0;
}

static int __maybe_unused msdc_resume(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	//struct msdc_host *host = mmc_priv(mmc);

	dev_dbg(dev, "%s: %s\n", mmc_hostname(mmc), __func__);

	if (!(mmc->caps2 & MMC_CAP2_NO_SDIO)) /* SDIO */
		return 0;

	return pm_runtime_force_resume(dev);
}

static const struct dev_pm_ops msdc_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(msdc_suspend, msdc_resume)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(msdc_suspend_noirq, msdc_resume_noirq)
	SET_RUNTIME_PM_OPS(msdc_runtime_suspend, msdc_runtime_resume, NULL)
};

static struct platform_driver mt_msdc_driver = {
	.probe = msdc_drv_probe,
	.remove = msdc_drv_remove,
	.driver = {
		.name = "mtk-sd",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = msdc_of_ids,
		.pm = &msdc_dev_pm_ops,
	},
};

module_platform_driver(mt_msdc_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek SD/MMC Card Driver");
