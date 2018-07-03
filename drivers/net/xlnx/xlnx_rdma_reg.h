/*Just get it from tnic_dma_test*/
/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Xilnx, Inc
 */

#ifndef __XLNX_RDMA_REG_H__
#define __XLNX_RDMA_REG_H__

#include <rte_io.h>

/* DMA Registers */
#define  RDMA_BASE_ADDR (0x0000)

/* Tx DMA Channel Registers */
#define RDMA_TX_BASE_ADDR (RDMA_BASE_ADDR+ 0x0000)

/*  DMA Ctrl 32bit Register*/
#define RDMA_TX_CTRL (RDMA_TX_BASE_ADDR + 0x0000)

/* DMA ring buffer address LOW 32bit Register*/
#define RDMA_TXRING_START_ADDR_L (RDMA_TX_BASE_ADDR + 0x0004)

/* DMA ring buffer address High 32bit Register*/
#define RDMA_TXRING_START_ADDR_H (RDMA_TX_BASE_ADDR + 0x0008)

/* DMA ring buffer size 32bit Register*/
#define RDMA_TXRING_SIZE (RDMA_TX_BASE_ADDR + 0x000C)

/* DMA ring buffer producer point Register*/
#define RDMA_TXRING_PRODUCER (RDMA_TX_BASE_ADDR + 0x0010)

/* DMA ring buffer consumer point Register*/
#define RDMA_TXRING_CONSUMER (RDMA_TX_BASE_ADDR + 0x0014)

/* DMA ring buffer consumer writeback address Register*/

#define RDMA_TXRING_CONSUMER_ADDR_L (RDMA_TX_BASE_ADDR + 0x0018)

#define RDMA_TXRING_CONSUMER_ADDR_H (RDMA_TX_BASE_ADDR + 0x001C)

/* DMA ring buffer prefect batch size  Register*/
#define RDMA_TXPKT_BATCH_SIZE (RDMA_TX_BASE_ADDR + 0x0020)

/* Interrupt Registers */

/* DMA Interrupt Ctrl Set Register*/
#define RDMA_TX_INT_CTRL (RDMA_TX_BASE_ADDR + 0x0024)

/* DMA Interrupt Threshold and Delay  Register*/
#define RDMA_TX_INT_THRESHOLD_DELAY (RDMA_TX_BASE_ADDR + 0x0028)

/* DMA Interrupt  resend timeout ctrol Register*/
#define RDMA_TX_INT_RESEND_CTRL  (RDMA_TX_BASE_ADDR + 0x002c)

/*  DMA  Interrupt status buffer Low 32bit address  */
#define RDMA_TX_INT_STATUS_ADDR_L (RDMA_TX_BASE_ADDR + 0x0030)

/*  DMA  Interrupt status buffer High 32bit address  */
#define RDMA_TX_INT_STATUS_ADDR_H  (RDMA_TX_BASE_ADDR + 0x0034)

/*  DMA Interrupt Counter low 32bit  */
#define RDMA_TX_INT_COUNTER_L (RDMA_TX_BASE_ADDR + 0x0038)

/* DMA Interrupt Counter High 32bit  */
#define RDMA_TX_INT_COUNTER_H (RDMA_TX_BASE_ADDR + 0x003c)

/*  DMA Reset Register */
#define RDMA_TX_RESET (RDMA_TX_BASE_ADDR + 0x0040)

/*
***************************RX Registers*****************************
*/

/* Rx DMA Channel Registers */
#define RDMA_RX_BASE_ADDR  (RDMA_BASE_ADDR + 0x60)

/* DMA Ctrl 32bit Register*/
#define RDMA_RX_CTRL (RDMA_RX_BASE_ADDR + 0x0000)

/* DMA ring buffer address LOW 32bit Register*/
#define RDMA_RXRING_START_ADDR_L (RDMA_RX_BASE_ADDR + 0x0004)

/* DMA ring buffer address High 32bit Register*/
#define RDMA_RXRING_START_ADDR_H (RDMA_RX_BASE_ADDR + 0x0008)


/* DMA ring buffer size 32bit Register*/
#define RDMA_RXRING_SIZE (RDMA_RX_BASE_ADDR + 0x000C)

/* DMA ring buffer producer point Register*/
#define RDMA_RXRING_PRODUCER (RDMA_RX_BASE_ADDR + 0x0010)

/* DMA ring buffer consumer point Register*/
#define RDMA_RXRING_CONSUMER (RDMA_RX_BASE_ADDR + 0x0014)

/* DMA ring buffer consumer writeback address Register*/

#define RDMA_RXRING_CONSUMER_ADDR_L (RDMA_RX_BASE_ADDR + 0x0018)

#define RDMA_RXRING_CONSUMER_ADDR_H (RDMA_RX_BASE_ADDR + 0x001C)

/* DMA ring buffer prefect batch size  Register*/
#define RDMA_RXPKT_BATCH_SIZE (RDMA_RX_BASE_ADDR + 0x0020)

/* Interrupt Registers */

/* DMA Interrupt Ctrl Set Register*/
#define RDMA_RX_INT_CTRL (RDMA_RX_BASE_ADDR + 0x0024)

/* DMA Interrupt Threshold and Delay  Register*/
#define RDMA_RX_INT_THRESHOLD_DELAY (RDMA_RX_BASE_ADDR + 0x0028)

/* DMA Interrupt  resend timeout ctrol Register*/
#define RDMA_RX_INT_RESEND_CTRL  (RDMA_RX_BASE_ADDR + 0x002c)

/*  DMA  Interrupt status buffer Low 32bit address  */
#define RDMA_RX_INT_STATUS_ADDR_L (RDMA_RX_BASE_ADDR + 0x0030)

/*  DMA  Interrupt status buffer High 32bit address  */
#define RDMA_RX_INT_STATUS_ADDR_H  (RDMA_RX_BASE_ADDR + 0x0034)

/*  DMA Interrupt Counter low 32bit  */
#define RDMA_RX_INT_COUNTER_L (RDMA_RX_BASE_ADDR + 0x0038)

/*  DMA Interrupt Counter High 32bit  */
#define RDMA_RX_INT_COUNTER_H (RDMA_RX_BASE_ADDR + 0x003c)

/*  DMA Reset Register */
#define RDMA_RX_RESET (RDMA_RX_BASE_ADDR + 0x0040)

/* bit mask */
#define RDMA_RX_CTRL_RXEN	0x00000001
#define RDMA_TX_CTRL_TXEN	0x00000001

#define RDMA_REG_RD32(reg) rte_read32((reg))
#define RDMA_REG_WR32(value, reg) rte_write32((value), (reg))

#endif /* __XLNX_RDMA_REG_H__*/
