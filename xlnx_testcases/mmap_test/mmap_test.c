/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2010-2015 Intel Corporation
 */

#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <inttypes.h>
#include <unistd.h>
#include <ctype.h>
#include <rte_bus_vdev.h>

#include <rte_io.h>

#define REG_READ(reg) rte_read32((reg))
#define REG_WRITE(value, reg) rte_write32((value), (reg))

static inline void
reg_write(void *regs_vbase, uint32_t offset, uint32_t value)
{
        REG_WRITE(value, (uint32_t *)((uint8_t *)regs_vbase + offset));
}

static inline uint32_t
reg_read(void *regs_vbase, uint32_t offset)
{
        return REG_READ((uint32_t *)((uint8_t *)regs_vbase + offset));
}

/*
 * The main function,
 */
int
 main(void)
{
	void *regs_vbase;
	uint32_t rx_base,tx_base;
	uint64_t regs_pbase = 0xa0000000;
        int fd;

        fd = open("/dev/mem", O_RDWR | O_SYNC);
        regs_vbase =  mmap(NULL, 4096,
                        PROT_READ | PROT_WRITE, MAP_SHARED,
                        fd, regs_pbase);
	tx_base=0;
	printf("==============Transmit Registers============\n");
	printf("RDMA_TX_CTRL 		 	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0000));
	printf("RDMA_TXRING_START_ADDR_L 	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0004));
	printf("RDMA_TXRING_START_ADDR_H 	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0008));
	printf("RDMA_TXRING_SIZE 	 	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x000C));
	printf("RDMA_TXRING_PRODUCER 	 	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0010));
 	printf("RDMA_TXRING_CONSUMER 	 	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0014));
        printf("RDMA_TXRING_CONSUMER_ADDR_L	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0018));
	printf("RDMA_TXRING_CONSUMER_ADDR_H  	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x001C));
	printf("RDMA_TXPKT_BATCH_SIZE        	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0020));
	printf("RDMA_TX_INT_CTRL             	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0024));
	printf("RDMA_TX_INT_THRESHOLD_DELAY  	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0028));
	printf("RDMA_TXRING_CONSUMER         	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x002c));
	printf("RDMA_TX_INT_STATUS_ADDR_L    	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0030));
	printf("RDMA_TX_INT_STATUS_ADDR_H    	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0034));
   	printf("RDMA_TX_INT_COUNTER_L        	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0038));
    	printf("RDMA_TX_INT_COUNTER_H        	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x003C));
    	printf("RDMA_TX_RESET                	= 0x%" PRIx32 "\n",reg_read(regs_vbase, tx_base+0x0040));
	rx_base = reg_read(regs_vbase, 0x60);
	printf("==============Recieve Registers=============\n");

	printf("RDMA_RX_CTRL			= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0000));
	printf("RDMA_RXRING_START_ADDR_L	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0004));
	printf("RDMA_RXRING_START_ADDR_H 	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0008));
	printf("RDMA_RXRING_SIZE 		= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x000C));
	printf("RDMA_RXRING_PRODUCER 		= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0010));
 	printf("RDMA_RXRING_CONSUMER 		= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0014));
    	printf("RDMA_RXRING_CONSUMER_ADDR_L 	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0018));
    	printf("RDMA_RXRING_CONSUMER_ADDR_H 	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x001C));
    	printf("RDMA_RXPKT_BATCH_SIZE       	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0020));
    	printf("RDMA_RX_INT_CTRL            	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0024));
    	printf("RDMA_RX_INT_THRESHOLD_DELAY 	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0028));
   	printf("RDMA_RX_INT_RESEND_CTRL     	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x002c));
    	printf("RDMA_RX_INT_STATUS_ADDR_L   	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0030));
    	printf("RDMA_RX_INT_STATUS_ADDR_H   	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0034));
    	printf("RDMA_RX_INT_COUNTER_L      	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0038));
    	printf("RDMA_RX_INT_COUNTER_H       	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x003C));
    	printf("RDMA_RX_RESET               	= 0x%" PRIx32 "\n",reg_read(regs_vbase, rx_base+0x0040));

	close(fd);
	return 0;
}
