/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Xilinx, Inc
 */

#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rte_mbuf.h>
#include <rte_ethdev_driver.h>
#include <rte_ethdev_vdev.h>
#include <rte_malloc.h>
#include <rte_memcpy.h>
#include <rte_bus_vdev.h>
#include <rte_kvargs.h>
#include <rte_spinlock.h>
#include "xlnx_logs.h"
#include "xlnx_rdma.h"
#include "xlnx_rdma_reg.h"

#define DRIVER_NAME net_xlnx

#define XLNX_RDMA_MEM_ALIGN 0x1000

#define XLNX_MAX_QUEUE_PER_PORT	1

#define XLNX_RDMA_REGS_PBASE	"pbase"

static const char *valid_arguments[] = {
	XLNX_RDMA_REGS_PBASE,
	NULL
};


static struct ether_addr eth_addr = { .addr_bytes = {0} };
static struct rte_eth_link pmd_link = {
	.link_speed = ETH_SPEED_NUM_25G,
	.link_duplex = ETH_LINK_FULL_DUPLEX,
	.link_status = ETH_LINK_DOWN,
	.link_autoneg = ETH_LINK_AUTONEG,
};

int xlnx_net_logtype_init;
int xlnx_net_logtype_driver;

RTE_INIT(xlnx_net_init_log);
static void
xlnx_net_init_log(void)
{
	xlnx_net_logtype_init = rte_log_register("pmd.net.xlnx.init");
	if (xlnx_net_logtype_init >= 0)
		rte_log_set_level(xlnx_net_logtype_init, RTE_LOG_NOTICE);

	xlnx_net_logtype_driver = rte_log_register("pmd.net.xlnx.driver");
	if (xlnx_net_logtype_driver >= 0)
		rte_log_set_level(xlnx_net_logtype_driver, RTE_LOG_NOTICE);
}

static inline void
rdma_reg_write(void *regs_vbase, uint32_t offset, uint32_t value)
{
	RDMA_REG_WR32(value, (uint32_t *)((uint8_t *)regs_vbase + offset));
}

static inline uint32_t
rdma_reg_read(void *regs_vbase, uint32_t offset)
{
	return RDMA_REG_RD32((uint32_t *)((uint8_t *)regs_vbase + offset));
}

static void
rdma_flash_ring(struct rdma_queue *q)
{
	uint64_t start, stop;

	start = (uint64_t)q->ring_vaddr;
	stop = (uint64_t)q->ring_vend;

	invalidate_dcache_range(start, stop);
}


static uint16_t
eth_xlnx_rx_mbuf_supplement(struct rdma_queue *rxq, uint16_t nb_bufs)
{
	struct rte_mbuf **bufs;
	union rdma_rx_desc *rdesc;
	int i;

	bufs = rxq->mbufs_info;
	for (i = 0; i < nb_bufs; i++)
	{
		bufs[rxq->sw_p] = rte_pktmbuf_alloc(rxq->mb_pool);
		rdesc = (union rdma_rx_desc *)rxq->ring_vaddr + rxq->sw_p;
		rdesc->read.pkt_addr = bufs[rxq->sw_p]->buf_iova;
		rdesc->read.pkt_size = XLNX_MAX_PKT_SIZE;
		rxq->sw_p = (rxq->sw_p + 1) % rxq->ring_size;
		rxq->hw_p = (rxq->sw_p + 1) % rxq->ring_size;
	}

	RDMA_REG_WR32(rxq->hw_p, rxq->hw_producer);

	return nb_bufs;
}

static uint16_t
eth_xlnx_rx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	struct rdma_queue *rxq = q;
	uint32_t cur_mbuf_num; /* Number of pkts received in HW */
	uint32_t ret_mbuf_num; /* Number of pkts returned */
	uint32_t i;

	xlnx_log_dbg("%s\n", __func__);

	if ((q == NULL) || (bufs == NULL))
		return 0;

	rxq->hw_p = RDMA_REG_RD32(rxq->hw_producer);
	rxq->hw_c = RDMA_REG_RD32(rxq->hw_consumer);

	/* empty rx ring */
	if ((rxq->in_use == 0) && (rxq->sw_c == rxq->hw_c))
		return 0;

	rxq->in_use = 1;

	/* Count the pkts received already */
	if (rxq->sw_c > rxq->hw_c)
		cur_mbuf_num = rxq->hw_c + rxq->ring_size - rxq->sw_c;
	else
		cur_mbuf_num = rxq->hw_c - rxq->sw_c;

	/* Count how many pkts can be returned */
	if (cur_mbuf_num >= nb_bufs)
		ret_mbuf_num = nb_bufs;
	else
		ret_mbuf_num = cur_mbuf_num;

	for (i = 0; i < ret_mbuf_num; i++)
	{
		bufs[i] = rxq->mbufs_info[rxq->sw_c];
		/* TODO:
		 * Need to get the length from HW
		 * Now just return the max pkt size
		 */
		bufs[i]->data_len = XLNX_MAX_PKT_SIZE;
		bufs[i]->pkt_len = XLNX_MAX_PKT_SIZE;
		bufs[i]->port = rxq->rdma_dev->port_id;
		rxq->sw_c++;
		/* ring back from end to start */
		if (rxq->sw_c == rxq->ring_size)
			rxq->sw_c = 0;
	}

	eth_xlnx_rx_mbuf_supplement(rxq, ret_mbuf_num);
	rdma_flash_ring(rxq);

	rte_atomic64_add(&(rxq->rx_pkts), ret_mbuf_num);

	return i;
}

static void
eth_xlnx_tx_mbuf_free(struct rdma_queue *txq)
{
	uint32_t cur;
	uint32_t nb_bufs;
	uint32_t i;
	struct rte_mbuf **bufs;

	/* Do not free at first time */
	if (txq->in_use == 0)
		return;

	txq->sw_c = txq->hw_c;
	bufs = txq->mbufs_info;

	if (txq->sw_c < txq->sw_p)
		nb_bufs = txq->ring_size - txq->sw_p + txq->sw_c;
	else
		nb_bufs = txq->sw_c - txq->sw_p;

	cur = txq->sw_p;
	for(i = 0; i < nb_bufs; i++)
	{
		rte_pktmbuf_free(bufs[cur]);
		cur = (cur + 1) % txq->ring_size;
	}
	txq->sw_p = cur;
}

static uint16_t
eth_xlnx_tx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	struct rdma_queue *txq = q;
	union rdma_tx_desc *tx_desc;
	uint32_t send_mbuf_num; /* Number of pkts can sent to HW */
	uint32_t unused_pd_num;
	uint32_t hw_p;
	uint32_t i;

	xlnx_log_dbg("%s\n", __func__);

	if ((q == NULL) || (bufs == NULL))
		return 0;

	txq->hw_p = RDMA_REG_RD32(txq->hw_producer);
	txq->hw_c = RDMA_REG_RD32(txq->hw_consumer);

	/* Count the number of unused PD */
	if (txq->in_use == 0)
		unused_pd_num = txq->ring_size;
	else {
		if (txq->hw_p > txq->sw_p)
			unused_pd_num = txq->ring_size - txq->hw_p + txq->sw_p;
		else
			unused_pd_num = txq->hw_p - txq->sw_p;

	}

	/* PD ring is full, can not send pkt more */
	if (unused_pd_num == 0)
		return 0;

	if (unused_pd_num < nb_bufs)
		send_mbuf_num = unused_pd_num;
	else
		send_mbuf_num = nb_bufs;

	/*
	 * Put bufs to DMA TX ring
	 * and update mbufs_info
	 */
	hw_p = txq->hw_p;
	for (i = 0; i < send_mbuf_num; i++) {
		tx_desc = (union rdma_tx_desc *)txq->ring_vaddr + hw_p;
		txq->mbufs_info[hw_p] = bufs[i];
		tx_desc->read.pkt_addr = rte_mbuf_data_iova(bufs[i]);
		tx_desc->read.pkt_size = rte_pktmbuf_data_len(bufs[i]);
		tx_desc->read.seop.sop = 0x1;
		tx_desc->read.seop.eop = 0x1;
		tx_desc->read.rsvd3 = 0x1;
		hw_p = (hw_p + 1) % txq->ring_size;
	}
	rdma_flash_ring(txq);
	RDMA_REG_WR32(hw_p, txq->hw_producer);

	eth_xlnx_tx_mbuf_free(txq);

	rte_atomic64_add(&(txq->tx_pkts), send_mbuf_num);
	txq->in_use = 1;

	return i;
}

static inline void
eth_xlnx_reset_rdma(struct rdma_dev *rdma_dev)
{
	rdma_reg_write(rdma_dev->regs_vbase, 0xC0, 0x0);
	usleep(100);
	rdma_reg_write(rdma_dev->regs_vbase, 0xC0, 0x2);
	usleep(100);
}

static __rte_unused int
eth_xlnx_disable_rx_queue(struct rdma_dev *rdma_dev)
{
	uint32_t val;
	uint32_t cnt = 10;

	val = rdma_reg_read(rdma_dev->regs_vbase, RDMA_RX_CTRL);
	rdma_reg_write(rdma_dev->regs_vbase,
		RDMA_RX_CTRL, val & ~RDMA_RX_CTRL_RXEN);

	/* hardware may need about 100us */
	do {
		val = rdma_reg_read(rdma_dev->regs_vbase, RDMA_RX_CTRL);
		usleep(10);
	} while (--cnt && (val & RDMA_RX_CTRL_RXEN));
	if (!cnt) {
		RTE_LOG(INFO, PMD, "failed to stop rx queue\n");
		return -EBUSY;
	}

	return 0;
}

static __rte_unused int
eth_xlnx_enable_rx_queue(struct rdma_dev *rdma_dev)
{
	uint32_t val;
	val = rdma_reg_read(rdma_dev->regs_vbase, RDMA_RX_CTRL);
	rdma_reg_write(rdma_dev->regs_vbase,
		RDMA_RX_CTRL, val | RDMA_RX_CTRL_RXEN);

	return 0;
}

static __rte_unused int
eth_xlnx_disable_tx_queue(struct rdma_dev *rdma_dev)
{
	uint32_t val;
	uint32_t cnt = 10;

	val = rdma_reg_read(rdma_dev->regs_vbase, RDMA_TX_CTRL);
	rdma_reg_write(rdma_dev->regs_vbase,
		RDMA_TX_CTRL, val & ~RDMA_TX_CTRL_TXEN);

	/* hardware may need about 100us */
	do {
		val = rdma_reg_read(rdma_dev->regs_vbase, RDMA_TX_CTRL);
		usleep(10);
	} while (--cnt && (val & RDMA_TX_CTRL_TXEN));
	if (!cnt) {
		RTE_LOG(INFO, PMD, "failed to stop tx queue\n");
		return -EBUSY;
	}

	return 0;
}

static __rte_unused int
eth_xlnx_enable_tx_queue(struct rdma_dev *rdma_dev)
{
	uint32_t val;
	val = rdma_reg_read(rdma_dev->regs_vbase, RDMA_TX_CTRL);
	rdma_reg_write(rdma_dev->regs_vbase,
		RDMA_TX_CTRL, val | RDMA_TX_CTRL_TXEN);

	return 0;
}

static int
eth_dev_configure(struct rte_eth_dev *dev)
{
	xlnx_log_info();
	struct rdma_dev *rdma_dev;

	if (dev == NULL)
		return -EINVAL;

	rdma_dev = dev->data->dev_private;
	/* Config the HW */
	rdma_dev->pkt_batch = XLNX_RDMA_PKT_BATCH;
	rdma_dev->pkt_th_delay = XLNX_RDMA_PKT_TH_DELAY;
	rdma_dev->irq_resent = XLNX_RDMA_IRQ_RESENT;
	rdma_dev->irq_threshold = XLNX_RDMA_IRQ_THRESHOLD;
	rdma_dev->irq_delay = XLNX_RDMA_IRQ_DELAY;

	eth_xlnx_disable_rx_queue(rdma_dev);
	eth_xlnx_disable_tx_queue(rdma_dev);

	return 0;
}

static int
eth_dev_start(struct rte_eth_dev *dev)
{
	struct rdma_dev *rdma_dev;
	xlnx_log_info();

	if (dev == NULL)
		return -EINVAL;

	rdma_dev = dev->data->dev_private;

	eth_xlnx_enable_tx_queue(rdma_dev);
	/* FIXME:
	 * Keep same as kernel driver, RX path should be enable later
	 * Move to do it in rx queue setup
	 */
	//eth_xlnx_enable_rx_queue(rdma_dev);

	eth_xlnx_reset_rdma(rdma_dev);

	dev->data->dev_link.link_status = ETH_LINK_UP;
	return 0;
}

static void
eth_dev_stop(struct rte_eth_dev *dev)
{
	struct rdma_dev *rdma_dev;
	xlnx_log_info();

	if (dev == NULL)
		return;

	rdma_dev = dev->data->dev_private;

	eth_xlnx_disable_tx_queue(rdma_dev);
	eth_xlnx_disable_rx_queue(rdma_dev);

	dev->data->dev_link.link_status = ETH_LINK_DOWN;
}

static int
eth_rx_queue_setup(struct rte_eth_dev *dev, uint16_t rx_queue_id,
		uint16_t nb_rx_desc,
		unsigned int socket_id __rte_unused,
		const struct rte_eth_rxconf *rx_conf __rte_unused,
		struct rte_mempool *mb_pool)
{
	struct rdma_dev *rdma_dev;
	struct rdma_queue *rxq;
	struct rte_mbuf *mbuf;
	union rdma_rx_desc *rdesc;
	uint32_t min_size;
	int i;
	int ret;

	xlnx_log_info();

	if ((dev == NULL) || (mb_pool == NULL))
		return -EINVAL;

	if (rx_queue_id >= dev->data->nb_rx_queues)
		return -ENODEV;

	if (!rte_is_power_of_2(nb_rx_desc)) {
		RTE_LOG(ERR, PMD,
			"Unsupported size of RX queue: %d not a power of 2\n",
			nb_rx_desc);
		return -EINVAL;
	}

	if (nb_rx_desc > XLNX_MAX_RING_SIZE) {
		RTE_LOG(ERR, PMD,
			"Unsupported size of RX queue (max size: %d)\n",
			nb_rx_desc);
		return -EINVAL;
	}

	min_size = rte_pktmbuf_data_room_size(mb_pool) - RTE_PKTMBUF_HEADROOM;
	if (min_size < XLNX_MAX_PKT_SIZE) {
		RTE_LOG(ERR, PMD,
			"Mbuf size(=%d, now) must be increased to %u bytes to hold up to %u bytes of data.\n",
			min_size,
			XLNX_MAX_PKT_SIZE + RTE_PKTMBUF_HEADROOM,
			XLNX_MAX_PKT_SIZE);
		return -EINVAL;
	}

	rdma_dev = dev->data->dev_private;
	rxq = &rdma_dev->rx_queues[rx_queue_id];

	rxq->ring_vaddr = rte_zmalloc("rxq->ring_vaddr",
			sizeof(union rdma_rx_desc) * nb_rx_desc,
			XLNX_RDMA_MEM_ALIGN);
	if (!rxq->ring_vaddr) {
		RTE_LOG(ERR, PMD, "failed to alloc mem for rx ring\n");
		return -ENOMEM;
	}
	rxq->ring_paddr = rte_mem_virt2phy(rxq->ring_vaddr);
	rxq->ring_vend = (char *)rxq->ring_vaddr + sizeof(union rdma_rx_desc) * nb_rx_desc;

	rxq->status_vaddr = rte_zmalloc("rxq->status_vaddr",
			XLNX_QUEUE_STATUS_MSIZE, XLNX_RDMA_MEM_ALIGN);
	if (!rxq->status_vaddr) {
		RTE_LOG(ERR, PMD, "failed to alloc mem for rx queue status\n");
		ret = -ENOMEM;
		goto free_ring;
	}
	rxq->status_paddr = rte_mem_virt2phy(rxq->status_vaddr);

	rxq->mbufs_info = rte_zmalloc("rxq->mbufs_info",
			sizeof(struct rte_mbuf *) * nb_rx_desc,
			XLNX_RDMA_MEM_ALIGN);
	if (!rxq->mbufs_info) {
		RTE_LOG(ERR, PMD, "failed to alloc mem for rx mbuf info\n");
		goto enomem1;
	}

	ret = rte_pktmbuf_alloc_bulk(mb_pool, rxq->mbufs_info, nb_rx_desc);
	if (unlikely(ret)) {
		RTE_LOG(ERR, PMD, "failed to alloc mem for rx mbuf info\n");
		goto enomem2;
	}

	/* Fill the PD ring */
	rdesc = (union rdma_rx_desc *)rxq->ring_vaddr;
	for (i = 0; i < nb_rx_desc; i++)
	{
		mbuf = rxq->mbufs_info[i];
		rdesc->read.pkt_addr = mbuf->buf_iova;
		rdesc->read.pkt_size = XLNX_MAX_PKT_SIZE;
		rdesc++;
	}
	rdma_flash_ring(rxq);

	rxq->rdma_dev = rdma_dev;
	rxq->mb_pool = mb_pool;
	rxq->ring_size = nb_rx_desc;
	rxq->sw_p = 0;
	rxq->sw_c = 0;
	rxq->hw_p = 0;
	rxq->hw_c = 0;
	rxq->in_use = 0;
	rxq->hw_producer = (uint32_t *)((uint8_t *)rdma_dev->regs_vbase + RDMA_RXRING_PRODUCER);
	rxq->hw_consumer = (uint32_t *)((uint8_t *)rdma_dev->regs_vbase + RDMA_RXRING_CONSUMER);

	rte_atomic64_init(&rxq->rx_pkts);
	rte_atomic64_init(&rxq->err_pkts);

	rdma_reg_write(rdma_dev->regs_vbase, RDMA_RXRING_START_ADDR_L,
			rxq->ring_paddr & RDMA_ADDRL_MASK);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_RXRING_START_ADDR_H,
			(rxq->ring_paddr & RDMA_ADDRH_MASK) >> 32);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_RXRING_SIZE,
			rxq->ring_size);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_RXRING_CONSUMER_ADDR_L,
			rxq->status_paddr & RDMA_ADDRL_MASK);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_RXRING_CONSUMER_ADDR_H,
			(rxq->status_paddr & RDMA_ADDRH_MASK) >> 32);

	rdma_reg_write(rdma_dev->regs_vbase, RDMA_RXPKT_BATCH_SIZE,
			(rdma_dev->pkt_batch << 16) | rdma_dev->pkt_th_delay);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_RX_INT_RESEND_CTRL,
			rdma_dev->irq_resent);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_RX_INT_THRESHOLD_DELAY,
			(rdma_dev->irq_threshold << 16) | rdma_dev->irq_delay);

	rdma_reg_write(rdma_dev->regs_vbase, RDMA_RXRING_PRODUCER, 0);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_RXRING_CONSUMER, 0);

	dev->data->rx_queues[rx_queue_id] =
		&rdma_dev->rx_queues[rx_queue_id];

	/*
	 * FIXME:
	 *
	 * Keep same as kernel driver, enable RX transfer at here
	 */
	rdma_reg_write(rdma_dev->regs_vbase, 0xC4, XLNX_MAX_PKT_SIZE);
	eth_xlnx_enable_rx_queue(rdma_dev);
	rxq->hw_p = rxq->ring_size - 1;
	RDMA_REG_WR32(rxq->hw_p, rxq->hw_producer);

	rxq->configured = 1;
	return 0;

enomem2:
	rte_free(rxq->mbufs_info);
enomem1:
	rte_free(rxq->status_vaddr);
free_ring:
	rte_free(rxq->ring_vaddr);

	return ret;
}

static int
eth_tx_queue_setup(struct rte_eth_dev *dev, uint16_t tx_queue_id,
		uint16_t nb_tx_desc,
		unsigned int socket_id __rte_unused,
		const struct rte_eth_txconf *tx_conf __rte_unused)
{
	struct rdma_dev *rdma_dev;
	struct rdma_queue *txq;
	int ret __rte_unused;

	xlnx_log_info();

	if (dev == NULL)
		return -EINVAL;

	if (!rte_is_power_of_2(nb_tx_desc)) {
		RTE_LOG(ERR, PMD,
			"Unsupported size of TX queue: %d not a power of 2\n",
			nb_tx_desc);
		return -EINVAL;
	}

	if (nb_tx_desc > XLNX_MAX_RING_SIZE) {
		RTE_LOG(ERR, PMD,
			"Unsupported size of TX queue (max size: %d)\n",
			nb_tx_desc);
		return -EINVAL;
	}

	if (tx_queue_id >= dev->data->nb_tx_queues)
		return -ENODEV;

	rdma_dev = dev->data->dev_private;
	txq = &rdma_dev->tx_queues[tx_queue_id];

	txq->ring_vaddr = rte_zmalloc("txq->ring_vaddr",
			sizeof(union rdma_tx_desc) * nb_tx_desc,
			XLNX_RDMA_MEM_ALIGN);
	if (!txq->ring_vaddr) {
		RTE_LOG(ERR, PMD, "failed to alloc mem for tx ring\n");
		return -ENOMEM;
	}
	txq->ring_paddr = rte_mem_virt2phy(txq->ring_vaddr);
	txq->ring_vend = (char *)txq->ring_vaddr + sizeof(union rdma_tx_desc) * nb_tx_desc;

	txq->status_vaddr = rte_zmalloc("txq->status_vaddr",
			XLNX_QUEUE_STATUS_MSIZE, XLNX_RDMA_MEM_ALIGN);
	if (!txq->status_vaddr) {
		RTE_LOG(ERR, PMD, "failed to alloc mem for tx queue status\n");
		ret = -ENOMEM;
		goto free_ring;
	}
	txq->status_paddr = rte_mem_virt2phy(txq->status_vaddr);

	txq->mbufs_info = rte_zmalloc("txq->mbufs_info",
			sizeof(struct rte_mbuf *) * nb_tx_desc,
			XLNX_RDMA_MEM_ALIGN);
	if (!txq->mbufs_info) {
		RTE_LOG(ERR, PMD, "failed to alloc mem for tx mbuf info\n");
		goto enomem;
	}

	txq->rdma_dev = rdma_dev;
	txq->ring_size = nb_tx_desc;
	txq->sw_p = 0;
	txq->sw_c = 0;
	txq->hw_p = 0;
	txq->hw_c = 0;
	txq->in_use = 0;
	txq->hw_producer = (uint32_t *)((uint8_t *)rdma_dev->regs_vbase + RDMA_TXRING_PRODUCER);
	txq->hw_consumer = (uint32_t *)((uint8_t *)rdma_dev->regs_vbase + RDMA_TXRING_CONSUMER);

	rte_atomic64_init(&txq->tx_pkts);
	rte_atomic64_init(&txq->err_pkts);

	rdma_reg_write(rdma_dev->regs_vbase, RDMA_TXRING_START_ADDR_L,
			txq->ring_paddr & RDMA_ADDRL_MASK);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_TXRING_START_ADDR_H,
			(txq->ring_paddr & RDMA_ADDRH_MASK) >> 32);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_TXRING_SIZE,
			txq->ring_size);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_TXRING_CONSUMER_ADDR_L,
			txq->status_paddr & RDMA_ADDRL_MASK);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_TXRING_CONSUMER_ADDR_H,
			(txq->status_paddr & RDMA_ADDRH_MASK) >> 32);

	rdma_reg_write(rdma_dev->regs_vbase, RDMA_TXPKT_BATCH_SIZE,
			(rdma_dev->pkt_batch << 16) | rdma_dev->pkt_th_delay);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_TX_INT_RESEND_CTRL,
			rdma_dev->irq_resent);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_TX_INT_THRESHOLD_DELAY,
			(rdma_dev->irq_threshold << 16) | rdma_dev->irq_delay);

	rdma_reg_write(rdma_dev->regs_vbase, RDMA_TXRING_PRODUCER, 0);
	rdma_reg_write(rdma_dev->regs_vbase, RDMA_TXRING_CONSUMER, 0);

	dev->data->tx_queues[tx_queue_id] =
		&rdma_dev->tx_queues[tx_queue_id];

	txq->configured = 1;

	return 0;

enomem:
	rte_free(txq->status_vaddr);
free_ring:
	rte_free(txq->ring_vaddr);
	return -ENOMEM;
}

static int
eth_mtu_set(struct rte_eth_dev *dev __rte_unused, uint16_t mtu __rte_unused)
{
	xlnx_log_info();

	return 0;
}

static void
eth_dev_info(struct rte_eth_dev *dev,
		struct rte_eth_dev_info *dev_info)
{
	struct rdma_dev *rdma_dev;

	xlnx_log_info();

	if ((dev == NULL) || (dev_info == NULL))
		return;

	rdma_dev = dev->data->dev_private;
	dev_info->driver_name = RTE_STR(DRIVER_NAME);
	dev_info->speed_capa = ETH_LINK_SPEED_25G;
	dev_info->max_mac_addrs = 1;
	dev_info->max_rx_pktlen = XLNX_MAX_PKT_SIZE;
	dev_info->max_rx_queues = RTE_DIM(rdma_dev->rx_queues);
	dev_info->max_tx_queues = RTE_DIM(rdma_dev->tx_queues);
	dev_info->min_rx_bufsize = 0;
	dev_info->pci_dev = NULL;
}

static int
eth_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *igb_stats)
{
	unsigned i, num_stats;
	unsigned long rx_total = 0, tx_total = 0, tx_err_total = 0;
	const struct rdma_dev *rdma_dev;

	xlnx_log_info();

	if ((dev == NULL) || (igb_stats == NULL))
		return -EINVAL;

	rdma_dev = dev->data->dev_private;
	num_stats = RTE_MIN((unsigned)RTE_ETHDEV_QUEUE_STAT_CNTRS,
			RTE_MIN(dev->data->nb_rx_queues,
				RTE_DIM(rdma_dev->rx_queues)));
	for (i = 0; i < num_stats; i++) {
		igb_stats->q_ipackets[i] =
			rdma_dev->rx_queues[i].rx_pkts.cnt;
		rx_total += igb_stats->q_ipackets[i];
	}

	num_stats = RTE_MIN((unsigned)RTE_ETHDEV_QUEUE_STAT_CNTRS,
			RTE_MIN(dev->data->nb_tx_queues,
				RTE_DIM(rdma_dev->tx_queues)));
	for (i = 0; i < num_stats; i++) {
		igb_stats->q_opackets[i] =
			rdma_dev->tx_queues[i].tx_pkts.cnt;
		igb_stats->q_errors[i] =
			rdma_dev->tx_queues[i].err_pkts.cnt;
		tx_total += igb_stats->q_opackets[i];
		tx_err_total += igb_stats->q_errors[i];
	}

	igb_stats->ipackets = rx_total;
	igb_stats->opackets = tx_total;
	igb_stats->oerrors = tx_err_total;

	return 0;
}

static void
eth_stats_reset(struct rte_eth_dev *dev)
{
	unsigned i;
	struct rdma_dev *rdma_dev;

	xlnx_log_info();

	if (dev == NULL)
		return;

	rdma_dev = dev->data->dev_private;
	for (i = 0; i < RTE_DIM(rdma_dev->rx_queues); i++)
		rdma_dev->rx_queues[i].rx_pkts.cnt = 0;
	for (i = 0; i < RTE_DIM(rdma_dev->tx_queues); i++) {
		rdma_dev->tx_queues[i].tx_pkts.cnt = 0;
		rdma_dev->tx_queues[i].err_pkts.cnt = 0;
	}
}

static void
eth_rx_queue_release(void *q)
{
	struct rdma_queue *rxq;
	uint32_t i;
	rxq = q;

	xlnx_log_info();

	if (rxq == NULL)
		return;

	rxq->configured = 0;
	rte_free(rxq->ring_vaddr);
	rte_free(rxq->status_vaddr);
	for (i = 0; i < rxq->ring_size; i++)
		rte_pktmbuf_free(rxq->mbufs_info[i]);
	rte_free(rxq->mbufs_info);
}

static void
eth_tx_queue_release(void *q)
{
	struct rdma_queue *txq;

	txq = q;

	xlnx_log_info();

	if (txq == NULL)
		return;

	txq->configured = 0;
	rte_free(txq->ring_vaddr);
	rte_free(txq->status_vaddr);
	rte_free(txq->mbufs_info);
}

static int
eth_link_update(struct rte_eth_dev *dev __rte_unused,
		int wait_to_complete __rte_unused)
{
	struct rte_eth_link *link = &dev->data->dev_link;

	xlnx_log_info();

	link->link_status = 1;
	link->link_speed = ETH_SPEED_NUM_25G;
	link->link_duplex = ETH_LINK_FULL_DUPLEX;

	return 0;
}

static void eth_rx_queue_release_all(struct rte_eth_dev *dev)
{
	struct rdma_queue **queues = (struct rdma_queue **)dev->data->rx_queues;
	int nb_queues = dev->data->nb_rx_queues;
	int i;

	for (i = 0; i < nb_queues; i++)
		eth_rx_queue_release(queues[i]);
}

static void eth_tx_queue_release_all(struct rte_eth_dev *dev)
{
	struct rdma_queue **queues = (struct rdma_queue **)dev->data->tx_queues;
	int nb_queues = dev->data->nb_tx_queues;
	int i;

	for (i = 0; i < nb_queues; i++)
		eth_tx_queue_release(queues[i]);
}

static void
eth_dev_close(struct rte_eth_dev *dev)
{
	xlnx_log_info();

	dev->data->dev_link.link_status = ETH_LINK_DOWN;

	/* TODO
	 * HW: disable EN bit for TX and RX channel in HW
	 * HW: Clear PD buffer in HW(reset DMA, write disable bit=1, wait until
	 * hardware set it to 0
	 * */
	eth_rx_queue_release_all(dev);
	eth_tx_queue_release_all(dev);
}

static void
eth_mac_address_set(__rte_unused struct rte_eth_dev *dev,
		    __rte_unused struct ether_addr *addr)
{
	xlnx_log_info();

	return;
}

static const struct eth_dev_ops ops = {
	.dev_start = eth_dev_start,
	.dev_stop = eth_dev_stop,
	.dev_close = eth_dev_close,
	.dev_configure = eth_dev_configure,
	.dev_infos_get = eth_dev_info,
	.rx_queue_setup = eth_rx_queue_setup,
	.tx_queue_setup = eth_tx_queue_setup,
	.rx_queue_release = eth_rx_queue_release,
	.tx_queue_release = eth_tx_queue_release,
	.mtu_set = eth_mtu_set,
	.link_update = eth_link_update,
	.mac_addr_set = eth_mac_address_set,
	.stats_get = eth_stats_get,
	.stats_reset = eth_stats_reset,
};

static struct rte_vdev_driver pmd_xlnx_drv;

static void
xlnx_regs_mmap(struct rdma_dev *rdma_dev)
{
	int fd;

	fd = open("/dev/mem", O_RDWR | O_SYNC);

	rdma_dev->regs_vbase = mmap(NULL, 4096,
			PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, rdma_dev->regs_pbase);

	close(fd);
}

static int
eth_xlnx_dev_create(struct rte_vdev_device *dev, uint64_t regs_pbase)
{
	const unsigned nb_rx_queues = 1;
	const unsigned nb_tx_queues = 1;
	struct rte_eth_dev_data *data = NULL;
	struct rdma_dev *rdma_dev = NULL;
	struct rte_eth_dev *eth_dev = NULL;

	if (dev->device.numa_node == SOCKET_ID_ANY)
		dev->device.numa_node = rte_socket_id();

	RTE_LOG(INFO, PMD, "Creating xlnx ethdev on numa socket %u\n",
		dev->device.numa_node);

	/* now do all data allocation - for eth_dev structure, dummy driver
	 * and private data
	 */
	data = rte_zmalloc_socket(rte_vdev_device_name(dev), sizeof(*data), 0,
		dev->device.numa_node);
	if (!data)
		return -ENOMEM;

	eth_dev = rte_eth_vdev_allocate(dev, sizeof(*rdma_dev));
	if (!eth_dev) {
		rte_free(data);
		return -ENOMEM;
	}

	/* now put it all together
	 * - store queue data in rdma_dev,
	 * - store numa_node info in ethdev data
	 * - point eth_dev_data to rdma_dev
	 * - and point eth_dev structure to new eth_dev_data structure
	 */
	/* NOTE: we'll replace the data element, of originally allocated eth_dev
	 * so the xlnxs are local per-process */

	rdma_dev = eth_dev->data->dev_private;
	rdma_dev->port_id = eth_dev->data->port_id;
	rdma_dev->regs_pbase = regs_pbase;
	xlnx_regs_mmap(rdma_dev);

	rte_memcpy(data, eth_dev->data, sizeof(*data));
	data->nb_rx_queues = (uint16_t)nb_rx_queues;
	data->nb_tx_queues = (uint16_t)nb_tx_queues;
	data->dev_link = pmd_link;
	data->mac_addrs = &eth_addr;

	eth_dev->data = data;
	eth_dev->dev_ops = &ops;

	/* finally assign rx and tx ops */
	eth_dev->rx_pkt_burst = eth_xlnx_rx;
	eth_dev->tx_pkt_burst = eth_xlnx_tx;

	return 0;
}

static inline int
get_rdma_regs_pbase(const char *key __rte_unused,
		const char *value, void *extra_args)
{
	const char *a = value;
	unsigned *regs_pbase = extra_args;

	if ((value == NULL) || (extra_args == NULL))
		return -EINVAL;

	*regs_pbase = (unsigned)strtoul(a, NULL, 0);
	if (*regs_pbase == UINT_MAX)
		return -1;

	return 0;
}

static int
rte_pmd_xlnx_probe(struct rte_vdev_device *dev)
{
	const char *name, *params;
	struct rte_kvargs *kvlist = NULL;
	uint64_t regs_pbase;
	int ret;

	if (!dev)
		return -EINVAL;

	name = rte_vdev_device_name(dev);
	params = rte_vdev_device_args(dev);
	RTE_LOG(INFO, PMD, "Initializing pmd_xlnx for %s\n", name);

	if (params != NULL) {
		kvlist = rte_kvargs_parse(params, valid_arguments);
		if (kvlist == NULL)
			return -1;

		if (rte_kvargs_count(kvlist, XLNX_RDMA_REGS_PBASE) == 1) {

			ret = rte_kvargs_process(kvlist,
					XLNX_RDMA_REGS_PBASE,
					&get_rdma_regs_pbase, &regs_pbase);
			if (ret < 0)
				goto free_kvlist;
		}
	}

	RTE_LOG(INFO, PMD, "Configure pmd_xlnx, regs_pbase=%lu\n", regs_pbase);

	ret = eth_xlnx_dev_create(dev, regs_pbase);

free_kvlist:
	if (kvlist)
		rte_kvargs_free(kvlist);
	return ret;
}

static int
rte_pmd_xlnx_remove(struct rte_vdev_device *dev)
{
	struct rte_eth_dev *eth_dev = NULL;

	if (!dev)
		return -EINVAL;

	RTE_LOG(INFO, PMD, "Closing xlnx ethdev on numa socket %u\n",
			rte_socket_id());

	/* find the ethdev entry */
	eth_dev = rte_eth_dev_allocated(rte_vdev_device_name(dev));
	if (eth_dev == NULL)
		return -1;

	rte_free(eth_dev->data->dev_private);
	rte_free(eth_dev->data);

	rte_eth_dev_release_port(eth_dev);

	return 0;
}

static struct rte_vdev_driver pmd_xlnx_drv = {
	.probe = rte_pmd_xlnx_probe,
	.remove = rte_pmd_xlnx_remove,
};

RTE_PMD_REGISTER_VDEV(net_xlnx, pmd_xlnx_drv);
RTE_PMD_REGISTER_ALIAS(net_xlnx, eth_xlnx);
RTE_PMD_REGISTER_PARAM_STRING(net_xlnx,
	"size=<int> ");
