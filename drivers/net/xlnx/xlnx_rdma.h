/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Xilnx, Inc
 */

#ifndef __XLNX_RDMA_H__
#define __XLNX_RDMA_H__

#define XLNX_MAX_RING_SIZE 4096
#define XLNX_MAX_PKT_SIZE 9600
#define XLNX_MAX_QUEUE_PER_PORT	1

struct rdma_dev;

/* Receive Descriptor */
union rdma_rx_desc {
	struct {
		uint32_t pkt_rsvd; /* rsvd */
		uint16_t rsv;
		uint16_t pkt_size; /* Packet buffer size */
		uint64_t pkt_addr; /* Packet buffer address */
	} read;
	struct {
		uint64_t rsvd0; /* Reserved */
		uint64_t rsvd1;
	} wb;
};

/* Transmit Descriptor */
union rdma_tx_desc {
	struct {
		uint32_t magic;
		struct {
			uint16_t rsvd0:14;
			uint16_t eop:1;
			uint16_t sop:1;
		} seop;
		uint16_t pkt_size;
		uint64_t pkt_addr; /* Address of descriptor's data buf */
		/* metadata */
		uint64_t rsvd2;
		uint64_t rsvd3;
	} read;
	struct {
		uint64_t rsvd0; /* Reserved */
		uint64_t rsvd1;
		uint64_t rsvd2;
		uint64_t rsvd3;
	} wb;
};

/* Tx/Rx queue */
struct rdma_queue {
	struct rdma_dev * rdma_dev;

	uint32_t sw_p;
	uint32_t sw_c;
	uint32_t hw_p;
	uint32_t hw_c;

	uint32_t ring_size;
	struct rte_mbuf ** mbufs_info;
	void * ring_vaddr;
	phys_addr_t ring_paddr;

	struct rte_mempool * mb_pool;

	rte_atomic64_t rx_pkts;
	rte_atomic64_t tx_pkts;
	rte_atomic64_t err_pkts;

	uint32_t configured;
};

/* rdma_dev */
struct rdma_dev {
	uint64_t regs_pbase;
	void * regs_vbase;

	uint16_t port_id;

	struct rdma_queue rx_queues[XLNX_MAX_QUEUE_PER_PORT];
	struct rdma_queue tx_queues[XLNX_MAX_QUEUE_PER_PORT];

	rte_spinlock_t reg_lock;
};


#endif /* __XLNX_RDMA_H__*/
