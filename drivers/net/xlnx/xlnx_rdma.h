/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Xilnx, Inc
 */

#ifndef __XLNX_RDMA_H__
#define __XLNX_RDMA_H__

#define XLNX_MAX_RING_SIZE 4096
#define XLNX_MAX_PKT_SIZE 2000
#define XLNX_MAX_QUEUE_PER_PORT	1
#define XLNX_QUEUE_STATUS_MSIZE 4096

struct rdma_dev;

/* Receive Descriptor */
#if 0
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
#endif
union rdma_rx_desc {
	struct {
        uint64_t pkt_addr;
        uint32_t pkt_size;
        uint32_t rsv0;
        uint64_t rsv1;
        uint64_t rsv3;
    } read;
	struct {
        uint64_t pkt_addr;
        uint32_t pkt_size;
        uint32_t rsv0;
        uint64_t rsv1;
        uint64_t rsv3;
    } wb;
};

#if 0
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
#endif
union rdma_tx_desc {
	struct {
        uint64_t pkt_addr;
        uint32_t pkt_size;
        uint32_t rsv0;
        uint64_t rsv1;
        uint64_t rsv3;
    } read;
	struct {
        uint64_t pkt_addr;
        uint32_t pkt_size;
        uint32_t rsv0;
        uint64_t rsv1;
        uint64_t rsv3;
    } wb;
};
/* Tx/Rx queue */
struct rdma_queue {
	struct rdma_dev * rdma_dev;
	uint32_t *hw_producer;
	uint32_t *hw_consumer;
	struct rte_mempool * mb_pool;
	struct rte_mbuf ** mbufs_info;
	void * ring_vaddr;
	phys_addr_t ring_paddr;
	void * ring_vend;

	uint32_t sw_p;
	uint32_t sw_c;
	uint32_t hw_p;
	uint32_t hw_c;
	uint32_t in_use; /* 0: unused, 1: in use */
	uint32_t ring_size;

	void * status_vaddr;
	phys_addr_t status_paddr;

	rte_atomic64_t rx_pkts;
	rte_atomic64_t tx_pkts;
	rte_atomic64_t err_pkts;

	uint32_t configured;
}__rte_cache_aligned;

#define XLNX_RDMA_PKT_BATCH 1
#define XLNX_RDMA_PKT_TH_DELAY 32
#define XLNX_RDMA_IRQ_RESENT 100
#define XLNX_RDMA_IRQ_THRESHOLD 16
#define XLNX_RDMA_IRQ_DELAY 50

/* rdma_dev */
struct rdma_dev {
	uint64_t regs_pbase;
	void * regs_vbase;

	uint16_t port_id;

	uint32_t pkt_batch; /* Packet prefect batch size */
	uint32_t pkt_th_delay; /* Packet prefect batch size delay */
	uint32_t irq_resent; /* Interrupt delay in us */
	uint32_t irq_threshold; /* Interrupt threshold */
	uint32_t irq_delay; /* Interrupt delay in us */

	rte_spinlock_t reg_lock;

	struct rdma_queue rx_queues[XLNX_MAX_QUEUE_PER_PORT];
	struct rdma_queue tx_queues[XLNX_MAX_QUEUE_PER_PORT];
}__rte_cache_aligned;

#define CONFIG_RTE_CACHE_LINE_SIZE 128
static inline void
invalidate_dcache_range(uint64_t start, uint64_t stop)
{
	uint32_t cache_line_size = CONFIG_RTE_CACHE_LINE_SIZE;

	__asm__ __volatile__ (
			"sub     x2, %[cls], #1\n\t"
			"bic     %[input_start], %[input_start], x2\n\t"
			"1:\n\t"
			"dc      civac, %[input_start]\n\t"
			"add     %[input_start], %[input_start], %[cls]\n\t"
			"cmp     %[input_start], %[input_stop]\n\t"
			"b.lo    1b\n\t"
			"dsb     sy\n\t"
			: /*This is an empty output operand list */
			: [input_start] "r" (start), [input_stop] "r" (stop), [cls] "r" (cache_line_size));
}

#endif /* __XLNX_RDMA_H__*/
