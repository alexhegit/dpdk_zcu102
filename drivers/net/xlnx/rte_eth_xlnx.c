/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Xilinx, Inc
 */

#include <rte_mbuf.h>
#include <rte_ethdev_driver.h>
#include <rte_ethdev_vdev.h>
#include <rte_malloc.h>
#include <rte_memcpy.h>
#include <rte_bus_vdev.h>
#include <rte_kvargs.h>
#include <rte_spinlock.h>
#include "xlnx_logs.h"

#define XLNX_MAX_QUEUE_PER_PORT	1

#define ETH_NULL_PACKET_SIZE_ARG	"size"
#define ETH_NULL_PACKET_COPY_ARG	"copy"

static unsigned default_packet_size = 64;
static unsigned default_packet_copy;

static const char *valid_arguments[] = {
	ETH_NULL_PACKET_SIZE_ARG,
	ETH_NULL_PACKET_COPY_ARG,
	NULL
};

struct pmd_internals;

struct xlnx_queue {
	struct pmd_internals *internals;

	struct rte_mempool *mb_pool;
	struct rte_mbuf *dummy_packet;

	rte_atomic64_t rx_pkts;
	rte_atomic64_t tx_pkts;
	rte_atomic64_t err_pkts;
};

struct pmd_internals {
	unsigned packet_size;
	unsigned packet_copy;
	uint16_t port_id;

	struct xlnx_queue rx_xlnx_queues[XLNX_MAX_QUEUE_PER_PORT];
	struct xlnx_queue tx_xlnx_queues[XLNX_MAX_QUEUE_PER_PORT];

	rte_spinlock_t reg_lock;
};


static struct ether_addr eth_addr = { .addr_bytes = {0} };
static struct rte_eth_link pmd_link = {
	.link_speed = ETH_SPEED_NUM_10G,
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


static uint16_t
eth_xlnx_rx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	int i;
	struct xlnx_queue *h = q;
	unsigned packet_size;

	xlnx_log_dbg("%s\n", __func__);

	if ((q == NULL) || (bufs == NULL))
		return 0;

	packet_size = h->internals->packet_size;
	for (i = 0; i < nb_bufs; i++) {
		bufs[i] = rte_pktmbuf_alloc(h->mb_pool);
		if (!bufs[i])
			break;
		/* TODO:
		 * put bufs to DMA HW to receive data
		 */
		bufs[i]->data_len = (uint16_t)packet_size;
		bufs[i]->pkt_len = packet_size;
		bufs[i]->port = h->internals->port_id;
	}

	rte_atomic64_add(&(h->rx_pkts), i);

	return i;
}

static uint16_t
eth_xlnx_copy_rx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	int i;
	struct xlnx_queue *h = q;
	unsigned packet_size;

	xlnx_log_dbg("%s\n", __func__);

	if ((q == NULL) || (bufs == NULL))
		return 0;

	packet_size = h->internals->packet_size;
	for (i = 0; i < nb_bufs; i++) {
		bufs[i] = rte_pktmbuf_alloc(h->mb_pool);
		if (!bufs[i])
			break;
		rte_memcpy(rte_pktmbuf_mtod(bufs[i], void *), h->dummy_packet,
					packet_size);
		bufs[i]->data_len = (uint16_t)packet_size;
		bufs[i]->pkt_len = packet_size;
		bufs[i]->port = h->internals->port_id;
	}

	rte_atomic64_add(&(h->rx_pkts), i);

	return i;
}

static uint16_t
eth_xlnx_tx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	int i;
	struct xlnx_queue *h = q;

	xlnx_log_dbg("%s\n", __func__);

	if ((q == NULL) || (bufs == NULL))
		return 0;

	/* TODO:
	 * put bufs to DMA HW
	 */

	for (i = 0; i < nb_bufs; i++)
		rte_pktmbuf_free(bufs[i]);

	rte_atomic64_add(&(h->tx_pkts), i);

	return i;
}

static uint16_t
eth_xlnx_copy_tx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	int i;
	struct xlnx_queue *h = q;
	unsigned packet_size;

	xlnx_log_dbg("%s\n", __func__);

	if ((q == NULL) || (bufs == NULL))
		return 0;

	packet_size = h->internals->packet_size;
	for (i = 0; i < nb_bufs; i++) {
		rte_memcpy(h->dummy_packet, rte_pktmbuf_mtod(bufs[i], void *),
					packet_size);
		rte_pktmbuf_free(bufs[i]);
	}

	rte_atomic64_add(&(h->tx_pkts), i);

	return i;
}

static int
eth_dev_configure(struct rte_eth_dev *dev __rte_unused)
{
	xlnx_log_info();

	return 0;
}

static int
eth_dev_start(struct rte_eth_dev *dev)
{
	xlnx_log_info();

	if (dev == NULL)
		return -EINVAL;

	dev->data->dev_link.link_status = ETH_LINK_UP;
	return 0;
}

static void
eth_dev_stop(struct rte_eth_dev *dev)
{
	xlnx_log_info();

	if (dev == NULL)
		return;

	dev->data->dev_link.link_status = ETH_LINK_DOWN;
}

static int
eth_rx_queue_setup(struct rte_eth_dev *dev, uint16_t rx_queue_id,
		uint16_t nb_rx_desc __rte_unused,
		unsigned int socket_id __rte_unused,
		const struct rte_eth_rxconf *rx_conf __rte_unused,
		struct rte_mempool *mb_pool)
{
	struct rte_mbuf *dummy_packet;
	struct pmd_internals *internals;
	unsigned packet_size;

	xlnx_log_info();

	if ((dev == NULL) || (mb_pool == NULL))
		return -EINVAL;

	internals = dev->data->dev_private;

	if (rx_queue_id >= dev->data->nb_rx_queues)
		return -ENODEV;

	packet_size = internals->packet_size;

	internals->rx_xlnx_queues[rx_queue_id].mb_pool = mb_pool;
	dev->data->rx_queues[rx_queue_id] =
		&internals->rx_xlnx_queues[rx_queue_id];
	dummy_packet = rte_zmalloc_socket(NULL,
			packet_size, 0, dev->data->numa_node);
	if (dummy_packet == NULL)
		return -ENOMEM;

	internals->rx_xlnx_queues[rx_queue_id].internals = internals;
	internals->rx_xlnx_queues[rx_queue_id].dummy_packet = dummy_packet;

	return 0;
}

static int
eth_tx_queue_setup(struct rte_eth_dev *dev, uint16_t tx_queue_id,
		uint16_t nb_tx_desc __rte_unused,
		unsigned int socket_id __rte_unused,
		const struct rte_eth_txconf *tx_conf __rte_unused)
{
	struct rte_mbuf *dummy_packet;
	struct pmd_internals *internals;
	unsigned packet_size;

	xlnx_log_info();

	if (dev == NULL)
		return -EINVAL;

	internals = dev->data->dev_private;

	if (tx_queue_id >= dev->data->nb_tx_queues)
		return -ENODEV;

	packet_size = internals->packet_size;

	dev->data->tx_queues[tx_queue_id] =
		&internals->tx_xlnx_queues[tx_queue_id];
	dummy_packet = rte_zmalloc_socket(NULL,
			packet_size, 0, dev->data->numa_node);
	if (dummy_packet == NULL)
		return -ENOMEM;

	internals->tx_xlnx_queues[tx_queue_id].internals = internals;
	internals->tx_xlnx_queues[tx_queue_id].dummy_packet = dummy_packet;

	return 0;
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
	struct pmd_internals *internals;

	xlnx_log_info();

	if ((dev == NULL) || (dev_info == NULL))
		return;

	internals = dev->data->dev_private;
	dev_info->max_mac_addrs = 1;
	dev_info->max_rx_pktlen = (uint32_t)-1;
	dev_info->max_rx_queues = RTE_DIM(internals->rx_xlnx_queues);
	dev_info->max_tx_queues = RTE_DIM(internals->tx_xlnx_queues);
	dev_info->min_rx_bufsize = 0;
}

static int
eth_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *igb_stats)
{
	unsigned i, num_stats;
	unsigned long rx_total = 0, tx_total = 0, tx_err_total = 0;
	const struct pmd_internals *internal;

	xlnx_log_info();

	if ((dev == NULL) || (igb_stats == NULL))
		return -EINVAL;

	internal = dev->data->dev_private;
	num_stats = RTE_MIN((unsigned)RTE_ETHDEV_QUEUE_STAT_CNTRS,
			RTE_MIN(dev->data->nb_rx_queues,
				RTE_DIM(internal->rx_xlnx_queues)));
	for (i = 0; i < num_stats; i++) {
		igb_stats->q_ipackets[i] =
			internal->rx_xlnx_queues[i].rx_pkts.cnt;
		rx_total += igb_stats->q_ipackets[i];
	}

	num_stats = RTE_MIN((unsigned)RTE_ETHDEV_QUEUE_STAT_CNTRS,
			RTE_MIN(dev->data->nb_tx_queues,
				RTE_DIM(internal->tx_xlnx_queues)));
	for (i = 0; i < num_stats; i++) {
		igb_stats->q_opackets[i] =
			internal->tx_xlnx_queues[i].tx_pkts.cnt;
		igb_stats->q_errors[i] =
			internal->tx_xlnx_queues[i].err_pkts.cnt;
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
	struct pmd_internals *internal;

	xlnx_log_info();

	if (dev == NULL)
		return;

	internal = dev->data->dev_private;
	for (i = 0; i < RTE_DIM(internal->rx_xlnx_queues); i++)
		internal->rx_xlnx_queues[i].rx_pkts.cnt = 0;
	for (i = 0; i < RTE_DIM(internal->tx_xlnx_queues); i++) {
		internal->tx_xlnx_queues[i].tx_pkts.cnt = 0;
		internal->tx_xlnx_queues[i].err_pkts.cnt = 0;
	}
}

static void
eth_queue_release(void *q)
{
	struct xlnx_queue *nq;

	xlnx_log_info();

	if (q == NULL)
		return;

	nq = q;
	rte_free(nq->dummy_packet);
}

static int
eth_link_update(struct rte_eth_dev *dev __rte_unused,
		int wait_to_complete __rte_unused)
{
	xlnx_log_info();

	return 0;
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
	.dev_configure = eth_dev_configure,
	.dev_infos_get = eth_dev_info,
	.rx_queue_setup = eth_rx_queue_setup,
	.tx_queue_setup = eth_tx_queue_setup,
	.rx_queue_release = eth_queue_release,
	.tx_queue_release = eth_queue_release,
	.mtu_set = eth_mtu_set,
	.link_update = eth_link_update,
	.mac_addr_set = eth_mac_address_set,
	.stats_get = eth_stats_get,
	.stats_reset = eth_stats_reset,
};

static struct rte_vdev_driver pmd_xlnx_drv;

static int
eth_dev_xlnx_create(struct rte_vdev_device *dev,
		unsigned packet_size,
		unsigned packet_copy)
{
	const unsigned nb_rx_queues = 1;
	const unsigned nb_tx_queues = 1;
	struct rte_eth_dev_data *data = NULL;
	struct pmd_internals *internals = NULL;
	struct rte_eth_dev *eth_dev = NULL;

	if (dev->device.numa_node == SOCKET_ID_ANY)
		dev->device.numa_node = rte_socket_id();

	RTE_LOG(INFO, PMD, "Creating xlnx ethdev on numa socket %u\n",
		dev->device.numa_node);

	/* now do all data allocation - for eth_dev structure, dummy pci driver
	 * and internal (private) data
	 */
	data = rte_zmalloc_socket(rte_vdev_device_name(dev), sizeof(*data), 0,
		dev->device.numa_node);
	if (!data)
		return -ENOMEM;

	eth_dev = rte_eth_vdev_allocate(dev, sizeof(*internals));
	if (!eth_dev) {
		rte_free(data);
		return -ENOMEM;
	}

	/* now put it all together
	 * - store queue data in internals,
	 * - store numa_node info in ethdev data
	 * - point eth_dev_data to internals
	 * - and point eth_dev structure to new eth_dev_data structure
	 */
	/* NOTE: we'll replace the data element, of originally allocated eth_dev
	 * so the xlnxs are local per-process */

	internals = eth_dev->data->dev_private;
	internals->packet_size = packet_size;
	internals->packet_copy = packet_copy;
	internals->port_id = eth_dev->data->port_id;

	rte_memcpy(data, eth_dev->data, sizeof(*data));
	data->nb_rx_queues = (uint16_t)nb_rx_queues;
	data->nb_tx_queues = (uint16_t)nb_tx_queues;
	data->dev_link = pmd_link;
	data->mac_addrs = &eth_addr;

	eth_dev->data = data;
	eth_dev->dev_ops = &ops;

	/* finally assign rx and tx ops */
	if (packet_copy) {
		eth_dev->rx_pkt_burst = eth_xlnx_copy_rx;
		eth_dev->tx_pkt_burst = eth_xlnx_copy_tx;
	} else {
		eth_dev->rx_pkt_burst = eth_xlnx_rx;
		eth_dev->tx_pkt_burst = eth_xlnx_tx;
	}

	return 0;
}

static inline int
get_packet_size_arg(const char *key __rte_unused,
		const char *value, void *extra_args)
{
	const char *a = value;
	unsigned *packet_size = extra_args;

	if ((value == NULL) || (extra_args == NULL))
		return -EINVAL;

	*packet_size = (unsigned)strtoul(a, NULL, 0);
	if (*packet_size == UINT_MAX)
		return -1;

	return 0;
}

static inline int
get_packet_copy_arg(const char *key __rte_unused,
		const char *value, void *extra_args)
{
	const char *a = value;
	unsigned *packet_copy = extra_args;

	if ((value == NULL) || (extra_args == NULL))
		return -EINVAL;

	*packet_copy = (unsigned)strtoul(a, NULL, 0);
	if (*packet_copy == UINT_MAX)
		return -1;

	return 0;
}

static int
rte_pmd_xlnx_probe(struct rte_vdev_device *dev)
{
	const char *name, *params;
	unsigned packet_size = default_packet_size;
	unsigned packet_copy = default_packet_copy;
	struct rte_kvargs *kvlist = NULL;
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

		if (rte_kvargs_count(kvlist, ETH_NULL_PACKET_SIZE_ARG) == 1) {

			ret = rte_kvargs_process(kvlist,
					ETH_NULL_PACKET_SIZE_ARG,
					&get_packet_size_arg, &packet_size);
			if (ret < 0)
				goto free_kvlist;
		}

		if (rte_kvargs_count(kvlist, ETH_NULL_PACKET_COPY_ARG) == 1) {

			ret = rte_kvargs_process(kvlist,
					ETH_NULL_PACKET_COPY_ARG,
					&get_packet_copy_arg, &packet_copy);
			if (ret < 0)
				goto free_kvlist;
		}
	}

	RTE_LOG(INFO, PMD, "Configure pmd_xlnx: packet size is %d, "
			"packet copy is %s\n", packet_size,
			packet_copy ? "enabled" : "disabled");

	ret = eth_dev_xlnx_create(dev, packet_size, packet_copy);

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
	"size=<int> "
	"copy=<int>");
