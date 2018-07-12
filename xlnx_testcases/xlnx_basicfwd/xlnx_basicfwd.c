/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2010-2015 Intel Corporation
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/queue.h>
#include <netinet/in.h>
#include <setjmp.h>
#include <stdarg.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>

#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_cycles.h>
#include <rte_lcore.h>
#include <rte_mbuf.h>
#include <unistd.h>
#include <ctype.h>

#define RX_RING_SIZE 1024
#define TX_RING_SIZE 1024
#define NUM_MBUFS 8191
#define MBUF_CACHE_SIZE 250
#define MBUF_DATA_SIZE 64
#define MAX_NUM_PKTS 100

/* Number of Packets : Parameters provided by user */
int num_pkts;
/* Test mode : Parameter provided by user */
char mode;

/* message buffer for transmission and recieving */
struct message {
	char data[MBUF_DATA_SIZE];
};

static const char short_options[] =
        "p:"  /* number of packets */
	"m:" /* test mode */
        ;

static const struct rte_eth_conf port_conf_default = {
	.rxmode = {
		.max_rx_pkt_len = ETHER_MAX_LEN,
		.ignore_offload_bitfield = 1,
	},
};

/*  Basic DPDK test  */


/* Function to display the statistics - Number of packets recieved and
 *  transmitted
*/
static void
stats_display(uint16_t port_id)
{
        struct rte_eth_stats stats;
        rte_eth_stats_get(port_id, &stats);

        printf("  RX-packets: %-10"PRIu64" RX-missed: %-10"PRIu64" RX-bytes:  "
               "%-"PRIu64"\n",
               stats.ipackets, stats.imissed, stats.ibytes);
        printf("  RX-errors: %-10"PRIu64" RX-nombuf:  %-10"PRIu64"\n",
               stats.ierrors, stats.rx_nombuf);
        printf("  TX-packets: %-10"PRIu64" TX-errors: %-10"PRIu64" TX-bytes:  "
               "%-"PRIu64"\n",
               stats.opackets, stats.oerrors, stats.obytes);
}

/*
 * Initializes a given port using global settings and with the RX buffers
 * coming from the mbuf_pool passed as a parameter.
 */

static inline int
port_init(uint16_t port, struct rte_mempool *mbuf_pool)
{
        struct rte_eth_conf port_conf = port_conf_default;
        const uint16_t rx_rings = 1, tx_rings = 1;
        uint16_t nb_rxd = RX_RING_SIZE;
        uint16_t nb_txd = TX_RING_SIZE;
        int retval;
        uint16_t q;
        struct rte_eth_dev_info dev_info;
        struct rte_eth_txconf txconf;

        if (port >= rte_eth_dev_count())
                return -1;

        rte_eth_dev_info_get(port, &dev_info);
        if (dev_info.tx_offload_capa & DEV_TX_OFFLOAD_MBUF_FAST_FREE)
                port_conf.txmode.offloads |=
                DEV_TX_OFFLOAD_MBUF_FAST_FREE;

/* Configure the Ethernet device. */
        retval = rte_eth_dev_configure(port, rx_rings, tx_rings, &port_conf);
        if (retval != 0)
                return retval;

        retval = rte_eth_dev_adjust_nb_rx_tx_desc(port, &nb_rxd, &nb_txd);
        if (retval != 0)
                return retval;

/* Allocate and set up 1 RX queue per Ethernet port. */
        for (q = 0; q < rx_rings; q++) {
                retval = rte_eth_rx_queue_setup(port, q, nb_rxd,
                rte_eth_dev_socket_id(port), NULL, mbuf_pool);
        if (retval < 0)
                return retval;
	}

        txconf = dev_info.default_txconf;
        txconf.txq_flags = ETH_TXQ_FLAGS_IGNORE;
        txconf.offloads = port_conf.txmode.offloads;
/* Allocate and set up 1 TX queue per Ethernet port. */
        for (q = 0; q < tx_rings; q++) {
                retval = rte_eth_tx_queue_setup(port, q, nb_txd,
                rte_eth_dev_socket_id(port), &txconf);
        	if (retval < 0)
                	return retval;
        }

/* Start the Ethernet port. */
        retval = rte_eth_dev_start(port);
        if (retval < 0)
        return retval;

/* Display the port MAC address. */
        struct ether_addr addr;
        rte_eth_macaddr_get(port, &addr);
        printf("Port %u MAC: %02" PRIx8 " %02" PRIx8 " %02" PRIx8
                   " %02" PRIx8 " %02" PRIx8 " %02" PRIx8 "\n",
                port,
                addr.addr_bytes[0], addr.addr_bytes[1],
                addr.addr_bytes[2], addr.addr_bytes[3],
                addr.addr_bytes[4], addr.addr_bytes[5]);

/* Enable RX in promiscuous mode for the Ethernet device. */
        rte_eth_promiscuous_enable(port);

        return 0;
}

/*
 * The loopback main.
 * This is the main thread that does the work, reading from
 * an input port and writing to an output port.
 */
static __attribute__(())
 int  loopback_main(struct rte_mempool *mbuf_pool)
{
        struct rte_mbuf *txpkt;
        struct rte_mbuf *rxpkt;
        uint32_t rx_len;
        struct message obj;
        struct message *msg;
        struct message *temp_msg;
        int nb_rx = 0, nb_tx = 0,  pkt_size = 0;
        int count = 0;
        int nb_pkt;
        char k = 1;

        for (count = 0; count < MBUF_DATA_SIZE; count++){
                obj.data[count] = k++;
        }

        msg = &obj;

        txpkt = rte_pktmbuf_alloc(mbuf_pool);
        rxpkt = rte_pktmbuf_alloc(mbuf_pool);

        pkt_size = sizeof(struct message);
        txpkt->data_len = pkt_size;
        txpkt->pkt_len = pkt_size;

        struct message * data;

        data = rte_pktmbuf_mtod(txpkt, struct message *);
        if (data != NULL)
                rte_memcpy(data, msg, sizeof(struct message));
        for (nb_pkt=1; nb_pkt <= num_pkts; nb_pkt++){
                nb_tx = rte_eth_tx_burst(0 , 0, &txpkt, nb_pkt);
                nb_rx = rte_eth_rx_burst(0 , 0, &rxpkt, nb_pkt);
                rte_pktmbuf_dump(stdout, rxpkt, 1024);
                rx_len  = rte_pktmbuf_data_len(rxpkt);
                printf ("recieve data length = %d \n",rx_len);
                temp_msg = rte_pktmbuf_mtod(rxpkt, struct message *);
                printf ("=====recieved data========= \n");
                for (count = 0; count < MBUF_DATA_SIZE; count++){
                       printf (" %02" PRIx8 ,temp_msg->data[count]);
   	        }
        }
        rte_pktmbuf_free(txpkt);
        rte_pktmbuf_free(rxpkt);

        printf("----\nData size: %d\nPacket size: %d\nRX : %d, TX : %d\n\n", MBUF_DATA_SIZE, pkt_size, nb_rx, nb_tx);
        stats_display(0);
        return 0;

} /* End of loopback_main function */

/* txonly main function */
static __attribute__(())
 int txonly_main(struct rte_mempool *mbuf_pool) {
        struct rte_mbuf *txpkt;
        struct message obj;
        struct message *msg;
        int  nb_tx = 0,  pkt_size = 0;
        int count = 0;
        int nb_pkt;
        char k = 1;
        for (count = 0; count < MBUF_DATA_SIZE; count++){
                obj.data[count] = k++;
        }
        msg = &obj;
        txpkt = rte_pktmbuf_alloc(mbuf_pool);
        pkt_size = sizeof(struct message);
        txpkt->data_len = pkt_size;
        txpkt->pkt_len = pkt_size;
        struct message * data;
        data = rte_pktmbuf_mtod(txpkt, struct message *);
        if (data != NULL)
    	        rte_memcpy(data, msg, sizeof(struct message));
        for (nb_pkt=1; nb_pkt <= num_pkts; nb_pkt++) {
                nb_tx = rte_eth_tx_burst(0 , 0, &txpkt, nb_pkt);
                rte_pktmbuf_dump(stdout, txpkt, 1024);
        }
        rte_pktmbuf_free(txpkt);
        printf("----\nData size: %d\nPacket size: %d\n, TX : %d\n\n", MBUF_DATA_SIZE, pkt_size,  nb_tx);
        stats_display(0);
        return 0;

} /*end of txonly function */

static __attribute__(())
 int  rxonly_main(struct rte_mempool *mbuf_pool) {
        struct rte_mbuf *rxpkt;
        uint32_t rx_len;
        struct message *temp_msg;
        int nb_rx = 0,count;
        int nb_pkt;
        rxpkt = rte_pktmbuf_alloc(mbuf_pool);
        for (nb_pkt=1; nb_pkt <= num_pkts; nb_pkt++) {
                nb_rx = rte_eth_rx_burst(0, 0, &rxpkt, nb_pkt);
                rte_pktmbuf_dump(stdout, rxpkt, 1024);
                rx_len  = rte_pktmbuf_data_len(rxpkt);
                printf ("recieve data length = %d \n",rx_len);
                temp_msg = rte_pktmbuf_mtod(rxpkt, struct message *);
                printf ("recieved data \n");
                for (count = 0; count < MBUF_DATA_SIZE; count++){
                        printf (" %02" PRIx8 ,temp_msg->data[count]);
            }
        }
        rte_pktmbuf_free(rxpkt);

        printf("----\nData size: %d\nnRX : %d, \n\n", MBUF_DATA_SIZE, nb_rx );
        stats_display(0);
        return 0;

} /* End of rxonly function */

/* Parser function */
static int
xlnx_parse_numpkts(const char *q_arg)
{
        char *end = NULL;
        int n;

        /* parse number string */
        n = strtol(q_arg, &end, 10);
        if ((q_arg[0] == '\0') || (end == NULL) || (*end != '\0'))
                return -1;
        if (n >= MAX_NUM_PKTS)
                return -1;

        return n;
}
static int
xlnx_parse_mode(const char *q_arg)
{
        char *end = NULL;
        int m;

        /* parse number string */
        m = strtol(q_arg, &end, 10);
        if ((q_arg[0] == '\0') || (end == NULL) || (*end != '\0'))
                return -1;
        return m;
}

/* Parse the argument given in the command line of the application */
static int
xlnx_parse_args(int argc, char **argv)
{
        int opt;
        char **argvopt;
        int option_index;

        argvopt = argv;
        while ((opt = getopt_long(argc, argvopt, short_options,
                                0 , &option_index)) != EOF) {
            switch (opt) {
        /* Num of packets */
                case 'p':
                    num_pkts=xlnx_parse_numpkts(optarg);
                    printf ("num_pkts = %d \n", num_pkts);
                    if (num_pkts == 0) {
                            return -1;
                    }
                break;
               /* Mode */
                case 'm':
                        mode=xlnx_parse_mode(optarg);
                        printf ("mode = %d \n", mode);
                        if (mode > 3) {
                                printf ("invalid mode \n");
                                return -1;
                         }

                  break;


                default:
                        num_pkts = 1;
                        mode = 0;
                        return 0;
                }

        }

        return 0;
}

/*
 * The main function, which does initialization and calls the per-lcore
 * functions.
 */
int
main(int argc, char *argv[])
{
        struct rte_mempool *mbuf_pool;
        unsigned nb_ports;
        uint16_t portid;

        /* Initialize the Environment Abstraction Layer (EAL). */
        int ret = rte_eal_init(argc, argv);
        if (ret < 0)
                rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");

        argc -= ret;
        argv += ret;
       /* parse application arguments (after the EAL ones) */
        ret = xlnx_parse_args(argc, argv);
        if (ret < 0)
                rte_exit(EXIT_FAILURE, "Invalid xlnx arguments\n");


	/* Check that there is an even number of ports to send/receive on. */
        nb_ports = rte_eth_dev_count();
        /* Creates a new mempool in memory to hold the mbufs. */
        mbuf_pool = rte_pktmbuf_pool_create("MBUF_POOL", NUM_MBUFS * nb_ports,
                MBUF_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id());

        if (mbuf_pool == NULL)
                rte_exit(EXIT_FAILURE, "Cannot create mbuf pool\n");

        /* Initialize all ports. */
        for (portid = 0; portid < nb_ports; portid++)
                if (port_init(portid, mbuf_pool) != 0)
                        rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu16 "\n",
                                portid);

        if (rte_lcore_count() > 1)
                printf("\nWARNING: Too many lcores enabled. Only 1 used.\n");

        /* Call lcore_main on the master core only. */
        if (mode == 0){
                printf ("loopback mode test \n");
                loopback_main(mbuf_pool);
        }
        if (mode == 1){
                printf ("TxOnly \n");
                txonly_main(mbuf_pool);
        }
        if (mode == 2) {
                printf ("RxOnly \n");
                rxonly_main(mbuf_pool);
        }


        return 0;
}
