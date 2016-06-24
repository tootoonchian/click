// -*- mode: c++; c-basic-offset: 4 -*-
/*
 * dpdkinfo.{cc,hh} -- element for interacting with dpdk
 * Maziar Manesh, Amin Tootoonchian
 *
 * Copyright (c) 2011-2014 Maziar Manesh
 * Copyright (c) 2014-2015 Amin Tootoonchian
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, subject to the conditions
 * listed in the Click LICENSE file. These conditions include: you must
 * preserve this copyright notice, and you cannot mention the copyright
 * holders in advertising related to the Software without their permission.
 * The Software is provided WITHOUT ANY WARRANTY, EXPRESS OR IMPLIED. This
 * notice is a summary of the Click LICENSE file; the license in that file is
 * legally binding.
 */

#include <click/config.h>
#include <click/args.hh>

#if HAVE_DPDK
#include "elements/userlevel/dpdkinfo.hh"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <unistd.h>

#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_errno.h>
#include <rte_malloc.h>
#include <rte_ring.h>

CLICK_DECLS

namespace {
int init_port(uint8_t port_id) {
    // FIXME: check if port is started
    if (rte_eal_process_type() != RTE_PROC_PRIMARY)
        return 0;

    static bool initialized[DPDKInfo::MAX_PORTS] = {0};
    if (initialized[port_id])
        return 0;

    int i, retval;
    uint16_t queue_id, pool_size;
    char pool_name[RTE_MEMPOOL_NAMESIZE];

    struct rte_eth_dev &dev = rte_eth_devices[port_id];
    struct rte_eth_dev_info dev_info;
    struct rte_eth_conf port_conf;
    struct rte_eth_rxconf *rxconf;
    struct rte_eth_txconf *txconf;
    struct rte_eth_fc_conf fc_conf;

    if (!dev.attached)
        return 0;

    /* prepare configs */
    memset(&port_conf, 0, sizeof(port_conf));
    rte_eth_dev_info_get(port_id, &dev_info);
    rxconf = &dev_info.default_rxconf;
    txconf = &dev_info.default_txconf;

    port_conf.rxmode.mq_mode                    = ETH_MQ_RX_NONE;
    port_conf.rxmode.max_rx_pkt_len             = ETHER_MAX_LEN;
    port_conf.rxmode.split_hdr_size             = DPDKInfo::SPLIT_HDR_SIZE;
    port_conf.rxmode.header_split               = DPDKInfo::HEADER_SPLIT;
    port_conf.rxmode.hw_ip_checksum             = DPDKInfo::HW_IP_CHECKSUM;
    port_conf.rxmode.hw_vlan_filter             = DPDKInfo::HW_VLAN_FILTER;
    port_conf.rxmode.jumbo_frame                = DPDKInfo::JUMBO_FRAME;
    port_conf.rxmode.hw_strip_crc               = DPDKInfo::HW_STRIP_CRC;
    port_conf.rx_adv_conf.rss_conf.rss_key      = NULL;
    port_conf.rx_adv_conf.rss_conf.rss_hf       = ETH_RSS_IP;
    port_conf.txmode.mq_mode                    = ETH_MQ_TX_NONE;
    port_conf.intr_conf.lsc                     = 0;

    rxconf->rx_drop_en                          = 1;
    //txconf->txq_flags                           = ETH_TXQ_FLAGS_NOMULTSEGS | ETH_TXQ_FLAGS_NOOFFLOADS;

    rte_eth_dev_stop(port_id);

    /* init port */
    retval = rte_eth_dev_configure(port_id,
            DPDKInfo::NB_RX_QUEUE,
            DPDKInfo::NB_TX_QUEUE,
            &port_conf);
    if (retval < 0) {
        rte_exit(EXIT_FAILURE,
                 "Cannot configure device: err=%d, port=%u\n",
                 retval, port_id);
    }

    retval = rte_eth_dev_flow_ctrl_get(port_id, &fc_conf);
    if (retval != 0 && retval != -ENOTSUP) {
        rte_exit(EXIT_FAILURE, "rte_eth_dev_flow_ctrl_get: "
                "err=%d, port=%d, %s", retval, port_id, rte_strerror(-retval));
    }
    if (retval == 0) {
        fc_conf.autoneg                             = 0;
        fc_conf.mode                                = RTE_FC_NONE;
        //fc_conf.pause_time                          = 1337;
        //fc_conf.send_xon                            = 1;

        retval = rte_eth_dev_flow_ctrl_set(port_id, &fc_conf);
        if (retval < 0 && retval != -ENOTSUP)
            rte_exit(EXIT_FAILURE, "rte_eth_dev_flow_ctrl_set: "
                    "err=%d, port=%d, %s", retval, port_id, rte_strerror(-retval));
    }

    /* init TX queues */
    for (queue_id = 0; queue_id < DPDKInfo::NB_TX_QUEUE; queue_id++) {
        retval = rte_eth_tx_queue_setup(port_id,
                queue_id,
                DPDKInfo::NB_TX_DESC,
                rte_eth_dev_socket_id(port_id),
                txconf);
        if (retval < 0)
            rte_exit(EXIT_FAILURE, "rte_eth_tx_queue_setup: "
                     "err=%d, port=%u queue=%u\n",
                     retval, port_id, queue_id);
    }

    sprintf(pool_name, "click_%s", dev.data->name);

    pool_size = DPDKInfo::POOL_SIZE;
    if (strncmp(dev_info.driver_name, "rte_bond_pmd", 12) == 0) {
        pool_size *= 4;
    }
    pool_size -= 1;

    struct rte_mempool *mempool =
        rte_pktmbuf_pool_create(pool_name,
                pool_size,
                DPDKInfo::POOL_CACHE_SIZE,
                0,
                DPDKInfo::MBUF_SIZE,
                rte_eth_dev_socket_id(port_id));

    if (mempool == NULL)
        rte_exit(EXIT_FAILURE, "Cannot init mbuf pool\n");


    /* init RX queues */
    for (queue_id = 0; queue_id < DPDKInfo::NB_RX_QUEUE; queue_id++) {
        retval = rte_eth_rx_queue_setup(port_id,
                queue_id,
                DPDKInfo::NB_RX_DESC,
                rte_eth_dev_socket_id(port_id),
                rxconf,
                mempool);
        if (retval < 0)
            rte_exit(EXIT_FAILURE, "rte_eth_rx_queue_setup: "
                     "err=%d, port=%u, queue=%u\n",
                     retval, port_id, queue_id);
    }

    /* start the device */
    retval = rte_eth_dev_start(port_id);
    if (retval < 0)
        rte_exit(EXIT_FAILURE, "rte_eth_dev_start: "
                 "err=%d, port=%u\n", retval, port_id);

    rte_eth_promiscuous_enable(port_id);

    initialized[port_id] = true;
    return 0;
}
}

int DPDKInfo::port_id(String &port_name_click, uint8_t *port_id)
{
    int i;
    bool is_virtual = false;
    char tmp[256];
    struct rte_pci_addr pci_addr;
    String port_name_dpdk = _name_map.find(port_name_click, "");

    if (eal_parse_pci_BDF(port_name_dpdk.c_str(), &pci_addr) != 0 &&
            eal_parse_pci_DomBDF(port_name_dpdk.c_str(), &pci_addr) != 0)
        is_virtual = true;

    if (!is_virtual) {
        sprintf(tmp, "%d:%d.%d", pci_addr.bus, pci_addr.devid, pci_addr.function);
        port_name_dpdk = tmp;
    } else {
        port_name_dpdk =
            port_name_dpdk.substring(0, port_name_dpdk.find_left(','));
    }

    struct rte_eth_dev *dev = rte_eth_dev_allocated(port_name_dpdk.c_str());
    if (dev) {
        *port_id = dev->data->port_id;
        return init_port(*port_id);
    }

    return -ENODEV;
}

int DPDKInfo::configure(Vector<String> &conf, ErrorHandler *errh)
{
    /* temporary variables */
    int i, retval;
    uint8_t port_id, nr_ports;
    char tmp1[256], tmp2[256];
    std::stringstream ss_devs;
    std::string dev;

    /* init EAL */
    Vector<String> params;
    Vector<String> vdevs;

    uint16_t _nr_channels;
    String port_name_click, port_name_dpdk;
    String _socket_mem;
    String _devs;
    String _huge_dir;
    String _prefix;
    String _cores;

    if (Args(conf, this, errh)
            .read_or_set("PREFIX", _prefix, "click")
            .read_or_set("HUGE_DIR", _huge_dir, "/dev/hugepages")
            .read_or_set("NR_CHANNELS", _nr_channels, 4)
            .read_or_set("SOCKET_MEM", _socket_mem, "100")
            .read_m("CORES", _cores)
            .read_m("DEVS", _devs)
            .complete() < 0)
        return -1;

    params.push_back("click");

    ss_devs.str(_devs.c_str());
    while (ss_devs >> dev) {
        retval = sscanf(dev.c_str(), "%256[^(](%256[^)])", tmp1, tmp2);
        port_name_click = cp_unquote(tmp1);
        port_name_dpdk = tmp2;
        _name_map.insert(port_name_click, port_name_dpdk);
        if (port_name_dpdk.starts_with("eth_")) {
            vdevs.push_back(port_name_dpdk);
        }
    }

    params.push_back("--huge-dir");
    params.push_back(_huge_dir.c_str());

    params.push_back("--file-prefix");
    params.push_back(_prefix.c_str());

    params.push_back("--proc-type");
    params.push_back("auto");

    params.push_back("--log-level");
    params.push_back("8");

    params.push_back("-l");
    params.push_back(_cores);

    params.push_back("-n");
    params.push_back(String(_nr_channels));

    params.push_back("--socket-mem");
    params.push_back(_socket_mem);

    char** argv = new char*[params.size()];

    for (i = 0; i < params.size(); i++) {
        argv[i] = const_cast<char*>(params[i].c_str());
    }

    retval = rte_eal_init(params.size(), argv);
    free(argv);

    // attach to vdevs after rte_eal_init() -> pdev port ids always come first'
    for (i = 0; i < vdevs.size(); i++) {
        retval = rte_eth_dev_attach(vdevs[i].c_str(), &port_id);
        if (retval != 0) {
             rte_exit(EXIT_FAILURE, "Failed to initialize vdev %s", vdevs[i]);
        }
    }

    if (retval < 0)
        rte_exit(EXIT_FAILURE, "Invalid EAL arguments\n");

    nr_ports = rte_eth_dev_count();
    if (nr_ports == 0)
        rte_exit(EXIT_FAILURE, "No Ethernet ports\n");
    if (nr_ports > MAX_PORTS)
        rte_exit(EXIT_FAILURE, "More Ethernet ports than supported\n");

    return 0;
}

CLICK_ENDDECLS

ELEMENT_LIBS((-Wl,--whole-archive,-ldpdk,--no-whole-archive))
#endif
ELEMENT_REQUIRES(userlevel)
EXPORT_ELEMENT(DPDKInfo)
