#ifndef CLICK_DPDKINFO_HH
#define CLICK_DPDKINFO_HH 1

#if HAVE_DPDK
#include <click/element.hh>
#include <click/hashmap.hh>
#include <click/packet.hh>
#include <click/error.hh>

#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdint.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/queue.h>
#include <stdarg.h>

#include <vector>

#include <rte_config.h>
#include <rte_common.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_pci.h>

CLICK_DECLS

class DPDKInfo : public Element {
public:
    enum {
        MAX_PORTS               = RTE_MAX_ETHPORTS,
        MBUF_SIZE               = RTE_MBUF_DEFAULT_BUF_SIZE,
        POOL_CACHE_SIZE         = 128,
        POOL_SIZE               = 384,
        NB_RX_QUEUE             = 1,
        NB_TX_QUEUE             = 1,
        NB_RX_DESC              = 128,
        NB_TX_DESC              = 128,
        HW_STRIP_CRC            = 1,
        HW_IP_CHECKSUM          = 0,
        SPLIT_HDR_SIZE          = 0,
        HEADER_SPLIT            = 0,
        HW_VLAN_FILTER          = 0,
        JUMBO_FRAME             = 0,
    };

    DPDKInfo() CLICK_COLD               {}
    ~DPDKInfo() CLICK_COLD              {}

    const char* class_name() const      { return "DPDKInfo"; }
    int configure_phase() const         { return CONFIGURE_PHASE_INFO; }
    int configure(Vector<String>&, ErrorHandler*);
    int port_id(String &, uint8_t *port_id);

private:
    HashMap<String, String> _name_map;
};
CLICK_ENDDECLS
#endif // HAVE_DPDK

#endif
