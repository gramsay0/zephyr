/*
 * Copyright (c) 2023 Enphase Energy
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_ivshmem_eth

#include <zephyr/drivers/pcie/cap.h>
#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <ethernet/eth_stats.h>

#include "eth.h"
#include "eth_ivshmem_priv.h"

LOG_MODULE_REGISTER(eth_ivshmem, CONFIG_ETHERNET_LOG_LEVEL);

#define PCI_VENDOR_ID_SIEMENS		0x110A
#define PCI_DEVICE_ID_IVSHMEM		0x4106
#define PCI_ID_SIEMENS_IVSHMEM		PCIE_ID(PCI_VENDOR_ID_SIEMENS, PCI_DEVICE_ID_IVSHMEM)

#define PCIE_CONF_CMDSTAT_INTX_DISABLE	0x0400
#define PCIE_CONF_INTR_PIN(x)		(((x) >> 8) & 0xFFu)
#define PCIE_INTX_PIN_MIN		1
#define PCIE_INTX_PIN_MAX		4

#define IVSHMEM_PCIE_REG_BAR_IDX	0
#define IVSHMEM_PCIE_MSI_X_BAR_IDX	1
#define IVSHMEM_PCIE_SHMEM_BAR_IDX	2

#define IVSHMEM_CFG_ID			0x00
#define IVSHMEM_CFG_NEXT_CAP		0x01
#define IVSHMEM_CFG_LENGTH		0x02
#define IVSHMEM_CFG_PRIV_CNTL		0x03
#define IVSHMEM_PRIV_CNTL_ONESHOT_INT	BIT(0)
#define IVSHMEM_CFG_STATE_TAB_SZ	0x04
#define IVSHMEM_CFG_RW_SECTION_SZ	0x08
#define IVSHMEM_CFG_OUTPUT_SECTION_SZ	0x10
#define IVSHMEM_CFG_ADDRESS		0x18

#define IVSHMEM_INT_ENABLE		BIT(0)

#define IVSHMEM_PROTO_UNDEFINED		0x0000
#define IVSHMEM_PROTO_NET		0x0001

#define ETH_IVSHMEM_STATE_RESET		0
#define ETH_IVSHMEM_STATE_INIT		1
#define ETH_IVSHMEM_STATE_READY		2
#define ETH_IVSHMEM_STATE_RUN		3

static const char * const eth_ivshmem_state_names[] = {
	[ETH_IVSHMEM_STATE_RESET] = "RESET",
	[ETH_IVSHMEM_STATE_INIT] = "INIT",
	[ETH_IVSHMEM_STATE_READY] = "READY",
	[ETH_IVSHMEM_STATE_RUN] = "RUN"
};

struct ivshmem_regs {
	uint32_t id;
	uint32_t max_peers;
	uint32_t int_control;
	uint32_t doorbell;
	uint32_t state;
};

struct eth_ivshmem_dev_data {
	struct net_if *iface;

	volatile struct ivshmem_regs *ivshmem_regs;
	const volatile uint32_t *state_table;
	uint32_t tx_rx_vector;
	uint32_t peer_id;
	uint8_t mac_addr[6];
	struct k_sem int_sem;
	struct eth_ivshmem_queue ivshmem_queue;

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ETH_IVSHMEM_THREAD_STACK_SIZE);
	struct k_thread thread;
	bool enabled;
	uint32_t state;
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
};

struct eth_ivshmem_cfg_data {
	const char *name;
	void (*generate_mac_addr)(uint8_t mac_addr[6]);
	struct intx_info {
		uint32_t irq;
		uint32_t priority;
		uint32_t flags;
	} intx_info[PCIE_INTX_PIN_MAX];
};

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *eth_ivshmem_get_stats(const struct device *dev)
{
	struct eth_ivshmem_dev_data *dev_data = dev->data;

	return &dev_data->stats;
}
#endif

static int eth_ivshmem_start(const struct device *dev)
{
	struct eth_ivshmem_dev_data *dev_data = dev->data;

	dev_data->enabled = true;

	/* Wake up thread to check/update state */
	k_sem_give(&dev_data->int_sem);

	return 0;
}

static int eth_ivshmem_stop(const struct device *dev)
{
	struct eth_ivshmem_dev_data *dev_data = dev->data;

	dev_data->enabled = false;

	/* Wake up thread to check/update state */
	k_sem_give(&dev_data->int_sem);

	return 0;
}

static enum ethernet_hw_caps eth_ivshmem_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T | ETHERNET_LINK_1000BASE_T;
}

static int eth_ivshmem_send(const struct device *dev, struct net_pkt *pkt)
{
	struct eth_ivshmem_dev_data *dev_data = dev->data;
	size_t len = net_pkt_get_len(pkt);

	void *data;
	int res = eth_ivshmem_queue_tx_get_buff(&dev_data->ivshmem_queue, &data, len);

	if (res != 0) {
		LOG_ERR("Failed to allocate tx buffer");
		eth_stats_update_errors_tx(dev_data->iface);
		return res;
	}

	if (net_pkt_read(pkt, data, len)) {
		LOG_ERR("Failed to read tx packet");
		eth_stats_update_errors_tx(dev_data->iface);
		return -EIO;
	}

	res = eth_ivshmem_queue_tx_commit_buff(&dev_data->ivshmem_queue);
	if (res == 0) {
		/* Notify peer */
		sys_write32(dev_data->tx_rx_vector | (dev_data->peer_id << 16),
			(mem_addr_t)&dev_data->ivshmem_regs->doorbell);
	}

	return res;
}

static struct net_pkt *eth_ivshmem_rx(struct eth_ivshmem_dev_data *dev_data)
{
	const void *rx_data;
	size_t rx_len;

	int res = eth_ivshmem_queue_rx(&dev_data->ivshmem_queue, &rx_data, &rx_len);

	if (res != 0) {
		if (res != -EWOULDBLOCK) {
			LOG_ERR("Queue RX failed");
			eth_stats_update_errors_rx(dev_data->iface);
		}
		return NULL;
	}

	struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(
		dev_data->iface, rx_len, AF_UNSPEC, 0, K_MSEC(100));
	if (pkt == NULL) {
		LOG_ERR("Failed to allocate rx buffer");
		eth_stats_update_errors_rx(dev_data->iface);
		goto dequeue;
	}

	if (net_pkt_write(pkt, rx_data, rx_len) != 0) {
		LOG_ERR("Failed to write rx packet");
		eth_stats_update_errors_rx(dev_data->iface);
		net_pkt_unref(pkt);
	}

dequeue:
	if (eth_ivshmem_queue_rx_complete(&dev_data->ivshmem_queue) == 0) {
		/* Notify peer */
		sys_write32(dev_data->tx_rx_vector | (dev_data->peer_id << 16),
			(mem_addr_t)&dev_data->ivshmem_regs->doorbell);
	}

	return pkt;
}

static void eth_ivshmem_set_state(struct eth_ivshmem_dev_data *dev_data, uint32_t state)
{
	LOG_DBG("State update: %s -> %s",
		eth_ivshmem_state_names[dev_data->state],
		eth_ivshmem_state_names[state]);
	dev_data->state = state;
	sys_write32(dev_data->state, (mem_addr_t) &dev_data->ivshmem_regs->state);
}

static void eth_ivshmem_state_update(struct eth_ivshmem_dev_data *dev_data)
{
	uint32_t peer_state = sys_read32((mem_addr_t)&dev_data->state_table[dev_data->peer_id]);

	switch (dev_data->state) {
	case ETH_IVSHMEM_STATE_RESET:
		switch (peer_state) {
		case ETH_IVSHMEM_STATE_RESET:
		case ETH_IVSHMEM_STATE_INIT:
			eth_ivshmem_set_state(dev_data, ETH_IVSHMEM_STATE_INIT);
			break;
		default:
			/* Wait for peer to reset */
			break;
		}
		break;
	case ETH_IVSHMEM_STATE_INIT:
		if (dev_data->iface == NULL || peer_state == ETH_IVSHMEM_STATE_RESET) {
			/* Peer is not ready for init */
			break;
		}
		eth_ivshmem_queue_reset(&dev_data->ivshmem_queue);
		eth_ivshmem_set_state(dev_data, ETH_IVSHMEM_STATE_READY);
		break;
	case ETH_IVSHMEM_STATE_READY:
	case ETH_IVSHMEM_STATE_RUN:
		switch (peer_state) {
		case ETH_IVSHMEM_STATE_RESET:
			net_eth_carrier_off(dev_data->iface);
			eth_ivshmem_set_state(dev_data, ETH_IVSHMEM_STATE_RESET);
			break;
		case ETH_IVSHMEM_STATE_READY:
		case ETH_IVSHMEM_STATE_RUN:
			if (dev_data->enabled && dev_data->state == ETH_IVSHMEM_STATE_READY) {
				eth_ivshmem_set_state(dev_data, ETH_IVSHMEM_STATE_RUN);
				net_eth_carrier_on(dev_data->iface);
			} else if (!dev_data->enabled && dev_data->state == ETH_IVSHMEM_STATE_RUN) {
				net_eth_carrier_off(dev_data->iface);
				eth_ivshmem_set_state(dev_data, ETH_IVSHMEM_STATE_RESET);
			}
			break;
		}
		break;
	}
}

FUNC_NORETURN static void eth_ivshmem_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct eth_ivshmem_dev_data *dev_data = dev->data;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		k_sem_take(&dev_data->int_sem, K_FOREVER);

		eth_ivshmem_state_update(dev_data);
		if (dev_data->state != ETH_IVSHMEM_STATE_RUN) {
			continue;
		}

		while (true) {
			struct net_pkt *pkt = eth_ivshmem_rx(dev_data);

			if (pkt == NULL) {
				break;
			}

			if (net_recv_data(dev_data->iface, pkt) < 0) {
				/* Upper layers are not ready to receive packets */
				net_pkt_unref(pkt);
			}

			k_yield();
		};
	}
}

static void eth_ivshmem_isr(const void *arg)
{
	const struct device *dev = arg;
	struct eth_ivshmem_dev_data *dev_data = dev->data;

	k_sem_give(&dev_data->int_sem);
}

static uint64_t pcie_conf_read_u64(pcie_bdf_t bdf, unsigned int reg)
{
	uint64_t lo = pcie_conf_read(bdf, reg);
	uint64_t hi = pcie_conf_read(bdf, reg + sizeof(uint32_t)/4);

	return hi << 32 | lo;
}

static bool eth_ivshmem_pci_lookup_cb(pcie_bdf_t bdf, pcie_id_t id, void *cb_data)
{
	struct pcie_dev *pci_dev = cb_data;

	if (id != PCI_ID_SIEMENS_IVSHMEM) {
		return true;
	}

	uint32_t class = pcie_conf_read(bdf, PCIE_CONF_CLASSREV);

	if (PCIE_CONF_CLASSREV_PROGIF(class) != IVSHMEM_PROTO_NET) {
		return true;
	}

	pci_dev->bdf = bdf;

	return false;
}

int eth_ivshmem_initialize(const struct device *dev)
{
	struct eth_ivshmem_dev_data *dev_data = dev->data;
	const struct eth_ivshmem_cfg_data *cfg_data = dev->config;
	int res;

	struct pcie_dev pcie = {
		.bdf = PCIE_BDF_NONE,
		.id = PCI_ID_SIEMENS_IVSHMEM
	};
	struct pcie_scan_opt pcie_scan_opt = {
		.cb = eth_ivshmem_pci_lookup_cb,
		.cb_data = &pcie,
		.flags = (PCIE_SCAN_RECURSIVE | PCIE_SCAN_CB_ALL),
	};

	pcie_scan(&pcie_scan_opt);

	if (pcie.bdf == PCIE_BDF_NONE) {
		LOG_ERR("Failed to find PCIe device");
		return -ENODEV;
	}

	LOG_INF("PCIe: ID 0x%08X, BDF 0x%X", pcie.id, pcie.bdf);

	struct pcie_bar mbar_regs, mbar_msi_x, mbar_shmem;

	if (!pcie_get_mbar(pcie.bdf, IVSHMEM_PCIE_REG_BAR_IDX, &mbar_regs)) {
		LOG_ERR("PCIe register bar not found");
		return -EINVAL;
	}

	pcie_set_cmd(pcie.bdf, PCIE_CONF_CMDSTAT_MEM |
		PCIE_CONF_CMDSTAT_MASTER, true);

	bool msi_x_bar_present = pcie_get_mbar(pcie.bdf, IVSHMEM_PCIE_MSI_X_BAR_IDX, &mbar_msi_x);
	bool shmem_bar_present = pcie_get_mbar(pcie.bdf, IVSHMEM_PCIE_SHMEM_BAR_IDX, &mbar_shmem);

	LOG_INF("MSI-X bar present: %s", msi_x_bar_present ? "yes" : "no");
	LOG_INF("SHMEM bar present: %s", shmem_bar_present ? "yes" : "no");

	z_phys_map((uint8_t **)&dev_data->ivshmem_regs,
		mbar_regs.phys_addr, mbar_regs.size,
		K_MEM_CACHE_WB | K_MEM_PERM_RW);

	LOG_INF("Registers at 0x%lX (mapped to 0x%lX), size 0x%04zX",
			mbar_regs.phys_addr, (uintptr_t)dev_data->ivshmem_regs, mbar_regs.size);

	LOG_INF("ivshmem_regs: id %u, max_peers %u",
		dev_data->ivshmem_regs->id, dev_data->ivshmem_regs->max_peers);
	if (dev_data->ivshmem_regs->id > 1) {
		LOG_ERR("Invalid ivshmem ID %u", dev_data->ivshmem_regs->id);
		return -EINVAL;
	}
	if (dev_data->ivshmem_regs->max_peers != 2) {
		LOG_ERR("Invalid ivshmem max peers %u", dev_data->ivshmem_regs->max_peers);
		return -EINVAL;
	}
	dev_data->peer_id = (dev_data->ivshmem_regs->id == 0) ? 1 : 0;

	uint32_t vendor_cap = pcie_get_cap(pcie.bdf, PCI_CAP_ID_VNDR);

	uintptr_t shmem_phys_addr;
	uint32_t cap_pos;

	if (shmem_bar_present) {
		shmem_phys_addr = mbar_shmem.phys_addr;
	} else {
		cap_pos = vendor_cap + IVSHMEM_CFG_ADDRESS / 4;
		shmem_phys_addr = pcie_conf_read_u64(pcie.bdf, cap_pos);
	}

	cap_pos = vendor_cap + IVSHMEM_CFG_STATE_TAB_SZ / 4;
	uint32_t state_table_size = pcie_conf_read(pcie.bdf, cap_pos);

	LOG_INF("state_table_size 0x%X", state_table_size);

	cap_pos = vendor_cap + IVSHMEM_CFG_RW_SECTION_SZ / 4;
	uint64_t rw_section_size = pcie_conf_read_u64(pcie.bdf, cap_pos);

	LOG_INF("rw_section_size 0x%llX", rw_section_size);

	cap_pos = vendor_cap + IVSHMEM_CFG_OUTPUT_SECTION_SZ / 4;
	uint64_t output_section_size = pcie_conf_read_u64(pcie.bdf, cap_pos);

	LOG_INF("output_section_size 0x%llX", output_section_size);

	uintptr_t shmem_addr;
	size_t shmem_size = state_table_size + rw_section_size + 2 * output_section_size;

	z_phys_map((uint8_t **)&shmem_addr,
		shmem_phys_addr, shmem_size,
		K_MEM_CACHE_WB | K_MEM_PERM_RW);

	LOG_INF("Shared memory at 0x%lX (mapped to 0x%lX), size 0x%04zX",
		shmem_phys_addr, shmem_addr, shmem_size);

	dev_data->state_table = (void *)shmem_addr;

	uintptr_t output_section_addr = shmem_addr + state_table_size + rw_section_size;
	/* ID 0 outputs to the first shmem section, ID 1 to the second */
	bool tx_buffer_first = dev_data->ivshmem_regs->id == 0;

	res = eth_ivshmem_queue_init(
		&dev_data->ivshmem_queue, output_section_addr,
		output_section_size, tx_buffer_first);
	if (res != 0) {
		LOG_ERR("Failed to init ivshmem queue");
		return res;
	}
	LOG_INF("shmem queue: desc_len 0x%hX, header size 0x%X, data size 0x%X",
		dev_data->ivshmem_queue.desc_max_len,
		dev_data->ivshmem_queue.vring_header_size,
		dev_data->ivshmem_queue.vring_data_max_len);

	k_sem_init(&dev_data->int_sem, 0, 1);

	cfg_data->generate_mac_addr(dev_data->mac_addr);
	LOG_INF("MAC Address %02X:%02X:%02X:%02X:%02X:%02X",
		dev_data->mac_addr[0], dev_data->mac_addr[1],
		dev_data->mac_addr[2], dev_data->mac_addr[3],
		dev_data->mac_addr[4], dev_data->mac_addr[5]);

	k_tid_t tid = k_thread_create(
		&dev_data->thread, dev_data->thread_stack,
		K_KERNEL_STACK_SIZEOF(dev_data->thread_stack),
		eth_ivshmem_thread,
		(void *) dev, NULL, NULL,
		CONFIG_ETH_IVSHMEM_THREAD_PRIORITY,
		K_ESSENTIAL, K_NO_WAIT);
	k_thread_name_set(tid, cfg_data->name);

	/* Ensure one-shot ISR mode is disabled */
	cap_pos = vendor_cap + IVSHMEM_CFG_PRIV_CNTL / 4;
	uint32_t cfg_priv_cntl = pcie_conf_read(pcie.bdf, cap_pos);

	cfg_priv_cntl &= ~(IVSHMEM_PRIV_CNTL_ONESHOT_INT << ((IVSHMEM_CFG_PRIV_CNTL % 4) * 8));
	pcie_conf_write(pcie.bdf, cap_pos, cfg_priv_cntl);

	if (msi_x_bar_present) {
		LOG_ERR("MSI-X not yet supported");
		return -ENOTSUP;
	}

	uint32_t cfg_int = pcie_conf_read(pcie.bdf, PCIE_CONF_INTR);
	uint32_t cfg_intx_pin = PCIE_CONF_INTR_PIN(cfg_int);

	if (!IN_RANGE(cfg_intx_pin, PCIE_INTX_PIN_MIN, PCIE_INTX_PIN_MAX)) {
		LOG_ERR("Invalid INTx pin %u", cfg_intx_pin);
		return -EINVAL;
	}

	/* Ensure INTx is enabled */
	pcie_set_cmd(pcie.bdf, PCIE_CONF_CMDSTAT_INTX_DISABLE, false);

	const struct intx_info *intx = &cfg_data->intx_info[cfg_intx_pin - 1];

	LOG_INF("Enabling INTx IRQ %u (pin %u)", intx->irq, cfg_intx_pin);
	if (!pcie_connect_dynamic_irq(
			pcie.bdf, intx->irq, intx->priority,
			eth_ivshmem_isr, dev, intx->flags)) {
		LOG_ERR("Failed to connect INTx ISR %u", cfg_intx_pin);
		return -EINVAL;
	}

	pcie_irq_enable(pcie.bdf, intx->irq);

	dev_data->tx_rx_vector = 0;

	eth_ivshmem_set_state(dev_data, ETH_IVSHMEM_STATE_RESET);

	sys_write32(IVSHMEM_INT_ENABLE,
		    (mem_addr_t)&dev_data->ivshmem_regs->int_control);

	/* Wake up thread to check/update state */
	k_sem_give(&dev_data->int_sem);

	return 0;
}

static void eth_ivshmem_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_ivshmem_dev_data *dev_data = dev->data;

	if (dev_data->iface == NULL) {
		dev_data->iface = iface;
	}

	net_if_set_link_addr(
		iface, dev_data->mac_addr,
		sizeof(dev_data->mac_addr),
		NET_LINK_ETHERNET);

	ethernet_init(iface);

	/* Do not start the interface until PHY link is up */
	net_if_carrier_off(iface);

	/* Wake up thread to check/update state */
	k_sem_give(&dev_data->int_sem);
}

static const struct ethernet_api eth_ivshmem_api = {
	.iface_api.init		= eth_ivshmem_iface_init,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats		= eth_ivshmem_get_stats,
#endif
	.start			= eth_ivshmem_start,
	.stop			= eth_ivshmem_stop,
	.get_capabilities	= eth_ivshmem_caps,
	.send			= eth_ivshmem_send,
};

#define ETH_IVSHMEM_INTX_INFO(intx_idx, drv_idx) {					\
		.irq = DT_IRQ_BY_IDX(DT_DRV_INST(drv_idx), intx_idx, irq),		\
		.priority = DT_IRQ_BY_IDX(DT_DRV_INST(drv_idx), intx_idx, priority),	\
		.flags = DT_IRQ_BY_IDX(DT_DRV_INST(drv_idx), intx_idx, flags),		\
	}

#define ETH_IVSHMEM_RANDOM_MAC_ADDR(inst)						\
	static void generate_mac_addr_##inst(uint8_t mac_addr[6])			\
	{										\
		uint32_t entropy = sys_rand32_get();					\
		mac_addr[0] = (entropy >> 16) & 0xff;					\
		mac_addr[1] = (entropy >>  8) & 0xff;					\
		mac_addr[2] = (entropy >>  0) & 0xff;					\
		/* Clear multicast bit */						\
		mac_addr[0] &= 0xFE;							\
		gen_random_mac(mac_addr, mac_addr[0], mac_addr[1], mac_addr[2]);	\
	}

#define ETH_IVSHMEM_LOCAL_MAC_ADDR(inst)						\
	static void generate_mac_addr_##inst(uint8_t mac_addr[6])			\
	{										\
		const uint8_t addr[6] = DT_INST_PROP(0, local_mac_address);		\
		memcpy(mac_addr, addr, sizeof(addr));					\
	}

#define ETH_IVSHMEM_GENERATE_MAC_ADDR(inst)						\
	BUILD_ASSERT(DT_INST_PROP(inst, zephyr_random_mac_address) ||			\
		NODE_HAS_VALID_MAC_ADDR(DT_DRV_INST(inst)),				\
		"eth_ivshmem requires either a fixed or random mac address");		\
	COND_CODE_1(DT_INST_PROP(inst, zephyr_random_mac_address),			\
			(ETH_IVSHMEM_RANDOM_MAC_ADDR(inst)),				\
			(ETH_IVSHMEM_LOCAL_MAC_ADDR(inst)))

#define ETH_IVSHMEM_INIT(inst)								\
	ETH_IVSHMEM_GENERATE_MAC_ADDR(inst);						\
	static struct eth_ivshmem_dev_data eth_ivshmem_dev_##inst = {};			\
	static const struct eth_ivshmem_cfg_data eth_ivshmem_cfg_##inst = {		\
		.name = "ivshmem_eth" STRINGIFY(inst),					\
		.generate_mac_addr = generate_mac_addr_##inst,				\
		.intx_info =								\
		{ FOR_EACH_FIXED_ARG(ETH_IVSHMEM_INTX_INFO, (,), inst, 0, 1, 2, 3) }	\
	};										\
	ETH_NET_DEVICE_DT_INST_DEFINE(inst,						\
				      eth_ivshmem_initialize,				\
				      NULL,						\
				      &eth_ivshmem_dev_##inst,				\
				      &eth_ivshmem_cfg_##inst,				\
				      CONFIG_ETH_INIT_PRIORITY,				\
				      &eth_ivshmem_api,					\
				      NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_IVSHMEM_INIT);
