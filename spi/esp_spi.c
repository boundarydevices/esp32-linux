/*
 * Copyright (C) 2015-2020 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include "esp_spi.h"
#include "esp_if.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#ifdef CONFIG_SUPPORT_ESP_SERIAL
#include "esp_serial.h"
#endif

static struct sk_buff * read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, u8 *buf, u32 size);
static void spi_exit(struct esp_spi_context *esp);

volatile u8 data_path = 0;

static struct esp_if_ops if_ops = {
	.read		= read_packet,
	.write		= write_packet,
};


static void open_data_path(void)
{
	msleep(200);
	data_path = OPEN_DATAPATH;
}

static void close_data_path(void)
{
	data_path = CLOSE_DATAPATH;
	msleep(200);
}

static irqreturn_t spi_interrupt_handler(int irq, void * priv)
{
	struct esp_spi_context *esp = (struct esp_spi_context *)priv;

	/* ESP32 is ready for next transaction */
	if (esp->spi_workqueue)
		queue_work(esp->spi_workqueue, &esp->spi_work);

	return IRQ_HANDLED;
}

static struct sk_buff * read_packet(struct esp_adapter *adapter)
{
	struct esp_spi_context *esp;
	struct sk_buff *skb = NULL;

	if (!data_path) {
		return NULL;
	}

	if (!adapter || !adapter->if_context) {
		printk (KERN_ERR "%s: Invalid args\n", __func__);
		return NULL;
	}

	esp = adapter->if_context;

	if (esp->esp_spi_dev) {
		skb = skb_dequeue(&(esp->rx_q));
	} else {
		printk (KERN_ERR "%s: Invalid args\n", __func__);
		return NULL;
	}

	return skb;
}

static int write_packet(struct esp_adapter *adapter, u8 *buf, u32 size)
{
	struct esp_spi_context *esp;
	struct sk_buff *skb;
	u8 *tx_buf = NULL;

	if (!adapter || !adapter->if_context || !buf || !size || (size > SPI_BUF_SIZE)) {
		printk (KERN_ERR "%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	if (!data_path) {
		return -EPERM;
	}

	/* Adjust length to make it multiple of 4 bytes  */
	size += 4 - (size & 3);

	esp = adapter->if_context;

	skb = esp_alloc_skb(size);

	if (!skb)
		return -ENOMEM;

	tx_buf = skb_put(skb, size);

	if (!tx_buf) {
		dev_kfree_skb(skb);
		return -ENOMEM;
	}

	/* TODO: This memecpy can be avoided if this function receives SKB as an argument */
	memcpy(tx_buf, buf, size);

	/* Enqueue SKB in tx_q */
	skb_queue_tail(&esp->tx_q, skb);

	return 0;
}

static void process_capabilities(struct esp_spi_context *esp, u8 cap)
{
	printk (KERN_INFO "ESP32 capabilities: 0x%x", cap);

	/* Reset BT */
	esp_deinit_bt(esp->adapter);

	if ((cap & ESP_BT_SPI_SUPPORT) || (cap & ESP_BT_SDIO_SUPPORT)) {
		msleep(200);
		esp_init_bt(esp->adapter);
	}
}

static void process_init_event(struct esp_spi_context *esp, u8 *evt_buf, u8 len)
{
	u8 len_left = len, tag_len;
	u8 *pos;

	if (!evt_buf)
		return;

	pos = evt_buf;

	while (len_left) {
		tag_len = *(pos + 1);
		if (*pos == ESP_PRIV_CAPABILITY) {
			process_capabilities(esp, *(pos + 2));
		} else {
			printk (KERN_WARNING "Unsupported tag in event");
		}
		len_left = len_left - (tag_len + 2);
	}
}

static void process_event(struct esp_spi_context *esp, u8 *evt_buf, u16 len)
{
	struct esp_priv_event *event;

	if (!evt_buf || !len)
		return;

	event = (struct esp_priv_event *) evt_buf;

	if (event->event_type == ESP_PRIV_EVENT_INIT) {
		printk (KERN_INFO "Received INIT event from esp32");
		process_init_event(esp, event->event_data, event->event_len);
	} else {
		printk (KERN_WARNING "Drop unknown event");
	}
}

static void process_priv_communication(struct esp_spi_context *esp, struct sk_buff *skb)
{
	struct esp_payload_header *header;
	u8 *payload;
	u16 len;

	if (!skb || !skb->data)
		return;

	header = (struct esp_payload_header *) skb->data;

	payload = skb->data + le16_to_cpu(header->offset);
	len = le16_to_cpu(header->len);

	if (header->priv_pkt_type == ESP_PACKET_TYPE_EVENT) {
		process_event(esp, payload, len);
	}

	dev_kfree_skb(skb);
}

static int process_rx_buf(struct esp_spi_context *esp, struct sk_buff *skb)
{
	struct esp_payload_header *header;
	u16 len = 0;
	u16 offset = 0;

	if (!skb)
		return -EINVAL;

	header = (struct esp_payload_header *) skb->data;

	offset = le16_to_cpu(header->offset);

	/* Validate received SKB. Check len and offset fields */
	if (offset != sizeof(struct esp_payload_header))
		return -EINVAL;

	len = le16_to_cpu(header->len);
	if (!len)
		return -EINVAL;

	len += sizeof(struct esp_payload_header);

	if (len > SPI_BUF_SIZE)
		return -EINVAL;

	/* Trim SKB to actual size */
	skb_trim(skb, len);

	if (header->if_type == ESP_PRIV_IF) {
		process_priv_communication(esp, skb);
		return 0;
	}

	if (!data_path)
		return -EPERM;

	/* enqueue skb for read_packet to pick it */
	skb_queue_tail(&esp->rx_q, skb);

	/* indicate reception of new packet */
	esp_process_new_packet_intr(esp->adapter);

	return 0;
}

static void esp_spi_work(struct work_struct *work)
{
	struct esp_spi_context *esp = container_of(work,
			struct esp_spi_context, spi_work);
	struct spi_transfer trans;
	struct sk_buff *tx_skb, *rx_skb;
	u8 *rx_buf;
	int ret = 0;

	memset(&trans, 0, sizeof(trans));
	tx_skb = NULL;

	/* Setup and execute SPI transaction
	 * 	Tx_buf: Check if tx_q has valid buffer for transmission,
	 * 		else keep it blank
	 *
	 * 	Rx_buf: Allocate memory for incoming data. This will be freed
	 *		immediately if received buffer is invalid.
	 *		If it is a valid buffer, upper layer will free it.
	 * */

	/* Configure TX buffer if available */
	if (data_path)
		tx_skb = skb_dequeue(&esp->tx_q);

	if (tx_skb) {
		trans.tx_buf = tx_skb->data;
	}

	/* Configure RX buffer */
	rx_skb = esp_alloc_skb(SPI_BUF_SIZE);
	rx_buf = skb_put(rx_skb, SPI_BUF_SIZE);

	memset(rx_buf, 0, SPI_BUF_SIZE);

	trans.rx_buf = rx_buf;
	trans.len = SPI_BUF_SIZE;

	ret = spi_sync_transfer(esp->esp_spi_dev, &trans, 1);

	if (ret) {
		printk(KERN_ERR "SPI Transaction failed: %d", ret);
	}

	/* Free rx_skb if received data is not valid */
	if (process_rx_buf(esp, rx_skb)) {
		dev_kfree_skb(rx_skb);
	}

	if (tx_skb)
		dev_kfree_skb(tx_skb);
}

static int spi_init(struct esp_spi_context *esp, struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	int status = 0;

	esp->spi_workqueue = create_workqueue("ESP_SPI_WORK_QUEUE");
	if (!esp->spi_workqueue) {
		spi_exit(esp);
		return -EFAULT;
	}

	INIT_WORK(&esp->spi_work, esp_spi_work);

	skb_queue_head_init(&esp->tx_q);
	skb_queue_head_init(&esp->rx_q);

	esp->esp_spi_dev = spi;
	if (!esp->esp_spi_dev) {
		dev_err(dev, "Failed to add new SPI device\n");
		spi_exit(esp);
		return -ENODEV;
	}

	status = devm_request_irq(dev, spi->irq, spi_interrupt_handler,
				  IRQF_SHARED | IRQF_TRIGGER_RISING,
				  "esp_spi", esp);
	if (status) {
		dev_err(dev, "Failed to request IRQ %d\n", status);
		spi_exit(esp);
		return status;
	}

	open_data_path();

#ifdef CONFIG_SUPPORT_ESP_SERIAL
	status = esp_serial_init((void *) esp->adapter);
	if (status != 0) {
		spi_exit(esp);
		dev_err(dev, "Error initialising serial interface\n");
		return status;
	}
#endif

	status = esp_add_card(esp->adapter);
	if (status) {
		spi_exit(esp);
		dev_err(dev, "Failed to add card\n");
		return status;
	}

	msleep(200);

	return status;
}

static void spi_exit(struct esp_spi_context *esp)
{
	close_data_path();
	msleep(200);

	skb_queue_purge(&esp->tx_q);
	skb_queue_purge(&esp->rx_q);

	if (esp->spi_workqueue) {
		destroy_workqueue(esp->spi_workqueue);
		esp->spi_workqueue = NULL;
	}

	esp_serial_cleanup();
	esp_remove_card(esp->adapter);

	if (esp->adapter->hcidev)
		esp_deinit_bt(esp->adapter);

	memset(esp, 0, sizeof(*esp));
}

static int esp_init_interface_layer(struct esp_spi_context *esp, struct esp_adapter *adapter,
				    struct spi_device *spi)
{
	if (!adapter)
		return -EINVAL;

	memset(esp, 0, sizeof(*esp));

	adapter->if_context = esp;
	adapter->if_ops = &if_ops;
	adapter->if_type = ESP_IF_TYPE_SPI;
	esp->adapter = adapter;

	return spi_init(esp, spi);
}

static void esp_deinit_interface_layer(struct esp_spi_context *esp)
{
	spi_exit(esp);
}

static void esp32_spi_reset(struct esp_spi_context *esp)
{
	gpiod_direction_output(esp->reset_gpio, GPIOD_OUT_LOW);
	udelay(100);
	gpiod_direction_input(esp->reset_gpio);
}

static int esp32_spi_probe(struct spi_device *spi)
{
	struct esp_spi_context *esp;
	int ret = 0;
	struct device *dev = &spi->dev;
	struct esp_adapter *adapter;

	esp = devm_kzalloc(dev, sizeof(*esp), GFP_KERNEL);
	if (!esp)
		return -ENOMEM;
	esp->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(esp->reset_gpio)) {
		dev_err(dev, "Can't get reset gpio\n");
		return -ENODEV;
	}
	spi_set_drvdata(spi, esp);

	esp32_spi_reset(esp);

	/* Init adapter */
	adapter = init_adapter();
	if (!adapter)
		return -EFAULT;

	/* Init transport layer */
	ret = esp_init_interface_layer(esp, adapter, spi);
	if (ret != 0) {
		deinit_adapter();
	} else
		dev_info(&spi->dev, "probed!\n");

	return ret;
}

static int esp32_spi_remove(struct spi_device *spi)
{
	struct esp_spi_context *esp = spi_get_drvdata(spi);

	esp_deinit_interface_layer(esp);
	deinit_adapter();

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id esp32_spi_dt_ids[] = {
	{ .compatible = "espressif,esp32" },
	{},
};
MODULE_DEVICE_TABLE(of, esp32_spi_dt_ids);
#endif

static struct spi_driver esp32_spi_driver = {
	.driver = {
		.name = "esp32",
		.of_match_table = of_match_ptr(esp32_spi_dt_ids),
	},
	.probe = esp32_spi_probe,
	.remove = esp32_spi_remove,
};
module_spi_driver(esp32_spi_driver);

MODULE_AUTHOR("Amey Inamdar <amey.inamdar@espressif.com>");
MODULE_AUTHOR("Mangesh Malusare <mangesh.malusare@espressif.com>");
MODULE_AUTHOR("Yogesh Mantri <yogesh.mantri@espressif.com>");
MODULE_AUTHOR("Boundary Devices <info@boundarydevices.com>");
MODULE_DESCRIPTION("Host SPI driver for ESP32 Hosted solution");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:esp32");
