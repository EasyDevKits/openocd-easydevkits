// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Driver for CH347-JTAG interface V1.1                                  *
 *                                                                         *
 *   Copyright (C) 2022 by oidcat.                                         *
 *   Author: oidcatiot@163.com                                             *
 *                                                                         *
 *   Enhancements by EasyDevKits - info@easydevkits.com                    *
 *                                                                         *
 *   CH347 is a high-speed USB bus converter chip that provides UART, I2C  *
 *   and SPI synchronous serial ports and JTAG interface through USB bus.  *
 *                                                                         *
 *   The Jtag interface by CH347 can supports transmission frequency       *
 *   configuration up to 60MHz.                                            *
 *                                                                         *
 *   The USB2.0 to JTAG scheme based on CH347 can be used to build         *
 *   customized USB high-speed JTAG debugger and other products.           *
 *                                                                         *
 *            _____________                                                *
 *           |             |____JTAG/SWD (TDO,TDI,TMS,TCK,TRST)            *
 *      USB__|    CH347T   |                                               *
 *           |_____________|____UART(TXD1,RXD1,RTS1,CTS1,DTR1)             *
 *            ______|______                                                *
 *           |             |                                               *
 *           | 8 MHz XTAL  |                                               *
 *           |_____________|                                               *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if IS_CYGWIN == 1
#include "windows.h"
#undef LOG_ERROR
#endif

// project specific includes
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/swd.h>
#include <helper/time_support.h>
#include <helper/replacements.h>
#include <helper/list.h>
#include <helper/binarybuffer.h>
#include "libusb_helper.h"

// system includes
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define JTAGIO_STA_OUT_TDI				(0x10)
#define JTAGIO_STA_OUT_TMS				(0x02)
#define JTAGIO_STA_OUT_TCK				(0x01)
#define JTAGIO_STA_OUT_TRST				(0x20)
#define TDI_H							JTAGIO_STA_OUT_TDI
#define TDI_L							0
#define TMS_H							JTAGIO_STA_OUT_TMS
#define TMS_L							0
#define TCK_H							JTAGIO_STA_OUT_TCK
#define TCK_L							0
#define TRST_H							JTAGIO_STA_OUT_TRST
#define TRST_L							0
#define LED_ON							1
#define LED_OFF							0
#define GPIO_CNT						8 // the CH347 has 8 GPIO's
#define USEABLE_GPIOS					0x78 /* mask which GPIO's are available in mode 3 of CH347T
												only GPIO3 (Pin11 / SCL), GPIO4 (Pin15 / ACT),
												GPIO5 (Pin9 / TRST) and GPIO6 (Pin2 / CTS1) are possible
												Tested only with CH347T not CH347F chip
												pin numbers are for CH347T */
#define VENDOR_VERSION					0x5F // for getting the chip version


#define HW_TDO_BUF_SIZE					4096
#define SF_PACKET_BUF_SIZE				51200 // Command packet length
#define UCMDPKT_DATA_MAX_BYTES_USBHS	507   /* The data length contained in each command packet
												 during USB high-speed operation */
#define USBC_PACKET_USBHS				512   // Maximum data length per packet at USB high speed
#define USBC_PACKET_USBHS_SINGLE		510   // usb high speed max package length
#define CH347_CMD_HEADER				3     // Protocol header length
#define MAX_BITS_PER_BIT_OP				248   /* No more bits are allowed per CH347_CMD_JTAG_BIT_OP
												 command; this should be dividable by 8 */

// Protocol transmission format: CMD (1 byte)+Length (2 bytes)+Data
#define CH347_CMD_INFO_RD				0xCA /* Parameter acquisition, used to obtain firmware version,
												JTAG interface related parameters, etc */
#define CH347_CMD_GPIO					0xCC // GPIO Command
#define CH347_CMD_JTAG_INIT				0xD0 // JTAG Interface initialization command
#define CH347_CMD_JTAG_BIT_OP			0xD1 // JTAG interface pin bit control command
#define CH347_CMD_JTAG_BIT_OP_RD		0xD2 // JTAG interface pin bit control and read commands
#define CH347_CMD_JTAG_DATA_SHIFT		0xD3 // JTAG interface data shift command
#define CH347_CMD_JTAG_DATA_SHIFT_RD	0xD4 // JTAG interface data shift and read command
/* for SWD */
#define CH347_CMD_SWD_INIT				0xE5 // SWD Interface Initialization Command
#define CH347_CMD_SWD					0xE8 // SWD Command group header
#define CH347_CMD_SWD_REG_W				0xA0 // SWD Interface write reg
#define CH347_CMD_SWD_SEQ_W				0xA1 // SWD Interface write spec seq
#define CH347_CMD_SWD_REG_R				0xA2 // SWD Interface read  reg
#define CH347_MAX_SEND_CMD				0X20 // max send cmd number
#define CH347_MAX_SEND_BUF				0X200
#define CH347_MAX_RECV_BUF				0X200
#define CH347_MAX_CMD_BUF				128

#define CH347_EPOUT						0x06u // the usb endpoint number for writing
#define CH347_EPIN						0x86u // the usb endpoint number for reading
#define CH347_MPHSI_INTERFACE			2 // the JTAG interface is number 2
#define USB_WRITE_TIMEOUT				500	// write timeout in milliseconds
#define USB_READ_TIMEOUT				500 // read timeout in milliseconds
#define BYTEWISE_MODE_VERSION			0x241 /* BCD version for devices that can use bytewise mode
												below this version only bitwise mode can be used */
#define DEFAULT_VENDOR_ID				0x1a86 // if no vendor id is set use this CH347 default
#define DEFAULT_PRODUCT_ID				0x55dd // if no product id is set use this CH347 default

enum pack_size {
	STANDARD_PACK = 0,
	LARGER_PACK = 1,
};

typedef void (*write_read_fn)(struct scan_command *cmd, uint8_t *bits, int nb_bits, enum scan_type scan);

struct ch347_info {
	// Record the CH347 pin status
	int tms_pin;
	int tdi_pin;
	int tck_pin;
	int trst_pin;

	int buffer_idx;
	uint8_t buffer[SF_PACKET_BUF_SIZE];

	int len_idx;
	int len_value;
	uint8_t last_cmd;

	uint8_t read_buffer[SF_PACKET_BUF_SIZE];
	int read_idx;
	int read_count;
	struct bit_copy_queue read_queue;
	enum pack_size pack_size;
	write_read_fn write_read_fn;
};

struct ch347_swd_io {
	uint8_t usb_cmd; // 0xA0, 0xA1, 0xA2
	uint8_t cmd;
	uint32_t *dst;
	uint32_t value;
	struct list_head list_entry;
};

struct ch347_swd_context {
	uint8_t send_buf[CH347_MAX_SEND_BUF];
	uint8_t recv_buf[CH347_MAX_RECV_BUF];
	int send_len;
	int recv_len;
	int need_recv_len;
	int queued_retval;
	int sent_cmd_count;
	struct list_head send_cmd_head;
	struct list_head free_cmd_head;
	struct ch347_swd_io ch347_cmd_buf[CH347_MAX_CMD_BUF];
};

static struct ch347_swd_context ch347_swd_context;
static bool swd_mode;
static bool dev_is_opened; // Whether the device is turned on
static uint16_t ch347_vids[] = {DEFAULT_VENDOR_ID, 0};
static uint16_t ch347_pids[] = {DEFAULT_PRODUCT_ID, 0};
// these are the 8 possible speeds; values in Hertz
static const int ch347_clock_speeds[] = {
	468750,
	937500,
	1875000,
	3750000,
	7500000,
	15000000,
	30000000,
	60000000,
	INT_MAX
};
static char *ch347_device_desc;
static uint8_t ch347_activity_led_gpio_pin = 0xFF;
static bool ch347_activity_led_active_high;
static struct ch347_info ch347;
static struct libusb_device_handle *ch347_handle;

// foreward declarations
static void ch347_write_read(struct scan_command *cmd, uint8_t *bits, int nb_bits, enum scan_type scan);
static void ch347_write_read_bitwise(struct scan_command *cmd, uint8_t *bits, int nb_bits, enum scan_type scan);
static int ch347_swd_run_queue(void);

/**
 * @brief opens the CH347 device via libusb driver
 *
 * @return true at success
 */
static bool ch347_open_device(void)
{
	if (jtag_libusb_open(ch347_vids, ch347_pids, ch347_device_desc, &ch347_handle, NULL) != ERROR_OK) {
		LOG_ERROR("CH347 not found: vid=%04x, pid=%04x",  ch347_vids[0], ch347_pids[0]);
		return false;
	}

	struct libusb_device_descriptor ch347_device_descriptor;
	libusb_get_device_descriptor(libusb_get_device(ch347_handle), &ch347_device_descriptor);

	int err_code = libusb_claim_interface(ch347_handle, CH347_MPHSI_INTERFACE);
	if (err_code != ERROR_OK) {
		LOG_ERROR("CH347 unable to claim interface: %s", libusb_error_name(err_code));
		return false;
	}

	char firmware_version;
	if (jtag_libusb_control_transfer(ch347_handle,
				LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
				VENDOR_VERSION, 0, 0, &firmware_version, sizeof(firmware_version),
				USB_WRITE_TIMEOUT, NULL) != ERROR_OK) {
		LOG_ERROR("CH347 unable to get firmware version");
		return false;
	}

	char manufacturer[256 + 1];
	if (libusb_get_string_descriptor_ascii(ch347_handle, ch347_device_descriptor.iManufacturer,
		(unsigned char *)manufacturer, sizeof(manufacturer) - 1) < 0)	{
		strcpy(manufacturer, "(unknown)");
	}
	char product[256 + 1];
	if (libusb_get_string_descriptor_ascii(ch347_handle, ch347_device_descriptor.iProduct,
		(unsigned char *)product, sizeof(product) - 1) < 0) {
		strcpy(product, "(unknown)");
	}
	char serial_number[256 + 1];
	if (libusb_get_string_descriptor_ascii(ch347_handle, ch347_device_descriptor.iSerialNumber,
		(unsigned char *)serial_number, sizeof(serial_number) - 1) < 0)	{
		strcpy(serial_number, "(unknown)");
	}

	LOG_INFO("CH347 %s from vendor %s with serial number %s found. (Chip version=%X.%2X, Firmware=0x%02X)",
		product,
		manufacturer,
		serial_number,
		(ch347_device_descriptor.bcdDevice >> 8) & 0xFF,
		ch347_device_descriptor.bcdDevice & 0xFF,
		firmware_version);

	if (ch347_device_descriptor.bcdDevice < BYTEWISE_MODE_VERSION) {
		LOG_INFO("CH347 old version of the chip, JTAG only working in bitwise mode. For bytewise mode at least version %X.%2X is needed.",
			(BYTEWISE_MODE_VERSION >> 8) & 0xFF,
			BYTEWISE_MODE_VERSION & 0xFF);
		ch347.write_read_fn = ch347_write_read_bitwise;
	} else {
		ch347.write_read_fn = ch347_write_read;
	}
	return true;
}

/**
 * @brief writes data to the CH347 via libusb driver
 *
 * @param data Point to the data buffer
 * @param length Data length in and out
 * @return true at success
 */
static bool ch347_write_data(uint8_t *data, int *length)
{
	int tmp = 0;
	int ret = jtag_libusb_bulk_write(ch347_handle, CH347_EPOUT, (char *)data, *length, USB_WRITE_TIMEOUT, &tmp);
	*length = tmp;
	return ret == ERROR_OK;
}

/**
 * @brief reads data from the CH347 via libusb driver
 *
 * @param data Point to the data buffer
 * @param length Data length in and out
 * @return true at success
 */
static bool ch347_read_data(uint8_t *data, int *length)
{
	int tmp = 0;
	int size = *length;
	int ret = jtag_libusb_bulk_read(ch347_handle, CH347_EPIN, (char *)data, size, USB_READ_TIMEOUT, &tmp);
	*length = tmp;
	return ret == ERROR_OK;
}

/**
 * @brief closes the CH347 device via libusb driver
 *
 * @return Always true
 */
static bool ch347_close_device(void)
{
	jtag_libusb_close(ch347_handle);
	return true;
}

/**
 * @brief swd init function
 *
 * @param clock_index clock index for CH347_CMD_SWD_INIT (E5)
 * @return true at success
 */
static bool ch347_swd_init_cmd(uint8_t clock_index)
{
	uint8_t cmd_buf[128] = "";
	int i = 0;
	cmd_buf[i++] = CH347_CMD_SWD_INIT;
	cmd_buf[i++] = 8;
	cmd_buf[i++] = 0;
	cmd_buf[i++] = 0x40;
	cmd_buf[i++] = 0x42;
	cmd_buf[i++] = 0x0f; // Reserved bytes
	cmd_buf[i++] = 0x00; // Reserved bytes
	cmd_buf[i++] = clock_index; // JTAG clock speed index
	i += 3; // Reserved Bytes

	int len = i;
	if (!ch347_write_data(cmd_buf, &len) || len != i)
		return false;

	len = 4;
	memset(cmd_buf, 0, sizeof(cmd_buf));

	if (!ch347_read_data(cmd_buf, &len) || len != 4)
		return false;

	return true;
}

/**
 * @brief CH347 Write
 *
 * @param buffer Point to a buffer to place the data to be written out
 * @param length Pointing to the length unit, the input is the length to be
 * written out, and the return is the actual written length
 * @return Write success returns true, failure returns false
 */
static bool ch347_write(uint8_t *buffer, int *length)
{
	int written_len = *length;
	if (*length >= HW_TDO_BUF_SIZE)
		written_len = HW_TDO_BUF_SIZE;
	int i = 0;

	while (true) {
		if (!ch347_write_data(buffer + i, &written_len)) {
			*length = 0;
			return false;
		}
		if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
			char *str = buf_to_hex_str(buffer, written_len);
			LOG_DEBUG_IO("size=%d, buf=[%s]", written_len, str);
			free(str);
		}
		i += written_len;
		if (i >= *length)
			break;
		if ((*length - i) > HW_TDO_BUF_SIZE)
			written_len = HW_TDO_BUF_SIZE;
		else
			written_len = *length - i;
	}

	*length = i;
	return true;
}

/**
 * @brief CH347 Read
 *
 * @param buffer  Point to a buffer to place the data to be read in
 * @param length Pointing to the length unit, the input is the length to
 * be read, and the return is the actual read length
 * @return Write success returns true, failure returns false
 */
static bool ch347_read(uint8_t *buffer, int *length)
{
	int read_len = *length;
	/* The maximum allowable reading for a single read is 4096B of data.
	   If it exceeds the allowable reading limit, it will be calculated as 4096B */
	if (read_len > HW_TDO_BUF_SIZE)
		read_len = HW_TDO_BUF_SIZE;
	int i = 0;

	while (true) {
		if (!ch347_read_data(buffer + i, &read_len)) {
			LOG_ERROR("CH347 read fail");
			return false;
		}

		i += read_len;
		if (i >= *length)
			break;
		read_len = (*length - i) > HW_TDO_BUF_SIZE ? HW_TDO_BUF_SIZE : *length - i;
	}
	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
		char *str = buf_to_hex_str(buffer, i);
		LOG_DEBUG_IO("size=%d, buf=[%s]", i, str);
		free(str);
	}
	*length = i;
	return true;
}

/**
 * @brief Reads data back from CH347 and decode it byte- and bitwise into the buffer
 *
 * @param buffer Point to a buffer to place the data to be read in
 * @param length Data length in bytes that should be read via libusb
 */
static void ch347_read_scan(uint8_t *buffer, int length)
{
	int read_len = length;
	unsigned char *read_buf = calloc(sizeof(unsigned char), read_len);
	if (!read_buf) {
		LOG_ERROR("calloc failed");
		return;
	}

	if (!ch347_read(read_buf, &read_len)) {
		LOG_ERROR("CH347 read fail");
		return;
	}

	int data_len = 0;
	int bit_counter = 0;
	int rd_index = 0;
	int buf_index = 0;

	while (rd_index < read_len) {
		// for CH347_CMD_JTAG_DATA_SHIFT_RD copy the data bytes
		if (read_buf[rd_index] == CH347_CMD_JTAG_DATA_SHIFT_RD) {
			data_len = le_to_h_u16(&read_buf[++rd_index]);
			rd_index += 2;
			memcpy(buffer + buf_index, &read_buf[rd_index], data_len);
			buf_index += data_len;
			rd_index += data_len;
		} else if (read_buf[rd_index] == CH347_CMD_JTAG_BIT_OP_RD) {
			// for CH347_CMD_JTAG_BIT_OP_RD we need to copy bit by bit
			data_len = le_to_h_u16(&read_buf[++rd_index]);
			rd_index += 2;

			for (int i = 0; i < data_len; i++) {
				if (read_buf[rd_index + i] & 1)
					buffer[buf_index] |= (1 << i % 8);
				else
					buffer[buf_index] &= ~(1 << i % 8);
				bit_counter++;
				// advance the buffer index after every 8 bits stored
				if (bit_counter % 8 == 0)
					buf_index++;
			}
			rd_index += data_len;
			/* after a read advance the buffer, because we put the bits from the next
				commands in the next buffer byte. But don't advance twice. After the
				8th bit the index is alread advanced */
			if (bit_counter % 8 != 0)
				buf_index++;
		} else {
			LOG_ERROR("CH347 read command fail");
			buffer[buf_index] = read_buf[rd_index];
			buf_index++;
			rd_index++;
		}
	}
	free(read_buf);
}

/**
 * @brief Sends the write buffer via libusb
 * and if LARGER_PACK mode is active read also data back
 */
static void ch347_flush_buffer(void)
{
	int wr_len = ch347.buffer_idx;
	int nb = ch347.buffer_idx;
	int ret = ERROR_OK;

	while (ret == ERROR_OK && nb > 0) {
		ret = ch347_write(ch347.buffer, &wr_len);
		nb -= wr_len;
	}
	memset(&ch347.buffer, 0, sizeof(ch347.buffer));
	ch347.buffer_idx = 0;
	ch347.last_cmd = 0;
	ch347.len_idx = 0;
	ch347.len_value = 0;

	if (ch347.read_count == 0)
		return;
	if (ch347.pack_size == LARGER_PACK) {
		ch347_read_scan(ch347.read_buffer, ch347.read_count);
		bit_copy_execute(&ch347.read_queue);
		memset(ch347.read_buffer, 0, SF_PACKET_BUF_SIZE);
		ch347.read_count = 0;
		ch347.read_idx = 0;
	}
}

/**
 * @brief Puts a byte for writing into the output buffer.
 * If buffer gets full it calls ch347_flush_buffer.
 * @param byte the byte
 */
static void ch347_in_buffer(uint8_t byte)
{
	if ((SF_PACKET_BUF_SIZE - ch347.buffer_idx) < 1)
		ch347_flush_buffer();
	ch347.buffer[ch347.buffer_idx] = byte;
	ch347.buffer_idx++;
	if ((SF_PACKET_BUF_SIZE - ch347.buffer_idx) == 0)
		ch347_flush_buffer();
}

/**
 * @brief Puts multiple byte for writing into the output buffer
 * @param bytes the bytes
 * @param count number of bytes
 */
static void ch347_in_buffer_bytes(uint8_t *bytes, int count)
{
	if ((ch347.buffer_idx + count) > SF_PACKET_BUF_SIZE)
		ch347_flush_buffer();
	memcpy(&ch347.buffer[ch347.buffer_idx], bytes, count);
	ch347.buffer_idx += count;
	if ((SF_PACKET_BUF_SIZE - ch347.buffer_idx) < 1)
		ch347_flush_buffer();
}

/**
 * @brief If two packets for the same USB command are in the writing buffer,
 * the commands will be combined into one bigger command
 *
 * @param cmd
 * @param cur_idx
 * @param len
 */
static void ch347_combine_packets(uint8_t cmd, int cur_idx, int len)
{
	if (cmd != ch347.last_cmd) {
		ch347.buffer[cur_idx] = cmd;
		h_u16_to_le(&ch347.buffer[cur_idx + 1], len - CH347_CMD_HEADER);

		// update the ch347 struct
		ch347.last_cmd = cmd;
		ch347.len_idx = cur_idx + 1;
		ch347.len_value = (len - CH347_CMD_HEADER);
	} else {
		// update the ch347 struct cmd data length
		ch347.len_value += (len - CH347_CMD_HEADER);

		// update the cmd packet valid length
		h_u16_to_le(&ch347.buffer[ch347.len_idx], ch347.len_value);

		// update the buffer data length
		memcpy(&ch347.buffer[cur_idx], &ch347.buffer[cur_idx + CH347_CMD_HEADER], (len - CH347_CMD_HEADER));

		/* update the ch347 buffer index */
		ch347.buffer_idx -= CH347_CMD_HEADER;
	}
}
/**
 * @brief Function used to change the TMS value at the
 * rising edge of TCK to switch its TAP state
 *
 * @param tms TMS value to be changed; true = output TMS high; false = output TMS low
 * @param buf_index Protocol packet length
 * @return Return protocol packet length
 */
static int ch347_clock_tms(bool tms, int buf_index)
{
	uint8_t tms_bit = tms ? TMS_H : TMS_L;
	uint8_t tdi_bit = ch347.tdi_pin ? TDI_H : TDI_L;
	uint8_t trst_bit = ch347.trst_pin ? TRST_H : TRST_L;
	// TCK = L
	uint8_t data = tms_bit | tdi_bit | TCK_L | trst_bit;
	ch347_in_buffer(data);
	// TCK = H
	data = tms_bit | tdi_bit | TCK_H | trst_bit;
	ch347_in_buffer(data);
	ch347.tms_pin = tms_bit;
	ch347.tck_pin = TCK_H;
	return buf_index + 2;
}

/**
 * @brief Function to ensure that the clock is in a low state
 *
 * @param buf_index Protocol packet length
 * @return Return protocol packet length
 */
static int ch347_idle_clock(int buf_index)
{
	uint8_t data = TCK_L;
	data |= ch347.tms_pin ? TMS_H : TMS_L;
	data |= ch347.tdi_pin ? TDI_H : TDI_L;
	data |= ch347.trst_pin ? TRST_H : TRST_L;
	ch347_in_buffer(data);
	ch347.tck_pin = TCK_L;
	return buf_index + 1;
}

/**
 * @brief Function that performs state switching by changing the value of TMS
 *
 * @param tms_value The TMS values that need to be switched form one byte of data in the switching order
 * @param step The number of bit values that need to be read from the tms_value value
 * @param skip Count from the skip bit of tms_value to step
 */
static void ch347_tms_change(const uint8_t *tms_value, int step, int skip)
{
	int prev_index = ch347.buffer_idx;
	int buf_index = CH347_CMD_HEADER;
	LOG_DEBUG_IO("TMS Value: %02x..., step = %d, skip = %d", tms_value[0], step, skip);

	for (int i = 0; i < 3; i++)
		ch347_in_buffer(0);

	for (int i = skip; i < step; i++)
		buf_index = ch347_clock_tms((tms_value[i / 8] >> (i % 8)) & 0x01, buf_index);
	int cmd_len = ch347_idle_clock(buf_index);

	ch347_combine_packets(CH347_CMD_JTAG_BIT_OP, prev_index, cmd_len);
}

/**
 * @brief By ch347_executeQueue call
 *
 * @param cmd Upper layer transfer command parameters
 */
static void ch347_tms(struct tms_command *cmd)
{
	LOG_DEBUG_IO("step: %d", cmd->num_bits);
	ch347_tms_change(cmd->bits, cmd->num_bits, 0);
}

/**
 * @brief Sets a GPIO bit
 *
 * @param gpio GPIO bit number 0-7
 * @param data true for high; false for low
 */
static void ch347_gpio_set(int gpio, bool data)
{
	// fixed length of the GPIO control command
	const int gpio_cmd_len = CH347_CMD_HEADER + GPIO_CNT;

	// build a gpio control command
	int cmd_len = gpio_cmd_len;
	uint8_t gpio_cmd[gpio_cmd_len];
	memset(gpio_cmd, 0, sizeof(gpio_cmd));
	gpio_cmd[0] = CH347_CMD_GPIO;
	gpio_cmd[1] = GPIO_CNT;
	gpio_cmd[2] = 0;
	/* always set bits 7 and 6 for GPIO enable
		bits 5 and 4 for pin direction output
		bit 3 is the data bit */
	gpio_cmd[CH347_CMD_HEADER + gpio] = data == 0 ? 0xF0 : 0xF8;

	if (!ch347_write(gpio_cmd, &cmd_len) && cmd_len != gpio_cmd_len) {
		LOG_ERROR("send usb data failure.");
		return;
	} else if (!ch347_read(gpio_cmd, &cmd_len) && cmd_len != gpio_cmd_len) {
		LOG_ERROR("read usb data failure.");
		return;
	} else if ((gpio_cmd[CH347_CMD_HEADER + gpio] & 0x40) >> 6 != data) {
		LOG_ERROR("output not set.");
		return;
	}
}

/**
 * @brief Turn the activity LED on or off
 *
 * @param led_state LED_ON or LED_OFF
 */
static void ch347_set_activity_led(int led_state)
{
	if (ch347_activity_led_gpio_pin != 0xff)
		ch347_gpio_set(ch347_activity_led_gpio_pin, ch347_activity_led_active_high ? led_state : 1 - led_state);
}

/**
 * @brief Reset for resetting with pins; not used for reset via TMS
 *
 * @param trst TRST pin
 * @param srst SRST pin
 * @return Always ERROR_OK
 */
static int ch347_reset(int trst, int srst)
{
	LOG_DEBUG_IO("reset trst: %i srst %i", trst, srst);
	/* have seen in ftdi driver, that reset does only the reset via trst or srst pins.
		if both are unset the ftdi driver does nothing. => do also nothing if both are unset */
	if (!trst && !srst)
		return ERROR_OK;

	/* untested! if not in swd mode and trst is defined we can give
		a 50µs pulse to the TRST pin via bit operations */
	if (!swd_mode && trst != 0) {
		unsigned long byte_index = 0;

		ch347_in_buffer(CH347_CMD_JTAG_BIT_OP);
		ch347_in_buffer(0x01);
		ch347_in_buffer(0);

		ch347.trst_pin = TRST_L;
		ch347_idle_clock(byte_index);

		ch347_flush_buffer();

		usleep(50);

		ch347_in_buffer(CH347_CMD_JTAG_BIT_OP);
		ch347_in_buffer(0x01);
		ch347_in_buffer(0);

		ch347.trst_pin = TRST_H;
		ch347_idle_clock(byte_index);

		ch347_flush_buffer();
	}
	return ERROR_OK;
}

/**
 * @brief Obtain the current Tap status and switch to the status TMS value passed down by cmd
 *
 * @param cmd Upper layer transfer command parameters
 */
static void ch347_move_path(struct pathmove_command *cmd)
{
	int prev_index = ch347.buffer_idx;
	int buf_index = CH347_CMD_HEADER;

	for (int i = 0; i < 3; i++)
		ch347_in_buffer(0);
	LOG_DEBUG_IO("num_states=%d, last_state=%d",
			 cmd->num_states, cmd->path[cmd->num_states - 1]);

	for (int i = 0; i < cmd->num_states; i++) {
		if (tap_state_transition(tap_get_state(), false) ==	cmd->path[i])
			buf_index = ch347_clock_tms(0, buf_index);
		if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
			buf_index = ch347_clock_tms(1, buf_index);
		tap_set_state(cmd->path[i]);
	}

	int cmd_len = ch347_idle_clock(buf_index);

	ch347_combine_packets(CH347_CMD_JTAG_BIT_OP, prev_index, cmd_len);
}

/**
 * @brief Toggle the tap state to the target state
 *
 * @param state Pre switch target path
 * @param skip Number of digits to skip
 */
static void ch347_move_state(tap_state_t state, int skip)
{
	uint8_t tms_scan;
	int tms_len;

	LOG_DEBUG_IO("from %s to %s", tap_state_name(tap_get_state()), tap_state_name(state));
	// don't do anything if we are already in the right state; but do execute always the TAP_RESET
	if (tap_get_state() == state && state != TAP_RESET)
		return;
	tms_scan = tap_get_tms_path(tap_get_state(), state);
	tms_len = tap_get_tms_path_len(tap_get_state(), state);
	ch347_tms_change(&tms_scan, tms_len, skip);
	tap_set_state(state);
}

/**
 * @brief Used to put the data from the byte buffer into the scan command
 *
 * @param cmd The scan command
 * @param read_data data that are read from usb
 */
static void ch347_scan_data_to_fields(struct scan_command *cmd, uint8_t *read_data)
{
	if (!cmd) {
		LOG_ERROR("cmd is NULL!");
		return;
	}

	int offset = 0;
	int num_bits = 0;

	for (int i = 0; i < cmd->num_fields; i++) {
		// if neither in_value nor in_handler are specified we don't have to examine this field
		LOG_DEBUG("fields[%d].in_value[%d], offset: %d", i, cmd->fields[i].num_bits, offset);
		num_bits = cmd->fields[i].num_bits;
		if (cmd->fields[i].in_value) {
			if (ch347.pack_size == LARGER_PACK) {
				if (cmd->fields[i].in_value)
					bit_copy_queued(&ch347.read_queue, cmd->fields[i].in_value,	0,
						&ch347.read_buffer[ch347.read_idx], offset, num_bits);

				if (num_bits > 7)
					ch347.read_idx += DIV_ROUND_UP(offset, 8);
			} else {
				uint8_t *buffer = malloc(DIV_ROUND_UP(num_bits, 8));
				if (!buffer) {
					LOG_ERROR("malloc failed");
					return;
				}
				uint8_t *captured = buf_set_buf(read_data, offset, buffer, 0, num_bits);

				if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
					int size = num_bits > DEBUG_JTAG_IOZ ? DEBUG_JTAG_IOZ : num_bits;
					char *str = buf_to_hex_str(captured, size);
					LOG_DEBUG_IO("size=%d, buf=[%s]", size, str);
					free(str);
				}
				if (cmd->fields[i].in_value)
					buf_cpy(captured, cmd->fields[i].in_value, num_bits);
				free(buffer);
			}
		}
		offset += num_bits;
	}
}

/**
 * @brief CH347 Batch read/write function
 * That's the workaround function for write/read bit by bit because the
 * bytewise functions D3/D4 are not working correctly for all chip versions
 *
 * @param cmd The scan command
 * @param bits Read and write data this time
 * @param nb_bits Incoming data length
 * @param scan The transmission method of incoming data to determine whether to perform data reading
 */
static void ch347_write_read_bitwise(struct scan_command *cmd, uint8_t *bits, int nb_bits, enum scan_type scan)
{
	int chunk_data_length = 0;
	int chunk_bit_count = 0;
	int chunk_count = 0;
	uint8_t tms_bit = 0;
	uint8_t tdi_bit = 0;

	if (ch347.pack_size == LARGER_PACK) {
		if ((ch347.read_count >= (USBC_PACKET_USBHS_SINGLE * 1)))
			ch347_flush_buffer();
	} else {
		ch347_flush_buffer();
	}

	bool is_read = (scan == SCAN_IN || scan == SCAN_IO);
	int i = 0;

	while (i < nb_bits) {
		// we need two bytes for each bit (one for TCK_L and one for TCK_H)
		if ((nb_bits - i) > MAX_BITS_PER_BIT_OP) {
			chunk_bit_count = MAX_BITS_PER_BIT_OP;
			chunk_data_length = chunk_bit_count * 2;
		} else {
			// an additional TCK_L after the last byte is needed because that's the last chunk
			chunk_data_length = (nb_bits - i) * 2 + 1;
			chunk_bit_count = nb_bits - i;
		}

		// build the cmd header
		if (is_read)
			ch347_in_buffer(CH347_CMD_JTAG_BIT_OP_RD);
		else
			ch347_in_buffer(CH347_CMD_JTAG_BIT_OP);
		ch347_in_buffer((uint8_t)chunk_data_length & 0xFF);
		ch347_in_buffer((uint8_t)(chunk_data_length >> 8) & 0xFF);

		tms_bit = TMS_L;
		uint8_t trst_bit = ch347.trst_pin ? TRST_H : TRST_L;
		for (int j = 0; j < chunk_bit_count; j++) {
			tdi_bit = ((bits[i / 8] >> i % 8) & 0x01) ? TDI_H : TDI_L;
			// for the last bit set TMS high to exit the shift state
			if ((i + 1) == nb_bits)
				tms_bit = TMS_H;

			ch347_in_buffer(tms_bit | tdi_bit | TCK_L | trst_bit);
			ch347_in_buffer(tms_bit | tdi_bit | TCK_H | trst_bit);

			i++;
		}
		// one TCK_L after the last bit
		if (i == nb_bits)
			ch347_in_buffer(tms_bit | tdi_bit | TCK_L | trst_bit);

		chunk_count++;
	}

	ch347.tms_pin = tms_bit;
	ch347.tdi_pin = tdi_bit;
	ch347.tck_pin = TCK_L;

	if (is_read) {
		uint8_t *read_data = calloc(SF_PACKET_BUF_SIZE, 1);
		if (!read_data) {
			LOG_ERROR("calloc failed");
			return;
		}
		uint32_t read_len = nb_bits + chunk_count * CH347_CMD_HEADER;
		ch347.read_count = read_len;

		if (ch347.pack_size == STANDARD_PACK && bits && cmd) {
			ch347_flush_buffer();
			ch347_read_scan(read_data, read_len);
		}

		ch347_scan_data_to_fields(cmd, read_data);
		free(read_data);
	}

	int prev_index = ch347.buffer_idx;
	for (i = 0; i < CH347_CMD_HEADER; i++)
		ch347_in_buffer(0);
	int buf_index = CH347_CMD_HEADER;
	buf_index = ch347_idle_clock(buf_index);
	ch347_combine_packets(CH347_CMD_JTAG_BIT_OP, prev_index, buf_index);
}

/**
 * @brief CH347 Batch read/write function
 *
 * @param cmd The scan command
 * @param bits Read and write data this time
 * @param nb_bits Incoming data length
 * @param scan The transmission method of incoming data to determine whether to perform data reading
 */
static void ch347_write_read(struct scan_command *cmd, uint8_t *bits, int nb_bits, enum scan_type scan)
{
	int nb8 = nb_bits / 8;
	int nb1 = nb_bits % 8;
	static uint8_t byte0[SF_PACKET_BUF_SIZE];

	if (ch347.pack_size == LARGER_PACK) {
		if ((ch347.read_count >= (USBC_PACKET_USBHS_SINGLE * 1)))
			ch347_flush_buffer();
	} else {
		ch347_flush_buffer();
	}

	if (nb8 > 0 && nb1 == 0) {
		nb8--;
		nb1 = 8;
	}

	bool is_read = (scan == SCAN_IN || scan == SCAN_IO);
	int temp_len = 0;
	int pkt_data_len;
	int i = 0;

	while (i < nb8) {
		if ((nb8 - i) > UCMDPKT_DATA_MAX_BYTES_USBHS)
			pkt_data_len = UCMDPKT_DATA_MAX_BYTES_USBHS;
		else
			pkt_data_len = nb8 - i;

		if (is_read)
			ch347_in_buffer(CH347_CMD_JTAG_DATA_SHIFT_RD);
		else
			ch347_in_buffer(CH347_CMD_JTAG_DATA_SHIFT);

		// packet data don't deal D3 & D4
		if (ch347.last_cmd != CH347_CMD_JTAG_DATA_SHIFT_RD ||
			ch347.last_cmd != CH347_CMD_JTAG_DATA_SHIFT) {
			// update the ch347 struct
			ch347.last_cmd = 0;
			ch347.len_idx = 0;
			ch347.len_value = 0;
		}

		ch347_in_buffer((uint8_t)(pkt_data_len >> 0) & 0xFF);
		ch347_in_buffer((uint8_t)(pkt_data_len >> 8) & 0xFF);

		if (bits)
			ch347_in_buffer_bytes(&bits[i], pkt_data_len);
		else
			ch347_in_buffer_bytes(byte0, pkt_data_len);
		i += pkt_data_len;

		temp_len += (pkt_data_len + CH347_CMD_HEADER);
	}

	int total_rd_len = temp_len;
	int read_len = 0;

	if (is_read) {
		ch347.read_count += temp_len;
		read_len += temp_len;
	}

	int d_len = 0;

	uint8_t tms_bit = 0;
	uint8_t tdi_bit = 0;
	uint8_t cmd_bit;

	if (bits) {
		cmd_bit = is_read ? CH347_CMD_JTAG_BIT_OP_RD : CH347_CMD_JTAG_BIT_OP;
		d_len = (nb1 * 2) + 1;

		if (cmd_bit != ch347.last_cmd) {
			ch347_in_buffer(cmd_bit);
			ch347_in_buffer((uint8_t)(d_len >> 0) & 0xFF);
			ch347_in_buffer((uint8_t)(d_len >> 8) & 0xFF);
			ch347.last_cmd = cmd_bit;
			ch347.len_idx = ch347.buffer_idx - 2;
			ch347.len_value = d_len;
		} else {
			// update the ch347 struct cmd data length
			ch347.len_value += d_len;
			// update the cmd packet valid length
			ch347.buffer[ch347.len_idx] =
				(uint8_t)(ch347.len_value >> 0) & 0xFF;
			ch347.buffer[ch347.len_idx + 1] =
				(uint8_t)(ch347.len_value >> 8) & 0xFF;
		}

		tms_bit = TMS_L;
		uint8_t trst_bit = ch347.trst_pin ? TRST_H : TRST_L;

		for (int j = 0; j < nb1; j++) {
			tdi_bit = (bits[nb8] >> j) & 0x01 ? TDI_H : TDI_L;

			if ((j + 1) == nb1)
				tms_bit = TMS_H;

			ch347_in_buffer(tms_bit | tdi_bit | TCK_L | trst_bit);
			ch347_in_buffer(tms_bit | tdi_bit | TCK_H | trst_bit);
		}
		ch347_in_buffer(tms_bit | tdi_bit | TCK_L | trst_bit);
	}

	ch347.tms_pin = tms_bit;
	ch347.tdi_pin = tdi_bit;
	ch347.tck_pin = TCK_L;

	if (is_read) {
		temp_len = ((d_len / 2) + CH347_CMD_HEADER);
		total_rd_len += temp_len;
		ch347.read_count += temp_len;
		read_len += temp_len;
	}
	if (is_read && total_rd_len > 0) {
		uint8_t *read_data = calloc(SF_PACKET_BUF_SIZE, 1);
		if (!read_data) {
			LOG_ERROR("calloc failed");
			return;
		}
		if (ch347.pack_size == STANDARD_PACK && bits && cmd) {
			ch347_flush_buffer();
			ch347_read_scan(read_data, read_len);
		}

		ch347_scan_data_to_fields(cmd, read_data);
		free(read_data);
	}

	int prev_index = ch347.buffer_idx;
	for (i = 0; i < CH347_CMD_HEADER; i++)
		ch347_in_buffer(0);
	int buf_index = CH347_CMD_HEADER;
	buf_index = ch347_idle_clock(buf_index);

	ch347_combine_packets(CH347_CMD_JTAG_BIT_OP, prev_index, buf_index);
}

/**
 * @brief Toggle the Tap state to run test/idle
 *
 * @param cycles
 * @param state
 */
static void ch347_run_test(int cycles, tap_state_t state)
{
	LOG_DEBUG_IO("cycles=%d, end_state=%d", cycles, state);
	if (tap_get_state() != TAP_IDLE)
		ch347_move_state(TAP_IDLE, 0);

	uint8_t tms_value = 0;
	ch347_tms_change(&tms_value, 7, 1);

	ch347.write_read_fn(NULL, NULL, cycles, SCAN_OUT);
	ch347_move_state(state, 0);
}

/**
 * @brief ???
 *
 * @param cycles
 * @param state
 */
static void ch347_stable_clocks(int cycles)
{
	LOG_DEBUG_IO("cycles=%d", cycles);
	ch347.write_read_fn(NULL, NULL, cycles, SCAN_OUT);
}

/**
 * @brief Switch to SHIFT-DR or SHIFT-IR status for scanning
 *
 * @param cmd Upper layer transfer command parameters
 * @return Always ERROR_OK
 */
static int ch347_scan(struct scan_command *cmd)
{
	static const char *const type2str[] = {"", "SCAN_IN", "SCAN_OUT", "SCAN_IO"};

	enum scan_type type = jtag_scan_type(cmd);
	uint8_t *buf = NULL;
	int scan_bits = jtag_build_buffer(cmd, &buf);

	if (cmd->ir_scan)
		ch347_move_state(TAP_IRSHIFT, 0);
	else
		ch347_move_state(TAP_DRSHIFT, 0);

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
		char *log_buf = buf_to_hex_str(buf, DIV_ROUND_UP(scan_bits, 8));
		LOG_DEBUG_IO("scan=%s, type=%s, bits=%d, buf=[%s], end_state=%d",
			 cmd->ir_scan ? "IRSCAN" : "DRSCAN",
			 type2str[type],
			 scan_bits, log_buf, cmd->end_state);
		free(log_buf);
	}

	ch347.write_read_fn(cmd, buf, scan_bits, type);

	free(buf);

	ch347_move_state(cmd->end_state, 1);

	return ERROR_OK;
}

/**
 * @brief Sleep for a specific timspan
 *
 * @param us Sleep time in microseconds
 */
static void ch347_sleep(int us)
{
	LOG_DEBUG_IO("us=%d", us);
	jtag_sleep(us);
}

/**
 * @brief Executes the command quene
 *
 * @return Success returns ERROR_OK
 */
static int ch347_execute_queue(void)
{
	struct jtag_command *cmd;
	int ret = ERROR_OK;

	ch347_set_activity_led(LED_ON);

	for (cmd = jtag_command_queue; ret == ERROR_OK && cmd;
		 cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RESET:
			LOG_DEBUG_IO("JTAG_RESET : %d %d",
					 cmd->cmd.reset->trst,
					 cmd->cmd.reset->srst);
			ch347_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			break;
		case JTAG_RUNTEST:
			ch347_run_test(cmd->cmd.runtest->num_cycles,
					  cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			ch347_stable_clocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			ch347_move_state(cmd->cmd.statemove->end_state, 0);
			break;
		case JTAG_PATHMOVE:
			ch347_move_path(cmd->cmd.pathmove);
			break;
		case JTAG_TMS:
			ch347_tms(cmd->cmd.tms);
			break;
		case JTAG_SLEEP:
			ch347_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			ret = ch347_scan(cmd->cmd.scan);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%X", cmd->type);
			ret = ERROR_FAIL;
			break;
		}
	}

	ch347_flush_buffer();
	ch347_set_activity_led(LED_OFF);
	return ret;
}

/**
 * @brief CH347 Initialization function
 *
 * @return Success returns 0, failure returns ERROR_FAIL
 */
static int ch347_init(void)
{
	dev_is_opened = ch347_open_device();

	if (!dev_is_opened) {
		LOG_ERROR("CH347 open error");
		return ERROR_FAIL;
	}

	LOG_DEBUG_IO("CH347 open success");

	if (!swd_mode) {
		// ch347 jtag init
		ch347.tck_pin = TCK_L;
		ch347.tms_pin = TMS_L;
		ch347.tdi_pin = TDI_L;
		ch347.trst_pin = TRST_H;
		ch347.buffer_idx = 0;

		memset(&ch347.buffer, 0, SF_PACKET_BUF_SIZE);
		ch347.len_idx = 0;
		ch347.len_value = 0;
		ch347.last_cmd = 0;

		memset(&ch347.read_buffer, 0, SF_PACKET_BUF_SIZE);
		ch347.read_count = 0;
		ch347.read_idx = 0;

		bit_copy_queue_init(&ch347.read_queue);

		tap_set_state(TAP_RESET);
	} else {
		ch347_swd_init_cmd(0);
	}
	return ERROR_OK;
}

/**
 * @brief CH347 Device Release Function
 *
 * @return always returns 0
 */
static int ch347_quit(void)
{
	// on close set the LED on, because the state without JTAG is on
	ch347_set_activity_led(LED_ON);

	int retlen = 4;
	uint8_t byte[4] = {CH347_CMD_JTAG_BIT_OP, 0x01, 0x00, ch347.trst_pin};
	if (!swd_mode) {
		ch347_write(byte, &retlen);
		bit_copy_discard(&ch347.read_queue);
	}
	if (dev_is_opened) {
		ch347_close_device();
		LOG_DEBUG_IO("CH347 close");
		dev_is_opened = false;
	}
	return 0;
}

/**
 * @brief Sends the CH347_CMD_JTAG_INIT (D0) command to get the jtag
 * interface initialized with the speed index
 *
 * @param clock_index Clock index
 * @return true if the device supports STANDARD_PACK mode;
 * false if the device supports LARGER_PACK mode
 */
static bool ch347_check_speed(uint8_t clock_index)
{
	int i = 0;
	uint8_t cmd_buf[32] = "";
	cmd_buf[i++] = CH347_CMD_JTAG_INIT;
	cmd_buf[i++] = 6;
	cmd_buf[i++] = 0;

	cmd_buf[i++] = 0;
	cmd_buf[i++] = clock_index;

	for (int j = 0; j < 4; j++)
		cmd_buf[i++] = ch347.tck_pin | ch347.tdi_pin | ch347.tms_pin | ch347.trst_pin;

	int length = i;
	if (!ch347_write_data(cmd_buf, &length) || length != i)
		return false;

	length = 4;
	memset(cmd_buf, 0, sizeof(cmd_buf));

	if (!ch347_read_data(cmd_buf, &length) || length != 4)
		return false;

	return cmd_buf[0] == CH347_CMD_JTAG_INIT && cmd_buf[CH347_CMD_HEADER] == 0;
}

/**
 * @brief Initializes the jtag interface
 *
 * @param clock_index Clock index
 * @return true if the device supports STANDARD_PACK mode;
 * false if the device supports LARGER_PACK mode
 */
static bool ch347_jtag_init(uint8_t clock_index)
{
	// when checking with speed index 9 we can see if the device supports STANDARD_PACK or LARGER_PACK mode
	ch347.pack_size = ch347_check_speed(0x09) ? STANDARD_PACK : LARGER_PACK;
	if (ch347.pack_size == STANDARD_PACK)
		return (clock_index - 2 < 0) ? ch347_check_speed(0) : ch347_check_speed(clock_index - 2);

	return ch347_check_speed(clock_index);
}

/**
 * @brief CH347 TCK frequency setting
 *
 * @param speed Frequency in Hz
 * @return Success returns ERROR_OK，failed returns FALSE
 */
static int ch347_speed(int speed)
{
	if (!swd_mode) {
		// needs to be -1 because we use i+1 in the loop
		int end = sizeof(ch347_clock_speeds) / sizeof(int) - 1;
		for (int i = 0; i < end; i++) {
			int ret_val = -1;
			if (speed >= ch347_clock_speeds[i] && speed <= ch347_clock_speeds[i + 1]) {
				ret_val = ch347_jtag_init(i + 1);
				if (!ret_val) {
					LOG_ERROR("Couldn't set CH347 TCK speed");
					return ret_val;
				}
				break;
			} else if (speed < ch347_clock_speeds[0]) {
				ret_val = ch347_jtag_init(0);
				if (!ret_val) {
					LOG_ERROR("Couldn't set CH347 TCK speed");
					return ret_val;
				}
				break;
			}
		}
	}
	return ERROR_OK;
}

/**
 * @brief divides the input speed by 1000
 *
 * @param speed Frequency size set
 * @param khz Output of the speed in kHz
 * @return Always ERROR_OK
 */
static int ch347_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;
}

/**
 * @brief multiplies the input speed by 1000
 *
 * @param khz Speed in kHz
 * @param jtag_speed Output frequency
 * @return ERROR_OK at success; ERROR_FAIL if khz is zero
 */
static int ch347_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		LOG_ERROR("Couldn't support the adapter speed");
		return ERROR_FAIL;
	}
	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

/**
 * @brief Sets the TRST pin
 *
 * @param status Pin status
 * @return ERROR_OK at success; ERROR_FAIL otherwise
 */
static int ch347_trst_out(uint8_t status)
{
	int byte_index = 0;
	uint8_t cmd_packet[4] = "";
	ch347.trst_pin = status ? TRST_H : TRST_L;
	cmd_packet[byte_index++] = CH347_CMD_JTAG_BIT_OP;
	cmd_packet[byte_index++] = 0x01;
	cmd_packet[byte_index++] = 0;
	uint8_t byte = ch347.tck_pin | ch347.tdi_pin | ch347.tms_pin | ch347.trst_pin;
	cmd_packet[byte_index++] = byte;

	if (!ch347_write(cmd_packet, &byte_index)) {
		LOG_ERROR("TRST set failure.");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/**
 * @brief The command handler for setting the device usb vid/pid
 *
 * @return ERROR_OK at success; ERROR_COMMAND_SYNTAX_ERROR otherwise
 */
COMMAND_HANDLER(ch347_handle_vid_pid_command)
{
	if (CMD_ARGC != 2) {
		LOG_ERROR("incomplete ch347 vid_pid configuration directive");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], ch347_vids[0]);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], ch347_pids[0]);

	return ERROR_OK;
}

/**
 * @brief The command handler for resetting the target device via trst pin
 *
 * @return Always ERROR_OK
 */
COMMAND_HANDLER(ch347_trst)
{
	ch347_trst_out(TRST_L);
	jtag_sleep(atoi(CMD_ARGV[0]) * 1000);
	ch347_trst_out(TRST_H);
	return ERROR_OK;
}

/**
 * @brief The command handler for setting the device description that should be found
 *
 * @return ERROR_OK at success; ERROR_COMMAND_SYNTAX_ERROR otherwise
 */
COMMAND_HANDLER(ch347_handle_device_desc_command)
{
	if (CMD_ARGC == 1) {
		if (ch347_device_desc)
			free(ch347_device_desc);
		ch347_device_desc = strdup(CMD_ARGV[0]);
	} else {
		LOG_ERROR("expected exactly one argument to ch347 device_desc <description>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

/**
 * @brief The command handler for configuring which GPIO pin is used as activity LED
 *
 * @return ERROR_OK at success; ERROR_COMMAND_SYNTAX_ERROR otherwise
 */
COMMAND_HANDLER(ch347_handle_activity_led_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint8_t gpio;
	if (CMD_ARGV[0][0] == 'n') {
		COMMAND_PARSE_NUMBER(u8, ++CMD_ARGV[0], gpio);
		ch347_activity_led_active_high = false;
		CMD_ARGV[0]--;
	} else {
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], gpio);
		ch347_activity_led_active_high = true;
	}

	if (gpio >= GPIO_CNT) {
		LOG_ERROR("activity_led out of range");
		return ERROR_COMMAND_SYNTAX_ERROR;
	} else if (((1 << gpio) & USEABLE_GPIOS) == 0) {
		LOG_ERROR("activity_led pin not in usable list");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	ch347_activity_led_gpio_pin = gpio;

	return ERROR_OK;
}

static const struct command_registration ch347_subcommand_handlers[] = {
	{
		.name = "vid_pid",
		.handler = &ch347_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of the CH347 device",
		.usage = "vid pid",
	},
	{
		.name = "jtag_ntrst_delay",
		.handler = &ch347_trst,
		.mode = COMMAND_ANY,
		.help = "resets via trst pin, parameter is the delay between low and high edge",
		.usage = "[milliseconds]",
	},
	{
		.name = "device_desc",
		.handler = &ch347_handle_device_desc_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB device description of the CH347 device",
		.usage = "description_string",
	},
	{
		.name = "activity_led",
		.handler = &ch347_handle_activity_led_command,
		.mode = COMMAND_CONFIG,
		.help = "if set this CH347 GPIO pin is the JTAG activity LED; start with n for active low output",
		.usage = "n4 or 4",
	},

	COMMAND_REGISTRATION_DONE};

static const struct command_registration ch347_command_handlers[] = {
	{
		.name = "ch347",
		.mode = COMMAND_ANY,
		.help = "perform ch347 management",
		.chain = ch347_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE};

/**
 * @brief Initialization for the swd mode
 *
 * @return Always ERROR_OK
 */
static int ch347_swd_init(void)
{
	LOG_INFO("CH347 SWD mode enabled");
	swd_mode = true;
	memset(&ch347_swd_context, 0, sizeof(ch347_swd_context));

	INIT_LIST_HEAD(&ch347_swd_context.send_cmd_head);
	INIT_LIST_HEAD(&ch347_swd_context.free_cmd_head);

	ch347_swd_context.queued_retval = ERROR_OK;
	// 0XE8 + 2byte len + N byte cmds
	ch347_swd_context.send_len = CH347_CMD_HEADER;
	// 0XE8 + 2byte len + N byte ack + data
	ch347_swd_context.need_recv_len = CH347_CMD_HEADER;
	struct ch347_swd_io *pswd_io = ch347_swd_context.ch347_cmd_buf;
	for (int i = 0; i < CH347_MAX_CMD_BUF; i++, pswd_io++) {
		INIT_LIST_HEAD(&pswd_io->list_entry);
		list_add_tail(&pswd_io->list_entry,	&ch347_swd_context.free_cmd_head);
	}
	return ERROR_OK;
}

static struct ch347_swd_io *ch347_get_one_swd_io(void)
{
	struct ch347_swd_io *pswd_io;
	if (list_empty(&ch347_swd_context.free_cmd_head))
		return NULL;

	pswd_io = list_first_entry(&ch347_swd_context.free_cmd_head,
				   struct ch347_swd_io, list_entry);
	list_del_init(&pswd_io->list_entry);
	pswd_io->cmd = 0;
	pswd_io->usb_cmd = CH347_CMD_SWD_SEQ_W;
	pswd_io->dst = NULL;
	return pswd_io;
}

static void ch347_swd_queue_flush(void)
{
	int length = ch347_swd_context.send_len;
	ch347_swd_context.send_buf[0] = (uint8_t)CH347_CMD_SWD;
	ch347_swd_context.send_buf[1] = (uint8_t)(ch347_swd_context.send_len - CH347_CMD_HEADER);
	ch347_swd_context.send_buf[2] = (uint8_t)((ch347_swd_context.send_len - CH347_CMD_HEADER) >> 8);
	if (!ch347_write_data(ch347_swd_context.send_buf, &length) || length != ch347_swd_context.send_len) {
		ch347_swd_context.queued_retval = ERROR_FAIL;
		LOG_DEBUG("CH347WriteData error");
		return;
	}
	ch347_swd_context.recv_len = 0;
	do {
		length = CH347_MAX_RECV_BUF - ch347_swd_context.recv_len;
		if (!ch347_read_data(&ch347_swd_context.recv_buf[ch347_swd_context.recv_len], &length)) {
			ch347_swd_context.queued_retval = ERROR_FAIL;
			LOG_DEBUG("CH347ReadData error");
			return;
		}
		ch347_swd_context.recv_len += length;
	} while (ch347_swd_context.recv_len < ch347_swd_context.need_recv_len);

	if (ch347_swd_context.need_recv_len > ch347_swd_context.recv_len) {
		LOG_ERROR("write/read failed %d %d",
			  ch347_swd_context.recv_len,
			  ch347_swd_context.need_recv_len);
	}
}

static void ch347_write_swd_reg(uint8_t cmd, const uint8_t *out, uint8_t parity)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = CH347_CMD_SWD_REG_W;
	// 8bit + 32bit +1bit
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x29;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = cmd;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out[0];
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out[1];
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out[2];
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out[3];
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = parity;
	// 0xA0 +  1 byte(3bit ACK)
	ch347_swd_context.need_recv_len += (1 + 1);
}

static void ch347_write_spec_seq(const uint8_t *out, uint8_t out_len)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] =
		CH347_CMD_SWD_SEQ_W;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out_len;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	for (uint8_t i = 0; i < DIV_ROUND_UP(out_len, 8); i++)
		ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out ? out[i] : 0x00;
	ch347_swd_context.need_recv_len += 1; /* 0xA1 */
}

static void ch347_read_swd_reg(uint8_t cmd)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = CH347_CMD_SWD_REG_R;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x22;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = cmd;
	// 0xA2 + 1 byte(3bit ACK) + 4 byte(data) + 1 byte(1bit parity+1bit trn)
	ch347_swd_context.need_recv_len += 1 + 1 + 4 + 1;
}

static int ch347_swd_switch_out(enum swd_special_seq seq, const uint8_t *out, unsigned int out_len)
{
	if ((ch347_swd_context.send_len + (1 + 2 + DIV_ROUND_UP(out_len, 8))) > CH347_MAX_SEND_BUF)
		return ERROR_FAIL;
	if ((ch347_swd_context.need_recv_len + 2) > CH347_MAX_RECV_BUF)
		return ERROR_FAIL;

	struct ch347_swd_io *pswd_io = ch347_get_one_swd_io();
	if (pswd_io) {
		ch347_write_spec_seq(out, out_len);
		list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);
		return ERROR_OK;
	} else {
		return ERROR_FAIL;
	}
}

// check read/write REG can fill in remaining buff
static bool ch347_chk_buf_size(uint8_t cmd, uint32_t ap_delay_clk)
{
	bool flush = false;
	int send_len = ch347_swd_context.send_len;
	int recv_len = ch347_swd_context.need_recv_len;
	int len;
	do {
		if (cmd & SWD_CMD_RNW) {
			len = 1 + 1 + 1 + 1; // 0xA2 + len + rev + cmd
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			send_len += len;
			len = 1 + 1 + 4 + 1;
			/* 0xA2 + 1byte(3bit ack) +  4byte(data) +
			   1byte(1bit parity+1bit trn) */
			if (recv_len + len > CH347_MAX_RECV_BUF)
				break;
			recv_len += len;
		} else {                         // write reg
			len = 1 + 1 + 1 + 1 + 4 + 1;
			// 0xA0 + len + rev  + cmd +data + parity
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			send_len += len;
			len = 1 + 1; // 0xA0 + 1byte(3bit ack)
			if (recv_len + len > CH347_MAX_RECV_BUF)
				break;
			recv_len += len;
		}
		if (cmd & SWD_CMD_APNDP) {
			len = 1 + 1 + 1 + DIV_ROUND_UP(ap_delay_clk, 8);
			// 0xA1 + Len  + rev  + n byte(delay)
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			len = 1; // 0xA1
			if ((recv_len + len) > CH347_MAX_RECV_BUF)
				break;
		}
		// swd packet requests
		flush = true;
	} while (false);

	return flush;
}

static void ch347_swd_send_idle(uint32_t ap_delay_clk)
{
	struct ch347_swd_io *pswd_io = ch347_get_one_swd_io();
	if (!pswd_io) {
		ch347_swd_run_queue();
		pswd_io = ch347_get_one_swd_io();
		if (!pswd_io) {
			LOG_DEBUG("ch347_swd_queue_cmd error");
			ch347_swd_context.queued_retval = ERROR_FAIL;
			return;
		}
	}
	ch347_write_spec_seq(NULL, ap_delay_clk);

	list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);
}

static int ch347_swd_run_queue(void)
{
	LOG_DEBUG_IO("Executing %u queued transactions", ch347_swd_context.sent_cmd_count);
	if (ch347_swd_context.queued_retval != ERROR_OK) {
		LOG_DEBUG_IO("Skipping due to previous errors: %d", ch347_swd_context.queued_retval);
		goto skip;
	}

	/* A transaction must be followed by another transaction or at least 8
	   idle cycles to ensure that data is clocked through the AP. */
	if ((ch347_swd_context.send_len + (1 + 2 + 1)) > CH347_MAX_SEND_BUF)
		goto skip_idle;

	if ((ch347_swd_context.need_recv_len + 1) > CH347_MAX_RECV_BUF)
		goto skip_idle;

	ch347_swd_send_idle((uint32_t)8);

skip_idle:

	ch347_swd_queue_flush();

	if (ch347_swd_context.queued_retval != ERROR_OK) {
		LOG_ERROR("CH347 usb write/read failed - queued_retval");
		goto skip;
	}
	uint8_t *recv_buf = ch347_swd_context.recv_buf;
	int recv_len = 0;
	if (recv_buf[recv_len++] != CH347_CMD_SWD) { // 0XE8
		ch347_swd_context.queued_retval = ERROR_FAIL;
		LOG_ERROR("CH347 usb write/read failed - not CH347_CMD_SWD");
		goto skip;
	}

	int cmds_len = le_to_h_u16(&recv_buf[recv_len]);
	recv_len += 2; // cmds_len
	if ((cmds_len + CH347_CMD_HEADER) > ch347_swd_context.recv_len) {
		ch347_swd_context.queued_retval = ERROR_FAIL;
		LOG_ERROR("CH347 usb write/read failed - too long");
		goto skip;
	}

	struct list_head *tmp;
	struct list_head *pos;
	struct ch347_swd_io *pswd_io;

	list_for_each_safe(pos, tmp, &ch347_swd_context.send_cmd_head) {
		pswd_io = list_entry(pos, struct ch347_swd_io, list_entry);
		if (pswd_io->usb_cmd == CH347_CMD_SWD_SEQ_W) {
			if (recv_buf[recv_len++] != CH347_CMD_SWD_SEQ_W) {
				ch347_swd_context.queued_retval = ERROR_FAIL;
				LOG_ERROR("CH347 usb write/read failed - not CH347_CMD_SWD_SEQ_W");
				goto skip;
			}
		} else { // read/write Reg
			int ack;
			bool check_ack;
			// read  Reg
			if (recv_buf[recv_len] == CH347_CMD_SWD_REG_R) {
				recv_len++;
				ack = buf_get_u32(&recv_buf[recv_len++], 0, 3);
				/* Devices do not reply to DP_TARGETSEL write
				   cmd, ignore received ack */
				check_ack = swd_cmd_returns_ack(pswd_io->cmd);
				if (ack != SWD_ACK_OK && check_ack) {
					ch347_swd_context.queued_retval = swd_ack_to_error_code(ack);
					LOG_ERROR("ack != SWD_ACK_OK");
					goto skip;
				}
				if (pswd_io->cmd & SWD_CMD_RNW) {
					int data = buf_get_u32(&recv_buf[recv_len], 0, 32);
					int parity = buf_get_u32(&recv_buf[recv_len], 32, 1);
					if (parity != parity_u32(data)) {
						LOG_ERROR("SWD Read data parity mismatch");
						ch347_swd_context.queued_retval = ERROR_FAIL;
						goto skip;
					}

					LOG_DEBUG_IO("%s%s %s %s reg %X = %08X - " PRIx32,
							 check_ack ? "" : "ack ignored ",
							 ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" :
								ack == SWD_ACK_FAULT  ? "FAULT" : "JUNK",
							 pswd_io->cmd & SWD_CMD_APNDP ? "AP" : "DP",
							 pswd_io->cmd & SWD_CMD_RNW ? "read" : "write",
							 (pswd_io->cmd & SWD_CMD_A32) >> 1,
							 data);

					if (pswd_io->dst)
						*pswd_io->dst = data;
				} else {
					ch347_swd_context.queued_retval = ERROR_FAIL;
					LOG_ERROR("CH347 usb write/read failed - not SWD_CMD_RNW");
					goto skip;
				}
				recv_len += 5;
			} else if (recv_buf[recv_len] == CH347_CMD_SWD_REG_W) {
				recv_len++;
				ack = buf_get_u32(&recv_buf[recv_len++], 0, 3);
				/* Devices do not reply to DP_TARGETSEL write
				   cmd, ignore received ack */
				check_ack = swd_cmd_returns_ack(pswd_io->cmd);
				if (ack != SWD_ACK_OK && check_ack) {
					ch347_swd_context.queued_retval = swd_ack_to_error_code(ack);
					LOG_ERROR("SWD Read data parity mismatch");
					goto skip;
				}
				LOG_DEBUG_IO("%s%s %s %s reg %X = %08X - " PRIx32,
						 check_ack ? "" : "ack ignored ",
						 ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" :
							ack == SWD_ACK_FAULT  ? "FAULT" : "JUNK",
						 pswd_io->cmd & SWD_CMD_APNDP ? "AP" : "DP",
						 pswd_io->cmd & SWD_CMD_RNW ? "read" : "write",
						 (pswd_io->cmd & SWD_CMD_A32) >> 1,
						 pswd_io->value);
			} else {
				ch347_swd_context.queued_retval = ERROR_FAIL;
				LOG_ERROR("CH347 usb write/read failed recv_len = %d", recv_len);
				goto skip;
			}
		}
		list_del_init(&pswd_io->list_entry);
		list_add_tail(&pswd_io->list_entry,
				  &ch347_swd_context.free_cmd_head);
	}

skip:
	if (!list_empty(&ch347_swd_context.send_cmd_head)) {
		list_for_each_safe(pos, tmp, &ch347_swd_context.send_cmd_head) {
			pswd_io = list_entry(pos, struct ch347_swd_io, list_entry);
			list_del_init(&pswd_io->list_entry);
			list_add_tail(&pswd_io->list_entry,
					  &ch347_swd_context.free_cmd_head);
		}
	}
	// 0xE8 + 2byte len
	ch347_swd_context.send_len = CH347_CMD_HEADER;
	// 0xE8 + 2byte len
	ch347_swd_context.need_recv_len = CH347_CMD_HEADER;
	ch347_swd_context.recv_len = 0;
	ch347_swd_context.sent_cmd_count = 0;
	int retval = ch347_swd_context.queued_retval;
	ch347_swd_context.queued_retval = ERROR_OK;

	return retval;
}

static void ch347_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk)
{
	if (ap_delay_clk > 255)
		LOG_DEBUG("ap_delay_clk = %d", ap_delay_clk);

	if (ch347_swd_context.sent_cmd_count >= CH347_MAX_SEND_CMD)
		ch347_swd_run_queue();

	if (!ch347_chk_buf_size(cmd, ap_delay_clk))
		ch347_swd_run_queue();

	struct ch347_swd_io *pswd_io = ch347_get_one_swd_io();
	if (!pswd_io) {
		ch347_swd_run_queue();
		pswd_io = ch347_get_one_swd_io();
		if (!pswd_io) {
			LOG_DEBUG("Error");
			ch347_swd_context.queued_retval = ERROR_FAIL;
			return;
		}
	}

	pswd_io->cmd = cmd | SWD_CMD_START | SWD_CMD_PARK;

	if (pswd_io->cmd & SWD_CMD_RNW) {
		pswd_io->usb_cmd = CH347_CMD_SWD_REG_R;
		pswd_io->dst = dst;
		ch347_read_swd_reg(pswd_io->cmd);
	} else {
		pswd_io->usb_cmd = CH347_CMD_SWD_REG_W;
		pswd_io->value = data;
		ch347_write_swd_reg(pswd_io->cmd, (uint8_t *)&data,	parity_u32(data));
	}

	ch347_swd_context.sent_cmd_count++;
	list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);
	// Insert idle cycles after AP accesses to avoid WAIT
	if (cmd & SWD_CMD_APNDP) {
		if (ap_delay_clk == 0)
			LOG_DEBUG("ap_delay_clk == 0");
		ch347_swd_send_idle(ap_delay_clk);
	}
}

static int ch347_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		return ch347_swd_switch_out(seq, swd_seq_line_reset, swd_seq_line_reset_len);
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		return ch347_swd_switch_out(seq, swd_seq_jtag_to_swd, swd_seq_jtag_to_swd_len);
	case JTAG_TO_DORMANT:
		LOG_DEBUG("JTAG-to-DORMANT");
		return ch347_swd_switch_out(seq, swd_seq_jtag_to_dormant, swd_seq_jtag_to_dormant_len);
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		return ch347_swd_switch_out(seq, swd_seq_swd_to_jtag, swd_seq_swd_to_jtag_len);
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		return ch347_swd_switch_out(seq, swd_seq_swd_to_dormant, swd_seq_swd_to_dormant_len);
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		return ch347_swd_switch_out(seq, swd_seq_dormant_to_swd, swd_seq_dormant_to_swd_len);
	case DORMANT_TO_JTAG:
		LOG_DEBUG("DORMANT-to-JTAG");
		return ch347_swd_switch_out(seq, swd_seq_dormant_to_jtag, swd_seq_dormant_to_jtag_len);
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}
}

static void ch347_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RNW);
	ch347_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

static void ch347_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RNW));
	ch347_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static const struct swd_driver ch347_swd = {
	.init = ch347_swd_init,
	.switch_seq = ch347_swd_switch_seq,
	.read_reg = ch347_swd_read_reg,
	.write_reg = ch347_swd_write_reg,
	.run = ch347_swd_run_queue,
};

static const char *const ch347_transports[] = {"jtag", "swd", NULL};

static struct jtag_interface ch347_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = ch347_execute_queue,
};

struct adapter_driver ch347_adapter_driver = {
	.name = "ch347",
	.transports = ch347_transports,
	.commands = ch347_command_handlers,

	.init = ch347_init,
	.quit = ch347_quit,
	.reset = ch347_reset,
	.speed = ch347_speed,
	.khz = ch347_khz,
	.speed_div = ch347_speed_div,

	.jtag_ops = &ch347_interface,
	.swd_ops = &ch347_swd,
};
