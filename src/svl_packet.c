#include "svl_packet.h"
#include <stdint.h>
#include <stddef.h>

#include <am_mcu_apollo.h>

// CRC-16/UMTS
// width=16 poly=0x8005 init=0x0000 refin=false refout=false xorout=0x0000
//   check=0xfee8 residue=0x0000 name="CRC-16/UMTS"
// https://reveng.sourceforge.io/crc-catalogue/16.htm
static const uint16_t CRC_Table[8 * 32] = {
	0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
	0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
	0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
	0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
	0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
	0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
	0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
	0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
	0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
	0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
	0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
	0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
	0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
	0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
	0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
	0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
	0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
	0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
	0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
	0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
	0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
	0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
	0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
	0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
	0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
	0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
	0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

//Update CRC with given byte
static uint16_t updateCRC(uint16_t crc, uint8_t num)
{
	uint8_t CRCH = crc >> 8;
	uint8_t CRCL = crc & 0xFF;
	uint16_t idx = (num ^ CRCH);
	return (((CRC_Table[idx] >> 8) ^ CRCL) << 8) | (CRC_Table[idx] & 0xFF);
}

static svl_packet_driver_t driver;

static size_t svl_packet_read(uint8_t *c, size_t amount)
{
	size_t retval = 0x00;
	if (driver.read)
	{
		retval = driver.read(driver.param, c, amount);
	}
	return retval;
}

static size_t svl_packet_write(const uint8_t *c, size_t amount)
{
	size_t retval = 0x00;
	if (driver.write)
	{
		retval = driver.write(driver.param, c, amount);
	}
	return retval;
}

static size_t svl_packet_millis(void)
{
	size_t retval = 0x00;
	if (driver.millis)
	{
		retval = driver.millis();
	}
	return retval;
}

void svl_packet_driver_register(const svl_packet_driver_t *driver_)
{
	driver = *driver_;
}

// Doesn't return until all bytes have been at least queued for TX by the
// underlying function.
static size_t svl_packet_write_all(const uint8_t *buffer, size_t size)
{
	size_t write = svl_packet_write(buffer, size);
	while (write < size)
	{
		write += svl_packet_write(buffer + write, size - write);
	}
	return write;
}

void svl_packet_send(const svl_packet_t *packet)
{
	uint16_t crc = updateCRC(0, packet->cmd); //Add this byte to CRC
	for (uint32_t x = 0; x < packet->pl_len; x++)
	{
		crc = updateCRC(crc, packet->pl[x]); //Add this byte to CRC
	}

	// Send the len in network byte order. Includes command and CRC bytes.
	// command byte
	const uint8_t front[3] = {
		(packet->pl_len + 3) >> 8,
		(packet->pl_len + 3) & 0xFF,
		packet->cmd
	};
	svl_packet_write_all(front, sizeof(front));

	// payload
	if ((packet->pl != NULL) && (packet->pl_len != 0))
	{
		svl_packet_write_all(packet->pl, packet->pl_len);
	}

	// Send the CRC in network byte order
	const uint8_t back[2] = {
		crc >> 8,
		crc & 0xFF
	};
	svl_packet_write_all(back, sizeof(back));
}

void svl_packet_send_command(uint8_t command)
{
	uint16_t crc = updateCRC(0, command); //Add this byte to CRC
	uint8_t output[5] = {
		0, 3, command, crc >> 8, crc & 0xFF
	};
	svl_packet_write_all(output, sizeof(output));
}

static size_t svl_packet_read_timeout(uint8_t *buffer, size_t size, uint32_t timeout_ms)
{
	uint32_t start = svl_packet_millis();
	size_t read = svl_packet_read(buffer, size);
	while (read < size && (svl_packet_millis() < (timeout_ms + start)))
	{
		read += svl_packet_read(buffer + read, size - read);
	}
	return read;
}

uint8_t svl_packet_wait(svl_packet_t *packet)
{
	// wait for 2 bytes (the length bytes)
	// wait for length bytes to come in
	// make sure that 'length' bytes are enough to satisfy the desired payload length

	if (packet == NULL)
	{
		return (SVL_PACKET_ERR);
	}

	uint8_t len_buffer[2] = {0};
	size_t len_read = svl_packet_read_timeout(len_buffer, sizeof(len_buffer), 500);

	if (len_read < 2)
	{
		return (SVL_PACKET_ERR_TIMEOUT | SVL_PACKET_LEN);
	}

	// Transmitted in network byte order
	uint16_t len = len_buffer[0] << 8 | len_buffer[1];

	if (len == 0)
	{
		return (SVL_PACKET_ERR_ZLP);
	}
	if ((len - 3) > packet->max_pl_len)
	{
		return (SVL_PACKET_ERR_MEM | SVL_PACKET_PL);
	}

	// Get the CMD byte
	uint8_t cmd = 0;
	if (!svl_packet_read_timeout(&cmd, 1, 500))
		return (SVL_PACKET_ERR_TIMEOUT | SVL_PACKET_PL);

	packet->cmd = cmd;
	uint16_t crc = updateCRC(0, cmd); //Add this byte to CRC

	packet->pl_len = (len - 3);

	// Now read the data coming in
	if ((packet->pl != NULL) && (packet->max_pl_len != 0))
	{
		size_t payload_len = svl_packet_read_timeout(packet->pl, packet->pl_len, 500);
		if (payload_len != packet->pl_len)
			return (SVL_PACKET_ERR_TIMEOUT | SVL_PACKET_PL);

		for (uint32_t x = 0; x < packet->pl_len; x++)
		{
			crc = updateCRC(crc, packet->pl[x]); //Add this byte to CRC
		}
	}

	// and get the CRC bytes
	uint8_t crc_buffer[2] = {0};
	size_t crc_len =
		svl_packet_read_timeout(crc_buffer, sizeof(crc_buffer), 500);

	if (crc_len != 2)
		return (SVL_PACKET_ERR_TIMEOUT | SVL_PACKET_PL);
	uint16_t remote_crc = crc_buffer[0] << 8 | crc_buffer[1];

	if (remote_crc != crc)
	{
		return (SVL_PACKET_ERR_CRC);
	}

	return (SVL_PACKET_OK);
}
