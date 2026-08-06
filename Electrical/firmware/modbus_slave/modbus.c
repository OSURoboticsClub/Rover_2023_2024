/*
 * modbus.c
 *
 * Created:  2/3/2022 3:38:07 PM
 *  Authors: Anthony Grana,
			 Blake Hakkila,
			 Kurtis Dinelle,
			 Matthew Beitler
 */ 
#include <stdbool.h>
#include <modbus.h>
#include "modbus_interface.h"

#define FC_WRITE_MULT	0x10			// write multiple registers function code
#define FC_READ_MULT	0x03			// read multiple registers function code


#define INT_REG_OFFSET		0			// Offset for translating uint16_t array index to register index
#define FLOAT_REG_OFFSET	256			// Offset for translating float array index to register index
#define CHAR_REG_OFFSET		512			// Offset for translating char array index to register index
#define BOOL_REG_OFFSET		768			// Offset for translating bool array index to register index

#define INT_REG_BYTE_SZ		2			// Number of bytes for an int register
#define FLOAT_REG_BYTE_SZ	4			// Number of bytes for a float register
#define CHAR_REG_BYTE_SZ	1			// Number of bytes for a char register
#define BOOL_REG_BYTE_SZ	1			// Number of bytes for a bool register

#define SLAVE_ID_IDX		0			// packet byte index for slave ID byte
#define FC_IDX				1			// packet byte index for Function Code byte
#define START_REG_H_IDX		2			// packet byte index for the high side of the start register number
#define START_REG_L_IDX		3			// packet byte index for the low side of the start register number
#define NUM_REG_H_IDX		4			// packet byte index for the high side of the end register number
#define NUM_REG_L_IDX		5			// packet byte index for the low side of the end register number
#define WR_DATA_SIZE_IDX	6			// packet byte index for the size of the data to follow in bytes (write multiple only)
#define RD_DATA_SIZE_IDX	2			// packet byte index for the size of the data to follow in bytes (read multiple only)

#define WR_DATA_BYTE_START	7			// packet byte index for the start of the data to be written (write multiple only)
#define RD_DATA_BYTE_START	3			// packet byte index for start of data in read response (read multiple only

#define WR_RESP_PACKET_SIZE	8			// packet size for write multiple response packet
#define RD_RESP_PACKET_MIN_SIZE	5		// packet size for read multiple response with no data bytes added yet

#define ABS_MIN_PACKET_SIZE 7			// this is the smallest possible packet size in the protocol in bytes
#define ABS_MIN_WRITE_PACKET_SIZE 10	// this is the smallest possible packet size for a write command from the master
#define WRITE_RES_PACKET_SIZE 8			// this is the only possible packet size for a write response from the slave This is the same as the size of read from master
#define ABS_MIN_READ_RES_PACKET_SIZE 6  // this is the smallest possible packet size for a read response from the slave

#define CRC_SIZE 2						// size of CRC in bytes

#define MASTER_ADRESS 0x00

static uint8_t	slaveID; // The assigned ID of this node

static uint16_t	packetSize;			// The size of the last received packet
static uint32_t lastComplete = 0;	// The time that a last complete packet was received
				
uint8_t responsePacket[TX_BUFFER_SIZE];	// The outgoing response packet
uint16_t responsePacketSize;			// The size of outgoing response packet

struct ringBuffer rxBuffer; // Holds all incoming serial data awaiting to be processed

// The registers where final data is sent after being processed in a packet
uint16_t	intRegisters[REGISTER_AR_SIZE];
float		floatRegisters[REGISTER_AR_SIZE];
char		charRegisters[REGISTER_AR_SIZE];
bool		boolRegisters[REGISTER_AR_SIZE];


/* All convert functions assume they are receiving an array of 4 unsigned values in
 * big-endian format. */
#define MERGE_FOUR_BYTES(x) \
    (((x)[0] << 24) | ((x)[1] << 16) | ((x)[2] << 8) | (x)[3])

#define convert_to_int(x)	((uint16_t)(((x)[0] << 8) | (x)[1]))
#define convert_to_char(x)	((char)(x)[0])
#define convert_to_bool(x)	((bool)(x)[0])

// Converts 4 bytes to a float
static float convert_to_float(const uint8_t data[4]) {
    union {
        uint32_t data;
        float data_f;
    } u;

    u.data = MERGE_FOUR_BYTES(data);
    return u.data_f;
}

// Convert a float into an array of 4 bytes
static uint8_t* float_to_bytes(float f) {
	union {
		uint32_t data;
		float data_f;
	} u;

	static uint8_t floatCoversionBytes[FLOAT_REG_BYTE_SZ];

	u.data_f = f;
	floatCoversionBytes[0] = (u.data >> 24) & 0xFF;
	floatCoversionBytes[1] = (u.data >> 16) & 0xFF;
	floatCoversionBytes[2] = (u.data >> 8) & 0xFF;
	floatCoversionBytes[3] = u.data & 0xFF;
	
	return floatCoversionBytes;
}

// Pops a completed packet from the serial buffer
static uint8_t* pop_packet(void) {
	static uint8_t returnPacket[RX_BUFFER_SIZE];
	
	//copy packet data to return array
	for(int i = 0; i < packetSize; i++) {
		returnPacket[i] = rxBuffer.data[rxBuffer.tail];
		rxBuffer.tail = PKT_WRAP_ARND(rxBuffer.tail + 1);
	}
	
	return returnPacket;
}

// Pops data from the serial buffer up to the next valid function code
static void pop_to_fc(void) {
	uint16_t FCLoc = PKT_WRAP_ARND(rxBuffer.tail + FC_IDX + 1);
	uint8_t checkFCByte = rxBuffer.data[FCLoc];
	
	while (checkFCByte != FC_READ_MULT && checkFCByte != FC_WRITE_MULT && FCLoc != rxBuffer.head) {
		FCLoc = PKT_WRAP_ARND(FCLoc + 1);
		checkFCByte = rxBuffer.data[FCLoc];
	}
	
	if (PKT_WRAP_ARND(FCLoc - 1) >= rxBuffer.tail) {
		packetSize = PKT_WRAP_ARND(FCLoc - 1) - rxBuffer.tail;
	} else {
		packetSize = (RX_BUFFER_SIZE - rxBuffer.tail) + PKT_WRAP_ARND(FCLoc - 1);
	}

	pop_packet();
}

// Calculates the CRC of a packet
static uint16_t calculate_crc(uint8_t* buf, int len) {
	uint16_t crc = 0xFFFF;

	for (int pos = 0; pos < len; pos++) {
		crc ^= (uint8_t)buf[pos];          // XOR byte into least sig. byte of crc

		for (int i = 8; i != 0; i--) {    // Loop over each bit
			if ((crc & 0x0001) != 0) {      // If the LSB is set
				crc >>= 1;                    // Shift right and XOR 0xA001
				crc ^= 0xA001;
			} else {                           // Else LSB is not set
				crc >>= 1; 
			}                   // Just shift right
		}
	}
	
	return crc;
}

// Builds a response packet to a read request
static void read_handler(uint8_t* packet, uint16_t start_reg, uint16_t end_reg) {
	int i = start_reg;
	
	while (i < REGISTER_AR_SIZE + INT_REG_OFFSET && i < end_reg) {
		uint16_t data = intRegisters[i - INT_REG_OFFSET];
		packet[0] = (data >> 8) & 0xFF;
		packet[1] = data & 0xFF;
		packet += INT_REG_BYTE_SZ;
		i++;
	}
	
	while (i < REGISTER_AR_SIZE + FLOAT_REG_OFFSET && i < end_reg) {
		uint8_t* floatConversionBytes = float_to_bytes(floatRegisters[i - FLOAT_REG_OFFSET]);
		for (int j = 0; j < FLOAT_REG_BYTE_SZ; j++) {
			packet[j] = floatConversionBytes[j];
		}
		packet += FLOAT_REG_BYTE_SZ;
		i++;
	}
	
	while (i < REGISTER_AR_SIZE + CHAR_REG_OFFSET && i < end_reg) {
		packet[0] = charRegisters[i - CHAR_REG_OFFSET];
		packet += CHAR_REG_BYTE_SZ;
		i++;
	}
	
	while (i < REGISTER_AR_SIZE + BOOL_REG_OFFSET && i < end_reg) {
		packet[0] = boolRegisters[i - BOOL_REG_OFFSET];
		packet += BOOL_REG_BYTE_SZ;
		i++;
	}
}

// Builds a response packet to a write request
static void write_handler(uint8_t* data_packet, uint16_t start_reg, uint16_t end_reg) {
	int i = start_reg;
	
	while (i < REGISTER_AR_SIZE + INT_REG_OFFSET && i < end_reg) {
		intRegisters[i - INT_REG_OFFSET] = convert_to_int(data_packet);
		data_packet += INT_REG_BYTE_SZ;
		i++;
	}
	
	while (i < REGISTER_AR_SIZE + FLOAT_REG_OFFSET && i < end_reg) {
		floatRegisters[i - FLOAT_REG_OFFSET] = convert_to_float(data_packet);
		data_packet += FLOAT_REG_BYTE_SZ;
		i++;
	}
	
	while (i < REGISTER_AR_SIZE + CHAR_REG_OFFSET && i < end_reg) {
		charRegisters[i - CHAR_REG_OFFSET] = convert_to_char(data_packet);
		data_packet += CHAR_REG_BYTE_SZ;
		i++;
	}
	
	while (i < REGISTER_AR_SIZE+BOOL_REG_OFFSET && i < end_reg) {
		boolRegisters[i - BOOL_REG_OFFSET] = convert_to_bool(data_packet);
		data_packet += BOOL_REG_BYTE_SZ;
		i++;
	}
}

// Calculates the size in bytes of the data in a read response
static uint16_t get_read_response_data_sz(uint16_t start_reg, uint16_t end_reg) {
	uint16_t size = 0;

	if (start_reg < REGISTER_AR_SIZE + INT_REG_OFFSET) {
		if (end_reg > REGISTER_AR_SIZE + INT_REG_OFFSET) {
			size += (REGISTER_AR_SIZE + INT_REG_OFFSET - start_reg) * INT_REG_BYTE_SZ;
			start_reg = REGISTER_AR_SIZE + INT_REG_OFFSET;
		} else {
			size += (end_reg - start_reg) * INT_REG_BYTE_SZ;
			return size;
		}
	}
	
	if (start_reg < REGISTER_AR_SIZE + FLOAT_REG_OFFSET) {
		if (end_reg > REGISTER_AR_SIZE + FLOAT_REG_OFFSET) {
			size += (REGISTER_AR_SIZE + FLOAT_REG_OFFSET - start_reg) * FLOAT_REG_BYTE_SZ;
			start_reg = REGISTER_AR_SIZE + FLOAT_REG_OFFSET;
		} else {
			size += (end_reg - start_reg) * FLOAT_REG_BYTE_SZ;
			return size;
		}
	}
	
	if (start_reg < REGISTER_AR_SIZE + CHAR_REG_OFFSET) {
		if (end_reg > REGISTER_AR_SIZE + CHAR_REG_OFFSET) {
			size += (REGISTER_AR_SIZE + CHAR_REG_OFFSET - start_reg) * CHAR_REG_BYTE_SZ;
			start_reg = REGISTER_AR_SIZE + CHAR_REG_OFFSET;
		} else {
			size += (end_reg - start_reg) * CHAR_REG_BYTE_SZ;
			return size;
		}
	}
	
	if (start_reg < REGISTER_AR_SIZE + BOOL_REG_OFFSET) {
		size += (end_reg - start_reg) * BOOL_REG_BYTE_SZ;
		return size;
	}
		
	return size;	
}

// Gets the number of bytes in the serial buffer
static uint16_t buffer_get_data_sz(void) {
	if (rxBuffer.head >= rxBuffer.tail) {
		return rxBuffer.head - rxBuffer.tail;
	} else {
		return (RX_BUFFER_SIZE - rxBuffer.tail) + rxBuffer.head;
	}
}

// Checks if a completed, valid packet is in the serial buffer
static bool packet_complete(void) {
	packetSize = 0;	// Reset this in case packet is not complete
	
	uint8_t slave_id = rxBuffer.data[PKT_WRAP_ARND(rxBuffer.tail + SLAVE_ID_IDX)];
	uint8_t func_code = rxBuffer.data[PKT_WRAP_ARND(rxBuffer.tail + FC_IDX)];
	uint8_t start_reg_hi = rxBuffer.data[PKT_WRAP_ARND(rxBuffer.tail + START_REG_H_IDX)];
	uint8_t start_reg_lo = rxBuffer.data[PKT_WRAP_ARND(rxBuffer.tail + START_REG_L_IDX)];
	uint8_t num_reg_hi = rxBuffer.data[PKT_WRAP_ARND(rxBuffer.tail + NUM_REG_H_IDX)];
	uint8_t num_reg_lo = rxBuffer.data[PKT_WRAP_ARND(rxBuffer.tail + NUM_REG_L_IDX)];
	uint8_t num_data_bytes = rxBuffer.data[PKT_WRAP_ARND(rxBuffer.tail + WR_DATA_SIZE_IDX)];

	// if the function code isn't write or read or start register/number of registers high bytes are too big, we know somethings fucked up
	// also we may as well skip if the slave id doesn't match
	if ((func_code != FC_WRITE_MULT && func_code != FC_READ_MULT) || start_reg_hi >= 4 || num_reg_hi >= 4 || slave_id != slaveID) {
		pop_to_fc(); // Pops out any garbage preceding the next valid function code
		return false;
	}
	
	uint16_t num_bytes_check_start = start_reg_hi << 8 | start_reg_lo;
	uint16_t num_bytes_check_end = num_bytes_check_start + (num_reg_hi << 8 | num_reg_lo);
	
	// lets make sure the number of data bytes is correct
	if (func_code == FC_WRITE_MULT && num_data_bytes != get_read_response_data_sz(num_bytes_check_start, num_bytes_check_end)) {
		pop_to_fc();
		return false;
	}
	
	num_data_bytes = 0;	// Default 0 for packets with no data bytes
	uint16_t base_pkt_sz; // size of packet not including data bytes
	
	// Handle write command from master
	if (func_code == FC_WRITE_MULT) {
		if (buffer_get_data_sz() < ABS_MIN_WRITE_PACKET_SIZE) {
			return false; //if the data size is less than this, we know the packet is incomplete
		}
		
		num_data_bytes = rxBuffer.data[PKT_WRAP_ARND(rxBuffer.tail + WR_DATA_SIZE_IDX)]; //get supposed number of data bytes from the packet
		base_pkt_sz = ABS_MIN_WRITE_PACKET_SIZE - 1;
	}
	
	// Handle write response from slave or read command from master (identical packet structure)
	else {
		if (buffer_get_data_sz() < WRITE_RES_PACKET_SIZE) {
			return false; // if the data size is less than this, we know the packet is incomplete
		}
		
		base_pkt_sz = WRITE_RES_PACKET_SIZE; // we know the final packet size
	}
	
	uint16_t full_pkt_sz = num_data_bytes + base_pkt_sz;
	
	if (buffer_get_data_sz() < full_pkt_sz) {
		return false;
	}
	
	packetSize = full_pkt_sz; // Set global packetSize to completed packet size
	
	// pull packet into linear buffer for crc check
	uint8_t packetNoCRC[packetSize - CRC_SIZE];
	for (int i = 0; i < packetSize - CRC_SIZE; i++) {
		packetNoCRC[i] = rxBuffer.data[PKT_WRAP_ARND(rxBuffer.tail + i)];
	}
	
	// pull out the crc from the packet
	uint8_t packetCRC[CRC_SIZE];
	for (int i = 0; i < CRC_SIZE; i++){
		packetCRC[i] = rxBuffer.data[PKT_WRAP_ARND(rxBuffer.tail + (packetSize - CRC_SIZE) + i)];
	}
	
	uint16_t expectedCRC = calculate_crc(packetNoCRC, packetSize - CRC_SIZE);
	
	if (((expectedCRC >> 8) & 0xFF) == packetCRC[1] && (expectedCRC & 0xFF) == packetCRC[0]) {
		return true; // packet is complete and passes crc
	} else { // on crc fail remove first byte from buffer This is the only known incorrect byte
		packetSize = 1;
		pop_packet();
		return false;
	}
	
}

// Resets the time a completed packet was last received
static void reset_timeout(void) {
	lastComplete = get_elapsed_ms();
}

// Initiate the modbus slave library
void modbus_slave_init(const uint8_t slave_id) {
	slaveID = slave_id;
	rxBuffer.head = 0;
	rxBuffer.tail = 0;
}

// Does the heavy lifting of the modbus slave library
void modbus_slave_update(void) {
	// if not enough data has been received just break out
	// check if an entire packet has been received otherwise return, also resolves overflow errors
	if (buffer_get_data_sz() < ABS_MIN_PACKET_SIZE || !packet_complete()) {
		return;
	}
	
	// packet is complete, so pull it out
	uint8_t* packet = pop_packet();
	if (packet[SLAVE_ID_IDX] != slaveID) {
		return;						//disregard if the packet doesn't apply to this slave
	}
	
	// extract register info from packet
	uint16_t start_reg = packet[START_REG_H_IDX] << 8 | packet[START_REG_L_IDX];
	uint16_t num_registers = packet[NUM_REG_H_IDX] << 8 | packet[NUM_REG_L_IDX];
	int end_reg = start_reg + num_registers; // this register number is exclusive, so all valid register numbers are less than end_reg
	
	// call read and write handlers based on function code
	switch(packet[FC_IDX]) {
		case FC_READ_MULT:
		{
			uint16_t read_num_bytes = get_read_response_data_sz(start_reg, end_reg);
			responsePacketSize = RD_RESP_PACKET_MIN_SIZE + read_num_bytes;
			responsePacket[SLAVE_ID_IDX] = slaveID;
			responsePacket[FC_IDX] = packet[FC_IDX];
			responsePacket[RD_DATA_SIZE_IDX] = read_num_bytes;
			read_handler(responsePacket+RD_DATA_BYTE_START, start_reg, end_reg);
			break;
		}
		case FC_WRITE_MULT:
		{
			responsePacketSize = WR_RESP_PACKET_SIZE;
			responsePacket[SLAVE_ID_IDX] = slaveID;	
			responsePacket[FC_IDX] = packet[FC_IDX];
			responsePacket[START_REG_H_IDX] = packet[START_REG_H_IDX];
			responsePacket[START_REG_L_IDX] = packet[START_REG_L_IDX];
			responsePacket[NUM_REG_H_IDX] = packet[NUM_REG_H_IDX];
			responsePacket[NUM_REG_L_IDX] = packet[NUM_REG_L_IDX];
			write_handler(&packet[WR_DATA_BYTE_START], start_reg, end_reg);
			break;
		}
	}
	
	uint16_t responceCRC = calculate_crc(responsePacket, responsePacketSize - CRC_SIZE);
	
	responsePacket[responsePacketSize - 2] = responceCRC & 0xFF;
	responsePacket[responsePacketSize - 1] = (responceCRC>>8) & 0xFF;
	
	// write out response packet
	serial_port_write(responsePacket, responsePacketSize);

	reset_timeout();
}

// Checks if a packet was last received before a timeout occurred
bool modbus_slave_comm_good() {
	return !(get_elapsed_ms() - lastComplete > timeout);
}
