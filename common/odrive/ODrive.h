// A class which handles USB/UART communications with an ODrive motor controller.
// It gets the JSON interface definition from the ODrive and populates a root
// Endpoint class. The Endpoint class abstracts all the send and recieve 
// calls, such that assigning a value to an endpoint name within the root
// Endpoint, will communicate that to the ODrive. Assigning a variable with
// an endpoint name within the roon Endpoint, will get the appropriate value
// from the ODrive. All types have to be explicitly stated.
//
// Uses libusb for USB interface
// Uses the nlohmann JSON library 

// Here is a small usage example:
//
// ODrive odrive;
// odrive.connect_usb();
// 
// float bus_voltage = odrive.root("vbus_voltage").get2<float>(); // This gets the 'vbus_voltage' paramater from the ODrive and assigns it to bus_voltage
// Endpoint& axis0 = odrive.root("axis0");  
// float motor0_position;
// axis0("controller")("input_pos").get(motor0_position); // This gets the 'input_pos' paramater of the first axis from the ODrive
// axis0("controller")("input_pos").set(20.0f); // This sets the 'input_pos' paramater of the first axis

// The original code is from: https://github.com/tokol0sh/Odrive_USB
// and was modified to also handle UART, be more reliable, handle function calls
// and be easier to use. The code is still a bit messy though.
// It should work with all firmware versions >= 0.5.1 on ODrive 3. I haven't tested it on ODrive Pro/S1
// but it should work there with minimal changes too.
// If you copy the files in this folder into your project, this should work with minimal changes.

#pragma once
#include "endpoint.h"
#include <string>
#include <iostream>
#include <vector>

// If you don't need USB or UART support, you can adjust these defines.
// Right now UART won't work on Windows.
#define ODRIVE_INCLUDE_USB
#ifndef _MSC_VER
#define ODRIVE_INCLUDE_UART
#endif

#ifdef ODRIVE_INCLUDE_USB
struct libusb_context;
struct libusb_device_handle;
#endif

typedef std::vector<u8> serial_buffer;
typedef u8* serial_buffer_iterator;
//typedef serial_buffer::iterator serial_buffer_iterator;

inline serial_buffer_iterator get_it(serial_buffer& buf)
{
	return buf.data();
}

class ODrive
{
public:
	bool connect_uart(const char* uart_address, int baud_rate, bool stop_bits_2);

	// Right know we assume that there is at most one ODrive connected via USB, so there
	// is no parameter to specify which ODrive to connect to yet.
	bool connect_usb();
	void close();

public:
	Endpoint root;
	bool is_connected = false;
	bool communication_error = false;
	int endpoint_request_counter = 0;
	
	ODriveVersion odrive_fw_version;
	bool odrive_fw_is_milana;

public:
	// The following functions shouldn't be used directly. They are only public
	// because the Endpoint class needs them.
	// This needs to be cleaned up...
	void endpoint_request(int endpoint_id, serial_buffer& received_payload, const serial_buffer& payload, bool ack, int length, bool length_must_match=true);

	void call(int id);
	void call(int id, int in1, float* out1);
	void call(int id, int in1, u64* out1);
	void call(int id, bool* out1);
	
	template<typename T>
	void set_value(int id, const T& value)
	{
		serial_buffer send_payload;
		send_payload.reserve(sizeof(T));
		serial_buffer receive_payload;
		serialize(send_payload, value);
		endpoint_request(id, receive_payload, send_payload, false, sizeof(T));
	}
	template<typename T>
	void get_value(int id, T& value)
	{
		serial_buffer send_payload;
		serial_buffer receive_payload;
		endpoint_request(id, receive_payload, send_payload, true, sizeof(T));
		if (receive_payload.size() == sizeof(T))
		{
			auto it = get_it(receive_payload);
			deserialize(it, value);
		}
	}

private:
	// An instance of this is never connected to USB and UART both, so maybe this should
	// be cleaned up with polymorphism or some other way.
#ifdef ODRIVE_INCLUDE_USB
	libusb_context* ctx = nullptr;
	libusb_device_handle* usb_device = nullptr;
	int usb_interface;
	int usb_write_endpoint = -1, usb_read_endpoint = -1;
#endif
#ifdef ODRIVE_INCLUDE_UART
	int uart_file = -1;
#endif

	u16 firmware_crc = 0;
	u16 seq_no = 0;

private:
	bool get_json_interface();
	void send_to_odrive(std::vector<u8>& packet);
	bool receive_from_odrive(u8* packet, int max_bytes_to_receive, int* received_bytes, int expected_length);

	// Templates for basic serialization and deserialization
	void serialize(serial_buffer& serial_buffer, const u8& value);
	void serialize(serial_buffer& serial_buffer, const s8& value);
	void serialize(serial_buffer& serial_buffer, const s16& value);
	void serialize(serial_buffer& serial_buffer, const u16& value);
	void serialize(serial_buffer& serial_buffer, const s32& value);
	void serialize(serial_buffer& serial_buffer, const s64& value);
	void serialize(serial_buffer& serial_buffer, const float& valuef);
	void serialize(serial_buffer& serial_buffer, const bool& value);

	void deserialize(serial_buffer_iterator& it, s8& value);
	void deserialize(serial_buffer_iterator& it, u8& value);
	void deserialize(serial_buffer_iterator& it, s16& value);
	void deserialize(serial_buffer_iterator& it, u16& value);
	void deserialize(serial_buffer_iterator& it, s32& value);
	void deserialize(serial_buffer_iterator& it, u32& value);
	void deserialize(serial_buffer_iterator& it, s64& value);
	void deserialize(serial_buffer_iterator& it, u64& value);
	void deserialize(serial_buffer_iterator& it, float& value);
	void deserialize(serial_buffer_iterator& it, bool& value);

	serial_buffer create_odrive_packet(u16 seq_no, int endpoint, u16 response_size, const serial_buffer& payload);
	std::vector<u8> packet_to_stream(std::vector<u8>& packet);
	int stream_to_packet(std::vector<u8>& stream, u8* packet, int max_length);
};
