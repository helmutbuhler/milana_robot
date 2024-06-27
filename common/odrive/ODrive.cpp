#include "ODrive.h"
#include "json.hpp"

#ifdef ODRIVE_INCLUDE_USB
#ifdef _MSC_VER
#include "../../3rdparty/libusb/include/libusb.h"
#if _MSC_VER >= 1930 // Visual Studio 2022
#pragma comment(lib, "../3rdparty/libusb/VS2022/MS64/static/libusb-1.0.lib")
#elif _MSC_VER >= 1920 // Visual Studio 2019
#pragma comment(lib, "../3rdparty/libusb/VS2019/MS64/static/libusb-1.0.lib")
#elif _MSC_VER >= 1910 // Visual Studio 2017
#pragma comment(lib, "../3rdparty/libusb/VS2017/MS64/static/libusb-1.0.lib")
#endif
#else
#include <libusb-1.0/libusb.h>
#endif
#endif

#ifdef ODRIVE_INCLUDE_UART
#include <sys/fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "../../common/time_helper.h"

using nlohmann::json;

#ifdef ODRIVE_INCLUDE_USB
// ODrive specific USB Identifier:
#define VID     0x1209
#define PID     0x0D32

libusb_device* check_usb_device_is_odrive(libusb_device* device, int pass,
		int* usb_interface_out, int* usb_write_endpoint_out, int* usb_read_endpoint_out)
{
	libusb_device_descriptor desc;
	if (libusb_get_device_descriptor(device, &desc) < 0)
		return nullptr;
	if (desc.idVendor != VID || desc.idProduct != PID)
		return nullptr;
	//printf("found ODrive USB\n");
	libusb_config_descriptor* config;
	if (libusb_get_active_config_descriptor(device, &config) != 0)
		return nullptr;
	//printf("Interfaces: %d\n", (int)config->bNumInterfaces);
	for (int i = 0; i < (int)config->bNumInterfaces; i++)
	{
		const libusb_interface* inter = &config->interface[i];
		//printf("num_altsetting: %d\n", (int)inter->num_altsetting);
		for (int j = 0; j < inter->num_altsetting; j++)
		{
			const libusb_interface_descriptor* interdesc = &inter->altsetting[j];
			bool ok;
			if (pass == 0)
				ok = interdesc->bInterfaceClass == 0x00 && interdesc->bInterfaceSubClass == 0x01;
			else
				ok = interdesc->bInterfaceClass == 0x0a && interdesc->bInterfaceSubClass == 0x00;
			//printf("ok: %d\n", (int)ok);
			if (!ok)
				continue;
			int write_endpoint = -1, read_endpoint = -1;
			//printf("bNumEndpoints: %d\n", (int)interdesc->bNumEndpoints);
			for (int k = 0; k < (int)interdesc->bNumEndpoints; k++)
			{
				const libusb_endpoint_descriptor* epdesc = &interdesc->endpoint[k];
				if ((epdesc->bEndpointAddress & LIBUSB_ENDPOINT_IN) != 0)
					read_endpoint = (int)epdesc->bEndpointAddress;
				else
					write_endpoint = (int)epdesc->bEndpointAddress;
			}
			//printf("write_endpoint: %d\n", write_endpoint);
			//printf("read_endpoint: %d\n", read_endpoint);
			if (write_endpoint == -1 || read_endpoint == -1)
				continue;
			*usb_interface_out = (int)interdesc->bInterfaceNumber;
			*usb_write_endpoint_out = write_endpoint;
			*usb_read_endpoint_out = read_endpoint;
			libusb_free_config_descriptor(config);
			return device;
		}
	}
	libusb_free_config_descriptor(config);
	return nullptr;
}

libusb_device_handle* get_odrive_usb_device(libusb_context* ctx,
		int* usb_interface_out, int* usb_write_endpoint_out, int* usb_read_endpoint_out)
{
	// This function enumerates all usb devices and returns the first ODrive it finds.
	// It also returns the interface number for fibre communication and the read and write endpoint number.
	// In case you want to connect multiple ODrives via USB you will need to extend this function somehow...
	// This code is roughly the same algorithm as this python code found in ODrive/Firmware/fibre/python/fibre/usbbulk_transport.py:
/*
    self.cfg = self.dev.get_active_configuration()
    custom_interfaces = [i for i in self.cfg.interfaces() if i.bInterfaceClass == 0x00 and i.bInterfaceSubClass == 0x01]
    cdc_interfaces = [i for i in self.cfg.interfaces() if i.bInterfaceClass == 0x0a and i.bInterfaceSubClass == 0x00]
    all_compatible_interfaces = custom_interfaces + cdc_interfaces
    if len(all_compatible_interfaces) == 0:
      raise Exception("the device has no compatible interfaces")
    self.intf = all_compatible_interfaces[0]

    # Try to detach kernel driver from interface
    try:
      if self.dev.is_kernel_driver_active(self.intf.bInterfaceNumber):
        self.dev.detach_kernel_driver(self.intf.bInterfaceNumber)
        self._logger.debug("Detached Kernel Driver")
      else:
        self._logger.debug("Kernel Driver was not attached")
    except NotImplementedError:
      pass #is_kernel_driver_active not implemented on Windows

    # find write endpoint (first OUT endpoint)
    self.epw = usb.util.find_descriptor(self.intf,
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT
    )
    assert self.epw is not None
    self._logger.debug("EndpointAddress for writing {}".format(self.epw.bEndpointAddress))
    # find read endpoint (first IN endpoint)
    self.epr = usb.util.find_descriptor(self.intf,
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_IN
    )
	*/

	libusb_device* dev = nullptr;
	libusb_device** devices; //pointer to pointer of device, used to retrieve a list of devices
	ssize_t num_devices = libusb_get_device_list(ctx, &devices);
	for (ssize_t i = 0; i < num_devices; i++)
	{
		for (int p = 0; p < 2; p++)
		{
			dev = check_usb_device_is_odrive(devices[i], p, usb_interface_out, usb_write_endpoint_out, usb_read_endpoint_out);
			if (dev) break;
		}
		if (dev) break;
	}
	libusb_device_handle* handle = nullptr;
	if (dev)
	{
		libusb_open(dev, &handle);
		if (!handle)
		{
			printf("Found ODrive via USB, but could not open it.\n");
#ifndef _MSC_VER
			printf("Try running this with sudo.\n");
			printf("Alternatively, try following these steps: https://askubuntu.com/a/980887\n");
			printf("With idVendor = %d and idProduct = %d\n", VID, PID);
#endif
		}
	}
	libusb_free_device_list(devices, 1);
	return handle;
}
#endif

bool ODrive::connect_usb()
{
	close();
#ifdef ODRIVE_INCLUDE_USB
	libusb_init(&ctx);

	//libusb_set_option(ctx, LIBUSB_OPTION_USE_USBDK);

	//libusb_set_debug(ctx, LIBUSB_LOG_LEVEL_INFO);

	usb_device = get_odrive_usb_device(ctx, &usb_interface, &usb_write_endpoint, &usb_read_endpoint);
	if (!usb_device)
	{
		printf("Cannot find ODrive via USB!\n");
		close();
		return false;
	}

	int r;
	/*printf("Conf...\n");
	/r = libusb_set_configuration(usb_device, 1);
	if (r != 0)
	{
		printf("libusb_set_configuration failed!\n");
	}*/
	//printf("Claiming...\n");
	r = libusb_claim_interface(usb_device, usb_interface);
	if (r != 0)
	{
		printf("libusb_claim_interface failed!\n");
		close();
		return false;
	}
	printf("Connecting to ODrive via USB... ");
	fflush(stdout);
	if (!get_json_interface())
	{
		close();
		return false;
	}
	is_connected = true;
	return true;
#else
	printf("Cannot connect to ODrive via USB. Feature not compiled in.\n");
	return false;
#endif
}

void ODrive::close()
{
#ifdef ODRIVE_INCLUDE_USB
	if (usb_device)
	{
		libusb_release_interface(usb_device, usb_interface);
		libusb_close(usb_device);
	}
	if (ctx)
		libusb_exit(ctx);
	ctx = nullptr;
	usb_device = nullptr;
#endif
#ifdef ODRIVE_INCLUDE_UART
	if (uart_file != -1)
	    ::close(uart_file);
	uart_file = -1;
#endif
	root = Endpoint();
	is_connected = false;
	communication_error = false;
}

bool ODrive::connect_uart(const char* uart_address, int baud_rate, bool stop_bits_2)
{
	close();

#ifdef ODRIVE_INCLUDE_UART
	int br;
	switch (baud_rate)
	{
	case 115200 : br = B115200 ; break;
	case 230400 : br = B230400 ; break;
	case 500000 : br = B500000 ; break;
	case 921600 : br = B921600 ; break;
	case 1152000: br = B1152000; break;
	case 1500000: br = B1500000; break;
	case 2000000: br = B2000000; break;
	default:
		printf("Invalid baudrate: %d\n", br);
		return false;
	}

    //------------------------------------------------
	//  OPEN THE UART
    //------------------------------------------------
	// The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR   - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//	    O_NDELAY / O_NONBLOCK (same function) 
    //               - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //                 if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//				   immediately with a failure status if the output can't be written immediately.
    //                 Caution: VMIN and VTIME flags are ignored if O_NONBLOCK flag is set.
	//	    O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.fid = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

    uart_file = open(uart_address, O_RDWR | O_NOCTTY);

	//tcflush(fid, TCIFLUSH);
 	//tcflush(fid, TCIOFLUSH);
 	
    //usleep(1000000);  // 1 sec delay

    if (uart_file == -1)
	{
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
		return false;
	}

	termios port_options = {0};
    tcgetattr(uart_file, &port_options); // Get the current attributes of the Serial port 
	
    //------------------------------------------------
	// CONFIGURE THE UART
    //------------------------------------------------
	// flags defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html
	//	Baud rate:
    //         - B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, 
    //           B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, 
    //           B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE: - CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD  - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL  - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)

    port_options.c_cflag &= ~PARENB;            // Disables the Parity Enable bit(PARENB),So No Parity   
	if (stop_bits_2)
		port_options.c_cflag |= CSTOPB; // 2 Stop bits 
	else
		port_options.c_cflag &= ~CSTOPB; // 1 Stop bit 
    port_options.c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size             
    port_options.c_cflag |=  CS8;               // Set the data bits = 8                                 	 
    port_options.c_cflag &= ~CRTSCTS;           // No Hardware flow Control                         
    port_options.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines       				
    port_options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both input & output
    port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode                            
    port_options.c_iflag &= ~ICRNL;                        
    port_options.c_oflag &= ~OPOST;                           // No Output Processing

    port_options.c_lflag = 0;               //  enable raw input instead of canonical,
 		
    //port_options.c_cc[VMIN]  = VMINX;       // Read at least 1 character
    port_options.c_cc[VMIN]  = 0;       // Read at least 0 character
    //port_options.c_cc[VTIME] = 1;
    port_options.c_cc[VTIME] = 0;
		
    cfsetispeed(&port_options, br); // Set Read Speed 
    cfsetospeed(&port_options, br); // Set Write Speed 
	/*printf("port options\n");
	for (int i = 0; i < sizeof(port_options); i++)
		printf("0x%x\n", ((u8*)&port_options)[i]);
	printf("port options\n");*/

    // Set the attributes to the termios structure
    int att = tcsetattr(uart_file, TCSANOW, &port_options);

    if (att != 0)
    {
        printf("ERROR in Setting port attributes");
		close();
		return false;
    }

    // Flush Buffers
    tcflush(uart_file, TCIFLUSH);
    tcflush(uart_file, TCIOFLUSH);

	printf("Connecting to ODrive via UART (%s)... ", uart_address);
	if (!get_json_interface())
	{
		close();
		return false;
	}
	is_connected = true;
	return true;
#else
	printf("Cannot connect to ODrive via UART. Feature is not implemented on Windows yet!\n");
	return false;
#endif
}

static void populate_from_json(const json& j, Endpoint& endpoint)
{
	for (const json& obj : j)
	{
		Endpoint new_child;
		new_child.odrive = endpoint.odrive;
		std::string name = obj["name"];
		std::string type = obj["type"];
		new_child.name = endpoint.name + "." + name;
		new_child.type = type;
		new_child.access = "none";

		if (obj.count("access"))
		{
			//new_child.access = obj["access"]; // This line somehow doesn't compile in VS2019
			std::string tmp = obj["access"];
			new_child.access = std::move(tmp);
		}
		if (obj.count("id"))
			new_child.id = obj["id"];
		//std::cout << "Name: " << name << " type: " << type << " access: " << access << " id: " << id << std::endl;
		endpoint.children[name] = std::move(new_child);
		if (obj.count("members"))
		{
			const json& members = obj["members"];
			populate_from_json(members, endpoint.children[obj["name"]]);
		}
	}
}

void ODrive::endpoint_request(int endpoint_id,
		serial_buffer& received_payload, const serial_buffer& payload,
		bool ack, int length, bool length_must_match)
{
	if (communication_error)
		return;
	endpoint_request_counter++;

	if (ack)
		endpoint_id |= 0x8000;

	seq_no = (seq_no + 1) & 0x7fff;
	seq_no |= 0x80;

	// Send the packet
	const int max_bytes_to_receive = 64;
	u8 data[max_bytes_to_receive];
	int received_bytes = 0;
	serial_buffer packet = create_odrive_packet(seq_no, endpoint_id, (u16)length, payload);
	u32_micros start_time = time_micros();
	while (true)
	{
		if (time_micros() - start_time > 500000)
		{
			communication_error = true;
			received_payload.clear();
			printf("endpoint request timeout\n");
			return;
		}
		send_to_odrive(packet);

		if (!ack)
		{
			received_payload.clear();
			return;
		}

		receive_again:
		// Immediately wait for response from Odrive
		bool r = receive_from_odrive(data, max_bytes_to_receive, &received_bytes, length);
		if (!r)
			continue;

		if (communication_error)
		{
			printf("communication error!\n");
			received_payload.clear();
			return;
		}
		if (received_bytes < 2)
		{
			printf("%d: unexpected length %d\n", endpoint_id, received_bytes);
			communication_error = true;
			received_payload.clear();
			return;
		}
		/*if (received_bytes-2 != length)
		{
			printf("%d: expected length %d, but received %d\n", endpoint_id, length, received_bytes-2);
		}*/
		u16 received_seq_no = 0;
		serial_buffer_iterator it = data;
		deserialize(it, received_seq_no);
		if ((u16)((u16)seq_no | 0x8000) == (u16)received_seq_no+1)
		{
			// If a response takes longer than usual, we might timeout before we receive it.
			// In that case the next request gets the old response and the sequence number is one off.
			// So we just skip this response and read in the next.
			//printf("%d: receive_again\n", endpoint_id);
			goto receive_again;
		}
		if ((u16)((u16)seq_no | 0x8000) != (u16)received_seq_no)
		{
			//communication_error = true;
			//printf("%d: expected seq %d, but received %d\n", endpoint_id, ((u16)seq_no | 0x8000), received_seq_no);
			continue;
			//return;
		}

		if (length_must_match && received_bytes-2 != length)
		{
			communication_error = true;
			received_payload.clear();
			printf("%d: expected length %d, but received %d\n", endpoint_id, length, received_bytes-2);
			return;
		}

		received_payload.resize(received_bytes-2);
		memcpy(received_payload.data(), data+2, received_bytes-2);
		break;
	}
}

inline u16 firmware_id_to_crc(int id)
{
	return (u16)((id >> 16) & 0xffff);
}

bool ODrive::get_json_interface()
{
	//printf("get_json_interface...\n");

	serial_buffer send_payload;
	serial_buffer receive_payload;
	serial_buffer received_json;

	int crc = 0;
	{
		serialize(send_payload, (int)0xffffffff);
		endpoint_request(0, receive_payload, send_payload, true, 4);
		if (communication_error)
			return false;
		send_payload.clear();
		auto it = get_it(receive_payload);
		deserialize(it, crc);
	}
	firmware_crc = firmware_id_to_crc(crc);

	u32_micros start_time = time_micros();
	do {
		serialize(send_payload, (int)received_json.size());
		endpoint_request(0, receive_payload, send_payload, true, 64, false);
		send_payload.clear();
		for (u8 byte : receive_payload)
		{
			received_json.push_back(byte);
			//printf("%c", byte);
			//fflush(stdout);
		}
	} while (receive_payload.size());
	printf("done. time: %dms\n", (int)((time_micros() - start_time) * .001f));
	if (communication_error)
		return false;

	bool allow_exceptions = false;
	json j = json::parse(received_json, nullptr, allow_exceptions);
	if (j.is_discarded())
	{
		printf("invalid json!\n");
		communication_error = true;
		return false;
	}
	//printf("Received %i bytes!\n", received_json.size());
	root = Endpoint();
	root.odrive = this;
	populate_from_json(j, root);

	u8 odrive_fw_version_major = 0;
	u8 odrive_fw_version_minor = 0;
	u8 odrive_fw_version_revision = 0;
	root("fw_version_major"   ).get(odrive_fw_version_major);
	root("fw_version_minor"   ).get(odrive_fw_version_minor);
	root("fw_version_revision").get(odrive_fw_version_revision);
	odrive_fw_version = odrive_fw_version_major*1000000 + odrive_fw_version_minor*1000 + odrive_fw_version_revision;

	odrive_fw_is_milana = false;
	if (root.has_child("fw_version_milana"))
		root("fw_version_milana").get(odrive_fw_is_milana);
	//printf("odrive_fw_version: %d Milana: %d\n", odrive_fw_version, (int)odrive_fw_is_milana);
	return !communication_error;
}


void ODrive::call(int id)
{
	// call
	serial_buffer send_payload;
	serial_buffer receive_payload;
	endpoint_request(id, receive_payload, send_payload, false, 0);
}

void ODrive::call(int id, int in1, float* out1)
{
	{
		// param1
		serial_buffer send_payload;
		serial_buffer receive_payload;
		serialize(send_payload, in1);
		endpoint_request(id+1, receive_payload, send_payload, false, 0);
	}
	{
		// call
		serial_buffer send_payload;
		serial_buffer receive_payload;
		endpoint_request(id, receive_payload, send_payload, false, 0);
	}
	{
		// return
		serial_buffer send_payload;
		serial_buffer receive_payload;
		endpoint_request(id+2, receive_payload, send_payload, true, sizeof(*out1));
		auto it = get_it(receive_payload);
		deserialize(it, *out1);
	}
}
void ODrive::call(int id, int in1, u64* out1)
{
	{
		// param1
		serial_buffer send_payload;
		serial_buffer receive_payload;
		serialize(send_payload, in1);
		endpoint_request(id+1, receive_payload, send_payload, false, 0);
	}
	{
		// call
		serial_buffer send_payload;
		serial_buffer receive_payload;
		endpoint_request(id, receive_payload, send_payload, false, 0);
	}
	{
		// return
		serial_buffer send_payload;
		serial_buffer receive_payload;
		endpoint_request(id+2, receive_payload, send_payload, true, sizeof(*out1));
		auto it = get_it(receive_payload);
		deserialize(it, *out1);
	}
}

void ODrive::call(int id, bool* out1)
{
	{
		// call
		serial_buffer send_payload;
		serial_buffer receive_payload;
		endpoint_request(id, receive_payload, send_payload, false, 0);
	}
	{
		// return
		serial_buffer send_payload;
		serial_buffer receive_payload;
		endpoint_request(id+1, receive_payload, send_payload, true, sizeof(*out1));
		auto it = get_it(receive_payload);
		deserialize(it, *out1);
	}
}

// Calculates an arbitrary CRC for one byte.
// Adapted from https://barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
template<typename T, unsigned POLYNOMIAL>
static T calc_crc(T remainder, uint8_t value)
{
    constexpr T BIT_WIDTH = (8 * sizeof(T));
    constexpr T TOPBIT = ((T)1 << (BIT_WIDTH - 1));
    
    // Bring the next byte into the remainder.
    remainder ^= (value << (BIT_WIDTH - 8));

    // Perform modulo-2 division, a bit at a time.
    for (uint8_t bit = 8; bit; --bit) {
        if (remainder & TOPBIT) {
            remainder = (remainder << 1) ^ POLYNOMIAL;
        } else {
            remainder = (remainder << 1);
        }
    }

    return remainder;
}

template<typename T, unsigned POLYNOMIAL>
static T calc_crc(T remainder, const uint8_t* buffer, size_t length) {
    while (length--)
        remainder = calc_crc<T, POLYNOMIAL>(remainder, *(buffer++));
    return remainder;
}

template<unsigned POLYNOMIAL>
static uint8_t calc_crc8(uint8_t remainder, uint8_t value) {
    return calc_crc<uint8_t, POLYNOMIAL>(remainder, value);
}

template<unsigned POLYNOMIAL>
static uint16_t calc_crc16(uint16_t remainder, uint8_t value) {
    return calc_crc<uint16_t, POLYNOMIAL>(remainder, value);
}

template<unsigned POLYNOMIAL>
static uint8_t calc_crc8(uint8_t remainder, const uint8_t* buffer, size_t length) {
    return calc_crc<uint8_t, POLYNOMIAL>(remainder, buffer, length);
}

template<unsigned POLYNOMIAL>
static uint16_t calc_crc16(uint16_t remainder, const uint8_t* buffer, size_t length) {
    return calc_crc<uint16_t, POLYNOMIAL>(remainder, buffer, length);
}

constexpr uint8_t CANONICAL_CRC8_POLYNOMIAL = 0x37;
constexpr uint8_t CANONICAL_CRC8_INIT = 0x42;
constexpr uint16_t CANONICAL_CRC16_POLYNOMIAL = 0x3d65;
constexpr uint16_t CANONICAL_CRC16_INIT = 0x1337;

std::vector<u8> ODrive::packet_to_stream(std::vector<u8>& packet)
{
	assert(packet.size() < 128);

	serial_buffer data;
	data.reserve(3+packet.size()+2);

	serialize(data, (u8)0xaa);
	serialize(data, (u8)packet.size());
    u8 crc8 = calc_crc8<CANONICAL_CRC8_POLYNOMIAL>(CANONICAL_CRC8_INIT, data.data(), 2);
	serialize(data, crc8);

	for (u8 b : packet)
		data.push_back(b);

    u16 crc16 = calc_crc16<CANONICAL_CRC16_POLYNOMIAL>(CANONICAL_CRC16_INIT, packet.data(), packet.size());
    serialize(data, (u8)((crc16 >> 8) & 0xff));
    serialize(data, (u8)((crc16 >> 0) & 0xff));
	return data;
}

int ODrive::stream_to_packet(std::vector<u8>& stream, u8* packet, int max_length)
{
	if (stream.size() >= 128 || stream.size() >= max_length+5 || stream.size() < 5)
	{
		//printf("invalid stream packet size %d\n", stream.size());
		//for (int i = 0; i < stream.size(); i++)
		//	printf("0x%02x\n", stream[i]);
		return -1;
	}

	if (stream[0] != 0xaa)
	{
		//printf("invalid stream packet 1!\n");
		return -1;
	}
	if (stream[1] != stream.size()-5)
	{
		//printf("invalid stream packet 2! %d %d\n", stream[1], stream.size()-5);
		return -1;
	}
    u8 crc8 = calc_crc8<CANONICAL_CRC8_POLYNOMIAL>(CANONICAL_CRC8_INIT, stream.data(), 2);
	if (stream[2] != crc8)
	{
		//printf("invalid stream packet 3! 0x%x!=0x%x\n", stream[2], crc8);
		return -1;
	}
    u16 crc16_1 = calc_crc16<CANONICAL_CRC16_POLYNOMIAL>(CANONICAL_CRC16_INIT, stream.data()+3, stream.size()-5);
    u16 crc16_2;
	crc16_2 = ((u16)stream[stream.size()-2]) << 8;
	crc16_2 |= stream[stream.size()-1] << 0;
	if (crc16_1 != crc16_2)
	{
		//printf("invalid stream packet 4! %d %d  0x%x!=0x%x\n", stream.size(), max_length, crc16_1, crc16_2);
		//for (int i = 0; i < stream.size(); i++)
		//	printf("0x%02x\n", stream[i]);
		//printf("\n");
		return -1;
	}
	memcpy(packet, stream.data()+3, stream.size()-5);
	return (int)stream.size()-5;
}


void ODrive::send_to_odrive(std::vector<u8>& packet)
{
#ifdef ODRIVE_INCLUDE_USB
	if (usb_device)
	{
		int sent_bytes = 0;
		int r = libusb_bulk_transfer(usb_device,
			usb_write_endpoint,
			packet.data(),
			(int)packet.size(),
			&sent_bytes,
			0);
		if (r != 0 || sent_bytes != packet.size())
		{
			communication_error = true;
			printf("libusb_bulk_transfer send return %d %d\n", (int)packet.size(), sent_bytes);
		}
	}
#endif
#ifdef ODRIVE_INCLUDE_UART
	if (uart_file != -1)
	{
		std::vector<u8> stream_packet = packet_to_stream(packet);
		int sent_bytes = write(uart_file, stream_packet.data(), stream_packet.size());
		//printf("done\n");
		if (sent_bytes != stream_packet.size())
		{
			communication_error = true;
			printf("send return %d\n", sent_bytes);
		}
	}
#endif
}


bool ODrive::receive_from_odrive(u8* packet, int max_bytes_to_receive, int* received_bytes, int expected_length)
{
	// returns false if the packet to odrive should be resent
	//printf("recv...\n");
#ifdef ODRIVE_INCLUDE_USB
	if (usb_device)
	{
		const int timeout = 1000;
		int r = libusb_bulk_transfer(usb_device,
			usb_read_endpoint,
			packet,
			max_bytes_to_receive,
			received_bytes,
			timeout);
		if (r != 0)
		{
			communication_error = true;
			printf("libusb_bulk_transfer recv return %d\n", r);
		}
		return r == 0;
	}
#endif
#ifdef ODRIVE_INCLUDE_UART
	if (uart_file != -1)
	{
		std::vector<u8> stream(max_bytes_to_receive);
		*received_bytes = 0;
		u32_micros start_time = time_micros();
		int timeouts = 0;
		while (true)
		{
			if (time_micros() - start_time > (expected_length == 64 ? 2000 : 800))
			{
				//printf("recv timeout!\n");
				return false;
			}

			int received = read(uart_file, stream.data()+*received_bytes, max_bytes_to_receive-*received_bytes);
			//printf("recv: %d %d\n", max_bytes_to_receive, *received_bytes);
			if (received == 0)
			{
				timeouts++;
				continue;
			}
			if (received < 0)
			{
				communication_error = true;
				printf("recv return: %d\n", *received_bytes);
				*received_bytes = 0;
				return true;
			}
			*received_bytes += received;
			if (*received_bytes < 5)
			{
				//printf("recv again %d\n", *received_bytes);
				continue;
			}
			if (*received_bytes-5 < stream[1])
			{
				//printf("recv again %d < %d\n", *received_bytes-5, stream[1]);
				continue;
			}
			break;
		}
		stream.resize(*received_bytes);
		*received_bytes = stream_to_packet(stream, packet, max_bytes_to_receive);
		if (*received_bytes < 0)
		{
			//communication_error = true;
			*received_bytes = 0;
			return false;
		}
		/*if (timeouts)
		{
			printf("expectedlength: %d. timeouts: %d. time: %dus\n", expected_length, timeouts, (int)((time_micros() - start_time)));
		}*/
	}
#endif
	return true;
}

void ODrive::serialize(serial_buffer& serial_buffer, const s64& value) {
	serial_buffer.push_back((value >> 0) & 0xFF);
	serial_buffer.push_back((value >> 8) & 0xFF);
	serial_buffer.push_back((value >> 16) & 0xFF);
	serial_buffer.push_back((value >> 24) & 0xFF);
	serial_buffer.push_back((value >> 32) & 0xFF);
	serial_buffer.push_back((value >> 40) & 0xFF);
	serial_buffer.push_back((value >> 48) & 0xFF);
	serial_buffer.push_back((value >> 56) & 0xFF);
}

void ODrive::serialize(serial_buffer& serial_buffer, const s32& value) {
	serial_buffer.push_back((value >> 0) & 0xFF);
	serial_buffer.push_back((value >> 8) & 0xFF);
	serial_buffer.push_back((value >> 16) & 0xFF);
	serial_buffer.push_back((value >> 24) & 0xFF);
}

void ODrive::serialize(serial_buffer& serial_buffer, const float& valuef) {
	union {
		float f;
		int i;
	}u;
	u.f = valuef;
	serialize(serial_buffer, u.i);
}

void ODrive::serialize(serial_buffer& serial_buffer, const s16& value) {
	serial_buffer.push_back((value >> 0) & 0xFF);
	serial_buffer.push_back((value >> 8) & 0xFF);
}

void ODrive::serialize(serial_buffer& serial_buffer, const u16& value) {
	serial_buffer.push_back((value >> 0) & 0xFF);
	serial_buffer.push_back((value >> 8) & 0xFF);
}

void ODrive::serialize(serial_buffer& serial_buffer, const u8& value) {
	serial_buffer.push_back(value);
}
void ODrive::serialize(serial_buffer& serial_buffer, const s8& value) {
	serial_buffer.push_back(value);
}
void ODrive::serialize(serial_buffer& serial_buffer, const bool& value) {
	serial_buffer.push_back(value);
}

void ODrive::deserialize(serial_buffer_iterator& it, s32& value) {
	value = *it++;
	value |= (*it++) << 8;
	value |= (*it++) << 16;
	value |= (*it++) << 24;
}

void ODrive::deserialize(serial_buffer_iterator& it, u32& value) {
	value = *it++;
	value |= (*it++) << 8;
	value |= (*it++) << 16;
	value |= (*it++) << 24;
}

void ODrive::deserialize(serial_buffer_iterator& it, s16& value) {
	value = *it++;
	value |= (*it++) << 8;
}

void ODrive::deserialize(serial_buffer_iterator& it, u16& value) {
	value = *it++;
	value |= (*it++) << 8;
}

void ODrive::deserialize(serial_buffer_iterator& it, s64& value) {
	value = *it++;
	value |= s64(*it++) << (1*8);
	value |= s64(*it++) << (2*8);
	value |= s64(*it++) << (3*8);
	value |= s64(*it++) << (4*8);
	value |= s64(*it++) << (5*8);
	value |= s64(*it++) << (6*8);
	value |= s64(*it++) << (7*8);
}

void ODrive::deserialize(serial_buffer_iterator& it, u64& value) {
	value = *it++;
	value |= u64(*it++) << (1*8);
	value |= u64(*it++) << (2*8);
	value |= u64(*it++) << (3*8);
	value |= u64(*it++) << (4*8);
	value |= u64(*it++) << (5*8);
	value |= u64(*it++) << (6*8);
	value |= u64(*it++) << (7*8);
}

void ODrive::deserialize(serial_buffer_iterator& it, float& value) {
	union {
		float f;
		int i;
	}u;
	deserialize(it, u.i);
	value = u.f;
}

void ODrive::deserialize(serial_buffer_iterator& it, bool& value) {
	u8 val = *it++;
	assert(val <= 1);
	value = val != 0;
}

void ODrive::deserialize(serial_buffer_iterator& it, s8& value) {
	value = *it++;
}

void ODrive::deserialize(serial_buffer_iterator& it, u8& value) {
	value = *it++;
}

serial_buffer ODrive::create_odrive_packet(u16 seq_no, int endpoint, u16 response_size, const serial_buffer& payload)
{
	serial_buffer data;
	data.reserve(8+payload.size());
	serialize(data, (u16)seq_no);
	serialize(data, (u16)endpoint);
	serialize(data, (u16)response_size);

	for (u8 b : payload)
		data.push_back(b);

	if ((endpoint & 0x7fff) == 0)
		serialize(data, (u16)1);
	else
		serialize(data, firmware_crc);

	return data;
}

