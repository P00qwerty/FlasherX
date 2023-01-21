/* Copyright 2018 Paul Stoffregen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef ethernet_h_
#define ethernet_h_
#define UDP_TX_PACKET_MAX_SIZE 256

 // All symbols exposed to Arduino sketches are contained in this header file
 //
 // Older versions had much of this stuff in EthernetClient.h, EthernetServer.h,
 // and socket.h.  Including headers in different order could cause trouble, so
 // these "friend" classes are now defined in the same header file.  socket.h
 // was removed to avoid possible conflict with the C library header files.


 // Configure the maximum number of sockets to support.  W5100 chips can have
 // up to 4 sockets.  W5200 & W5500 can have up to 8 sockets.  Several bytes
 // of RAM are used for each socket.  Reducing the maximum can save RAM, but
 // you are limited to fewer simultaneous connections.
#if defined(RAMEND) && defined(RAMSTART) && ((RAMEND - RAMSTART) <= 2048)
#define MAX_SOCK_NUM 4
#else
#define MAX_SOCK_NUM 8
#endif
#define FNET_SOCKET_DEFAULT_SIZE 1024 * 2
#define FNET_STACK_HEAP_DEFAULT_SIZE 64u * 1024u //64k
#define FNET_POLL_TIME 1000 //Time in microseconds

// By default, each socket uses 2K buffers inside the Wiznet chip.  If
// MAX_SOCK_NUM is set to fewer than the chip's maximum, uncommenting
// this will use larger buffers within the Wiznet chip.  Large buffers
// can really help with UDP protocols like Artnet.  In theory larger
// buffers should allow faster TCP over high-latency links, but this
// does not always seem to work in practice (maybe Wiznet bugs?)
//#define ETHERNET_LARGE_BUFFERS


#include <Arduino.h>
#include <fnet.h>
#include "Client.h"
#include "Server.h"
#include "Udp.h"

enum EthernetLinkStatus {
	Unknown,
	LinkON,
	LinkOFF
};

typedef void (*UdpServerCallback)(
	IPAddress src_ip,    ///< IP address of the sender
	uint16_t src_port,    ///< Port the packet was sent from
	uint8_t* data,   ///< UDP payload data
	uint16_t len);        ///< Length of the payload data


typedef struct {
	UdpServerCallback callback;
} UdpServerListener;

class EthernetClass {
private:
	static DMAMEM uint8_t** socket_buf_transmit;
	static DMAMEM uint16_t* socket_buf_len;
	static DMAMEM uint16_t* socket_port;
	static DMAMEM uint8_t** socket_addr;
	static IntervalTimer _fnet_poll;
	static volatile boolean link_status;
	static uint8_t* stack_heap_ptr;
	static size_t stack_heap_size;
	static ssize_t socket_size;
	static uint8_t socket_num;

	static volatile UdpServerListener* _callback;

public:
	int startListeningToPort(UdpServerCallback callback, uint16_t port);
	void packetLOOP();
	uint8_t sendCompletePacket(uint8_t sockindex, const uint8_t* buf, uint16_t len, IPAddress _address, uint16_t port);


	static volatile fnet_socket_t* socket_ptr;
	static DMAMEM uint8_t** socket_buf_receive;
	static DMAMEM uint16_t* socket_buf_index;

	static EthernetLinkStatus linkStatus();

	// Manaul configuration
	static uint8_t begin(uint8_t* mac, unsigned long timeout, IPAddress ip);
	static uint8_t begin(uint8_t* mac, unsigned long timeout, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet);

	static void MACAddress(uint8_t* mac_address);

	static IPAddress localIP();
	static IPAddress subnetMask();
	static IPAddress gatewayIP();
	static IPAddress dhcpServerIP();
	static IPAddress dnsServerIP() { return fnet_netif_get_ip4_dns(fnet_netif_get_default()); }

	void setMACAddress(const uint8_t* mac_address);
	void setLocalIP(const IPAddress local_ip);
	void setSubnetMask(const IPAddress subnet);
	void setGatewayIP(const IPAddress gateway);
	void setDnsServerIP(const IPAddress dns_server);

private:
	// Opens a socket(TCP or UDP or IP_RAW mode)
	static uint8_t socketStatus(uint8_t s);
	static uint8_t socketBegin(uint8_t protocol, uint16_t port);

	static bool initializeStack(uint8_t* mac, unsigned long timeout);
	// Close socket
	static void socketClose(uint8_t s);
	// Establish TCP connection (Active connection)
	static void socketConnect(uint8_t s, uint8_t* addr, uint16_t port);
	// disconnect the connection
	static void socketDisconnect(uint8_t s);
	// Establish TCP connection (Passive connection)
	static uint8_t socketListen(uint8_t s);
	// Send data (TCP)
	static uint16_t socketSend(uint8_t s, const uint8_t* buf, uint16_t len);
	static uint16_t socketSendAvailable(uint8_t s);
	// Receive data (TCP)
	static int socketRecv(uint8_t s, uint8_t* buf, int16_t len);
	static uint16_t socketRecvAvailable(uint8_t s);
	// sets up a UDP datagram, the data for which will be provided by one
	// or more calls to bufferData and then finally sent with sendUDP.
	// return true if the datagram was successfully set up, or false if there was an error
	static bool socketStartUDP(uint8_t s, IPAddress addr, uint16_t port);
	// copy up to len bytes of data from buf into a UDP datagram to be
	// sent later by sendUDP.  Allows datagrams to be built up from a series of bufferData calls.
	// return Number of bytes successfully buffered
	static uint16_t socketBufferData(uint8_t s, uint16_t offset, const uint8_t* buf, uint16_t len);
	// Send a UDP datagram built up from a sequence of startUDP followed by one or more
	// calls to bufferData.
	// return true if the datagram was successfully sent, or false if there was an error
	static bool socketSendUDP(uint8_t s, fnet_flag_t flags = 0);
	// Initialize the "random" source port number
	static void socketPortRand(uint16_t n);

	static fnet_return_t teensy_mutex_init(fnet_mutex_t* mutex);
	static void teensy_mutex_release(fnet_mutex_t* mutex);
	static void teensy_mutex_lock(fnet_mutex_t* mutex);
	static void teensy_mutex_unlock(fnet_mutex_t* mutex);
	static fnet_time_t timer_get_ms(void);
	static void link_callback(fnet_netif_desc_t netif, fnet_bool_t connected, void* callback_param);
	static void dhcp_cln_callback_updated(fnet_dhcp_cln_desc_t _dhcp_desc, fnet_netif_desc_t netif, void* p);
};

extern EthernetClass Ethernet;



#endif




#ifndef  W5100_H_INCLUDED
#define W5100_H_INCLUDED

#include <Arduino.h>

class SnMR {
public:
	//  static const uint8_t CLOSE  = 0x00; //Unused
	static const uint8_t TCP = 0x21;
	static const uint8_t UDP = 0x02;
	//  static const uint8_t IPRAW  = 0x03; //Unused
	//  static const uint8_t MACRAW = 0x04; //Unused
	//  static const uint8_t PPPOE  = 0x05; //Unused
	//  static const uint8_t ND     = 0x20; //Unused
	static const uint8_t MULTI = 0x80;
};

class SnSR {
public:
	static const uint8_t CLOSED = 0x00;
	static const uint8_t INIT = 0x13;
	static const uint8_t LISTEN = 0x14;
	static const uint8_t ESTABLISHED = 0x17;
	static const uint8_t FIN_WAIT = 0x18;
	static const uint8_t CLOSE_WAIT = 0x1C;
};

#endif
