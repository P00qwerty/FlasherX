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


#include <Arduino.h>
#include <fnet.h>
#include "zNativeEthernet.h"

uint8_t* EthernetClass::stack_heap_ptr = NULL;
size_t EthernetClass::stack_heap_size = 0;
ssize_t EthernetClass::socket_size = 0;
uint8_t EthernetClass::socket_num = 0;
IntervalTimer EthernetClass::_fnet_poll;
volatile boolean EthernetClass::link_status = 0;
DMAMEM uint8_t** EthernetClass::socket_buf_receive;
DMAMEM uint16_t* EthernetClass::socket_buf_index;
DMAMEM uint8_t** EthernetClass::socket_buf_transmit;
DMAMEM uint16_t* EthernetClass::socket_buf_len;
DMAMEM uint16_t* EthernetClass::socket_port;
DMAMEM uint8_t** EthernetClass::socket_addr;
volatile fnet_socket_t* EthernetClass::socket_ptr;
volatile UdpServerListener* EthernetClass::_callback;

#if ARDUINO >= 156 && !defined(ARDUINO_ARCH_PIC32)
extern void yield(void);
#else
#define yield()
#endif

// TODO: randomize this when not using DHCP, but how?
static uint16_t local_port = 49152;  // 49152 to 65535

uint8_t EthernetClass::begin(uint8_t* mac, unsigned long timeout, IPAddress ip)
{
    // Assume the DNS server will be the machine on the same network as the local IP
    // but with last octet being '1'
    IPAddress dns = ip;
    dns[3] = 1;

    // Assume the gateway will be the machine on the same network as the local IP
    // but with last octet being '1'
    IPAddress gateway = ip;
    gateway[3] = 1;

    IPAddress subnet(255, 255, 255, 0);
    return begin(mac, timeout, ip, dns, gateway, subnet);
}

uint8_t EthernetClass::begin(uint8_t* mac, unsigned long timeout, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet)
{
    if (!initializeStack(mac, (unsigned long)10000)) return 0x00;

    if (timeout > 0)
    {
        uint32_t startMillis = millis();
        Serial.println("    Initialize Ethernet with DHCP:");
        bool error = false;

        while(!fnet_dhcp_cln_is_enabled(fnet_dhcp_cln_get_by_netif(fnet_netif_get_default())))
        {
            //Wait for dhcp initialization
            if(millis() - startMillis >= timeout)
            {
                error = true;
                break;
            }
        }
        if (!error)
        {
            struct fnet_dhcp_cln_options current_options;
            do {//Wait for IP Address
                fnet_dhcp_cln_get_options(fnet_dhcp_cln_get_by_netif(fnet_netif_get_default()), &current_options);
                if(millis() - startMillis >= timeout)
                {
                    error = true;
                    break;
                }
            } while (!current_options.ip_address.s_addr);
        }
        if (error)
        {
            timeout = 0;
            Serial.println("    Failed to configure Ethernet using DHCP");
            Serial.println("    So we manually set the IP address");
        }
        else
        {
            return 0x02;
        }
    }
    if (timeout == 0)
    {
        fnet_dhcp_cln_release(fnet_dhcp_cln_get_by_netif(fnet_netif_get_default()));
        fnet_netif_set_ip4_addr(fnet_netif_get_default(), ip, subnet);
        fnet_netif_set_ip4_gateway(fnet_netif_get_default(), gateway);
        fnet_netif_set_ip4_dns(fnet_netif_get_default(), dns);
    }
    return 0x01;
}

bool EthernetClass::initializeStack(uint8_t* mac, unsigned long responseTimeout)
{
    if (!fnet_netif_is_initialized(fnet_netif_get_default()))
    {
        struct fnet_init_params     init_params;
        if (stack_heap_size == 0) {
            stack_heap_size = FNET_STACK_HEAP_DEFAULT_SIZE;
        }
        if (stack_heap_ptr == NULL) {
            stack_heap_ptr = new uint8_t[stack_heap_size];
        }
        if (socket_size == 0) {
            socket_size = FNET_SOCKET_DEFAULT_SIZE;
        }
        if (socket_num == 0) {
            socket_num = MAX_SOCK_NUM;
        }

        socket_buf_transmit = new uint8_t * [socket_num];
        socket_buf_receive = new uint8_t * [socket_num];
        socket_buf_len = new uint16_t[socket_num];
        socket_port = new uint16_t[socket_num];
        socket_addr = new uint8_t * [socket_num];
        socket_ptr = new fnet_socket_t[socket_num];
        socket_buf_index = new uint16_t[socket_num];
        _callback = new UdpServerListener[socket_num];

        for (uint8_t i = 0; i < socket_num; i++) {
            socket_buf_transmit[i] = new uint8_t[socket_size];
            socket_buf_receive[i] = new uint8_t[socket_size];
            socket_addr[i] = new uint8_t[4];
            socket_ptr[i] = nullptr;
        }

        init_params.netheap_ptr = stack_heap_ptr;
        init_params.netheap_size = stack_heap_size;

        static const fnet_mutex_api_t teensy_mutex_api = {
            .mutex_init = teensy_mutex_init,
            .mutex_release = teensy_mutex_release,
            .mutex_lock = teensy_mutex_lock,
            .mutex_unlock = teensy_mutex_unlock,
        };

        static const fnet_timer_api_t timer_api = { //Setup millis timer
          .timer_get_ms = timer_get_ms,
          .timer_delay = 0,
        };

        /* Input parameters for FNET stack initialization */
        init_params.mutex_api = &teensy_mutex_api;
        init_params.timer_api = &timer_api;
        /* FNET Initialization */
        if (fnet_init(&init_params) != FNET_ERR)
        {

            //Serial.println("TCP/IP stack initialization is done.\n");
            // Initialize networking interfaces using fnet_netif_init().
            //Get current net interface.
            if (fnet_netif_init(FNET_CPU_ETH0_IF, mac, 6) != FNET_ERR)
            {
                //            Serial.println("netif Initialized");
                if (fnet_netif_get_default() == 0)
                {
                    Serial.println("    ERROR: Network Interface is not configurated!");
                    return false;
                }
                else
                {
                    //              Serial.println("SUCCESS: Network Interface is configurated!");
                    fnet_link_params_t link_params;
                    link_params.netif_desc = fnet_netif_get_default();
                    link_params.callback = link_callback;
                    
                    static unsigned long _responseTimeout = responseTimeout;
                    link_params.callback_param = &_responseTimeout;
                    fnet_link_init(&link_params);
                    _fnet_poll.begin(fnet_poll, FNET_POLL_TIME);
                }
                return true;
            }
            else
            {
                Serial.println("    Error:netif initialization failed.\n");
                return false;
            }
        }
        else
        {
            Serial.println("    Error:TCP/IP stack initialization failed.\n");
            return false;
        }
    }
    Serial.println("    Warning:TCP/IP stack already initialized.");
    return true;

}

EthernetLinkStatus EthernetClass::linkStatus()
{
    switch ((uint8_t)link_status) {
    case 0:  return LinkOFF;
    case 1: return LinkON;
    default:       return Unknown;
    }
}

void EthernetClass::MACAddress(uint8_t* mac_address)
{
    fnet_netif_get_hw_addr(fnet_netif_get_default(), mac_address, 6);
}

IPAddress EthernetClass::localIP()
{
    return IPAddress(fnet_netif_get_ip4_addr(fnet_netif_get_default()));
}

IPAddress EthernetClass::subnetMask()
{
    return IPAddress(fnet_netif_get_ip4_subnet_mask(fnet_netif_get_default()));
}

IPAddress EthernetClass::gatewayIP()
{
    return IPAddress(fnet_netif_get_ip4_gateway(fnet_netif_get_default()));
}

IPAddress EthernetClass::dhcpServerIP()
{
    struct fnet_dhcp_cln_options current_options;
    fnet_dhcp_cln_get_options(fnet_dhcp_cln_get_by_netif(fnet_netif_get_default()), &current_options);

    return IPAddress(current_options.dhcp_server.s_addr);
}

void EthernetClass::setMACAddress(const uint8_t* mac_address)
{
    fnet_netif_set_hw_addr(fnet_netif_get_default(), (fnet_uint8_t*)mac_address, 6);
}

void EthernetClass::setLocalIP(const IPAddress local_ip)
{
    fnet_netif_set_ip4_addr(fnet_netif_get_default(), *const_cast<IPAddress*>(&local_ip), subnetMask());
}

void EthernetClass::setSubnetMask(const IPAddress subnet)
{
    fnet_netif_set_ip4_addr(fnet_netif_get_default(), localIP(), *const_cast<IPAddress*>(&subnet));
}

void EthernetClass::setGatewayIP(const IPAddress gateway)
{
    fnet_netif_set_ip4_gateway(fnet_netif_get_default(), *const_cast<IPAddress*>(&gateway));
}

void EthernetClass::setDnsServerIP(const IPAddress dns_server)
{
    fnet_netif_set_ip4_dns(fnet_netif_get_default(), *const_cast<IPAddress*>(&dns_server));
}

fnet_return_t EthernetClass::teensy_mutex_init(fnet_mutex_t* mutex) {
    return FNET_OK;
}

void EthernetClass::teensy_mutex_release(fnet_mutex_t* mutex) {
}

void EthernetClass::teensy_mutex_lock(fnet_mutex_t* mutex) {
}

void EthernetClass::teensy_mutex_unlock(fnet_mutex_t* mutex) {
}

fnet_time_t EthernetClass::timer_get_ms(void) { //Used for multi-thread version
    fnet_time_t result;
    result = millis();
    return result;
}

void EthernetClass::link_callback(fnet_netif_desc_t netif, fnet_bool_t connected, void* callback_param)
{
//    Serial.println(connected ? "Link Connected!" : "Link Disconnected!");
    link_status = connected;
    if (connected)
    {
//            Serial.println("Initialising Services!");
//            init_services(netif);
        if (localIP() == IPAddress(0, 0, 0, 0))
        {
//            Serial.println("Initializing DHCP");

            static fnet_dhcp_cln_params_t dhcp_params; //DHCP intialization parameters
            dhcp_params.netif = netif;
            // Enable DHCP client.
            if (fnet_dhcp_cln_init(&dhcp_params))
            {
                fnet_dhcp_cln_set_response_timeout(fnet_dhcp_cln_get_by_netif(netif), *(unsigned long*)callback_param);
                
                /*Register DHCP event handler callbacks.*/
//                fnet_dhcp_cln_set_callback_updated(fnet_dhcp_cln_get_by_netif(netif), dhcp_cln_callback_updated, NULL);
//                fnet_dhcp_cln_set_callback_discover(fnet_dhcp_cln_get_by_netif(netif), dhcp_cln_callback_updated, NULL);
//                Serial.println("DHCP initialization done!");
            }
            else
            {
                Serial.println("ERROR: DHCP initialization failed!");
            }
        }
    }
    else {
//            Serial.println("Releasing Services!");
//            release_services(netif);
        fnet_dhcp_cln_release(fnet_dhcp_cln_get_by_netif(netif));
    }
}

void EthernetClass::dhcp_cln_callback_updated(fnet_dhcp_cln_desc_t _dhcp_desc, fnet_netif_desc_t netif, void* p) { //Called when DHCP updates
    struct fnet_dhcp_cln_options current_options;
    fnet_dhcp_cln_get_options(_dhcp_desc, &current_options);

    uint8_t* ip = (uint8_t*)&current_options.ip_address.s_addr;
    Serial.print("IPAddress: ");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.println((uint8_t)*ip);

    ip = (uint8_t*)&current_options.netmask.s_addr;
    Serial.print("SubnetMask: ");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.println((uint8_t)*ip);

    ip = (uint8_t*)&current_options.gateway.s_addr;
    Serial.print("Gateway: ");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.println((uint8_t)*ip);

    ip = (uint8_t*)&current_options.dhcp_server.s_addr;
    Serial.print("DHCPServer: ");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.print((uint8_t)*ip++);
    Serial.print(".");
    Serial.println((uint8_t)*ip);


    Serial.print("State: ");
    Serial.println(fnet_dhcp_cln_get_state(_dhcp_desc));
    Serial.println();
    Serial.println();
}

int EthernetClass::startListeningToPort(UdpServerCallback callback, uint16_t port)
{

    uint8_t sockindex = socketBegin(SnMR::UDP, port);

    if (sockindex >= socket_num)
    {
        return 0;
    }

    _callback[sockindex] = (UdpServerListener){callback};

    return 1;
}

void EthernetClass::packetLOOP()
{
    for (uint8_t s = 0; s < socket_num; s++)
    {
        if (socket_ptr[s] != nullptr)
        {
            uint16_t _remaining;
            if ((_remaining = socketRecvAvailable(s)) > 0)
            {
                //HACK - hand-parse the UDP packet using TCP recv method
                struct fnet_sockaddr from;
                fnet_size_t fromlen = sizeof(from);

                fnet_ssize_t ret = fnet_socket_recvfrom(socket_ptr[s], socket_buf_receive[s], socket_size, 0, &from, &fromlen);

                socket_buf_index[s] = 0;

                //read 8 header bytes and get IP and port from it
                if (ret > 0)
                {
                    IPAddress _remoteIP = from.sa_data;

                    uint16_t _remotePort = FNET_HTONS(from.sa_port);

                    if (_remaining > 0)
                    {
                        if (_remaining <= UDP_TX_PACKET_MAX_SIZE)
                        {
                            uint8_t packetBuffer2[_remaining];
                            socketRecv(s, packetBuffer2, _remaining);
                            _callback[s].callback(_remoteIP, _remotePort, packetBuffer2, _remaining);
                        }
                        else
                        {
                            Serial.print("PACKET TO LONG ");
                            Serial.print(_remaining);
                            Serial.print(" --> UDP_TX_PACKET_MAX_SIZE = ");
                            Serial.println(UDP_TX_PACKET_MAX_SIZE);
                        }
                    }
                }
            }
            continue;
        }
        break;
    }
}

uint8_t EthernetClass::sendCompletePacket(uint8_t sockindex, const uint8_t* buf, uint16_t len, IPAddress _address, uint16_t port)
{
    if (link_status == 0x01)
    {
        if (socket_ptr[sockindex] != nullptr)
        {
            //Serial.printf("UDP beginPacket\n");
            socketStartUDP(sockindex, _address, port);
            //Serial.printf("UDP write %d\n", len);
            socketBufferData(sockindex, 0, buf, len);
            return socketSendUDP(sockindex);
        }
    }
    else
    {
        //Serial.println("Not Connected");
    }
    return 0;
}

/*****************************************/
/*          Socket management            */
/*****************************************/


void EthernetClass::socketPortRand(uint16_t n)
{
    n &= 0x3FFF;
    local_port ^= n;
    //Serial.printf("socketPortRand %d, srcport=%d\n", n, local_port);
}

uint8_t EthernetClass::socketBegin(uint8_t protocol, uint16_t port)
{
    uint8_t s, maxindex = socket_num;
    // look at all the hardware sockets, use any that are closed (unused)
    for (s = 0; s < maxindex; s++) {
        if (socket_ptr[s] == nullptr) {

            goto makesocket;
        }
    }
    // As a last resort, forcibly close any already closing
    for (s = 0; s < maxindex; s++) {
        uint8_t stat = socketStatus(s);
        if (stat == SnSR::FIN_WAIT) goto closemakesocket;
        if (stat == SnSR::CLOSE_WAIT) goto closemakesocket;
    }
    return socket_num;

closemakesocket:
    socketClose(s);
makesocket:
    struct fnet_sockaddr_in local_addr;

    const fnet_uint32_t bufsize_option = socket_size;
    const fnet_int32_t      tcpnodelay_option = 1;
    if (protocol == SnMR::UDP) {
        socket_ptr[s] = fnet_socket(AF_INET, SOCK_DGRAM, 0);
    }
    else if (protocol == SnMR::TCP) {
        socket_ptr[s] = fnet_socket(AF_INET, SOCK_STREAM, 0);
    }
    else {
        //           Serial.println("Invalid Protocol!");
        return socket_num;
    }

    // create listen socket
    if (socket_ptr[s] == FNET_NULL) {
        //        Serial.println("UDP/IP: Socket creation error.");
        return socket_num;
    }

    fnet_memset(&local_addr, 0, sizeof(local_addr));

    local_addr.sin_port = FNET_HTONS(port); //fnet_htons(UDP_PORT);
    local_addr.sin_addr.s_addr = INADDR_ANY; //fnet_htonl(INADDR_ANY);
    local_addr.sin_family = AF_INET;

    // bind the socket to the port
    if (FNET_ERR == fnet_socket_bind(socket_ptr[s], (struct fnet_sockaddr*)(&local_addr), sizeof(local_addr))) {
        //        Serial.println("UDP/IP: Socket bind error.");
        fnet_socket_close(socket_ptr[s]);
        return socket_num;
    }

    fnet_socket_setopt(socket_ptr[s], SOL_SOCKET, SO_RCVBUF, &bufsize_option, sizeof(bufsize_option));
    fnet_socket_setopt(socket_ptr[s], SOL_SOCKET, SO_SNDBUF, &bufsize_option, sizeof(bufsize_option));
    if (protocol == SnMR::TCP) {
        const struct fnet_linger    linger_option =
        {
            .l_onoff = FNET_TRUE,
            .l_linger = 4 /*sec*/
        };
        fnet_socket_setopt(socket_ptr[s], IPPROTO_TCP, TCP_NODELAY, &tcpnodelay_option, sizeof(tcpnodelay_option));
        fnet_socket_setopt(socket_ptr[s], SOL_SOCKET, SO_LINGER, &linger_option, sizeof(linger_option));
        //        fnet_socket_setopt(socket_ptr[s], IPPROTO_TCP, TCP_MSS, &bufsize_option, sizeof(bufsize_option));
    }
    return s;
}

// Return the socket's status
//
uint8_t EthernetClass::socketStatus(uint8_t s)
{
    if (socket_ptr[s] == nullptr) {
        return SnSR::CLOSED;
    }
    fnet_socket_state_t state;
    fnet_size_t state_size = sizeof(state);
    if (fnet_socket_getopt(socket_ptr[s], SOL_SOCKET, SO_STATE, &state, &state_size) == FNET_ERR) {
        int8_t error_handler = fnet_error_get();
        Serial.print("StateErr: ");
        Serial.send_now();
        Serial.println(error_handler);
        Serial.send_now();
    }
    switch (state) {
    case SS_CLOSED:
        return SnSR::CLOSED;
    case SS_CLOSING:
        return SnSR::CLOSE_WAIT;
    case SS_CONNECTING:
        return SnSR::INIT;
    case SS_CONNECTED:
        return SnSR::ESTABLISHED;
    case SS_LISTENING:
        return SnSR::LISTEN;
    default:
        break;
    }
    return true;
}

// Immediately close.  If a TCP connection is established, the
// remote host is left unaware we closed.
//
void EthernetClass::socketClose(uint8_t s)
{
    fnet_socket_close(socket_ptr[s]);

    //    while(Ethernet.socketStatus(s) != 1){
    //        
    //    }
    socket_ptr[s] = nullptr;
}


// Place the socket in listening (server) mode
//
uint8_t EthernetClass::socketListen(uint8_t s)
{
    fnet_return_t ret = fnet_socket_listen(socket_ptr[s], 1);
    if (ret != FNET_OK) {
        return 0;
    }
    else {
        return 1;
    }
}


// establish a TCP connection in Active (client) mode.
//
void EthernetClass::socketConnect(uint8_t s, uint8_t* addr, uint16_t port)
{
    // set destination IP
  //    Serial.println("Socket Connect");
    struct fnet_sockaddr_in remoteaddr;
    remoteaddr.sin_family = AF_INET;
    remoteaddr.sin_port = FNET_HTONS(port);
    remoteaddr.sin_addr.s_addr = *(fnet_ip4_addr_t*)addr;
    //    fnet_socket_connect(socket_ptr[s], (struct fnet_sockaddr*)&remoteaddr, sizeof(remoteaddr));
    fnet_return_t ret = fnet_socket_connect(socket_ptr[s], (struct fnet_sockaddr*)&remoteaddr, sizeof(remoteaddr));
    if (ret == FNET_ERR) {
        int8_t error_handler = fnet_error_get();
        Serial.print("Connect Err: ");
        Serial.send_now();
        Serial.println(error_handler);
        Serial.send_now();
    }
}



// Gracefully disconnect a TCP connection.
//
void EthernetClass::socketDisconnect(uint8_t s)
{
    //    socketClose(s);
    if (fnet_socket_shutdown(socket_ptr[s], SD_WRITE) == FNET_ERR) {
        //        Serial.println("Socket Shutdown Error");
        //        int8_t error_handler = fnet_error_get();
        //        Serial.print("RemainingErr: ");
        //        Serial.println(error_handler);
        //        Serial.send_now();
    };
}



/*****************************************/
/*    Socket Data Receive Functions      */
/*****************************************/

// Receive data.  Returns size, or -1 for no data, or 0 if connection closed
//
int EthernetClass::socketRecv(uint8_t s, uint8_t* buf, int16_t len)
{
    if (socket_buf_index[s] == socket_size) return -1;
    if (socket_buf_index[s] + len < socket_size) {
        if (buf != NULL) fnet_memcpy(buf, socket_buf_receive[s] + socket_buf_index[s], len);
        socket_buf_index[s] += len;
        return len;
    }
    else if (socket_buf_index[s] + len >= socket_size) {
        uint16_t truncate = socket_size - socket_buf_index[s];
        if (buf != NULL) fnet_memcpy(buf, socket_buf_receive[s] + socket_buf_index[s], truncate);
        socket_buf_index[s] += truncate;
        return truncate;
    }
    return -1;
}

uint16_t EthernetClass::socketRecvAvailable(uint8_t s)
{
    int ret = fnet_socket_recv(Ethernet.socket_ptr[s], socket_buf_receive[s], Ethernet.socket_size, MSG_PEEK);
    if (ret == -1)
    {
        //        int8_t error_handler = fnet_error_get();
        //            Serial.print("SocketRecvAvailErr: ");
        //            Serial.send_now();
        //            Serial.println(error_handler);
        //        Serial.print("Socket Index: ");
        //        Serial.println(s);
        //            Serial.send_now();
        return 0;
    }

    return ret;
}



/*****************************************/
/*    Socket Data Transmit Functions     */
/*****************************************/

/**
 * @brief This function used to send the data in TCP mode
 * @return  1 for success else 0.
 */
uint16_t EthernetClass::socketSend(uint8_t s, const uint8_t* buf, uint16_t len)
{
    while (socketSendAvailable(s) < len) {}
    fnet_ssize_t ret = -1;
    ret = fnet_socket_send(socket_ptr[s], buf, len, 0);
    if (ret == -1) {
        int8_t error_handler = fnet_error_get();
        Serial.print("SendErr: ");
        Serial.send_now();
        Serial.println(error_handler);
        return 0;
    }
    return  ret;
}

uint16_t EthernetClass::socketSendAvailable(uint8_t s)
{
    fnet_ssize_t _max, _pending;
    fnet_size_t _max_size, _pending_size;
    _pending_size = sizeof(_pending);
    _max_size = sizeof(_max_size);
    if (fnet_socket_getopt(socket_ptr[s], SOL_SOCKET, SO_SNDNUM, &_pending, &_pending_size) == FNET_ERR) {
        return 0;
    }
    if (fnet_socket_getopt(socket_ptr[s], SOL_SOCKET, SO_SNDBUF, &_max, &_max_size) == FNET_ERR) {
        return 0;
    }
    return (uint16_t)(_max - _pending);
}

uint16_t EthernetClass::socketBufferData(uint8_t s, uint16_t offset, const uint8_t* buf, uint16_t len)
{
    //Serial.printf("  bufferData, offset=%d, len=%d\n", offset, len);
    uint8_t* _buf = (uint8_t*)buf;
    if (offset + len >= socket_size) {
        fnet_memcpy(socket_buf_transmit[s] + offset, _buf, socket_size - offset);
        socket_buf_len[s] += socket_size - offset;
        return socket_size - offset;
    }
    else {
        fnet_memcpy(socket_buf_transmit[s] + offset, _buf, len);
        socket_buf_len[s] += len;

        return len;
    }
}

bool EthernetClass::socketStartUDP(uint8_t s, IPAddress addr, uint16_t port)
{
    if (((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
        ((port == 0x00))) {
        return false;
    }
    socket_buf_len[s] = 0;
    socket_addr[s][0] = addr[0];
    socket_addr[s][1] = addr[1];
    socket_addr[s][2] = addr[2];
    socket_addr[s][3] = addr[3];
    socket_port[s] = FNET_HTONS(port);
    return true;
}

bool EthernetClass::socketSendUDP(uint8_t s, fnet_flag_t flags)
{
    struct fnet_sockaddr_in remoteaddr;
    remoteaddr.sin_family = AF_INET;
    remoteaddr.sin_port = socket_port[s];
    remoteaddr.sin_addr.s_addr = FNET_IP4_ADDR_INIT(socket_addr[s][0], socket_addr[s][1], socket_addr[s][2], socket_addr[s][3]);

    return fnet_socket_sendto(socket_ptr[s], socket_buf_transmit[s], socket_buf_len[s], flags, (struct fnet_sockaddr*)&remoteaddr, sizeof(remoteaddr)) != FNET_ERR ? true : false;
}

EthernetClass Ethernet;
