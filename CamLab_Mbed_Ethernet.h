#ifndef CAMLAB_MBED_ETHERNET_H
#define CAMLAB_MBED_ETHERNET_H

#include "mbed.h"
#include "CamLab_Mbed_Serial.h" // Pull Definitions for Communication from here 
#include "EthernetInterface.h"

/* ----------------------------------------------------------------------------
**   -- CamLab_Mbed_Ethernet Communication Class 
**   ---------------------------------------------------------------------------- 
**
** Based upon the CamLab_Mbed_Serial Class, since the communication protocol will be the same at a high-level. 
** Uses DHCP - K64F must be connected via a router and assigned a dynamic IP address (IP4) 
** Pyton uses "arp -a" to find devices on the pre-selected PORT, then uses a similar Command-Response format. 
** Note that Byte order changes occurs due to casting of char[] to uint and vice-versa */

// A better way to code the multiple communication classes is probably to utilise inheritance, but I didn't consider this early enough 



class CamLab_Mbed_Ethernet
{
private:
    int _PORT; // PORT used for network communication 
    
    EthernetInterface eth;
    SocketAddress sock_addr; // socket address used to 
    TCPSocket socket; // socket used to listen on Chosen PORT 
    TCPSocket *eth_COMM; // socket used for communication 

    char buf_eth[MAXIMUM_BUFFER_SIZE] = "{0}";  // Buffer size comes from definition in CamLab_Mbed_Serial.h
public:
    bool eth_Connected = false; 
    bool socket_Connected = false; 

    /**
     * @brief Construct a new CamLab_Mbed_Ethernet object - acts as a TCP Server using DHCP
     * Initialises Ethernt and Scoket in Non-Blocking mode, binds PORT and starts listening for connections. 
     * 
     * @param PORT Defines the network PORT over which the Device listens for connections  
     */
    CamLab_Mbed_Ethernet(int PORT);

    /**
     * @brief Check if there is a physical connection to ethernet socket - update internal status + return stauts as bool.
     * 
     * @return true If there is a valid Ethernet connection return true & update class variable eth_Connected
     * @return false If there is no valid Ethernet connection return false & update class variable eth_Connected
     */
    bool Update_Eth_Status(void); 

    /**
     * @brief Close existing socket and listen for new connection.
     * 
     */
    void Refresh_Socket_Connection(void);

    bool Check_Socket_Valid(void);

    /**
     * @brief Checks if there is a valid Connection + Socket for Ethernet Communication. 
     * Note that this may not mean we can read or write, however that will be dealt with seperately.
     * If there is no valid Conenction/Socket, the device will listen for both everytime this function is called.
     * 
     * 
     * @return true 
     * @return false 
     */
    bool Check_Eth_Status(void);

    /**
     * @brief CURRENTLY NOT IMPLEMENTED - simply deals with connection and echos any recieved data. 
     * We need to refactor all communication protocols to inherit from a base communication class. 
     * We need to contain all communication format in one class, then implementation in separate such as Serial/Ethernet so 
     * that all methods share common instructions/response format. 
     * 
     */
    void Eth_Response(void); // Assumes we have a valid ethernet Connection + Socket 

};




#endif