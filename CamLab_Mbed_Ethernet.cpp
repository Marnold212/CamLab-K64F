#include "CamLab_Mbed_Ethernet.h"

CamLab_Mbed_Ethernet::CamLab_Mbed_Ethernet(int PORT) : _PORT(PORT) {
    eth_COMM = nullptr; 
    // There is a default timeout - but very long 
    // Default in blocking mode - long time before returning no connection 
    eth.set_blocking(false); // Change to asyncronous mode 
    eth.connect();
    
    eth.get_ip_address(&sock_addr);
    printf("IP address: %s\n", sock_addr.get_ip_address() ? sock_addr.get_ip_address() : "None");
    printf("Mac address: %s\n", eth.get_mac_address() ? eth.get_mac_address() : "None");
    // After running eth.connect(), returns 3 if not connected, and 1 if connected to router 

    socket.set_blocking(false); // Similar to eth, set non-blocking to prevent waiting 
    // socket.set_timeout(1);
    // eth_COMM.set_blocking(false); // Not sure if we want to do this or not? 
    socket.open(&eth);

    socket.bind(PORT); // PORT = 5050
    nsapi_error_t error = socket.listen();
    if(error != NSAPI_ERROR_OK){
        printf("Error in Listening\n");
    }

}

bool CamLab_Mbed_Ethernet::Update_Eth_Status(void){ // Returns true if valid physical connection, false otherwise 
    eth_Connected = (eth.get_connection_status() == NSAPI_STATUS_GLOBAL_UP); 
    return eth_Connected; 
}

void CamLab_Mbed_Ethernet::Refresh_Socket_Connection(void){
    socket_Connected = false; 
    eth_COMM->close();  // Destroy COMM Socket 
    eth_COMM = nullptr; // Set pointer to COMM socket back to nullptr. 
    socket.listen();
}

bool CamLab_Mbed_Ethernet::Check_Socket_Valid(void){
    if(!socket_Connected){ // If there is no existing socket, check if any available 
        if(eth_COMM == nullptr){ // Comm Socket not connected so no valid object for eth_COMM
            eth_COMM = socket.accept(); // Set to accept available Sockets on given PORT
            return false; 
        }else{
            socket_Connected = true; 
            return true; 
        }
    }
    else{ // If the connection was vaild in previous loop, assume still vaild 
        return true; 
    }
}

// If there is a failure in the read/write process, this will then set the socket to invalid 
bool CamLab_Mbed_Ethernet::Check_Eth_Status(void){
    if(!eth_Connected){ // If Current Ethernet State is disconnected 
        if(Update_Eth_Status()){ // Check if new ethernet connection available
            printf("Connected to Ethernet - Listening for socket\n");
            eth.get_ip_address(&sock_addr);
            printf("IP address: %s\n", sock_addr.get_ip_address() ? sock_addr.get_ip_address() : "None");
            return Check_Socket_Valid(); // Check if there is a valid socket to connect to  
        }
    }else{  // If Current Ethernet State is Connected
        if(Update_Eth_Status()){ // Check if new ethernet connection available
            return Check_Socket_Valid(); // Check if there is a valid socket to connect to  
        }else{
            printf("Disconnected\n");
        }
    }
    return false;
}

void CamLab_Mbed_Ethernet::Eth_Response(void){ // Assumes we have a valid ethernet Connection + Socket 
    int rcount = eth_COMM->recv(buf_eth, sizeof(buf_eth));
    if(rcount < 0){ // All error codes are negative - see nsapi_error_t
        // printf("Error code: %i with recv data - disconnecting socket\n", rcount);
        Refresh_Socket_Connection(); 
        // printf("Listening for new Sockets\n");
    }else if(rcount == 0){ // No Command recieved, but no error with channel 
        return;
    }else if(rcount <= MAXIMUM_BUFFER_SIZE){
        printf("recv %d [%.*s]\n", rcount, strstr(buf_eth, "\r\n") - buf_eth, buf_eth);  // -3004 is an error code for no connection
        int scount = eth_COMM->send(buf_eth, rcount);
        printf("echo %d [%.*s]\n", scount, strstr(buf_eth, "\r\n") - buf_eth, buf_eth);  // -3004 is an error code for no connection      
    }else{
        Refresh_Socket_Connection(); 
    }      
}