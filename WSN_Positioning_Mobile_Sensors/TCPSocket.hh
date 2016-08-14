#ifndef TCPSOCKET_HH
#define TCPSOCKET_HH

#include <string>

extern "C" {
   
   #include <sys/types.h>
   #include <sys/socket.h>
   #include <netinet/in.h>
}

class TCPSocket {
   
 public:
   
   TCPSocket( short port, std::string address );
   ~TCPSocket();
   
   bool receivePacket( std::string& packet, unsigned size );
   bool sendPacket( std::string& packet );
   
 private:
   
   int sockfd_;
   struct sockaddr_in addr_;

   TCPSocket( TCPSocket const& orig );
   TCPSocket& operator= ( TCPSocket const& orig );
   
};



#endif
