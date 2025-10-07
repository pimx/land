within Platform;

  // External function declaration

  function sendUDP
    input Real values[:];
    input String ipAddress;
    input Integer port;
    external "C" annotation(
      Library = {"ws2_32"},  // <- ADD THIS LINE for Windows sockets
      Include = "#include <stdio.h>
#include <string.h>
#ifdef _WIN32
  #include <winsock2.h>
  #pragma comment(lib, \"ws2_32.lib\")
#else
  #include <sys/socket.h>
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <unistd.h>
#endif

void sendUDP(const double* values, size_t nvalues, const char* ip, int port) {
    static int initialized = 0;
    static int sockfd;
    static struct sockaddr_in servaddr;
    
    #ifdef _WIN32
    if (!initialized) {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2,2), &wsa);
        initialized = 1;
    }
    #else
    initialized = 1;
    #endif
    
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) return;
    
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = inet_addr(ip);
    
    sendto(sockfd, (char*)values, nvalues * sizeof(double), 0,
           (struct sockaddr*)&servaddr, sizeof(servaddr));
           
    #ifdef _WIN32
    closesocket(sockfd);
    #else
    close(sockfd);
    #endif
}");
  end sendUDP;