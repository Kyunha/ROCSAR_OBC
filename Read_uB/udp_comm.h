//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  udp_comm.h                                                          //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  UDP communications class to interchange data between processes.     //
//                                                                      //
//  Server usage:                                                       //
//    Create server instance: UDP_Comm_Server udp_comm_server(<port>);  //
//    Send data throught: udp_comm_server.send(uchar* data, int len);   //
//    (data will be sent to all clients registered in this port)        //
//                                                                      //
//  Client usage:                                                       //
//    Create client instance: UDP_Comm_Client udp_comm_client(<port>);  //
//      or  UDP_Comm_Client udp_comm_client("hostname", <port>);        //
//    Receive data: udp_comm_client.recv(uchar* data, len);             //
//    ("recv" is blocking; should be called at least every minute to    //
//    revalidate registratio; obtain descriptor to wait on "select":    //
//    udp_comm_client.desc();)                                          //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Sergio Cunha (Nov 2005)                                             //
//                                                                      //
//////////////////////////////////////////////////////////////////////////


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <time.h>


const time_t DT_RENEW  = 60;
const time_t DT_EXPIRE = 125;


class UDP_Comm {
  struct sockaddr_in  addr;
  socklen_t           addr_len;
  time_t              expire;
  UDP_Comm*           next;

  friend class UDP_Comm_Server;

public:
  UDP_Comm(struct sockaddr_in client, socklen_t len);
  UDP_Comm(struct sockaddr_in client, socklen_t len, time_t dt);
  void renew(time_t dt);
};


class UDP_Comm_Server {
  int        skt;
  UDP_Comm*  first;

public:
  UDP_Comm_Server(unsigned short port);
  ~UDP_Comm_Server();
  
  size_t send(unsigned char* data, size_t len);
  int    desc(void) { return skt; }
};


class UDP_Comm_Client {
  int                 skt;
  struct sockaddr_in  addr;
  socklen_t           addr_len;
  time_t              renew;
  time_t              dt_expire;
  time_t              dt_renew;
  
public:
  UDP_Comm_Client(const char* server_name, unsigned short port, time_t expire = -1);
  ~UDP_Comm_Client();

  size_t recv(unsigned char* data, size_t len);
  int    desc(void) { return skt; }
  void   send_renew(void);
};
