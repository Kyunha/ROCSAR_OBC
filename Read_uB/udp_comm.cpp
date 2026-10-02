//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  udp_comm.cpp                                                        //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  UDP communications class to interchange data between processes.     //
//  Check "udp_comm.h" header file for usage.                           //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Sergio Cunha (Nov 2005)                                             //
//                                                                      //
//////////////////////////////////////////////////////////////////////////


#include <unistd.h>
#include <fcntl.h>
#include <netdb.h>
#include <strings.h>
#include "udp_comm.h"


UDP_Comm::UDP_Comm(struct sockaddr_in client, socklen_t len)
{
  addr = client;
  addr_len = len;
  next = nullptr;
}

UDP_Comm::UDP_Comm(struct sockaddr_in client, socklen_t len, time_t dt)
{
  addr = client;
  addr_len = len;
  renew(dt);
  next = nullptr;
}

void UDP_Comm::renew(time_t dt)
{
  expire = time(0) + dt;
}


UDP_Comm_Server::UDP_Comm_Server(unsigned short port)
{
  first = nullptr;

  skt = socket(AF_INET, SOCK_DGRAM, 0);
  if(skt < 0)
    return;

  // Binding the socket port:
  struct sockaddr_in addr;
  socklen_t addr_len = sizeof(addr); // size_t addr_len = sizeof(addr);
  getsockname(skt, (sockaddr *) & addr, & addr_len);
  addr.sin_port = htons(port);
  if(bind(skt, (sockaddr *) & addr, addr_len)) {
    close(skt);
    skt = -1;
    return;
  }
  
  // Making the socket non-blocking:
  fcntl(skt, F_SETFL, O_NONBLOCK);
}

UDP_Comm_Server::~UDP_Comm_Server()
{
  if(skt >= 0)
    close(skt);

  while(first) {
    UDP_Comm* to_delete = first;
    first = to_delete->next;
    delete to_delete;
  }
}

size_t UDP_Comm_Server::send(unsigned char* data, size_t len)
{
  time_t now = time(0);
  
  // Prune expired clients:
  UDP_Comm** volatile ptr = & first;
  while(*ptr)
    if((*ptr)->expire < now) {
      UDP_Comm* to_delete = *ptr;
      *ptr = to_delete->next;
      delete to_delete;
    } else
      ptr = & ((*ptr)->next);

  // Read socket for new registrations or registration renewals:
  for( ; ; ) {
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    unsigned char buf[2048];
    int recvlen = recvfrom(skt, buf, 2048, 0, (struct sockaddr*) &from, &from_len);
    if(recvlen < 0)
      break;
    // The first two bytes contain the time interval until expiry:
    time_t dt;
    if(recvlen >= 2) {
      dt = (int) buf[0] + 256 * (int) buf[1];
      if(! dt)
        dt = -3;
    } else
      dt = DT_EXPIRE;
    // Check for existing socket (renewal):
    UDP_Comm* ptr = first;
    while(ptr) {
      if(ptr->addr.sin_port == from.sin_port &&
         ptr->addr.sin_addr.s_addr == from.sin_addr.s_addr)
        break;
      ptr = ptr->next;
    }
    if(ptr)
      ptr->renew(dt);
    else {
      UDP_Comm* client = new UDP_Comm(from, from_len, dt);
      client->next = first;
      first = client;
    }
  }
  
  // Send data to all registered clients:
  if(first == nullptr)
    return 0;
  UDP_Comm* volatile p_client = first;
  for( ; ; ) {
    if(p_client == nullptr) 
      return 0;
    sendto(skt, data, len, 0, (struct sockaddr*) &(p_client->addr), p_client->addr_len);
    p_client = p_client->next;
  }
}


UDP_Comm_Client::UDP_Comm_Client(const char* server_name, unsigned short port, time_t expire)
{
  skt = socket(AF_INET, SOCK_DGRAM, 0);
  if(skt < 0)
    return;

  struct hostent* hp;
  if((hp = gethostbyname(server_name)) == NULL) {
    close(skt);
    skt = -1;
    return;
  }
  addr_len = sizeof(sockaddr_in);
  bzero((char *) &addr, addr_len);
  addr.sin_family = hp->h_addrtype;
  bcopy(hp->h_addr, (char *) &addr.sin_addr, hp->h_length);
  addr.sin_port = htons(port);
  
  if(expire < 0) {
    dt_expire = DT_EXPIRE;
    dt_renew  = DT_RENEW;
  } else {
    dt_expire = expire;
    dt_renew = (expire > 0 ? (expire/2)+1 : 1000000);
  }
  send_renew();
}

UDP_Comm_Client::~UDP_Comm_Client()
{
  close(skt);
}

void UDP_Comm_Client::send_renew(void)
{
  unsigned char buf[2];
  buf[0] = dt_expire % 256;
  buf[1] = (dt_expire % 65536) / 256;
  sendto(skt, buf, 2, MSG_DONTWAIT, (struct sockaddr*) &addr, addr_len);
  renew = time(0) + dt_renew;
}

size_t UDP_Comm_Client::recv(unsigned char* data, size_t len)
{
  if(renew < time(0))
    send_renew();

  struct sockaddr_in from;
  socklen_t from_len = sizeof(from);
  return recvfrom(skt, data, len, 0, (struct sockaddr*) &from, &from_len);
}
