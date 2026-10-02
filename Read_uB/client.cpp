#include <iostream>
using namespace std;
#include <sys/select.h>
// #include <sys/time.h>
// #include <sys/types.h>
// #include <unistd.h>
#include "udp_comm.h"

int main()
{
  UDP_Comm_Client client("127.0.0.1", 1968);

  char buf[2048];
  int len;

  for( ; ; ) {
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;

    fd_set rfds;
    FD_ZERO(& rfds);
    FD_SET(client.desc(), & rfds);

    int rs = select(client.desc()+1, &rfds,0,0, &timeout);

    if(! rs) {
      cerr << "timeout receiving data ...\n";
      client.send_renew();
      continue;
    }

    if(rs > 0)
      if(FD_ISSET(client.desc(), &rfds)) {
        len = client.recv((unsigned char*) buf, 2048);
        if(len > 0 && len < 2048) {
          buf[len] = '\0';
          cout << buf;
        }
      }
  }
}


