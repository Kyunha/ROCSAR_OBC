#include <string.h>
#include <unistd.h>
#include "udp_comm.h"

int main()
{
  UDP_Comm_Server server(1968);

  for( ; ; ) {
    time_t now = time(0);
    char* time_str = ctime(& now);
    server.send((unsigned char*) time_str, strlen(time_str));
    sleep(2);
  }
}


