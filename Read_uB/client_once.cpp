#include <iostream>
using namespace std;
#include <sys/select.h>
// #include <sys/time.h>
// #include <sys/types.h>
// #include <unistd.h>
#include "udp_comm.h"

main()
{
  UDP_Comm_Client client("127.0.0.1", 1968, 0);

  char buf[2048];
  int len = client.recv((unsigned char*) buf, 2048);
  if(len > 0 && len < 2048) {
    buf[len] = '\0';
    cout << buf;
  }
}

