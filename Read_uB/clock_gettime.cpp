#include <time.h>
#include <sys/time.h>
#include <unistd.h>

int clock_gettime(int dummy, struct timespec* ts)
{
  struct timeval tv;
  gettimeofday(& tv, 0);
  ts->tv_sec = tv.tv_sec;
  ts->tv_nsec = tv.tv_usec*1000;
}
