#include <math.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream.h>
#include <errno.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdlib.h>
#include "Read_Sep.h"
#include "nav_data.h"
#include "udp_comm.h"


#define FILE_PERM S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH

static time_t SOCK_RENEW = 10;
static time_t SOCK_TOUT  = 3;
static char dev_name[128];
static UDP_Comm_Server* udp_server = 0;
static struct sockaddr_in data_server;
static int  sock = -1;
static bool keepgoing = true;

static void Manage_Sep(unsigned char* buf, int len);
static void Process_Sep(unsigned char* msg);
static void ProcessData(unsigned char* buf, int & len, int comm);
static void ProcessMsg(unsigned char* buf, int len, int comm);
// static void ProcessPayload_5895(const unsigned char* msg);
static int SetSerialPort(char *port, int baud);
static int open_comm(const char* device);
static int close_comm(int comm);
static int config_comm(int comm, long baud, int bits, int stop, int parity);
static int raw(int fd);
static int unraw(int fd);
static int open_sock_udp(const char* host, const char* service);
static int close_sock(int sock);
static int OpenBypassUDP(const char* host, u_short port);
static int OpenBypassServUDP(const char* host, const char* service);
static void signal_handler(int sig_no);
static Point3D WGS84_to_ED73(const Point3D & WGS84_xyz);
static Point3D ED73_XYZ_to_LLH(const Point3D & xyz);
static Point3D ProjED73(const Point3D & ED73_llh);
static double  Sigma(double lat1, double lat2);


int main(int argc, char* argv[])
{
  // Catching <CTRL-C> to perform proper hang-up:
  signal(SIGHUP,  signal_handler);
  signal(SIGINT,  signal_handler);
  signal(SIGTERM, signal_handler);

  if(argc<3 || argc>7) {
    cerr << "Usage: " << argv[0] <<
            " <serial device> <udp_port> [<host>] [<service>] [<dc_addr> <dc_port>].\n";
    exit(1);
  }

  // Opening serial device:
  int comm;
  if(!strncmp(argv[1], "gps", 3))
    comm = open(argv[1], O_RDONLY);
  else {
    if(!strncmp(argv[1], "/dev/", 5))
      // comm = open_comm(argv[1]);
      comm = SetSerialPort(argv[1], B115200);
    else {
      char buf[256];
      sprintf(buf, "/dev/%s", argv[1]);
      // comm = open_comm(buf);
      comm = SetSerialPort(buf, B115200);
    }
  }
  if(comm < 0) {
    cerr << "Error opening serial device "<<argv[1]<<".\n";
    exit(1);
  }

  // Opening socket to remote server:
  sock = open_sock_udp((argc>3 ? argv[3] : "localhost"), (argc>4 ? argv[4] : UDP_PORT));
  if(sock < 0) {
    cerr << "Error opening connection to "<<argv[3]<<".\n";
    close_comm(comm);
    exit(1);
  }
  
  // Opening UDP server port to send data to clients:
  UDP_Comm_Server server((unsigned short) atoi(argv[2]));
  if(server.desc() >= 0)
    udp_server = & server;
  else
    cerr << "Error creating socket on port "<<argv[2]<<".\n";

  // Creating a client to receive differential corrections data:
  char* serv_addr = "";
  unsigned short serv_port = 0;
  if(argc >= 7) {
    serv_addr = argv[5];
    serv_port = atoi(argv[6]);
  }
  UDP_Comm_Client client(serv_addr, serv_port);
  UDP_Comm_Client* p_client = 0;
  if(client.desc() >= 0)
    p_client = & client;
  else {
    if(argc >= 7)
      cerr << "Error opening UDP socket to differential corrections server ...\n";
  }
  
  // Basename of device to open:
  strncpy(dev_name, basename(argv[1]), 127);
  dev_name[127] = '\0';

  // Opening file to append data from the GPS receiver:
  time_t timer = time(0);
  struct tm* tblock = localtime(& timer);
  char gps_fname[32];
  sprintf(gps_fname, "gps_%s_%02d%02d%02d_%02d%02d%02d",
                     dev_name,
                     tblock->tm_year-100, tblock->tm_mon+1, tblock->tm_mday,
                     tblock->tm_hour, tblock->tm_min, tblock->tm_sec);
  int gps_fd = open(gps_fname, O_WRONLY|O_CREAT|O_APPEND, FILE_PERM);
  
  // Buffers for data received from socket:
  unsigned char read_sock_buf[BUFFER_SZ];
  int read_sock_len = 0;

  // Exchange data:
  cerr << "Press <CTRL-C> to terminate.\r\n";
  char buf[BUFFER_SZ];
  while(keepgoing) {
    int max_fd = (sock>comm ? sock : comm);

    static time_t last_data = 0;
    if(! last_data)
      last_data = time(0);
    struct timeval timeout;
    timeout.tv_sec = SOCK_TOUT;
    timeout.tv_usec = 0;

    fd_set rfds;
    FD_ZERO(&rfds); FD_SET(comm, &rfds); // FD_SET(sock, &rfds);
    if(p_client) {
      FD_SET(p_client->desc(), &rfds);
      if(p_client->desc() > max_fd)
        max_fd = p_client->desc();
    }

    int rs = select(max_fd+1, &rfds,0,0, (p_client ? &timeout : 0));

    if(rs < 0)
      break;

    if(p_client && time(0)-last_data >= SOCK_RENEW) {
      client.send_renew();
      last_data = time(0);
    }

    if(FD_ISSET(comm, &rfds)) {
      unsigned char read_comm_buf[BUFFER_SZ];
      int n = read(comm, read_comm_buf, BUFFER_SZ);
      if(n <= 0)
        break;
      if(gps_fd >= 0) {
        unsigned char* ptr = read_comm_buf;
        int len = n;
        while(len) {
          int wr = write(gps_fd, ptr, len);
          if(wr <= 0)
            break;
          ptr += wr;
          len -= wr;
        }
      }
      Manage_Sep(read_comm_buf, n);
    }

    if(p_client && FD_ISSET(p_client->desc(), &rfds)) {
      unsigned char read_cdif_buf[BUFFER_SZ];
      int n = read(p_client->desc(), read_cdif_buf, BUFFER_SZ);
      if(n <= 0)
        break;
      if(comm >= 0) {
        unsigned char* ptr = read_cdif_buf;
        int len = n;
        while(len) {
          int wr = write(comm, ptr, len);
          if(wr <= 0)
            break;
          ptr += wr;
          len -= wr;
        }
      }
    }

    if(false && FD_ISSET(sock, &rfds)) {
      int n = read(sock, read_sock_buf+read_sock_len, BUFFER_SZ-read_sock_len);
      if(n <= 0)
        break;
      read_sock_len += n;
      ProcessData(read_sock_buf, read_sock_len, comm);
    }
  }
  cerr << "\r\n";

  close(gps_fd);
  close_comm(comm);
  close_sock(sock);
  exit(0);
}


void Manage_Sep(unsigned char* buf, int len)
{
  // UpdateEphemerisFile(60);

  static unsigned char sep_data[SEP_DATA_LEN];
  static int sep_data_len = 0;

  while(len) {
    int added = len;
    if(len + sep_data_len > SEP_DATA_LEN)
      added = SEP_DATA_LEN - sep_data_len;
    memcpy(sep_data+sep_data_len, buf, added);
    sep_data_len += added;
    buf += added;
    len -= added;

    // Processing the SEP binary data:
    while(sep_data_len) {
      unsigned char* msg = sep_data;
      int msg_bytes = sep_data_len;
      while(*msg != SOH1 && msg_bytes) {
        msg++;
        msg_bytes--;
      }
      if(! msg_bytes) {
        sep_data_len = 0;
        break;
      }
      // We have a SOH1: check header integrity:
      if(msg_bytes < 8) {
        if(msg != sep_data) {
          sep_data_len -= (msg-sep_data);
          memmove(sep_data, msg, sep_data_len);
        }
        break;		// waiting for more data ...
      }
      if(msg[1] != SOH2) {
        // This is not a new message after all:
        msg[0] = '\0';
        continue;
      }
      int msg_len = (int) msg[6] + 256*msg[7];
      if(msg_len < 8 || msg_len > 4096) {
        // This length is not possible. Message incorrect. Start again:
        msg[0] = '\0';
        continue;
      }
      // We have a possible message: check for completeness:
      if(msg_bytes < msg_len) {
        if(msg != sep_data) {
          sep_data_len -= (msg-sep_data);
          memmove(sep_data, msg, sep_data_len);
        }
        break;		// waiting for more data ...
      }
      // Checking the checksum:
      unsigned short chk = calc_crc(msg+4, msg_len-4);
      if((chk%0x0100) != msg[2] || (chk/0x0100) != msg[3]) {
        // This message is not correct. Start again:
        msg[0] = '\0';
        continue;
      }
      // We have a valid message. Process it:
      Process_Sep(msg);
      // Taking the message data out of the buffer:
      msg += msg_len;
      sep_data_len -= (msg-sep_data);
      if(sep_data_len)
        memmove(sep_data, msg, sep_data_len);
    }

    // Dumping data if buffer is full, just in case:
    if(sep_data_len == SEP_DATA_LEN)
      memmove(sep_data, sep_data+1, --sep_data_len);
  }
}

MeasMsg meas;
void Process_Sep(unsigned char* msg)
{
  // The message is assumed to be consistent ...
  int type = 256*msg[5] + (int) msg[4];
  int len  = (int) msg[6] + 256*msg[7];

  // static MeasMsg meas;
  static NavData nav_data;
  static double Vup_hist[8];
  static int Vup_i = 0, Vup_n = 0;
  // Leap seconds estimate:
  static time_t leap_seconds = 14;

  static bool initialized = false;
  if(! initialized) {
    meas.gps_tow = -1.0;
    meas.num_blks = meas.num_blks1 = meas.num_blks2 = 0;
    for(int i=0; i<16; i++)
      meas.gps_blk[i].sv = meas.gps_blk1[i].sv = meas.gps_blk2[i].sv = (char) 0xFF;
    meas.course = 0.0;
    meas.Cnn = meas.Cee = meas.Chh = meas.Ctt = meas.Cne = 99.9;
    meas.Cnh = meas.Cnt = meas.Ceh = meas.Cet = meas.Cht = 99.9;
    // NavData structure:
    for(int i=0; i<8; i++)
      nav_data.label[i] = '\0';
    strncpy(nav_data.label, dev_name, 8);
    nav_data.Roll = nav_data.Pitch = nav_data.Heading = 0.0;
    nav_data.stage = STG_UNKNW;
    nav_data.PREV_time = nav_data.LD35_time
                       = nav_data.LDNW_time = 0;
    nav_data.LD35_lat = nav_data.LD35_lon = 0.0;
    nav_data.LD35_cvNN = nav_data.LD35_cvNE =
      nav_data.LD35_cvEE = nav_data.LD35_sdMX = 99.9;
    nav_data.LDNW_lat = nav_data.LDNW_lon = 0.0;
    nav_data.LDNW_cvNN = nav_data.LDNW_cvNE =
      nav_data.LDNW_cvEE = nav_data.LDNW_sdMX = 99.9;
    nav_data.SENS_time.tv_sec = nav_data.SENS_time.tv_nsec = 0;
    for(int i=0; i<20; i++)
      nav_data.sensor[i] = 0.0;
    for(int j=0; j<8; j++)
      Vup_hist[j] = 0.0;
    // Now the variables are initialized:
    initialized = true;
  }

  // char mt_str[64];
  // sprintf(mt_str, "message %04d, length %d.", type, len);
  // cerr << mt_str << "\n";
  
  switch(type) {
    case 5904: if(len >= 76) {      // PVTGeodetic
                 // Message time:
                 double gps_tow = ((double) *((unsigned long*) (msg+8)))/1000.0;
                 double dT = gps_tow - meas.gps_tow;
                 if(dT > 302400.0)
                   dT -= 604800.0;
                 if(dT < -302400.0)
                   dT += 604800.0;
                 if(fabs(dT) > 0.01) {
                   meas.num_blks = meas.num_blks1 = meas.num_blks2 = 0;
                   for(int i=0; i<16; i++)
                     meas.gps_blk[i].sv = meas.gps_blk1[i].sv = meas.gps_blk2[i].sv = (char) 0xFF;
                 }
                 meas.gps_tow = gps_tow;
                 // Position related data:
                 meas.latitude  = *((double*) (msg+20));
                 meas.longitude = *((double*) (msg+28));
                 meas.height    = *((double*) (msg+36));
                 meas.Vn = *((float*) (msg+44));
                 meas.Ve = *((float*) (msg+48));
                 meas.Vd = - *((float*) (msg+52));
                 meas.Vh = sqrt(meas.Vn*meas.Vn + meas.Ve*meas.Ve);
                 if(meas.Vh > VEL_MIN)
                   meas.course = atan2(meas.Ve, meas.Vn);
                 meas.velocity = sqrt(meas.Vh*meas.Vh + meas.Vh*meas.Vh);
                 meas.clk_off   = *((double*) (msg+56));
                 meas.distance  = 0.0;  // not used; means nothing ...
                 meas.mode = (int) msg[16];
                 BroadcastData(meas);
                 nav_data.NAV_time.tv_sec = (time_t) (*((unsigned long *) (msg+8))/1000 +
                                                      *((short *) (msg+12)) * 604800L +
                                                      315964800L - leap_seconds);
                 nav_data.NAV_time.tv_nsec = (time_t) ((*((unsigned long *) (msg+8)))%1000) * 1000000;
                 nav_data.Latitude = meas.latitude * RAD2DEG;
                 nav_data.Longitude = meas.longitude * RAD2DEG;
                 nav_data.Altitude = meas.height;
                 nav_data.Vn = meas.Vn;
                 nav_data.Ve = meas.Ve;
                 nav_data.Vup = -meas.Vd;
                 nav_data.Vel = meas.velocity;
                 nav_data.Vh = meas.Vh;
                 nav_data.Course = meas.course*RAD2DEG;
                 nav_data.Course -= 360.0*floor(nav_data.Course/360.0);
                 if(fabs(nav_data.Vup) >= 0.5 || meas.mode > 0) {
                   // Velocity is trusted if large enough or error is small.
                   Vup_hist[Vup_i++] = nav_data.Vup;
                   Vup_i %= 8;
                   if(Vup_n < 8)
                     Vup_n++;
                   double Vup_use = nav_data.Vup;
                   if(fabs(nav_data.Vup) < 0.5) {
                     // In this case, error is small:
                     if(Vup_n == 8) {
                       Vup_use = 0;
                       for(int i=0; i<8; i++)
                         Vup_use += Vup_hist[i];
                       Vup_use /= 8.0;
                       nav_data.flags = LLH_VALID | VEL_VALID | TRUST_SUR;
                       if(Vup_use > 0.5)
                         nav_data.flags |= TREND_CLB;
                       else if(Vup_use < -0.5)
                         nav_data.flags |= TREND_DSC;
                       else
                         nav_data.flags |= TREND_STP;
                     } else
                       // Waiting for more data ...
                       nav_data.flags = LLH_VALID | VEL_VALID | TREND_INV | TRUST_DKN;
                   } else {
                     // In this case, velocity is large:
                     nav_data.flags = LLH_VALID | VEL_VALID;
                     if(Vup_use > 2.0)
                       nav_data.flags |= TREND_CLB | TRUST_SUR;
                     else if(Vup_use < -2.0)
                       nav_data.flags |= TREND_DSC | TRUST_SUR;
                     else
                       nav_data.flags |= TREND_STP | TRUST_MLD;  // doesn't hap.
                   }
                 } else {
                   Vup_i = Vup_n = 0;
                   nav_data.flags = LLH_VALID | VEL_VALID | TREND_INV | TRUST_DKN;
                 }
                 if(fabs(meas.Cnn - 99.9) > 0.1 || fabs(meas.Cee - 99.9) > 0.1) {
                   nav_data.LDNW_time = nav_data.NAV_time.tv_sec;
                   nav_data.LDNW_lat = nav_data.Latitude;
                   nav_data.LDNW_lon = nav_data.Longitude;
                   nav_data.LDNW_cvNN = meas.Cnn;
                   nav_data.LDNW_cvNE = meas.Cne;
                   nav_data.LDNW_cvEE = meas.Cee;
                   double aux1 = nav_data.LDNW_cvNN + nav_data.LDNW_cvEE;
                   double aux2 = nav_data.LDNW_cvNN - nav_data.LDNW_cvEE;
                   nav_data.LDNW_sdMX = 0.5*(aux1 + sqrt(aux2*aux2+4.0*nav_data.LDNW_cvNE*nav_data.LDNW_cvNE));
                   nav_data.LDNW_sdMX = sqrt(nav_data.LDNW_sdMX);
                   nav_data.flags |= PRV_VALID;
                 }
                 udp_server->send((unsigned char*) & nav_data, sizeof(nav_data));
               }
               break;
    case 5906: if(len >= 56) {      // PosCovGeodetic
                 meas.Cnn = *((float*) (msg+16));
                 meas.Cee = *((float*) (msg+20));
                 meas.Chh = *((float*) (msg+24));
                 meas.Ctt = *((float*) (msg+28));
                 meas.Cne = *((float*) (msg+32));
                 meas.Cnh = *((float*) (msg+36));
                 meas.Cnt = *((float*) (msg+40));
                 meas.Ceh = *((float*) (msg+44));
                 meas.Cet = *((float*) (msg+48));
                 meas.Cht = *((float*) (msg+52));
               }
               break;
    case 5909: if(len >= 32) {      // DOP
                 float dummy;
                 memcpy((void*) (& dummy), msg+24, 4);
                 double hpl = (double) dummy;
                 memcpy((void*) (& dummy), msg+28, 4);
                 double vpl = (double) dummy;
               }
               break;
    case 5944: if(len >= 20) {   // GenMeasEpoch
                 // Message time:
                 meas.gps_tow = ((double) *((unsigned long*) (msg+8)))/1000.0;
                 // Raw data:
                 int nsv = (int) msg[14];
                 int len_sb1 = (int) msg[15];
                 int len_sb2 = (int) msg[16];
                 int nb = meas.num_blks = meas.num_blks1 = meas.num_blks2 = 0;
                 for(int i=0; i<16; i++)
                   meas.gps_blk[i].sv = meas.gps_blk1[i].sv = meas.gps_blk2[i].sv = (char)  0xFF;
                 unsigned char* p_sb = msg+20;
                 for(int i=0; i<nsv; i++) {
                   // Read type 1 sub-block first:
                   if(p_sb+len_sb1 > msg+len)   // incomplete message
                     break;
                   if(p_sb[16] != 0xFF || p_sb[17] != 0xFF) {
                     meas.gps_blk[nb].sv      = (char) p_sb[2];
                     meas.gps_blk[nb].pr      = (p_sb[3]&0x0F)*4294967.296 +
                                                *((unsigned long*) (p_sb+4))*0.001;
                     long dcphase = 0;
                     memcpy((void*) & dcphase, (void *) (p_sb+12), 3);
                     if(dcphase & 0x00800000)
                       dcphase |= 0xFF000000;
                     meas.gps_blk[nb].cphase  = meas.gps_blk[nb].pr / GPS_HZ2M +
                                                0.001 * dcphase;
                     meas.gps_blk[nb].dp      = -(*((long *) (p_sb+8)))*0.0001;
                     meas.gps_blk[nb].snr     = (double) p_sb[15] * 0.25;
                     meas.gps_blk[nb].n_cslip = (int) ((unsigned short*) (p_sb+16));
                     int n_sb2 = (int) p_sb[19];
                     p_sb += len_sb1;
                     for(int j=0; j<n_sb2; j++) {
                       if(p_sb+len_sb2 > msg+len)   // incomplete message
                         break;
                       long prl = ((long) (p_sb[3]&0x07)) << 16;
                       prl |= (long) *((unsigned short*) (p_sb+6));
                       if(prl & 0x00040000)
                         prl |= 0xFFF80000;
                       double pr;
                       if(prl != 0xFFFC0000)
                         pr = meas.gps_blk[nb].pr + 0.001*(double)prl;
                       else
                         pr = -1.0;
                       long dcphase = ((long) p_sb[4]) << 16;
                       dcphase |= (long) *((unsigned short*) (p_sb+8));
                       if(dcphase & 0x00800000)
                         dcphase |= 0xFF000000;
                       double cphase = 0.001 * dcphase;
                       if(dcphase != 0xFF800000)
                         cphase  += ((p_sb[0]&0x1F)>=2 ? pr/LAM_L2 : pr/LAM_L1);
                       else
                         cphase = -1.0;
                       long dopl = ((long) (p_sb[3]&0xF8)) << (16-3);
                       dopl |= (long) *((unsigned short*) (p_sb+10));
                       if(dopl & 0x00100000)
                         dopl |= 0xFFE00000;
                       double dop = 0.0001*(double)dopl;
                       if(dopl != 0xFFF00000)
                         if((p_sb[0]&0x1F) >= 2)
                           dop += meas.gps_blk[nb].dp * (LAM_L1/LAM_L2);
                         else
                           dop += meas.gps_blk[nb].dp;
                       else
                         dop = -1.0;
                       switch(p_sb[0]&0x1F) {
                         case 1: meas.gps_blk1[nb].cphase = cphase;
                                 meas.gps_blk1[nb].pr = pr;
                                 meas.gps_blk1[nb].dp = dop;
                                 meas.gps_blk1[nb].snr = (double) p_sb[2] * 0.25;
                                 meas.gps_blk1[nb].n_cslip = (int) p_sb[1];
                                 meas.gps_blk1[nb].sv = meas.gps_blk[nb].sv;
                                 break;
                         case 2: meas.gps_blk2[nb].cphase = cphase;
                                 meas.gps_blk2[nb].pr = pr;
                                 meas.gps_blk2[nb].dp = dop;
                                 meas.gps_blk2[nb].snr = (double) p_sb[2] * 0.25;
                                 meas.gps_blk2[nb].n_cslip = (int) p_sb[1];
                                 meas.gps_blk2[nb].sv = meas.gps_blk[nb].sv;
                                 break;
                         default: cerr << "Unknown subframe in message 5944 ...\n";
                       }
                       p_sb += len_sb2;
                     }
                     nb++;
                   }
                 }
                 meas.num_blks = meas.num_blks1 = meas.num_blks2 = nb;
                 /*
				         if(raw4_fp) {
 				           fprintf(raw4_fp, "%d\t%d", *((int *) (msg+8)), nsv);
                   for(int i=0; i<nb; i++) {
				             // fprintf(raw4_fp, "\t%d\t%.4f\t%.4f\t%.5f\t%.4f",
                     //         (int) meas.gps_blk[i].sv,
                     //         meas.gps_blk[i].pr,
                     //         meas.gps_blk[i].cphase,
                     //         meas.gps_blk[i].dp,
                     //         meas.gps_blk2[i].cphase);
				             fprintf(raw4_fp, "\t%d\t%.4f\t%.4f\t%.5f\t%.1f",
                             (int) meas.gps_blk[i].sv,
                             meas.gps_blk[i].pr,
                             meas.gps_blk[i].cphase,
                             meas.gps_blk[i].dp,
                             meas.gps_blk[i].snr);
				             fprintf(raw4_fp, "\t%d\t%.4f\t%.4f\t%.5f\t%.1f",
                             (int) meas.gps_blk1[i].sv,
                             meas.gps_blk1[i].pr,
                             meas.gps_blk1[i].cphase,
                             meas.gps_blk1[i].dp,
                             meas.gps_blk1[i].snr);
				             fprintf(raw4_fp, "\t%d\t%.4f\t%.4f\t%.5f\t%.1f",
                             (int) meas.gps_blk2[i].sv,
                             meas.gps_blk2[i].pr,
                             meas.gps_blk2[i].cphase,
                             meas.gps_blk2[i].dp,
                             meas.gps_blk2[i].snr);
                   }
                   for( ; i<16; i++) {
				             // fprintf(raw4_fp, "\tNaN\tNaN\tNaN\tNaN\tNaN");
				             fprintf(raw4_fp, "\tNaN\tNaN\tNaN\tNaN\tNaN");
				             fprintf(raw4_fp, "\tNaN\tNaN\tNaN\tNaN\tNaN");
				             fprintf(raw4_fp, "\tNaN\tNaN\tNaN\tNaN\tNaN");
                   }
                   fprintf(raw4_fp, "\n");
                   fflush(raw4_fp);
				         }
                 */
				       }
               break;
    case 5895: if(len >= 56)
                 // ProcessPayload_5895(msg+14);
               break;
  }
}


void BroadcastData(const MeasMsg & meas)
{
  static UDP_message message;

  message.Start          = 0xAA;
  message.Epochs         = 1;
  message.Data.GPStime   = meas.gps_tow;
  message.Data.Latitude  = meas.latitude * RAD2DEG;
  message.Data.Longitude = meas.longitude * RAD2DEG;
  message.Data.Height    = meas.height;
  message.Data.Vnorth    = meas.Vn;
  message.Data.Veast     = meas.Ve;
  message.Data.Vdown     = meas.Vd;
  message.Data.Roll      = 0.0;
  message.Data.Pitch     = 0.0;
  message.Data.Heading   = meas.course * RAD2DEG;
  message.Data.AccBiX = message.Data.AccBiY = message.Data.AccBiZ = 0.0;
  message.Data.GyrBiX = message.Data.GyrBiY = message.Data.GyrBiZ = 0.0;
  message.Data.Ganom     = 0.0;
  message.End            = 0x99;

  // if(send(sock, (char *) &message, sizeof(message), 0) < 0)
  if(sendto(sock, (char *) &message, sizeof(message), 0,
     (const struct sockaddr*) &data_server, sizeof(data_server)) < 0)
    cerr << "Can't send message\n";
}


static void ProcessData(unsigned char* buf, int & len, int comm)
{
  for( ; ; ) {
    if(! len)
      return;
    unsigned char* start;
    if(!(start = (unsigned char*) memchr(buf, int('#'), len))) {
      len = 0;
      return;
    }
    if(start != buf) {
      len -= int(start - buf);
      memmove(buf, start, len);
      }
    if(len < 3)
      return;
    int msg_len = buf[1] + 256*buf[2];
    if(msg_len < 5 || msg_len > BUFFER_SZ) {
      buf[0] = '?';
      continue;
    }
    if(len < msg_len)
      return;
    if(buf[msg_len-1] != '%') {
      buf[0] = '?';
      continue;
    }
    // Now we have a valid message; let's processe it:
    ProcessMsg(buf, msg_len, comm);
    // Eliminate message from buffer:
    len -= msg_len;
    if(len)
      memmove(buf, buf+msg_len, len);
  }
}

static void ProcessMsg(unsigned char* buf, int len, int comm)
{
  switch(buf[3]) {
    case 'G':
      {
        unsigned char* ptr = buf+4;
        int n = len-5;
        while(n) {
          int wr = write(comm, ptr, n);
          if(wr <= 0)
            return;
          ptr += wr;
          n -= wr;
        }
        cout << "Message to GPS (" << len-5 << " bytes)";
        if(len>=9 && buf[4]==0xA0 && buf[5]==0xA2)
          cout << " (msg id = " << int(buf[8]) << ")";
        struct timespec tv;
        clock_gettime(CLOCK_REALTIME, & tv);
        char hora[16];
        sprintf(hora, "%03d.%03d", int(tv.tv_sec % 1000),
                                   int(tv.tv_nsec / 1000000));
        cout << " (" << hora << ")\n";
      }
      break;
    case 'B':
      {
        if(len < 12)
          return;
        long baud  = *((unsigned long*) (buf+4));
        int bits   = int(buf[8]);
        int stop   = int(buf[9]);
        int parity = int(buf[10]);
        cout << "Serial port configured to " << baud << ',' << bits << ','
             << (stop==1 ? "1/2" : (stop==2 ? "1" : (stop==4 ? "2" : "?"))) << ','
             << (parity==0 ? 'N' : (parity==1 ? 'O' : (parity==2 ? 'E' : '?'))) << '\n';
        if(config_comm(comm, baud, bits, stop, parity))
          cerr << "Failed to configure port !!!\n";
      }
      break;
    case 'I':
      {
        struct timespec tv;
        clock_gettime(CLOCK_REALTIME, & tv);
        char hora[16];
        sprintf(hora, "%03d.%03d", int(tv.tv_sec % 1000),
                                   int(tv.tv_nsec / 1000000));
        cout << "Query for ID (" << hora << ")\n";
      }
      break;
    default:
      break;
  }
}


static int SetSerialPort(char *port, int baud)
{
  int fd;
  struct termios oldtio, newtio;

  fprintf(stderr, "Opening port %s with baud rate %d.\n", port, baud);
  fd = open(port, O_RDWR | O_NOCTTY | O_SYNC); 
  // fd = open(port, O_RDONLY | O_NOCTTY | O_SYNC); 
  if ( fd <0 )
    return -1;

  /* save current port settings */
  if ( tcgetattr(fd, &oldtio) )
    return -1;
  
  bzero(&newtio, sizeof(newtio));
  //newtio.c_cflag = baud | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_cflag = baud | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  
  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;
  
  /* inter-character timer */
  newtio.c_cc[VTIME]    = 10;  
  /* blocking read until x chars received */
  newtio.c_cc[VMIN]     = 64;   
  
  if ( tcflush(fd, TCIFLUSH) < 0)
    return -1;

  if (tcsetattr(fd, TCSANOW, &newtio) < 0)
    return -1;

  return fd;
}


static int open_comm(const char* device)
{
  char buf[256];
  if(!strncmp(device, "/dev/", 5))
    sprintf(buf, "%s", device);
  else
    sprintf(buf, "/dev/%s", device);

  int fd;
  if((fd = open(buf, O_RDWR )) < 0) {
    cerr << "open_comm -- open: " << strerror(errno) << "\n";
    return -1;
  }

  // Putting the COMM port into NONBLOCKing mode:
  fcntl(fd, F_SETFL, O_NONBLOCK);

  /*
  struct termios ttystate;
  if(tcgetattr(fd, &ttystate) < 0) {
    cerr << "open_comm -- tcgetattr: " << strerror(errno) << "\n";
    return -1;
  }
  ttystate.c_iflag |= IGNCR;
  ttystate.c_oflag &= ~(OCRNL | NLDLY | CRDLY | TABDLY | ONLRET);
  ttystate.c_oflag |= (ONLCR | OPOST);
  ttystate.c_cflag &= ~(CBAUD | CSIZE | PARENB | CSTOPB);
  ttystate.c_cflag |= (B9600 | CS8 | CREAD);
  ttystate.c_lflag &= ~(ECHO);
  if(tcsetattr(fd, TCSANOW, &ttystate) < 0) {
    cerr << "open_comm -- tcsetattr: " << strerror(errno) << "\n";
    return -1;
  }
  */
  if(config_comm(fd, 115200, 8, 2, 0))
    cerr << "Initial COMM configuration failed !!!\n";

  return fd;
}

static int close_comm(int comm)
{
  if(close(comm) < 0) {
    cerr << "close_comm -- close: " << strerror(errno) << "\n";
    return -1;
  }
  return 0;
}

static int config_comm(int comm, long baud, int bits, int stop, int parity)
{
  struct termios termios_p;
  if(tcgetattr(comm, & termios_p))
    return -1 ;

  // Put in RAW mode:
  termios_p.c_cc[VMIN]  =  1;
  termios_p.c_cc[VTIME] =  0;
  /*
  termios_p.c_lflag &= ~( ECHO|ICANON|ISIG|
                         ECHOE|ECHOK|ECHONL );
  termios_p.c_oflag &= ~( OPOST );
  */
  termios_p.c_iflag = IGNBRK;
  // termios_p.c_oflag = 0;
  termios_p.c_oflag = OPOST;
  termios_p.c_cflag = CLOCAL | HUPCL | CREAD;
  termios_p.c_lflag = 0;
  
  // Set baud rate:
  if(cfsetispeed(& termios_p, (speed_t) baud))
    return -1;
  if(cfsetospeed(& termios_p, (speed_t) baud))
    return -1;
    
  // Set data bits:
  termios_p.c_cflag &= ~ CSIZE;
  switch(bits) {
    case  5: termios_p.c_cflag |= CS5;  break;
    case  6: termios_p.c_cflag |= CS6;  break;
    case  7: termios_p.c_cflag |= CS7;  break;
    case  8: termios_p.c_cflag |= CS8;  break;
    default: return -1;
  }

  // Set stop bits:
  switch(stop) {
    case  2: termios_p.c_cflag &= ~ CSTOPB;  break;
    case  4: termios_p.c_cflag |=   CSTOPB;  break;
    default: return -1;
  }

  // Set parity:
  switch(parity) {
    case  0: termios_p.c_cflag &= ~ PARENB;  break;
    case  1: termios_p.c_cflag |= PARENB;
             termios_p.c_cflag |= PARODD;
             break;
    case  2: termios_p.c_cflag |= PARENB;
             termios_p.c_cflag &= ~ PARODD;
             break;
    default: return -1;
  }

  return tcsetattr(comm, TCSADRAIN, & termios_p);
  // return tcsetattr(comm, TCSANOW, & termios_p);
}

static int raw(int fd)
{
  struct termios termios_p;

  if( tcgetattr( fd, &termios_p ) )
    return( -1 );

  termios_p.c_cc[VMIN]  =  1;
  termios_p.c_cc[VTIME] =  0;
  termios_p.c_lflag &= ~( ECHO|ICANON|ISIG|
                         ECHOE|ECHOK|ECHONL );
  termios_p.c_oflag &= ~( OPOST );
  return( tcsetattr( fd, TCSADRAIN, &termios_p ) );
}

static int unraw(int fd)
{
  struct termios termios_p;

  if( tcgetattr( fd, &termios_p ) )
    return( -1 );

  termios_p.c_lflag |= ( ECHO|ICANON|ISIG|
                        ECHOE|ECHOK|ECHONL );
  termios_p.c_oflag |= ( OPOST );
  return( tcsetattr( fd, TCSADRAIN, &termios_p ) );
}


static int open_sock_udp(const char* host, const char* service)
{
  int fd;
  if(*service >= '0' && *service <= '9')
    fd = OpenBypassUDP(host, atoi(service));
  else
    fd = OpenBypassServUDP(host, service);
  if(fd < 0) {
    cerr << "Error connecting to Bypass server in open_sock.\n";
    return -1;
  }

  return fd;
}

static int close_sock(int sock)
{
  shutdown(sock, 2);
  if(close(sock) < 0) {
    cerr << "close_sock -- close: " << strerror(errno) << "\n";
    return -1;
  }
  return 0;
}

#define BYE_SOCK(sock)	{ close(sock); return -1; }

static int OpenBypassUDP(const char* host, u_short port)
{
  struct hostent* hst;
  struct sockaddr_in server;
  int sock;

  if(!(hst = gethostbyname(host)))
    return -1;
  if((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    return -1;

  server.sin_family = AF_INET;
  memcpy((char*) &server.sin_addr, hst->h_addr, hst->h_length);
  server.sin_port = htons(port);
  
  data_server = server;

  /*
  if(connect(sock, (const struct sockaddr*) & server, sizeof(server)) < 0)
    BYE_SOCK(sock);
  */

  return sock;
}

static int OpenBypassServUDP(const char* host, const char* service)
{
  struct servent* svent;

  if(! (svent = getservbyname(service, "udp")))
    return -1;
  return OpenBypassUDP(host, ntohs(svent->s_port));
}


static void signal_handler(int sig_no)
{
  switch(sig_no) {
    case SIGHUP  :
    case SIGINT  :
    case SIGTERM : keepgoing = false;
                   cerr << "Caught signal: good bye ...\n";
                   break;
    default:       break;
  }
}


//
// LLH to XYZ and vice-versa transformation routines:
// (values in radians and meters)
//

Point3D LLH_to_XYZ(const Point3D & llh)
{
  static double lat0 = 100.0, lon0 = 0.0, h0 = -10000.0;
  static double cLat, sLat, cLon, sLon, N;
  static Point3D xyz;
  
  double lat = llh.X;
  double lon = llh.Y;
  double h   = llh.Z;
  
  if(fabs(lat-lat0) > (9e-9*DEG2RAD) || fabs(lon-lon0) > (9e-9*DEG2RAD)) {
    cLat = cos(lat);
    sLat = sin(lat);
    cLon = cos(lon);
    sLon = sin(lon);

    N = WGS84MAJ*WGS84MAJ / sqrt(WGS84MAJ*WGS84MAJ*cLat*cLat +
        WGS84MIN*WGS84MIN*sLat*sLat);
    lat0 = lat;
    lon0 = lon;
    h0 = h + 1.0;               // Just to provoque new height computation ...
  }
  if(fabs(h-h0) > 1e-2) {
     xyz.X = (N+h)*cLat*cLon;
     xyz.Y = (N+h)*cLat*sLon;
     xyz.Z = (WGS84MIN*WGS84MIN/(WGS84MAJ*WGS84MAJ)*N+h)*sLat;
     h0 = h;
  }

  return xyz;
}

Point3D XYZ_to_LLH(const Point3D & xyz)
{
  static double x0 = 0.0, y0 = 0.0, z0 = 0.0;
  static Point3D llh;

  double x = xyz.X;
  double y = xyz.Y;
  double z = xyz.Z;

  if(fabs(x-x0) < 1e-2 && fabs(y-y0) < 1e-2 && fabs(z-z0) < 1e-2)
    return llh;

  const double a = WGS84MAJ;	// Semi-major axis of Earth;
  const double b = WGS84MIN; 	// Semi-minor axis of Earth.

  double xy2 = x*x + y*y;
  double xy  = sqrt(xy2);

  const double en2 = (a*a-b*b)/b;
  const double ed2 = en2*b/a;

  double den2 = z*z*(a*a) + xy2*(b*b);
  double den  = sqrt(den2);
  double den3 = den2*den;

  double lat = atan2(z+(en2*a*a*a)*z*z*z/den3, xy-(ed2*b*b*b)*xy2*xy/den3);
  double lon = atan2(y, x);

  double CLat = cos(lat);
  double SLat = sin(lat);
  double N = a*a/sqrt(a*a*CLat*CLat + b*b*SLat*SLat);
  double h = xy/CLat - N;

  llh.X = lat;
  llh.Y = lon;
  llh.Z = h;

  x0 = x;
  y0 = y;
  z0 = z;

  return llh;
}

Point3D XYZ_to_NED(const Point3D & xyz, const Point3D & llh)
{
  double cLat = cos(llh.X);
  double sLat = sin(llh.X);
  double cLon = cos(llh.Y);
  double sLon = sin(llh.Y);

  Point3D ned, Un, Ue, Ud;

  Un.X = -sLat*cLon;
  Un.Y = -sLat*sLon;
  Un.Z =  cLat;
  ned.X = Un.X*xyz.X + Un.Y*xyz.Y + Un.Z*xyz.Z;

  Ue.X = -sLon;
  Ue.Y =  cLon;
  //Ue.Z = 0;
  ned.Y = Ue.X*xyz.X + Ue.Y*xyz.Y;

  Ud.X = -cLat*cLon;
  Ud.Y = -cLat*sLon;
  Ud.Z = -sLat;
  ned.Z = Ud.X*xyz.X + Ud.Y*xyz.Y + Ud.Z*xyz.Z;

  return ned;
}


//
// LLH and XYZ to ED73 transformation routines.
//

Point3D LLH_to_ED73(const Point3D & llh)
{
  double lat = llh.X;
  double lon = llh.Y;

  Point3D ed73;

  if(ED73_POLY) {
    const double x1  =  1118841.494;
    const double x2  =  -619973.813;
    const double x3  =  7817812.032;
    const double x4  =    17645.084;
    const double x5  =    -2713.340;
    const double x6  = -4189517.464;

    const double y1  = -4356775.635;
    const double y2  =  6317461.832;
    const double y3  =   455132.604;
    const double y4  =    30981.485;
    const double y5  =  1582932.374;
    const double y6  =    -8075.704;
  
    double latlat = lat*lat;
    double lonlon = lon*lon;
    double latlon = lat*lon;
  
    ed73.X = x1 + x2*lat + x3*lon + x4*latlat + x5*lonlon + x6*latlon;
    ed73.Y = y1 + y2*lat + y3*lon + y4*latlat + y5*lonlon + y6*latlon;
    ed73.Z = llh.Z;       // keeping height to ease debugging ...
  }
  else {
    Point3D WGS84_xyz = LLH_to_XYZ(llh);
    Point3D  ED73_xyz = WGS84_to_ED73(WGS84_xyz);
    Point3D  ED73_llh = ED73_XYZ_to_LLH(ED73_xyz);
    ed73 = ProjED73(ED73_llh);
  }

  // Checking and performing a (200,300) Km offset to simulate
  // military coordinates projection ...
  if(ED73_OFFSET) {
    ed73.X += 200000.0;
    ed73.Y += 300000.0;
  }

  return ed73;
}

Point3D XYZ_to_ED73(const Point3D & xyz)
{
  return LLH_to_ED73(XYZ_to_LLH(xyz));
}

//
// Constants related to ED73 projection:
//
static const double ah   = 6378388.0;                  // meters
static const double bh   = 6356911.946;                // meters
static const double eh2  = (ah*ah - bh*bh)/(ah*ah);
static const double eeh2 = (ah*ah - bh*bh)/(bh*bh);
static const double latED730 =  (39.0 + 40.0/60.0)*(M_PI/180.0);     // radians
static const double lonED730 =  (-8.0 - 7.0/60.0 - 54.862/3600.0)*(M_PI/180.0); // radians
static const double feED73   =  1.0;
static const double dmED73   =  180.598;               // meters
static const double dpED73   = -86.990;                // meters

//
//  Performs datum conversion from WGS84 to ED73 datums:
//
static Point3D WGS84_to_ED73(const Point3D & WGS84_xyz)
{
  // Translation parameters:
  const double dX =  239.749;
  const double dY =  -88.181;
  const double dZ =  -30.488;

  // Rotation parameters:
  const double rx = (0.26/3600.0)*(M_PI/180.0);
  const double ry = (0.08/3600.0)*(M_PI/180.0);
  const double rz = (1.21/3600.0)*(M_PI/180.0);

  double rot[3][3];
  rot[0][0] =  cos(rz)*cos(ry);
  rot[0][1] =  cos(rz)*sin(ry)*sin(rx) + sin(rz)*cos(rx);
  rot[0][2] = -cos(rz)*sin(ry)*cos(rx) + sin(rz)*sin(rx);

  rot[1][0] = -sin(rz)*cos(ry);
  rot[1][1] = -sin(rz)*sin(ry)*sin(rx) + cos(rz)*cos(rx);
  rot[1][2] =  sin(rz)*sin(ry)*cos(rx) + cos(rz)*sin(rx);

  rot[2][0] =  sin(ry);
  rot[2][1] = -cos(ry)*sin(rx);
  rot[2][2] =  cos(ry)*cos(rx);

  // Scale factor:
  const double m = -2.23e-6;

  double xWGS = WGS84_xyz.X;
  double yWGS = WGS84_xyz.Y;
  double zWGS = WGS84_xyz.Z;
  // ED73 cartesian coordinates:
  Point3D ED73_xyz;
  ED73_xyz.X = dX + (1.0+m)*(rot[0][0]*xWGS+rot[0][1]*yWGS + rot[0][2]*zWGS);
  ED73_xyz.Y = dY + (1.0+m)*(rot[1][0]*xWGS+rot[1][1]*yWGS + rot[1][2]*zWGS);
  ED73_xyz.Z = dZ + (1.0+m)*(rot[2][0]*xWGS+rot[2][1]*yWGS + rot[2][2]*zWGS);

  return ED73_xyz;
}

//
// Rigorous convertion from XYZ to LLH (ED73 datum):
//
static Point3D ED73_XYZ_to_LLH(const Point3D & xyz)
{
  double x = xyz.X;
  double y = xyz.Y;
  double z = xyz.Z;

  double a1 = ah*eh2;
  double a2 = a1*a1;
  double a3 = a1*eh2/2.0;
  double a4 = 5.0*a2/2.0;
  double a5 = a1 + a3;
  double a6 = 1.0 - eh2;

  double zp = fabs(z);
  double w2 = x*x + y*y;
  double w  = sqrt(w2);
  double z2 = z*z;
  double r2 = w2 + z2;
  double r  = sqrt(r2);

  double lat, lon, h;

  lon = atan2(y, x);

  double s2 = z2/r2;
  double c2 = w2/r2;
  double u  = a2/r;
  double v  = a3 - a4/r;

  double s, ss, c;
  if (c2 > 0.3) {
    s    = (zp/r)*(1.0 + c2*(a1 + u + s2*v)/r);
    lat = asin(s);
    ss   = s*s;
    c    = sqrt(1.0 - ss);
  }
  else {
    c    = (w/r)*(1.0 - s2*(a5 - u - c2*v)/r);
    lat = acos(c);
    ss   = 1.0 - c*c;
    s    = sqrt(ss);
  }

  double g  = 1.0 - eh2*ss;
  double rg = ah/sqrt(g);
  double rf = a6*rg;

  u = w - rg*c;
  v = zp - rf*s;

  double f = c*u + s*v;
  double m = c*v - s*u;
  double p = m/(rf/g + f);

  lat += p;
  h = f + m*p/2.0;

  if (z < 0.0)
    lat = -lat;

  Point3D llh;
  llh.X = lat;
  llh.Y = lon;
  llh.Z = h;
  return llh;
}

//
//  Performs projection (ED73 datum).
//
static Point3D ProjED73(const Point3D & ED73_llh)
{
  double lat = ED73_llh.X;
  double lon = ED73_llh.Y;

  double clat = cos(lat);
  double slat = sin(lat);

  double dL = lon - lonED730;
  double W  = sqrt(1.0 - eh2*slat*slat);
  double N  = ah/W;
  double M  = ah*(1.0 - eh2)/(W*W*W);

  double Sc = Sigma(latED730, lat);

  double t  = slat/clat;
  double t2 = t*t;
  double t4 = t2*t2;

  double NM  = N/M;
  double NM2 = NM*NM;
  double NM4 = NM2*NM2;

  double c  = clat;
  double c2 = c*c;
  double c4 = c2*c2;

  double dL2 = dL*dL;
  double dL4 = dL2*dL2;

  double K1 = NM - t2;
  double K2 = K1 + 4.0*NM2;
  double K3 = NM*(14.0 - 58.0*t2) + 40.0*t2 + t4 - 9.0;
  double K4 = 8.0*NM4*(11.0 - 24.0*t2) - 28.0*NM*NM2*(1.0 - 6.0*t2) + NM2*(1.0 - 32.0*t2) - 2.0*NM*t2 + t4;
  double K5 = 61.0 - 479.0*t2 + 179.0*t4 - t2*t4;
  double K6 = 1385.0 - 3111.0*t2 + 543.0*t4 - t2*t4;

  double m = dL*N*c*(1.0 + dL2*c2*K1/6.0 + dL4*c4*K3/120.0 + dL2*dL4*c2*c4*K5/5040.0);
  double p = Sc + dL2*N*slat*c*(1.0 + dL2*c2*K2/12.0 + dL4*c4*K4/360.0 + dL2*dL4*c2*c4*K6/20160.0)/2.0;

  m = m*feED73 + dmED73;
  p = p*feED73 + dpED73;

  Point3D ED73_mph;
  ED73_mph.X = m;
  ED73_mph.Y = p;
  ED73_mph.Z = ED73_llh.Z;      // just because ...
  return ED73_mph;
}

//
// Sigma
//
static double Sigma(double lat1, double lat2)
{
  double B[11];

  B[0] = ah*(1 - eh2);

  for (int i=1; i<=5; i++)
    B[2*i] = ((2.0*i + 1.0)/(2.0*i))*eh2*B[2*(i-1)];

  double Beta0  = B[0]       + B[2]/2.0      + 3.0*B[4]/8.0    + 5.0*B[6]/16.0  + 35.0*B[8]/128.0 + 63.0*B[10]/256.0;
  double Beta2  = B[2]/2.0   + B[4]/2.0      + 15.0*B[6]/32.0  + 28.0*B[8]/64.0 + 105.0*B[10]/256.0;
  double Beta4  = B[4]/8.0   + 3.0*B[6]/16.0 + 28.0*B[8]/128.0 + 15.0*B[10]/64.0;
  double Beta6  = B[6]/32.0  + 4.0*B[8]/64.0 + 45.0*B[10]/512.0;
  double Beta8  = B[8]/128.0 + 5.0*B[10]/256.0;
  double Beta10 = B[10]/512.0;

  double Sc = Beta0*(lat2 - lat1)
            - Beta2*(sin(2.0*lat2)   - sin(2.0*lat1))/2.0
            + Beta4*(sin(4.0*lat2)   - sin(4.0*lat1))/4.0
            - Beta6*(sin(6.0*lat2)   - sin(6.0*lat1))/6.0
            + Beta8*(sin(8.0*lat2)   - sin(8.0*lat1))/8.0
            - Beta10*(sin(10.0*lat2) - sin(10.0*lat1))/10.0;

  return (Sc);
}


unsigned long Read_IP(char* str) {
  unsigned long IP = atol(str);
  while(*str && *str++ != '.');
    if(! *str) {
      cerr << "The IP address is not correct\n";
      exit(1);
    }
  IP *= 256UL;
  IP += atol(str);
  while(*str && *str++ != '.');
    if(! *str) {
      cerr << "The IP address is not correct\n";
      exit(1);
    }
  IP *= 256UL;
  IP += atol(str);
  while(*str && *str++ != '.');
    if(! *str) {
      cerr << "The IP address is not correct\n";
      exit(1);
    }
  IP *= 256UL;
  IP += atol(str);
  return IP;
}
