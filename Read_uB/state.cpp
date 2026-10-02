//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  state.cpp                                                           //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  This process maintains the system state by blending information     //
//  from navigation and sensor data acquiring routines. This data is    //
//  made available to clients through an UDP port. This process also    //
//  decides to execute the cut-down procedure if it detects that the    //
//  the payload is falling and this has not been done yet.              //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Sergio Cunha (Nov 2005)                                             //
//                                                                      //
//////////////////////////////////////////////////////////////////////////


//#include <fcntl.h>
//#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <math.h>
#include "udp_comm.h"
#include "nav_data.h"


// Type definitions that are valid locally:
enum param_tag {
  unknown         =  0,
  stat_port       =  1,          // UDP Port to be used to release results
  gps1_addr       =  2,          // Internet address of GPS1
  gps1_port       =  3,          // UDP Port of GPS1
  gps2_addr       =  4,          // Internet address of GPS2
  gps2_port       =  5,          // UDP Port of GPS2
  gps3_addr       =  6,          // Internet address of GPS3
  gps3_port       =  7,          // UDP Port of GPS3
  dgps_addr       =  8,          // Internet address of DGPS module
  dgps_port       =  9,          // UDP Port of DGPS module
  sens_file       = 10,          // File where sensor data is fetched
  coff_file       = 11,          // File created upon executing cut-off
  coff_cmd        = 12,          // Command that causes balloon cut-off
  pcof_file       = 13,          // File created upon parachute cut-off
  pcof_cmd        = 14,          // Command that causes parachute cut-off
  wind_L100       = 15,          // Wind (direction and value) at FL100
  wind_L200       = 16,          // Wind (direction and value) at FL200
  wind_L300       = 17,          // Wind (direction and value) at FL300
  wind_L400       = 18           // Wind (direction and value) at FL400
};


// Definitions:
const char       CFG_FILE[]     = "state.ini";
const char       STATPORT_STR[] = "StatePort";
const char       GPS1ADDR_STR[] = "GPS1Addr";
const char       GPS1PORT_STR[] = "GPS1Port";
const char       GPS2ADDR_STR[] = "GPS2Addr";
const char       GPS2PORT_STR[] = "GPS2Port";
const char       GPS3ADDR_STR[] = "GPS3Addr";
const char       GPS3PORT_STR[] = "GPS3Port";
const char       DGPSADDR_STR[] = "DGPSAddr";
const char       DGPSPORT_STR[] = "DGPSPort";
const char       SENSFILE_STR[] = "SensorFile";
const char       COFFFILE_STR[] = "CutoffFile";
const char       COFF_CMD_STR[] = "CutoffCMD";
const char       PCOFFILE_STR[] = "ParaoffFile";
const char       PCOF_CMD_STR[] = "ParaoffCMD";
const char       WIND_100_STR[] = "Wind100";
const char       WIND_200_STR[] = "Wind200";
const char       WIND_300_STR[] = "Wind300";
const char       WIND_400_STR[] = "Wind400";
static const int LINELEN = 1024; // Maximum number of chars in a cfg line
const int        DT_STALE_GPS   = 3;
const int        DT_STALE_DGPS  = 10;

const double     DEG2RAD = M_PI / 180.0;
const double     RAD2DEG = 180.0 / M_PI;
const double     KTS2SI  = 1.856/3.6;


// Global variables:
NavData         State;
unsigned short  STAT_port;
char            GPS1_addr[128];
unsigned short  GPS1_port;
char            GPS2_addr[128];
unsigned short  GPS2_port;
char            GPS3_addr[128];
unsigned short  GPS3_port;
char            DGPS_addr[128];
unsigned short  DGPS_port;
char            SENS_file[128];
char            COFF_file[128];
char            COFF_cmd[128];
char            PCOF_file[128];
char            PCOF_cmd[128];
char            WD100_str[128];
char            WD200_str[128];
char            WD300_str[128];
char            WD400_str[128];
double          Wind100_N;
double          Wind100_E;
double          Wind200_N;
double          Wind200_E;
double          Wind300_N;
double          Wind300_E;
double          Wind400_N;
double          Wind400_E;
static bool     hang_up = false;


// Local function prototypes:
static void InitState(void);
static void DefaultParams(void);
static int ReadParams(const char* file_name);
static param_tag chk_name(char* name);
static void DecompWind(void);
static bool is_cutoff(void);
static bool is_paraoff(void);
static void State_SetNav(const NavData & data);
static char* my_fgets(char* line, int max_len, int fd);
static void signal_handler(int sig_no);

// External function prototypes:
void landing(void);


main(int argc, char** argv)
{
  // Signal handlers for proper program termination:
  signal(SIGHUP,  signal_handler);
  signal(SIGINT,  signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGPIPE, signal_handler);

  // Initialize state and read parameters:
  InitState();
  DefaultParams();
  ReadParams(argc > 1 ? argv[1] : CFG_FILE);
  
  // Opering port to serve data:
  UDP_Comm_Server server(STAT_port);
  
  // Opening UDP connections to obtain navigation data:
  UDP_Comm_Client gps1(GPS1_addr, GPS1_port);
  UDP_Comm_Client gps2(GPS2_addr, GPS2_port);
  UDP_Comm_Client gps3(GPS3_addr, GPS3_port);
  UDP_Comm_Client dgps(DGPS_addr, DGPS_port);
  int fd_gps1 = gps1.desc();
  int fd_gps2 = gps2.desc();
  int fd_gps3 = gps3.desc();
  int fd_dgps = dgps.desc();
  
  // Opening the SENSOR file to be read as is filled in:
  FILE* fp_sens = fopen(SENS_file, "r");
  int fd_sens = -1;
  if(fp_sens) {
    fd_sens = fileno(fp_sens);
    fseek(fp_sens, 0, SEEK_END);
  } else
    fprintf(stderr, "Failed to open sensor file: %s\n", SENS_file);
  /*
  int fd_sens = open(SENS_file, O_RDONLY);
  if(fd_sens >= 0) {
    fprintf(stderr, "Suceeded to open sensor file: %s\n", SENS_file);
    // lseek(fd_sens, 0, SEEK_END);
  } else
    fprintf(stderr, "Failed to open sensor file: %s\n", SENS_file);
  */

  int max_fd = fd_gps1;
  if(fd_gps2 > max_fd)
    max_fd = fd_gps2;
  if(fd_gps3 > max_fd)
    max_fd = fd_gps3;
  if(fd_dgps > max_fd)
    max_fd = fd_dgps;
  // if(fd_sens > max_fd)
  //   max_fd = fd_sens;
  max_fd++;

  fd_set rfds0;
  FD_ZERO(& rfds0);
  if(fd_gps1 >= 0)
    FD_SET(fd_gps1, & rfds0);
  if(fd_gps2 >= 0)
    FD_SET(fd_gps2, & rfds0);
  if(fd_gps3 >= 0)
    FD_SET(fd_gps3, & rfds0);
  if(fd_dgps >= 0)
    FD_SET(fd_dgps, & rfds0);
  // if(fd_sens >= 0)
  //   FD_SET(fd_sens, & rfds0);

  while(! hang_up) {
    // Adjusting flight stage:
    switch(State.stage) {
      case STG_INIT : if((State.flags & TRUST_FLD) == TRUST_SUR)
                        if((State.flags & TREND_FLD) == TREND_CLB)
                          State.stage = STG_CLIMB;
                        else if((State.flags & TREND_FLD) == TREND_DSC)
                          State.stage = STG_DESCD;
                      break;
      case STG_CLIMB: if((State.flags & TRUST_FLD) == TRUST_SUR &&
                         (State.flags & TREND_FLD) == TREND_DSC) {
                        State.stage = STG_DESCD;
                        system(COFF_cmd);   // CUT-OFF EXECUTED HERE !
                      }
                      break;
      case STG_DESCD: {
                        static time_t last_check = 0;
                        time_t now = time(0);
                        if(now > last_check) {
                          if(is_cutoff())
                            State.stage = STG_CTOFF;
                          last_check = now;
                        }
                      }
      case STG_CTOFF: if((State.flags & TRUST_FLD) == TRUST_SUR &&
                         (State.flags & TREND_FLD) == TREND_STP) {
                        State.stage = STG_LAND;
                        system(PCOF_cmd);   // PARACHUTE CUT-OFF EXEC. HERE !
                      }
                      break;
      case STG_LAND : {
                        static time_t last_check = 0;
                        time_t now = time(0);
                        if(now > last_check) {
                          if(is_paraoff())
                            State.stage = STG_PCOFF;
                          last_check = now;
                        }
                      }
      case STG_PCOFF: break;
    }

    struct timeval timeout;
    timeout.tv_sec  = 1;
    timeout.tv_usec = 0;
    fd_set rfds = rfds0;
    int rs = select(max_fd, &rfds,0,0, &timeout);

    static NavData State_GPS1, State_GPS2, State_GPS3, State_DGPS;
    static int last_gps = 0;
    static time_t last_gps1_data = 0, last_gps1_nav_valid = 0;
    static time_t last_gps2_data = 0, last_gps2_nav_valid = 0;
    static time_t last_gps3_data = 0, last_gps3_nav_valid = 0;
    static time_t last_dgps_data = 0, last_dgps_nav_valid = 0;
    static time_t started = 0;
    time_t now = time(0);
    if(! started)
      started = now;

    // Getting data from GPS1:
    if(rs > 0 && fd_gps1 >= 0 && FD_ISSET(fd_gps1, &rfds)) {
      NavData dummy;
      int len = gps1.recv((unsigned char*) & dummy, sizeof(dummy));
      if(len > 0)
        last_gps1_data = now;
      if(len >= sizeof(dummy) && (dummy.flags & LLH_VALID) &&
                                 (dummy.flags & VEL_VALID)) {
        State_GPS1 = dummy;
        last_gps1_nav_valid = now;
        if(last_gps == 0 || last_gps == 1 ||
           (last_gps == 2 && now-last_gps2_nav_valid > DT_STALE_GPS) ||
           (last_gps == 3 && now-last_gps3_nav_valid > DT_STALE_GPS) ||
           (last_gps == 4 && now-last_dgps_nav_valid > DT_STALE_DGPS)) {
          last_gps = 1;
          State_SetNav(State_GPS1);
        }
      }
    } else if(now-started > DT_RENEW && now-last_gps1_data > DT_RENEW)
      gps1.send_renew();

    // Getting data from GPS2:
    if(rs > 0 && fd_gps2 >= 0 && FD_ISSET(fd_gps2, &rfds)) {
      NavData dummy;
      int len = gps2.recv((unsigned char*) & dummy, sizeof(dummy));
      if(len > 0)
        last_gps2_data = now;
      if(len >= sizeof(dummy) && (dummy.flags & LLH_VALID) &&
                                 (dummy.flags & VEL_VALID)) {
        State_GPS2 = dummy;
        last_gps2_nav_valid = now;
        if(last_gps == 0 || last_gps == 2 ||
           (last_gps == 1 && now-last_gps1_nav_valid > DT_STALE_GPS) ||
           (last_gps == 3 && now-last_gps3_nav_valid > DT_STALE_GPS) ||
           (last_gps == 4 && now-last_dgps_nav_valid > DT_STALE_DGPS)) {
          last_gps = 2;
          State_SetNav(State_GPS2);
        }
      }
    } else if(now-started > DT_RENEW && now-last_gps2_data > DT_RENEW)
      gps2.send_renew();

    // Getting data from GPS3:
    if(rs > 0 && fd_gps3 >= 0 && FD_ISSET(fd_gps3, &rfds)) {
      NavData dummy;
      int len = gps3.recv((unsigned char*) & dummy, sizeof(dummy));
      if(len > 0)
        last_gps3_data = now;
      if(len >= sizeof(dummy) && (dummy.flags & LLH_VALID) &&
                                 (dummy.flags & VEL_VALID)) {
        State_GPS3 = dummy;
        last_gps3_nav_valid = now;
        if(last_gps == 0 || last_gps == 3 ||
           (last_gps == 1 && now-last_gps1_nav_valid > DT_STALE_GPS) ||
           (last_gps == 2 && now-last_gps2_nav_valid > DT_STALE_GPS) ||
           (last_gps == 4 && now-last_dgps_nav_valid > DT_STALE_DGPS)) {
          last_gps = 3;
          State_SetNav(State_GPS3);
        }
      }
    } else if(now-started > DT_RENEW && now-last_gps3_data > DT_RENEW)
      gps3.send_renew();

    // Getting data from DGPS:
    if(rs > 0 && fd_dgps >= 0 && FD_ISSET(fd_dgps, &rfds)) {
      NavData dummy;
      int len = dgps.recv((unsigned char*) & dummy, sizeof(dummy));
      if(len > 0)
        last_dgps_data = now;
      if(len >= sizeof(dummy) && (dummy.flags & LLH_VALID) &&
                                 (dummy.flags & VEL_VALID)) {
        State_DGPS = dummy;
        last_dgps_nav_valid = now;
        last_gps = 4;
        State_SetNav(State_DGPS);
      }
    } else if(now-started > DT_RENEW && now-last_dgps_data > DT_RENEW)
      dgps.send_renew();

    // Updating the landing forecast:
    landing();

    // Getting data from SENSORS:
    // if(rs > 0 && fd_sens >= 0 && FD_ISSET(fd_sens, &rfds)) {
    if(true) {
      char line[1024];
      if(fgets(line, 1023, fp_sens)) {
        line[1023] = '\0';
        double ttag, val[20];
        int n;
        if((n = sscanf(line, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
                       &ttag, val+0, val+1, val+2, val+3, val+4,
                       val+5, val+6, val+7, val+8, val+9,
                       val+10, val+11, val+12, val+13, val+14,
                       val+15, val+16, val+17, val+18, val+19)) >= 2) {
          State.SENS_time.tv_sec  = (time_t) ttag;
          State.SENS_time.tv_nsec = (int) ((ttag - State.SENS_time.tv_sec)*1e9+0.5);
          int k;
          for(k=0; k<n-1; k++)
            State.sensor[k] = val[k];
          for( ; k<20; k++)
            State.sensor[k] = 0.0;
          State.flags |= SNS_VALID;
        }
      }
    }

    // Output state:
    server.send((unsigned char*) & State, sizeof(State));
  }

  // Clean-up:
  if(fp_sens)
    fclose(fp_sens);
  return 0;
}


static void InitState(void)
{
  strncpy(State.label, "Straplex", 8);
  State.flags = STG_VALID;
  State.stage = STG_INIT;
}

static void DefaultParams(void)
{
  STAT_port = 2000;
  strcpy(GPS1_addr, "127.0.0.1");
  GPS1_port = 2001;
  strcpy(GPS2_addr, "127.0.0.1");
  GPS2_port = 2002;
  strcpy(GPS3_addr, "127.0.0.1");
  GPS3_port = 2003;
  strcpy(DGPS_addr, "127.0.0.1");
  DGPS_port = 2004;
  strcpy(SENS_file, "/home/straplex/sensors.txt");
  strcpy(COFF_file, "/home/straplex/cutoff.txt");
  strcpy(COFF_cmd,  "/home/straplex/cutoff");
  strcpy(PCOF_file, "/home/straplex/paraoff.txt");
  strcpy(PCOF_cmd,  "/home/straplex/paraoff");
  strcpy(WD100_str, "360/00");
  strcpy(WD200_str, "360/00");
  strcpy(WD300_str, "360/00");
  strcpy(WD400_str, "360/00");
  DecompWind();
}

static int ReadParams(const char* file_name)
{
  // Checking file existance:
  struct stat buf;
  if(stat(file_name, & buf))            // get file attributes ...
    return -1;
  if(! S_ISREG(buf.st_mode))            // check if this is a regular file ...
    return -1;
  
  // Reading the file ...
  FILE* fp = fopen(file_name, "r");
  if(! fp)
    return -1;             // we don't create files if they don't exist ...

  char line[LINELEN];
  int n = 0;
  while(fgets(line, LINELEN, fp)) {
    bool copy_whole_line = true;
    char* comment = line;
    // Look for name:
    char* ptr = line;
    while(*ptr == ' ' || *ptr == '\t')
      ptr++;
    if(isalpha(*ptr)) {
      char* name = ptr++;
      while(isalnum(*ptr))
        ptr++;
      if(*ptr == ' ' || *ptr == '\t' || *ptr == '=' || *ptr == ':') {
        char ch = *ptr;
        *ptr = '\0';
        param_tag par_tag = chk_name(name);
        *(ptr++) = ch;
        if(par_tag != unknown) {
          // Name is OK. Look for argument after separators:
          while(*ptr == ' ' || *ptr == '\t' || *ptr == '=' || *ptr == ':')
            ptr++;
          char* arg = ptr++;
          while(isalnum(*ptr) || *ptr == '.' || *ptr == '+' ||
                  *ptr == '-' || *ptr == '_' || *ptr == '/')
            ptr++;
          ch = *ptr;
          *ptr = '\0';
          switch(par_tag) {
            case stat_port: sscanf(arg, "%hd", & STAT_port); break;
            case gps1_addr: strncpy(GPS1_addr, arg, 128);    break;
            case gps1_port: sscanf(arg, "%hd", & GPS1_port); break;
            case gps2_addr: strncpy(GPS2_addr, arg, 128);    break;
            case gps2_port: sscanf(arg, "%hd", & GPS2_port); break;
            case gps3_addr: strncpy(GPS3_addr, arg, 128);    break;
            case gps3_port: sscanf(arg, "%hd", & GPS3_port); break;
            case dgps_addr: strncpy(DGPS_addr, arg, 128);    break;
            case dgps_port: sscanf(arg, "%hd", & DGPS_port); break;
            case sens_file: strncpy(SENS_file, arg, 128);    break;
            case coff_file: strncpy(COFF_file, arg, 128);    break;
            case coff_cmd : strncpy(COFF_cmd, arg, 128);     break;
            case pcof_file: strncpy(PCOF_file, arg, 128);    break;
            case pcof_cmd : strncpy(PCOF_cmd, arg, 128);     break;
            case wind_L100: strncpy(WD100_str, arg, 128);
                            DecompWind();                    break;
            case wind_L200: strncpy(WD200_str, arg, 128);
                            DecompWind();                    break;
            case wind_L300: strncpy(WD300_str, arg, 128);
                            DecompWind();                    break;
            case wind_L400: strncpy(WD400_str, arg, 128);
                            DecompWind();                    break;
          }
          *ptr = ch;
          n++;
        }
      }
    }
  }
  fclose(fp);
  DecompWind();
  return n;
}

static param_tag chk_name(char* name)
{
  if(! strncasecmp(name, STATPORT_STR, strlen(STATPORT_STR)))
    return stat_port;
  if(! strncasecmp(name, GPS1ADDR_STR, strlen(GPS1ADDR_STR)))
    return gps1_addr;
  if(! strncasecmp(name, GPS1PORT_STR, strlen(GPS1PORT_STR)))
    return gps1_port;
  if(! strncasecmp(name, GPS2ADDR_STR, strlen(GPS2ADDR_STR)))
    return gps2_addr;
  if(! strncasecmp(name, GPS2PORT_STR, strlen(GPS2PORT_STR)))
    return gps2_port;
  if(! strncasecmp(name, GPS3ADDR_STR, strlen(GPS3ADDR_STR)))
    return gps3_addr;
  if(! strncasecmp(name, GPS3PORT_STR, strlen(GPS3PORT_STR)))
    return gps3_port;
  if(! strncasecmp(name, DGPSADDR_STR, strlen(DGPSADDR_STR)))
    return dgps_addr;
  if(! strncasecmp(name, DGPSPORT_STR, strlen(DGPSPORT_STR)))
    return dgps_port;
  if(! strncasecmp(name, SENSFILE_STR, strlen(SENSFILE_STR)))
    return sens_file;
  if(! strncasecmp(name, COFFFILE_STR, strlen(COFFFILE_STR)))
    return coff_file;
  if(! strncasecmp(name, COFF_CMD_STR, strlen(COFF_CMD_STR)))
    return coff_cmd;
  if(! strncasecmp(name, PCOFFILE_STR, strlen(PCOFFILE_STR)))
    return pcof_file;
  if(! strncasecmp(name, PCOF_CMD_STR, strlen(PCOF_CMD_STR)))
    return pcof_cmd;
  if(! strncasecmp(name, WIND_100_STR, strlen(WIND_100_STR)))
    return wind_L100;
  if(! strncasecmp(name, WIND_200_STR, strlen(WIND_200_STR)))
    return wind_L200;
  if(! strncasecmp(name, WIND_300_STR, strlen(WIND_300_STR)))
    return wind_L300;
  if(! strncasecmp(name, WIND_400_STR, strlen(WIND_400_STR)))
    return wind_L400;
  return unknown;
}

static void DecompWind(void)
{
  double direction, speed;
  if(sscanf(WD100_str, "%lf/%lf", & direction, & speed) < 2)
    direction = speed = 0;
  Wind100_N = -cos(direction*DEG2RAD) * speed*KTS2SI;
  Wind100_E = -sin(direction*DEG2RAD) * speed*KTS2SI;
  if(sscanf(WD200_str, "%lf/%lf", & direction, & speed) < 2)
    direction = speed = 0;
  Wind200_N = -cos(direction*DEG2RAD) * speed*KTS2SI;
  Wind200_E = -sin(direction*DEG2RAD) * speed*KTS2SI;
  if(sscanf(WD300_str, "%lf/%lf", & direction, & speed) < 2)
    direction = speed = 0;
  Wind300_N = -cos(direction*DEG2RAD) * speed*KTS2SI;
  Wind300_E = -sin(direction*DEG2RAD) * speed*KTS2SI;
  if(sscanf(WD400_str, "%lf/%lf", & direction, & speed) < 2)
    direction = speed = 0;
  Wind400_N = -cos(direction*DEG2RAD) * speed*KTS2SI;
  Wind400_E = -sin(direction*DEG2RAD) * speed*KTS2SI;
}


static bool is_cutoff(void)
{
  struct stat buf;
  if(stat(COFF_file, & buf))            // get file attributes ...
    return false;
  true;
}

static bool is_paraoff(void)
{
  struct stat buf;
  if(stat(PCOF_file, & buf))            // get file attributes ...
    return false;
  true;
}


static void State_SetNav(const NavData & data)
{
  State.NAV_time = data.NAV_time;
  State.Latitude = data.Latitude;
  State.Longitude = data.Longitude;
  State.Altitude = data.Altitude;
  State.Vn = data.Vn;
  State.Ve = data.Ve;
  State.Vup = data.Vup;
  State.Vel = data.Vel;
  State.Vh = data.Vh;
  State.Course = data.Course;
  State.Roll = data.Roll;
  State.Pitch = data.Pitch;
  State.Heading = data.Heading;
  State.flags &= ~(LLH_VALID | VEL_VALID | ATT_VALID | TREND_FLD | TRUST_FLD);
  State.flags |= data.flags & (LLH_VALID | VEL_VALID | ATT_VALID | TREND_FLD | TRUST_FLD);
}


static char* my_fgets(char* line, int max_len, int fd)
{
  static char buf[1024];
  static int len = 0;

  int fl = 0;
  while(fl < len && !buf[fl])
    fl++;
  if(fl > 0) {
    len -= fl;
    if(len)
      memmove(buf, buf+fl, len);
  }

  int rd = read(fd, buf+len, 1024-len);

  if(rd <= 0)
    return 0;

  len += rd;
  int has_eol = -1;
  for(int i=len-rd; i<len; i++)
    if(buf[i] == '\r' || buf[i] == '\n') {
      buf[i] == '\0';
      if(has_eol < 0)
        has_eol = i;
    }

  if(has_eol > 0) {
    strncpy(line, buf, (max_len > 1024 ? 1024 : max_len));
    len -= has_eol+1;
    if(len > 0)
      memmove(buf, buf+has_eol+1, len);
    return line;
  }
  return 0;
}


static void signal_handler(int sig_no)
{
  switch(sig_no) {
    case SIGHUP  :
    case SIGINT  :
    case SIGTERM : hang_up = true;
                   fprintf(stderr, "Caught signal: bailing out ...\n");
                   break;
    case SIGPIPE : fprintf(stderr, "Broken connection: ignoring ...\n");
                   break;
    default:       break;
  }
}
