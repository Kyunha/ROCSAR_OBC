//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  read_state.cpp                                                      //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  This program connects to the "state" or "Read_uB" processes and,    //
//  upon reading the state in STRAPLEX format, converts it to UTM.      //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Sergio Cunha (Mai 2008)                                             //
//                                                                      //
//////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <sys/select.h>
// #include <sys/time.h>
// #include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include "udp_comm.h"
#include "nav_data.h"
#include "read_state_veleiro.h"


// Type definitions that are valid locally:
enum data_sw {
  sw_all       =  0,  // Send all available data in ASCII.
  sw_nav       =  1,  // Send navigation data for APRS.
  sw_lnd       =  2,  // Send estimate of landing spot (APRS).
  sw_cut       =  3,  // Similar estimate or immediate cut-off.
  sw_sns       =  4,  // Send sensor data.
  sw_cam       =  5   // Send advice on recording with webcams.
};


// Local constants:
static time_t SOCK_EXPIRE = 20;
static time_t SOCK_RENEW  = 10;
static time_t SOCK_TOUT   = 3;


// Local function prototypes:
void Usage(int exit_code);
static void convert_state(NavData& State, utm_coord* utm_state);
static void ProduceOutput(NavData & State, data_sw mode);


// Local global variables:
static UDP_Comm_Client* client = 0;
static int delta_t = 0;


void Usage(int exit_code)
{
  fprintf(stderr, "usage: read_state [-1anlcsw] [-t secs] [server] [port]\n");
  fprintf(stderr, "  switches: -1: read once and quit (default: continuous).\n");
  fprintf(stderr, "            -a: supply all data (this is the default).\n");
  fprintf(stderr, "            -n: navigation data for APRS.\n");
  fprintf(stderr, "            -l: landing estimate for APRS.\n");
  fprintf(stderr, "            -c: landing estimate if cut-down is now.\n");
  fprintf(stderr, "            -s: sensor data.\n");
  fprintf(stderr, "            -w: advise on turning webcams on or off.\n");
  fprintf(stderr, "            -h: this help message.\n");
  fprintf(stderr, "            -t secs: send data only at the given rate.\n");
  fprintf(stderr, "  server: hostname of IP address (defaults to localhost.\n");
  fprintf(stderr, "  port: UDP port of the service (defaults to 2000).\n");
  
  _exit(exit_code);
}


int init_connection(int argc, char** argv)
{
  data_sw mode = sw_all;
  char* server = "127.0.0.1";
  unsigned short port = 2000;
  char *server_arg = 0, *port_arg = 0;
  
  for(int i=1; i<argc; i++) {
    if(*argv[i] == '-') {
      int k = i;
      for(int j=1; argv[k][j]; j++)
        switch(argv[k][j]) {
          case '1': delta_t = -1;
                    break;
          case 't':
          case 'T': if(k+1 >= argc)
                      Usage(-1);
                    delta_t = atoi(argv[i = k+1]);
                    break;
          case 'a':
          case 'A': mode = sw_all;
                    break;
          case 'n':
          case 'N': mode = sw_nav;
                    break;
          case 'l':
          case 'L': mode = sw_lnd;
                    break;
          case 'c':
          case 'C': mode = sw_cut;
                    break;
          case 's':
          case 'S': mode = sw_sns;
                    break;
          case 'w':
          case 'W': mode = sw_cam;
                    break;
          case '?':
          case 'h':
          case 'H': Usage(0);
        }
    } else if(server_arg == 0)
      server_arg = argv[i];
    else if(port_arg == 0)
      port_arg = argv[i];
  }
  if(server_arg)
    server = server_arg;
  if(port_arg)
    port = (unsigned short) atoi(port_arg);

  client = new UDP_Comm_Client(server, port, (delta_t<0 ? 0 : SOCK_EXPIRE));
  if(client == 0 || client->desc() < 0) {
    fprintf(stderr, "Error opening UDP socket to server ...\n");
    _exit(-1);
  }

  return 0;
}


int read_state_veleiro(utm_coord* utm_state)
{
  NavData State;
  int len;

  if(! client)
    return -1;

  for(int k=0; k<1; k++) {
    static time_t last_data = 0;
    if(! last_data)
      last_data = time(0);

    struct timeval timeout;
    timeout.tv_sec = SOCK_TOUT;
    timeout.tv_usec = 0;

    fd_set rfds;
    FD_ZERO(& rfds);
    FD_SET(client->desc(), & rfds);

    int rs = select(client->desc()+1, &rfds,0,0, &timeout);

    if(! rs && delta_t >= 0) {
      if(time(0) - last_data >= SOCK_RENEW)
        client->send_renew();
      continue;
    }

    if(rs > 0)
      if(FD_ISSET(client->desc(), &rfds)) {
        len = client->recv((unsigned char*) &State, sizeof(State));
        time_t now = time(0);
        if(len > 0)
          last_data = now;
        if(len >= sizeof(State)) {
          static time_t last_out = 0;
          if(now - last_out >= delta_t) {
            // ProduceOutput(State, mode);
            convert_state(State, utm_state);
            last_out = now;
          }
        }
      }

    if(delta_t < 0)
      break;
  }

  return 0;
}


static void convert_state(NavData& State, utm_coord* utm_state)
{
  if(State.LLH_valid() && State.Latitude < 89.9 && State.Altitude > -9999.9) {
    LatLonToUtmWGS84((utm_state->utmXZone), (utm_state->utmYZone), 
                     (utm_state->Easting), (utm_state->Northing), 
		     State.Latitude, State.Longitude);
    utm_state->pos_valid = true;
  } else {
    utm_state->Easting = 0.0;
    utm_state->Northing = 0.0;
    utm_state->utmXZone = 0;
    utm_state->utmYZone = '-';
    utm_state->pos_valid = false;
  }

  if(State.VEL_valid() && utm_state->pos_valid) {
    utm_state->Vel = State.Vh;
    utm_state->Course = State.Course;
    utm_state->vel_valid = true;
    utm_state->course_valid = true;
  } else {
    utm_state->Vel = 0.0;
    utm_state->Course = 0.0;
    utm_state->vel_valid = false;
    utm_state->course_valid = false;
  }
}


static void ProduceOutput(NavData & State, data_sw mode)
{
  switch(mode) {
    case sw_all:
      printf("STRAPLEX: ");
      if(State.STG_valid())
        switch(State.stage) {
          case STG_UNKNW: printf("unknown stage"); break;
          case STG_INIT : printf("before launch"); break;
          case STG_CLIMB: printf("ascencion stage"); break;
          case STG_DESCD: printf("after burst"); break;
          case STG_CTOFF: printf("balloon cut-off"); break;
          case STG_LAND : printf("landed"); break;
          case STG_PCOFF: printf("parachute cut-off"); break;
          default:        printf("undefined stage");
        }
      else
        printf("invalid stage");
      printf(" and ");
      if(State.FT_Known()) {
        if(State.IsStopped())
          printf("stopped");
        else if(State.IsClimbing())
          printf("climbing");
        else if(State.IsDescending())
          printf("descending");
        else
          printf("undefined trend");
        if(State.FT_Sure())
          printf(" (OK)");
      } else
        printf("unknown trend");
      printf("\n");
      if(State.LLH_valid()) {
        time_t nav_time = State.NAV_time.tv_sec;
        struct tm* tm_time = gmtime(& nav_time);
        if(tm_time)
          printf("POS: %02d:%02d:%02d.%02d", tm_time->tm_hour, tm_time->tm_min,
                 tm_time->tm_sec, State.NAV_time.tv_nsec/10000000);
        else
          printf("POS: --:--:--.--");
        bool neg = State.Latitude < 0.0;
        int deg = (int) floor(neg ? -State.Latitude : State.Latitude);
        double min = 60.0 * ((neg ? -State.Latitude : State.Latitude) - deg);
        printf(" %c%02d.%06.3f", (neg ? 'S' : 'N'), deg, min);
        neg = State.Longitude < 0.0;
        deg = (int) floor(neg ? -State.Longitude : State.Longitude);
        min = 60.0 * ((neg ? -State.Longitude : State.Longitude) - deg);
        printf(" %c%03d.%06.3f", (neg ? 'W' : 'E'), deg, min);
        printf(" %+05d\n", (int) floor(State.Altitude + 0.5));
      } else
        printf("POS: invalid\n");
      if(State.VEL_valid()) {
        int deg = (int) floor(State.Course-360.0*floor(State.Course/360.0)+0.5);
        if(deg == 0)
          deg = 360;
        printf("VEL: %05.1f %03d %+05.1f\n", State.Vh, deg, State.Vup);
      } else
        printf("VEL: invalid\n");
      if(State.ATT_valid()) {
        double hdg = State.Heading;
        hdg = hdg - 360.0*floor(hdg/360.0);
        if(hdg < 0.05)
          hdg = 360.0;
        printf("ATT: %+04.1f %+04.1f %05.1f\n", State.Roll, State.Pitch, hdg);
      } else
        printf("ATT: invalid\n");
      if(State.PRV_valid()) {
        time_t prv_time = State.PREV_time;
        struct tm* tm_time = gmtime(& prv_time);
        if(tm_time)
          printf("LND: (%02d:%02d:%02d) ",
                 tm_time->tm_hour, tm_time->tm_min, tm_time->tm_sec);
        else
          printf("LND: (--:--:--) ");
        prv_time = State.LD35_time;
        tm_time = gmtime(& prv_time);
        if(tm_time)
          printf("burst %02d:%02d:%02d",
                 tm_time->tm_hour, tm_time->tm_min, tm_time->tm_sec);
        else
          printf("burst --:--:--");
        bool neg = State.LD35_lat < 0.0;
        int deg = (int) floor(neg ? -State.LD35_lat : State.LD35_lat);
        double min = 60.0 * ((neg ? -State.LD35_lat : State.LD35_lat) - deg);
        printf(" %c%02d.%06.3f", (neg ? 'S' : 'N'), deg, min);
        neg = State.LD35_lon < 0.0;
        deg = (int) floor(neg ? -State.LD35_lon : State.LD35_lon);
        min = 60.0 * ((neg ? -State.LD35_lon : State.LD35_lon) - deg);
        printf(" %c%03d.%06.3f", (neg ? 'W' : 'E'), deg, min);
        printf(" %06.2f ", State.LD35_sdMX);
        prv_time = State.LDNW_time;
        tm_time = gmtime(& prv_time);
        if(tm_time)
          printf("cut-off %02d:%02d:%02d",
                 tm_time->tm_hour, tm_time->tm_min, tm_time->tm_sec);
        else
          printf("cut-off --:--:--");
        neg = State.LDNW_lat < 0.0;
        deg = (int) floor(neg ? -State.LDNW_lat : State.LDNW_lat);
        min = 60.0 * ((neg ? -State.LDNW_lat : State.LDNW_lat) - deg);
        printf(" %c%02d.%06.3f", (neg ? 'S' : 'N'), deg, min);
        neg = State.LDNW_lon < 0.0;
        deg = (int) floor(neg ? -State.LDNW_lon : State.LDNW_lon);
        min = 60.0 * ((neg ? -State.LDNW_lon : State.LDNW_lon) - deg);
        printf(" %c%03d.%06.3f", (neg ? 'W' : 'E'), deg, min);
        printf(" %06.2f\n", State.LDNW_sdMX);
      } else
        printf("LND: landing estimate invalid\n");
      if(State.SNS_valid()) {
        time_t sns_time = State.SENS_time.tv_sec;
        struct tm* tm_time = gmtime(& sns_time);
        if(tm_time)
          printf("SNS: %02d:%02d:%02d",
                 tm_time->tm_hour, tm_time->tm_min, tm_time->tm_sec);
        else
          printf("SNS: --:--:--");
        int i;
        for(i=0; i<10; i++) {
          int ival = (int) floor(State.sensor[i]+0.5);
          if(ival > 9999)
            ival = 9999;
          else if(ival < -999)
            ival = -999;
          if(ival > 999)
            printf(" %d", ival);
          else
            printf(" %+03d", ival);
        }
        printf("\n             ");
        for( ; i<20; i++) {
          int ival = (int) floor(State.sensor[i]+0.5);
          if(ival > 9999)
            ival = 9999;
          else if(ival < -999)
            ival = -999;
          if(ival > 999)
            printf(" %d", ival);
          else
            printf(" %+03d", ival);
        }
        printf("\n");
      } else
        printf("SNS: invalid\n\n");
      break;
    case sw_nav:
      if(true || State.LLH_valid()) {
        time_t nav_time = State.NAV_time.tv_sec;
        struct tm* tm_time = gmtime(& nav_time);
        if(tm_time)
          printf("/%02d%02d%02dh",
                 tm_time->tm_hour, tm_time->tm_min, tm_time->tm_sec);
        else
          printf("/000000h");
        double latitude = State.Latitude;
        if(latitude > 90.0)
          latitude = 90.0;
        else if(latitude < -90.0)
          latitude = -90.0;
        bool neg = latitude < 0.0;
        int deg = (int) floor(neg ? -latitude : latitude);
        double min = 60.0 * ((neg ? -latitude : latitude) - deg);
        printf("%02d%05.2f%c", deg, min, (neg ? 'S' : 'N'));
        double longitude = State.Longitude;
        if(longitude > 180.0)
          longitude = 180.0;
        else if(longitude < -180.0)
          longitude = -180.0;
        neg = longitude < 0.0;
        deg = (int) floor(neg ? -longitude : longitude);
        min = 60.0 * ((neg ? -longitude : longitude) - deg);
        printf("/%03d%05.2f%c", deg, min, (neg ? 'W' : 'E'));
        deg = (int) floor(State.Course-360.0*floor(State.Course/360.0)+0.5);
        int vel = (int) floor(State.Vh*(3.6/1.856)+0.5);
        if(vel < 0)
          vel = 0;
        else if(vel > 999)
          vel = 999;
        printf("O%03d/%03d", deg, vel);
        int alt = (int) floor((State.Altitude-53.6)/0.3048+0.5);
        if(alt < 0)
          alt = 0;
        else if(alt > 999999)
          alt = 999999;
        int vup = (int) floor(State.Vup*(60/0.3048)+0.5);
        if(vup > 9999)
          vup = 9999;
        else if(vup < -9999)
          vup = -9999;
        printf("/A=%06d/Vup=%+04d", alt, vup);
        printf(" STRAPLEX\n");
      } else
        printf("/000000h4000.00N/00800.00WO000/000/A=000000/Vup=0000 ST_invalid\n");
      break;
    case sw_lnd:
      if(State.PRV_valid()) {
        time_t prv_time = State.LD35_time;
        struct tm* tm_time = gmtime(& prv_time);
        if(tm_time)
          printf("/%02d%02d%02dh",
                 tm_time->tm_hour, tm_time->tm_min, tm_time->tm_sec);
        else
          printf("/000000h");
        double latitude = State.LD35_lat;
        if(latitude > 90.0)
          latitude = 90.0;
        else if(latitude < -90.0)
          latitude = -90.0;
        bool neg = latitude < 0.0;
        int deg = (int) floor(neg ? -latitude : latitude);
        double min = 60.0 * ((neg ? -latitude : latitude) - deg);
        printf("%02d%05.2f%c", deg, min, (neg ? 'S' : 'N'));
        double longitude = State.LD35_lon;
        if(longitude > 180.0)
          longitude = 180.0;
        else if(longitude < -180.0)
          longitude = -180.0;
        neg = longitude < 0.0;
        deg = (int) floor(neg ? -longitude : longitude);
        min = 60.0 * ((neg ? -longitude : longitude) - deg);
        printf("/%03d%05.2f%c", deg, min, (neg ? 'W' : 'E'));
        printf("O000/000");
        int stdev = (int) floor(State.LD35_sdMX/185.6+0.5);
        if(stdev < 0)
          stdev = 0;
        else if(stdev > 999)
          stdev = 999;
        printf("/Err=%02d.%d", stdev/10, stdev%10);
        printf(" ST_land\n");
      } else
        printf("/000000h4000.00N/00800.00WO000/000/Err=99.9 ST_land\n");
      break;
    case sw_cut:
      if(State.PRV_valid()) {
        time_t prv_time = State.LDNW_time;
        struct tm* tm_time = gmtime(& prv_time);
        if(tm_time)
          printf("/%02d%02d%02dh",
                 tm_time->tm_hour, tm_time->tm_min, tm_time->tm_sec);
        else
          printf("/000000h");
        double latitude = State.LDNW_lat;
        if(latitude > 90.0)
          latitude = 90.0;
        else if(latitude < -90.0)
          latitude = -90.0;
        bool neg = latitude < 0.0;
        int deg = (int) floor(neg ? -latitude : latitude);
        double min = 60.0 * ((neg ? -latitude : latitude) - deg);
        printf("%02d%05.2f%c", deg, min, (neg ? 'S' : 'N'));
        double longitude = State.LDNW_lon;
        if(longitude > 180.0)
          longitude = 180.0;
        else if(longitude < -180.0)
          longitude = -180.0;
        neg = longitude < 0.0;
        deg = (int) floor(neg ? -longitude : longitude);
        min = 60.0 * ((neg ? -longitude : longitude) - deg);
        printf("/%03d%05.2f%c", deg, min, (neg ? 'W' : 'E'));
        printf("O000/000");
        int stdev = (int) floor(State.LDNW_sdMX/185.6+0.5);
        if(stdev < 0)
          stdev = 0;
        else if(stdev > 999)
          stdev = 999;
        printf("/Err=%02d.%d", stdev/10, stdev%10);
        printf(" ST_cut\n");
      } else
        printf("/000000h4000.00N/00800.00WO000/000/Err=99.9 ST_cut\n");
      break;
    case sw_sns:
      if(true || State.SNS_valid()) {
        time_t sns_time = State.SENS_time.tv_sec;
        struct tm* tm_time = gmtime(& sns_time);
        if(tm_time)
          printf(":CT8STP-10:%02d%02d%02dh",
                 tm_time->tm_hour, tm_time->tm_min, tm_time->tm_sec);
        else
          printf("000000h");
        for(int i=0; i<20; i++) {
          int ival = (int) floor(State.sensor[i]+0.5);
          if(ival > 9999)
            ival = 9999;
          else if(ival < -999)
            ival = -999;
          if(ival > 999)
            printf(" %d", ival);
          else
            printf(" %+03d", ival);
        }
        printf("\n");
      } else
        printf("000000h ---\n");
      break;
    case sw_cam:
      if(State.Altitude <= 5000.0 || State.Altitude >= 25000.0)
        printf("yes");
      else
        printf("no");
      if(State.Altitude <= 10000.0 || State.Altitude >= 30000.0)
        printf(" yes\n");
      else
        printf(" no\n");
      break;
    default:
      printf("unknown print mode ...\n");
  }
}
