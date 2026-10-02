

const char  EPH_FILE[]   = "eph.dat";
const char  UDP_PORT[]   = "1973";
const bool  ED73_POLY    = false;
const bool  ED73_OFFSET  = true;
const int   uB_DATA_LEN  = 1024;
const unsigned char SOH1 = 0xB5;
const unsigned char SOH2 = 0x62;
const int TERN_DATA_LEN  = 2048;
const unsigned char SOP  = '#';
const unsigned char EOP  = '%';

#define BUFFER_SZ       1024

const double DT_GPS_RAW  = 0.1;
const double DT_GPS_POS  = 0.5;

const double DPOS_MIN    = 5.0;
const double DPOS_MAJ    = 150.0;
const double ALFA_MIN    = 0.05;
const double ALFA_MAJ    = 0.5;

const double MAX_RES     = 0.2;
const double MAX_RES0    = 1.0;
const double MIN_SVAL    = 0.1;
const double MAX_ACC     = 9.8;

const double MAX_DCLK_STD   = 0.1;
const double MAX_DCLK_MAX   = 0.2;
const double MAX_CLK_dDRIFT = 0.5;
const double CLK_TAU_OK     = 10.0;
const int    NUM_CD_BAD     = 5;

const double WEIGHT_CLK_DRIFT = 1.0;
const double WEIGHT_HEIGHT    = 1.0;
const double WEIGHT_HDG       = 1.0;
const double WEIGHT_VEL       = 1.0;
const double MAX_VVERT        = 20.0;

const double MAX_DT_SYNC  = 0.6;
const double MAX_DT_SLEEP = 5.0;
const long   SYNC_SLEEP   = 5;

const double ODOM_FACTOR = 0.042;


#ifndef M_PI
#define M_PI        3.14159265358979323846
#endif

const double         WGS84MAJ    = 6378137.000;
const double         WGS84MIN    = 6356752.314;
const double         WGS84ECC2   = 6.694380023e-3;
const double         NYU_2       = 1.996498184321e7;
const double         OMEGAEDOT   = 7.2921151467e-5;
const double         SOL         = 299792458.0;
const double         GPS_HZ2M    = 299.792458 / 1575.42;

const double         VEL_MIN     = 1.0;

const double         DEG2RAD     = M_PI / 180.0;
const double         RAD2DEG     = 180.0 / M_PI;


struct GPS_blk {
  char          sv;
  double        snr;
  double        pr;
  double        dp;
  double        cphase;
  int           n_cslip;
};

struct MeasMsg{
  double    gps_tow;
  int       num_blks;
  GPS_blk   gps_blk[12];
  int       num_blks2;
  GPS_blk   gps_blk2[12];
  double    clk_off;
  double    clk_drift;
  double    latitude;
  double    longitude;
  double    height;
  double    course;
  double    velocity;
  double    Vh;
  double    Vn;
  double    Ve;
  double    Vd;
  double    distance;
};

struct EPH_DATA {
  double  Time;       // GPS time of last update (seconds)
  bool    Flag;       // true if ephemeris data valid; false otherwise
  long    IODE;       //  \     (not really used ...)
  double  Toc;        //   |
  double  Af2;        //   |
  double  Af1;        //   |
  double  Af0;        //   |
  double  Crs;        //   |
  double  DELTAN;     //   |
  double  MSUBO;      //   |
  double  Cuc;        //   |
  double  e;          //    \   According to the document:
  double  Cus;        //    /         ICD-GPS-200
  double  SQRTA;      //   |
  double  Toe;        //   |
  double  Cic;        //   |
  double  OMEGASUBO;  //   |
  double  Cis;        //   |
  double  ISUBO;      //   |
  double  Crc;        //   |
  double  OMEGA;      //   |
  double  OMEGADOT;   //   |
  double  IDOT;       //  /
};
typedef EPH_DATA * PEPH_DATA;

struct Point3D {
  double X;
  double Y;
  double Z;
};

#pragma pack(1)
struct GUI_buffer {				// Structure to pass data for publication.
	double GPStime;					// GPS time related to publication.
  double Latitude;				// Reference origin position latitude, in radians.
  double Longitude;      	// Reference origin position longitude, in radians.
  double Height;         	// Reference origin position height, in meters.
  double Vnorth;         	// IMU origin horizontal velocity to the North.
  double Veast;          	// IMU horizontal velocity to the East.
  double Vdown;          	// IMU vertical velocity, pointing down.
  double Roll;						// Attitude Euler angle 1 - Roll.
  double Pitch;          	// Attitude Euler angle 2 - Pitch.
  double Heading;        	// Attitude Euler angle 3 - Heading.
  double AccBiX;					// X accelerometer bias.
  double AccBiY;					// Y accelerometer bias.
  double AccBiZ;					// Z accelerometer bias.
  double GyrBiX;					// X gyro bias.
  double GyrBiY;					// Y gyro bias.
  double GyrBiZ;					// Z gyro bias.
  double Ganom;					 	// Vertical gravity anomaly.
};
#pragma pack()

#pragma pack(1)
struct UDP_message {
   unsigned char Start;
   uint32_t      Epochs;
   GUI_buffer    Data;
   unsigned char End;
};
#pragma pack()


void BroadcastData(const MeasMsg & meas);
Point3D XYZ_to_LLH(const Point3D & xyz);
Point3D XYZ_to_NED(const Point3D & xyz, const Point3D & llh);
Point3D LLH_to_ED73(const Point3D & llh);
Point3D XYZ_to_ED73(const Point3D & xyz);
Point3D XYZ_to_NED(const Point3D & xyz, const Point3D & llh);
unsigned long Read_IP(char* str);

