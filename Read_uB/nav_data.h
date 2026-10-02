//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  nav_data.h                                                          //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Structure to transfer navigation data between processes.            //
//                                                                      //
//  This struture is used to exchange measurements from the processes   //
//  that handle GPS receivers and process their data and the clients    //
//  of such data, under the UDP or other protocol.                      //
//                                                                      //
//  (Some member functions have been included to ease data handling.)   //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Sergio Cunha (Nov 2005)                                             //
//                                                                      //
//////////////////////////////////////////////////////////////////////////


#include <time.h>


const unsigned short LLH_VALID = 0x0001;  // Geographic coordinates are valid.
const unsigned short VEL_VALID = 0x0002;  // Velocity values are valid.
const unsigned short ATT_VALID = 0x0004;  // Attitude data is valid.
const unsigned short STG_VALID = 0x0008;  // Stage indicator is valid.
const unsigned short PRV_VALID = 0x0010;  // Landing prevision data is valid.
const unsigned short SNS_VALID = 0x0020;  // Sensor data is valid.

const unsigned short TREND_FLD = 0x0300;  // Flight trend flag field selector.
const unsigned short TREND_INV = 0x0000;  // Flight trend flag data invalid.
const unsigned short TREND_STP = 0x0100;  // Flight trend: capsule stopped.
const unsigned short TREND_CLB = 0x0200;  // Flight trend: capsule climbing.
const unsigned short TREND_DSC = 0x0300;  // Flight trend: capsule descending.
const unsigned short TRUST_FLD = 0x0C00;  // FT confidence flag field selector.
const unsigned short TRUST_INV = 0x0000;  // FT confidence flag data invalid.
const unsigned short TRUST_DKN = 0x0400;  // FT confidence: unknown flight phase.
const unsigned short TRUST_MLD = 0x0800;  // FT confidence: mild confidence.
const unsigned short TRUST_SUR = 0x0C00;  // FT confidence: trust values.

const unsigned short STG_UNKNW = 0x0000;  // Flight stage unknown.
const unsigned short STG_INIT  = 0x0001;  // Flight stage: before take off.
const unsigned short STG_CLIMB = 0x0002;  // Flight stage: climbing.
const unsigned short STG_DESCD = 0x0003;  // Flight stage: descending w/o cut-off.
const unsigned short STG_CTOFF = 0x0004;  // Flight stage: after cut-off.
const unsigned short STG_LAND  = 0x0005;  // Flight stage: landed.
const unsigned short STG_PCOFF = 0x0006;  // Flight stage: parachute cut-off.


#pragma pack(1)
struct NavData {
  char           label[8];    // Station name (zero-padded if needed).

  timespec       NAV_time;    // Seconds since 01/Jan/1970.
  double         Latitude;    //
  double         Longitude;   // Geographic coordinates relative to WGS-84.
  double         Altitude;    // (Values in degrees and meters.)

  double         Vn;          // Velocity towards North.
  double         Ve;          // Velocity towards East.
  double         Vup;         // Upwards velocity.
  double         Vel;         // Norm of the velocity vector (all axis).
  double         Vh;          // Norm of the horizontal velocity.
  double         Course;      // Direction of horizontal velocity (degrees).
  
  double         Roll;        //
  double         Pitch;       // Attitude (degrees).
  double         Heading;     //

  unsigned short flags;       // Several flags packed into two bytes.
  unsigned short stage;       // Stage of the flight.
  
  time_t         PREV_time;   // Time of forecast.
  time_t         LD35_time;   // Predicted landing time for natural burst.
  time_t         LDNW_time;   // Predicted landing time for imediate cut-off.
  double         LD35_lat;    // Predicted landing latitude for natural burst.
  double         LD35_lon;    // Predicted landing longitude for natural burst.
  double         LD35_cvNN;   // Covariance NN for natural burst.
  double         LD35_cvNE;   // Covariance NE for natural burst.
  double         LD35_cvEE;   // Covariance EE for natural burst.
  double         LD35_sdMX;   // Maximal standard dev. for natural burst.
  double         LDNW_lat;    // Predicted landing latitude for imed. cut-off.
  double         LDNW_lon;    // Predicted landing longitude for imed. cut-off.
  double         LDNW_cvNN;   // Covariance NN for immediate cut-off.
  double         LDNW_cvNE;   // Covariance NE for immediate cut-off.
  double         LDNW_cvEE;   // Covariance EE for immediate cut-off.
  double         LDNW_sdMX;   // Maximal standard dev. for immediate cut-off.
  
  timespec       SENS_time;   // Seconds since 01/Jan/1970.
  double         sensor[20];  // Values of up to 20 sensors.
  
  ///////////////////////////////////////////////////////////////////////////
  
  bool           LLH_valid(void)     { return (flags & LLH_VALID) > 0; }
  bool           VEL_valid(void)     { return (flags & VEL_VALID) > 0; }
  bool           ATT_valid(void)     { return (flags & ATT_VALID) > 0; }
  bool           STG_valid(void)     { return (flags & STG_VALID) > 0; }
  bool           PRV_valid(void)     { return (flags & PRV_VALID) > 0; }
  bool           SNS_valid(void)     { return (flags & SNS_VALID) > 0; }
  unsigned short FligthTrend(void)   { return flags & TREND_FLD; }
  unsigned short FT_Trust(void)      { return flags & TRUST_FLD; }
  bool           IsStopped(void)     { return (flags & TREND_FLD) == TREND_STP; }
  bool           IsClimbing(void)    { return (flags & TREND_FLD) == TREND_CLB; }
  bool           IsDescending(void)  { return (flags & TREND_FLD) == TREND_DSC; }
  bool           FT_Known(void)      { return (flags & TRUST_MLD) > 0; }
  bool           FT_Sure(void)       { return (flags & TRUST_FLD) == TRUST_SUR; }
};
#pragma pack()
