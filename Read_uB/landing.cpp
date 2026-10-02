//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  landing.cpp                                                         //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Routines to compute impact are for Straplex (kalman filter).        //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Nuno Lima e Sergio Cunha (Nov 2005)                                 //
//                                                                      //
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
// Matrix Function Header
#define WANT_STREAM			  // include.h will get stream fns
#define WANT_MATH         // include.h will get math fns
                          // newmatap.h will get include.h
#include <newmatap.h>     // need matrix applications
#include <newmatio.h>			// need matrix output routines
#ifdef use_namespace
using namespace NEWMAT;		// access NEWMAT namespace
#endif
//////////////////////////////////////////////////////////////////////////
#include <iostream.h>
#include "udp_comm.h"
#include <stdlib.h>
#include <math.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include "nav_data.h"


//////////////////////////////////////////////////////////////////////////
// Local global constants:

static const double v_up = 5.6;     // Vertical velocity
static const double S    = 1.0641;  // Parachute area
static const double Cx   = 1.42;    // Parachute drag coefficient
static const double M    = 3.0;     // Global mass of the falling payload
static const double Hmax = 35000.0; // Expected bursting altitude
static const time_t Kdt  = 10;      // Interval to update Kalman filter

static const double std_Ph  = 3.0;  // Horizontal position error std dev
static const double std_Pv  = 3.0;  // Vertical position error std dev
static const double std_dVh = 0.01; // Vertical velocity error std dev
static const double std_W   = 0.05; // Wind error std dev
static const double std_GPS = 5.0;  // GPS position error std dev

// Digital Terrain Model
static const double DTM = 300.0;		// INFO NEEDED

// General constants:
static const double WGS84_MAJ  = 6378137.000;    // WGS_84 Earth major axis.
static const double WGS84_ECC2 = 6.694380023e-3; // Squared excentricity of the Earth.
static const double RAD2DEG    = 180.0/M_PI;
static const double DEG2RAD    = M_PI/180.0;


// Global variables
//
extern NavData State;
extern double  Wind100_N;
extern double  Wind100_E;
extern double  Wind200_N;
extern double  Wind200_E;
extern double  Wind300_N;
extern double  Wind300_E;
extern double  Wind400_N;
extern double  Wind400_E;


// Local global variables:
//
static time_t           Kal_time = 0;
static Matrix           Kal_state(12,1);
static SymmetricMatrix  Kal_cov(12);
///////////////////////////////////////////////////////////////////////////

// Function prototypes:
//
static double fall_speed(double h);
static double prev(Matrix &state1, SymmetricMatrix &cov1, double H_cutdown, double dt);
static void update_Kalman(void);
void landing(void);


///////////////////////////////////////////////////////////////////////////
//UDP Port Open
//

void udp_open_port(void)
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

///////////////////////////////////////////////////////////////////////////

//
//
//

///////////////////////////////////////////////////////////////////////////
// Free fall velocity in standard air:

static double fall_speed(double h)
{
	double T, P;

	if (h >= 25000)	{
	  T = -131.21 + 0.00299*h;
	  P = 2.488 * pow(((T + 263.1)/216.6),(-11.388));
	}	else if (h >= 11000) {
	  T = -56.46 + 0.0*h;
	  P = 22.65 * exp(1.73 - 0.000157 * h); 
	}	else {
	  T = 15.04 - 0.00649 * h;
	  P = 101.29 * pow(((T + 273.1)/288.08),5.256);
	}
	
	double d = P / (0.2869 * (T + 273.1));

	P = P * 10;

	double v_h = sqrt(2 * M * 9.8 / Cx / d / S);
	return v_h;
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
// Prevision Function
static double prev(Matrix &state1, SymmetricMatrix &cov1, double H_cutdown, double dt)
{
	double alfa_100 = 0, alfa_200 = 0, alfa_300 = 0, alfa_400 = 0; // wind control
	double T;

	if (dt >= 999.9) {
    T = 18000.0;
    dt = 20;
	}	else if (dt > 20) {
    T = dt;
    dt = 20;
	}	else
	  T = dt;

	state1 = Kal_state;
	cov1   = Kal_cov;

	double t = 0.0;
	while (t < T) {
		//////////////////////////////////////////
		//wind altitude control
		double H_feet = state1(3,1)/0.3048;
		
		alfa_100 = 1.0 - fabs((H_feet-10000.0)/10000.0);
		if(alfa_100 < 0.0)
		  alfa_100 = 0.0;
		if(H_feet < 10000.0)
		  alfa_100 = 1.0;
		
		alfa_200 = 1.0 - fabs((H_feet-20000.0)/10000.0);
		if(alfa_200 < 0.0)
		  alfa_200 = 0.0;
		
		alfa_300 = 1.0 - fabs((H_feet-30000.0)/10000.0);
		if(alfa_300 < 0.0)
		  alfa_300 = 0.0;

		alfa_400 = 1.0 - fabs((H_feet-40000.0)/10000.0);
		if(alfa_400 < 0.0)
		  alfa_400 = 0.0;
		if(H_feet > 40000.0)
		  alfa_400 = 1.0;
		//////////////////////////////////////////

		double v_h = v_up;
		if (state1(3,1) >= H_cutdown || H_cutdown <= 0.0) {
		  v_h = -fall_speed(state1(3,1));	// replace this by the formulae ...
		  H_cutdown = -10000.0;
		}

		//Matrix Ak definition
		IdentityMatrix I12(12);
		Matrix Ak(12,12); Ak = I12;
		Ak(1, 5) = dt*alfa_100;
		Ak(1, 7) = dt*alfa_200;
		Ak(1, 9) = dt*alfa_300;
		Ak(1,11) = dt*alfa_400;
		Ak(2, 6) = dt*alfa_100;
		Ak(2, 8) = dt*alfa_200;
		Ak(2,10) = dt*alfa_300;
		Ak(2,12) = dt*alfa_400;
		Ak(3, 4) = dt;
		
		double CosLatitude = cos(state1(1,1) * DEG2RAD);
		double SinLatitude = sin(state1(1,1) * DEG2RAD);
    double den = sqrt(1.0 - WGS84_ECC2*(SinLatitude*SinLatitude));
    double Rm = WGS84_MAJ*(1-WGS84_ECC2)/(den*den*den) + state1(3,1);
    double Rp = (WGS84_MAJ/den + state1(3,1))*CosLatitude;

		Matrix LLH_Ak(12,12); LLH_Ak = Ak;
		LLH_Ak(1, 5) = Ak(1, 5)/Rm*RAD2DEG;
		LLH_Ak(1, 7) = Ak(1, 7)/Rm*RAD2DEG;
		LLH_Ak(1, 9) = Ak(1, 9)/Rm*RAD2DEG;
		LLH_Ak(1,11) = Ak(1,11)/Rm*RAD2DEG;
		LLH_Ak(2, 6) = Ak(2, 6)/Rp*RAD2DEG;
		LLH_Ak(2, 8) = Ak(2, 8)/Rp*RAD2DEG;
		LLH_Ak(2,10) = Ak(2,10)/Rp*RAD2DEG;
		LLH_Ak(2,12) = Ak(2,12)/Rp*RAD2DEG;

		state1 = LLH_Ak*state1;
		state1(3,1) += v_h*dt;

		Matrix Q(12,12); Q = 0.0;
		Q(1,1)   =  std_Ph*std_Ph;
		Q(2,2)   =  std_Ph*std_Ph;
		Q(3,3)   =  std_Pv*std_Pv;
		Q(4,4)   =  std_dVh*std_dVh;
		Q(5,5)   =  std_W*std_W;
		Q(6,6)   =  std_W*std_W;
		Q(7,7)   =  std_W*std_W;
		Q(8,8)   =  std_W*std_W;
		Q(9,9)   =  std_W*std_W;
		Q(10,10) =  std_W*std_W;
		Q(11,11) =  std_W*std_W;
		Q(12,12) =  std_W*std_W;

		cov1 << Ak*cov1*Ak.t() + Q*dt;

		t = t+dt;

		if (T > 17900.0 && state1(3,1) <= DTM && v_h < 0.0)
      break;
	}

  return t;
}
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// State and covariance update function:
static void update_Kalman(void)
{
  Matrix           state1(12,1);
  SymmetricMatrix  cov1(12);

  double h_max = Hmax;
  if(State.STG_valid() && State.stage >= STG_DESCD)
    h_max = -1000.0;
  prev(state1, cov1, h_max, State.NAV_time.tv_sec-Kal_time);
  Kal_state = state1;
  Kal_cov = cov1;

	Matrix Hk(3,12); Hk = 0.0;

	Hk(1,1) = 1.0;
	Hk(2,2) = 1.0;
	Hk(3,3) = 1.0;

	SymmetricMatrix cov_obs(3); cov_obs = 0.0;
	cov_obs(1,1) = std_GPS*std_GPS;
	cov_obs(2,2) = std_GPS*std_GPS;
	cov_obs(2,2) = std_GPS*std_GPS;

	Matrix K = (Kal_cov*Hk.t())*(Hk*Kal_cov*Hk.t()+cov_obs).i();

  double CosLatitude = cos(Kal_state(1,1) * DEG2RAD);
  double SinLatitude = sin(Kal_state(1,1) * DEG2RAD);
  double den = sqrt(1.0 - WGS84_ECC2*(SinLatitude*SinLatitude));
  double Rm = WGS84_MAJ*(1-WGS84_ECC2)/(den*den*den) + Kal_state(3,1);
  double Rp = (WGS84_MAJ/den + Kal_state(3,1))*CosLatitude;

  Matrix delta_obs(3,1);
  delta_obs(1,1) = (State.Latitude - Kal_state(1,1))*DEG2RAD*Rm;
  delta_obs(2,1) = (State.Longitude - Kal_state(2,1))*DEG2RAD*Rp;
  delta_obs(3,1) = (State.Altitude - Kal_state(3,1));

	Matrix LLH_K = K;
	LLH_K(1,1) = LLH_K(1,1)/Rm*RAD2DEG;
	LLH_K(1,2) = LLH_K(1,2)/Rm*RAD2DEG;
	LLH_K(1,3) = LLH_K(1,3)/Rm*RAD2DEG;
	LLH_K(2,1) = LLH_K(2,1)/Rp*RAD2DEG;
	LLH_K(2,2) = LLH_K(2,2)/Rp*RAD2DEG;
	LLH_K(2,3) = LLH_K(2,3)/Rp*RAD2DEG;

	Kal_state = Kal_state + LLH_K*delta_obs;

	IdentityMatrix I12(12);
	Kal_cov << (I12-K*Hk)*Kal_cov*(I12-K*Hk).t()+K*cov_obs*K.t();

	Kal_time = State.NAV_time.tv_sec;
}

void init_Kalman(void)
{
  if(! State.LLH_valid())
    return;

  Kal_time = State.NAV_time.tv_sec;

  Kal_state( 1,1) = State.Latitude;
  Kal_state( 2,1) = State.Longitude;
  Kal_state( 3,1) = State.Altitude;
  Kal_state( 4,1) = 0.0;
  Kal_state( 5,1) = Wind100_N;
  Kal_state( 6,1) = Wind100_E;
  Kal_state( 7,1) = Wind200_N;
  Kal_state( 8,1) = Wind200_E;
  Kal_state( 9,1) = Wind300_N;
  Kal_state(10,1) = Wind300_E;
  Kal_state(11,1) = Wind400_N;
  Kal_state(12,1) = Wind400_E;
  
  Kal_cov = 0.0;
  Kal_cov( 1, 1) = 50.0*50.0;
  Kal_cov( 2, 2) = 50.0*50.0;
  Kal_cov( 3, 3) = 50.0*50.0;
  Kal_cov( 4, 4) =  2.0*2.0;
  Kal_cov( 5, 5) =  4.0*4.0;
  Kal_cov( 6, 6) =  4.0*4.0;
  Kal_cov( 7, 7) =  5.0*5.0;
  Kal_cov( 8, 8) =  5.0*5.0;
  Kal_cov( 9, 9) =  5.0*5.0;
  Kal_cov(10,10) =  5.0*5.0;
  Kal_cov(11,11) =  5.0*5.0;
  Kal_cov(12,12) =  5.0*5.0;
}
///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
// Main function: update prevision:
//
void landing(void)
{
  // Initialize and proceed only if this it succeeds:
  if(! Kal_time)
    init_Kalman();
  if(! Kal_time)
    return;

  // Check if the Kalman filter needs to be updated:
  if(State.STG_valid() && State.LLH_valid() &&
     State.stage >= STG_CLIMB && State.stage <= STG_DESCD &&
     State.NAV_time.tv_sec-Kal_time >= Kdt)
    update_Kalman();

  // Make landing forecasts if needed:
  if(! State.PRV_valid() || Kal_time-State.PREV_time >= Kdt
                         || State.NAV_time.tv_sec-State.PREV_time >= Kdt ) {
    Matrix           state1(12,1);
    SymmetricMatrix  cov1(12);
    double T;

    T = prev(state1, cov1, -1000.0, 20000.0);
    State.PREV_time = Kal_time;
    State.LDNW_time = Kal_time + (time_t) (T+0.5);
    State.LDNW_lat  = state1(1,1);
    State.LDNW_lon  = state1(2,1);
    State.LDNW_cvNN = cov1(1,1);
    State.LDNW_cvNE = cov1(1,2);
    State.LDNW_cvEE = cov1(2,2);
    double aux1 = State.LDNW_cvNN + State.LDNW_cvEE;
    double aux2 = State.LDNW_cvNN - State.LDNW_cvEE;
    State.LDNW_sdMX = 0.5*(aux1 + sqrt(aux2*aux2+4.0*State.LDNW_cvNE*State.LDNW_cvNE));
    State.LDNW_sdMX = sqrt(State.LDNW_sdMX);

    if(State.STG_valid() && State.stage >= STG_DESCD) {
      State.LD35_time = State.LDNW_time;
      State.LD35_lat  = State.LDNW_lat;
      State.LD35_lon  = State.LDNW_lon;
      State.LD35_cvNN = State.LDNW_cvNN;
      State.LD35_cvNE = State.LDNW_cvNE;
      State.LD35_cvEE = State.LDNW_cvEE;
      State.LD35_sdMX = State.LDNW_sdMX;
    } else {
      T = prev(state1, cov1, Hmax, 20000.0);
      State.LD35_time = Kal_time + (time_t) (T+0.5);
      State.LD35_lat  = state1(1,1);
      State.LD35_lon  = state1(2,1);
      State.LD35_cvNN = cov1(1,1);
      State.LD35_cvNE = cov1(1,2);
      State.LD35_cvEE = cov1(2,2);
      aux1 = State.LD35_cvNN + State.LD35_cvEE;
      aux2 = State.LD35_cvNN - State.LD35_cvEE;
      State.LD35_sdMX = 0.5*(aux1 + sqrt(aux2*aux2+4.0*State.LD35_cvNE*State.LD35_cvNE));
      State.LD35_sdMX = sqrt(State.LD35_sdMX);
    }
    State.flags |= PRV_VALID;
  }
}
