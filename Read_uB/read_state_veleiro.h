//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  read_state_veleiro.h                                                //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Header file of "read_state_veleiro.cpp"                             //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Sergio Cunha (Mai 2008)                                             //
//                                                                      //    
//////////////////////////////////////////////////////////////////////////      


struct utm_coord {
  double Easting;
  double Northing;
  double Vel;
  double Course;

  int  utmXZone;
  char utmYZone;

  bool pos_valid;
  bool vel_valid;
  bool course_valid;
};


int init_connection(int argc, char** argv);
int read_state_veleiro(utm_coord* utm_state);
void LatLonToUtmWGS84 (int& utmXZone, char& utmYZone,
                       double& easting, double& northing,
		       double lat, double lon);

