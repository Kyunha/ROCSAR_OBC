//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  read_state_veleiro_main.cpp                                         //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Just to test "read_state_veleiro" ...                               //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  Sergio Cunha (Mai 2008)                                             //
//                                                                      //
//////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include "read_state_veleiro.h"


main(int argc, char** argv)
{
  utm_coord st;

  init_connection(argc, argv);

  while(1) {
    st.Easting = st.Northing = 0.0;
    st.utmXZone = 0;
    st.utmYZone = '-';
    st.pos_valid = st.vel_valid = st.course_valid = false;

    read_state_veleiro(& st);

    printf("POS: %s -> %.3lf, %.3lf (%d%c)\n", (st.pos_valid ? "ok" : "KO"),
					       st.Easting, st.Northing,
					       st.utmXZone, st.utmYZone);
    printf("VEL: %s -> %.3lf\n", (st.vel_valid ? "ok" : "KO"), st.Vel);
    printf("CRS: %s -> %.3lf\n", (st.course_valid ? "ok" : "KO"), st.Course);
  }
}

