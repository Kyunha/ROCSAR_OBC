#
#  Makefile
#


CC = gcc
CPP = g++
CFLAGS = -g -Wno-deprecated -fno-delete-null-pointer-checks -fno-strict-aliasing -O0 -Inewmat

UBOBJS = udp_comm.o Read_uB.o clock_gettime.o
UMOBJS = udp_comm.o Read_uB_meas.o clock_gettime.o
SPOBJS = udp_comm.o Read_Sep.o crc.o clock_gettime.o
STOBJS = udp_comm.o state.o landing.o
RDOBJS = udp_comm.o read_state.o
VLOBJS = udp_comm.o read_state_veleiro.o read_state_veleiro_main.o utm.o
SVOBJS = udp_comm.o server.o
CLOBJS = udp_comm.o client.o
COOBJS = udp_comm.o client_once.o

all: Read_uB Read_uB_meas Read_Sep state read_state read_state_veleiro server client client_once

Read_uB:$(UBOBJS)
	$(CPP) $(CFLAGS) -o Read_uB $(UBOBJS)

Read_uB_meas:$(UMOBJS)
	$(CPP) $(CFLAGS) -o Read_uB_meas $(UMOBJS)

Read_Sep:$(SPOBJS)
	$(CPP) $(CFLAGS) -o Read_Sep $(SPOBJS)

state: $(STOBJS)
	$(CPP) $(CFLAGS) -o state $(STOBJS) -Lnewmat -lnewmat -lm

read_state: $(RDOBJS)
	$(CPP) $(CFLAGS) -o read_state $(RDOBJS)

read_state_veleiro: $(VLOBJS)
	$(CPP) $(CFLAGS) -o read_state_veleiro $(VLOBJS)

server: $(SVOBJS)
	$(CPP) $(CFLAGS) -o server $(SVOBJS)

client: $(CLOBJS)
	$(CPP) $(CFLAGS) -o client $(CLOBJS)

client_once: $(COOBJS)
	$(CPP) $(CFLAGS) -o client_once $(COOBJS)

Read_uB.o: Read_uB.cpp Read_uB.h
	$(CPP) $(CFLAGS) -c Read_uB.cpp

Read_Sep.o: Read_Sep.cpp Read_Sep.h
	$(CPP) $(CFLAGS) -c Read_Sep.cpp

crc.o: crc.cpp
	$(CPP) $(CFLAGS) -c crc.cpp

udp_comm.o: udp_comm.cpp udp_comm.h
	$(CPP) $(CFLAGS) -c udp_comm.cpp

state.o: state.cpp udp_comm.h nav_data.h
	$(CPP) $(CFLAGS) -c state.cpp

landing.o: landing.cpp nav_data.h
	$(CPP) $(CFLAGS) -c landing.cpp

read_state.o: read_state.cpp udp_comm.h nav_data.h
	$(CPP) $(CFLAGS) -c read_state.cpp

read_state_veleiro.o: read_state_veleiro.cpp read_state_veleiro.h udp_comm.h nav_data.h
	$(CPP) $(CFLAGS) -c read_state_veleiro.cpp

read_state_veleiro_main.o: read_state_veleiro_main.cpp read_state_veleiro.h
	$(CPP) $(CFLAGS) -c read_state_veleiro_main.cpp

utm.o: utm.cpp read_state_veleiro.h
	$(CPP) $(CFLAGS) -c utm.cpp

server.o: server.cpp udp_comm.h
	$(CPP) $(CFLAGS) -c server.cpp

client.o: client.cpp udp_comm.h
	$(CPP) $(CFLAGS) -c client.cpp

client_once.o: client_once.cpp udp_comm.h
	$(CPP) $(CFLAGS) -c client_once.cpp

clock_gettime.o: clock_gettime.cpp
	$(CPP) $(CFLAGS) -c clock_gettime.cpp

