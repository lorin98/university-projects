PREFIX=../
CC=gcc
INCLUDE_DIRS=-I$(PREFIX)/src/common -I$(PREFIX)/src/orazio_host/ 
CC_OPTS=-Wall -Ofast -std=gnu99 $(INCLUDE_DIRS) #-D_VERBOSE_


LIBS=-lpthread -lwebsockets 


LOBJS = packet_handler.o\
	deferred_packet_handler.o\
	linenoise.o\
	orazio_client.o\
	orazio_print_packet.o\
	serial_linux.o\

OBJS = 	ventry.o\
	orazio_shell_globals.o\
	orazio_shell_commands.o\
	orazio_ws_server.o


HEADERS = packet_header.h\
	  packet_operations.h\
	  packet_handler.h\
	  deferred_packet_handler.h\
	  orazio_packets.h\
	  orazio_print_packet.h\
	  ventry.h

BINS= 	orazio\
	orazio_client_test\
	orazio_client_test_simplified\
	orazio_client_with_apriltag\
	log_parser


.phony:	clean all

all:	$(BINS) 

#common objects
%.o:	$(PREFIX)/src/common/%.c 
	$(CC) $(CC_OPTS) -c  $<

#host test
%.o:	$(PREFIX)/src/host_test/%.c 
	$(CC) $(CC_OPTS) -c  $<

#client 
%.o:	$(PREFIX)/src/orazio_host/%.c 
	$(CC) $(CC_OPTS) -c  $<

#host test - 'code' folder
%.o:	$(PREFIX)/src/host_test/code/%.c 
	$(CC) $(CC_OPTS) -c  $<

orazio: orazio.o $(OBJS) $(LOBJS)
	$(CC) $(CC_OPTS) -o $@ $< $(OBJS) $(LOBJS) $(LIBS)

orazio_client_test:  orazio_client_test.o orazio_client_test_getkey.o $(LOBJS)
	$(CC) $(CC_OPTS) -o $@ $^ $(LIBS)

orazio_client_test_simplified:  orazio_client_test_simplified.o orazio_client_test_getkey.o $(LOBJS)
	$(CC) $(CC_OPTS) -o $@ $^ $(LIBS)

orazio_client_with_apriltag:  orazio_client_with_apriltag.o orazio_client_test_getkey.o camera_tools.o pose_estimate.o $(LOBJS)
	$(CC) $(CC_OPTS) -o $@ $^ $(LIBS) -lopencv_core -lopencv_highgui -lopencv_imgproc -lapriltag -lm

log_parser: log_parser.o pose_estimate.o g2o_graph_build.o $(LOBJS)
	$(CC) $(CC_OPTS) -o $@ $^ $(LIBS) -lapriltag -lm
clean:
	rm -rf $(OBJS) $(BINS) *~ *.d *.o

