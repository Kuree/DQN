# Makefile
# Sample for RH_RF95 on Raspberry Pi with Lora/GPS hat
# http://www.dragino.com/products/module/item/106-lora-gps-hat.html
# Based on the sample for RH_NRF24 on Raspberry Pi
# Caution: requires bcm2835 library to be already installed
# http://www.airspayce.com/mikem/bcm2835/

CC            = g++
CFLAGS        = -DRASPBERRY_PI -DBCM2835_NO_DELAY_COMPATIBILITY
#LIBS          = -lbcm2835
LIBS          = -lwiringPi
RADIOHEADBASE = ./RadioHead
INCLUDE       = -I$(RADIOHEADBASE)
BUILDDIR			= build

all: builddir DQNserver

builddir:
	mkdir -p $(BUILDDIR)

RasPi.o: $(RADIOHEADBASE)/RHutil/RasPi.cpp
	$(CC) $(CFLAGS) -c $^ $(INCLUDE) -o $(BUILDDIR)/$@

RasPiWiring.o: $(RADIOHEADBASE)/RHutil/RasPiWiring.cpp
	$(CC) $(CFLAGS) -c $^ $(INCLUDE) -o $(BUILDDIR)/$@

basestation.o: basestation.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

RH_RF95.o: $(RADIOHEADBASE)/RH_RF95.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

RHMesh.o: $(RADIOHEADBASE)/RHMesh.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

RHRouter.o: $(RADIOHEADBASE)/RHRouter.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

RHReliableDatagram.o: $(RADIOHEADBASE)/RHReliableDatagram.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

RHDatagram.o: $(RADIOHEADBASE)/RHDatagram.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

RHHardwareSPI.o: $(RADIOHEADBASE)/RHHardwareSPI.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

RHNRFSPIDriver.o: $(RADIOHEADBASE)/RHNRFSPIDriver.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

RHSPIDriver.o: $(RADIOHEADBASE)/RHSPIDriver.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

RHGenericDriver.o: $(RADIOHEADBASE)/RHGenericDriver.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

RHGenericSPI.o: $(RADIOHEADBASE)/RHGenericSPI.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

DQNserver.o: DQNserver.cpp
	$(CC) $(CFLAGS) -c $(INCLUDE) $^ -o $(BUILDDIR)/$@

DQNserver: DQNserver.o RH_RF95.o RasPiWiring.o RHHardwareSPI.o RHSPIDriver.o RHGenericDriver.o RHGenericSPI.o
	$(CC) $(addprefix $(BUILDDIR)/,$^) $(LIBS) -o DQNserver

clean:
	rm -rf $(BUILDDIR) *.o DQNserver
