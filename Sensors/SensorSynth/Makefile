#DEFINITIONS

CFLAGS		= -pipe -02 -Wall -W 
CXXFLAGS	= -pipe -02 -Wall -W	
INCPATH		= -I. -I./src

DEPS = 	$(INCPATH)/Adafruit_VL6180x.h \
	$(INCPATH)/pigpio.h \
	$(INCPATH)/ledRead.h \
	$(INCPATH)/vl53l0x_api_calibration.h \
	$(INCPATH)/vl53l0x_api_ranging.h \
	$(INCPATH)/vl53l0x_api_strings.h \
	$(INCPATH)/vl53l0x_api_core.h \
	$(INCPATH)/vl53l0x_api.h \
	$(INCPATH)/vl53l0x_platform.h

OBJECTS =	objects/SensorLib.o \
		objects/vl53l0x_api_calibration.o \
		objects/vl53l0x_api_ranging.o \
		objects/vl53l0x_api_strings.o \
		objects/vl53l0x_api_core.o \
		objects/vl53l0x_api.o \
		objects/vl53l0x_platform.o \
		objects/ledRead.o

TARGET = 	Output/SensorTest

#Build rules

