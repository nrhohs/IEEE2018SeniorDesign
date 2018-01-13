#///////////////////////////////////////////////////////////////////////////
#//
#//  This file is part of RTIMULib
#//
#//  Copyright (c) 2014-2015, richards-tech, LLC
#//
#//  Permission is hereby granted, free of charge, to any person obtaining a copy of
#//  this software and associated documentation files (the "Software"), to deal in
#//  the Software without restriction, including without limitation the rights to use,
#//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
#//  Software, and to permit persons to whom the Software is furnished to do so,
#//  subject to the following conditions:
#//
#//  The above copyright notice and this permission notice shall be included in all
#//  copies or substantial portions of the Software.
#//
#//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
#//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
#//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
#//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Compiler, tools and options

RTIMULIBPATH  = Libraries/RTIMULib2/RTIMULib

CC    			= gcc
CXX   			= g++
DEFINES       	=
CFLAGS			= -pipe -O2 -Wall -W $(DEFINES)
CXXFLAGS      	= -pipe -O2 -Wall -W $(DEFINES)
INCPATH       	= -I. -I$(RTIMULIBPATH)
LINK  			= g++
LFLAGS			= -Wl,-O1
LIBS  			= -L/usr/lib/arm-linux-gnueabihf
COPY  			= cp -f
COPY_FILE     	= $(COPY)
COPY_DIR      	= $(COPY) -r
STRIP 			= strip
INSTALL_FILE  	= install -m 644 -p
INSTALL_DIR   	= $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      	= rm -f
SYMLINK       	= ln -f -s
DEL_DIR       	= rmdir
MOVE  			= mv -f
CHK_DIR_EXISTS	= test -d
MKDIR			= mkdir -p

# Output directory

OBJECTS_DIR   = objects/

# Files

DEPS  = $(RTIMULIBPATH)/RTMath.h \
    $(RTIMULIBPATH)/RTIMULib.h \
    $(RTIMULIBPATH)/RTIMULibDefs.h \
    $(RTIMULIBPATH)/RTIMUHal.h \
    $(RTIMULIBPATH)/RTFusion.h \
    $(RTIMULIBPATH)/RTFusionKalman4.h \
    $(RTIMULIBPATH)/RTFusionRTQF.h \
    $(RTIMULIBPATH)/RTIMUSettings.h \
    $(RTIMULIBPATH)/RTIMUAccelCal.h \
    $(RTIMULIBPATH)/RTIMUMagCal.h \
    $(RTIMULIBPATH)/RTIMUCalDefs.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMU.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUNull.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUMPU9150.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUMPU9250.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUGD20HM303D.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUGD20M303DLHC.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUGD20HM303DLHC.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMULSM9DS0.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMULSM9DS1.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUBMX055.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUBNO055.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressure.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureBMP180.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureLPS25H.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureMS5611.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureMS5637.h \
    $(RTIMULIBPATH)/pigpio.h \
    $(RTIMULIBPATH)/vl53l0x_api_calibration.h \
    $(RTIMULIBPATH)/vl53l0x_api_ranging.h \
    $(RTIMULIBPATH)/vl53l0x_api_strings.h \
    $(RTIMULIBPATH)/vl53l0x_api_core.h \
    $(RTIMULIBPATH)/vl53l0x_api.h \
    $(RTIMULIBPATH)/vl53l0x_platform.h
 

OBJECTS = objects/Robot.o \
    objects/RTMath.o \
    objects/RTIMUHal.o \
    objects/RTFusion.o \
    objects/RTFusionKalman4.o \
    objects/RTFusionRTQF.o \
    objects/RTIMUSettings.o \
    objects/RTIMUAccelCal.o \
    objects/RTIMUMagCal.o \
    objects/RTIMU.o \
    objects/RTIMUNull.o \
    objects/RTIMUMPU9150.o \
    objects/RTIMUMPU9250.o \
    objects/RTIMUGD20HM303D.o \
    objects/RTIMUGD20M303DLHC.o \
    objects/RTIMUGD20HM303DLHC.o \
    objects/RTIMULSM9DS0.o \
    objects/RTIMULSM9DS1.o \
    objects/RTIMUBMX055.o \
    objects/RTIMUBNO055.o \
    objects/RTPressure.o \
    objects/RTPressureBMP180.o \
    objects/RTPressureLPS25H.o \
    objects/RTPressureMS5611.o \
    objects/RTPressureMS5637.o \
    objects/Adafruit_VL6180x.o \
    objects/ledRead.o \
    objects/vl53l0x_api_calibration.o \
    objects/vl53l0x_api_core.o \
    objects/vl53l0x_api_ranging.o \
    objects/vl53l0x_api_strings.o \
    objects/vl53l0x_api.o \
    objects/vl53l0x_platform.o \
    objects/rs232.o  
     

MAKE_TARGET	= Robot
DESTDIR		= Output/
TARGET		= Output/$(MAKE_TARGET)

# Build rules

$(TARGET): $(OBJECTS)
	@$(CHK_DIR_EXISTS) Output/ || $(MKDIR) Output/
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS) ./Libraries/wiringPi/devLib/lcd.o -lwiringPi -pthread -lrt -lpigpio

clean:
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core

# Compile

$(OBJECTS_DIR)%.o : $(RTIMULIBPATH)/%.cpp $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $< $(CFLAGS) $(INCPATH)
	
$(OBJECTS_DIR)%.o : $(RTIMULIBPATH)/IMUDrivers/%.cpp $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/Adafruit_VL6180x.o : Libraries/Adafruit_VL6180x.cpp 
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/rs232.o : Libraries/rs232.c
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/ledRead.o : Libraries/ledRead.c $(DEPS) 
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_api_calibration.o : Libraries/vl53l0x_api_calibration.c $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_api_core.o : Libraries/vl53l0x_api_core.c $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_api_ranging.o : Libraries/vl53l0x_api_ranging.c $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_api_strings.o : Libraries/vl53l0x_api_strings.c $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_api.o : Libraries/vl53l0x_api.c $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_platform.o : Libraries/vl53l0x_platform.c $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

$(OBJECTS_DIR)Robot.o : Robot.cpp $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/ 
	$(CXX) -c -o $@ Robot.cpp $(CFLAGS) $(INCPATH) -lm -lwiringPi

# Install

install_target: FORCE
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/usr/local/bin/ || $(MKDIR) $(INSTALL_ROOT)/usr/local/bin/
	-$(INSTALL_PROGRAM) "Output/$(MAKE_TARGET)" "$(INSTALL_ROOT)/usr/local/bin/$(MAKE_TARGET)"
	-$(STRIP) "$(INSTALL_ROOT)/usr/local/bin/$(MAKE_TARGET)"

uninstall_target:  FORCE
	-$(DEL_FILE) "$(INSTALL_ROOT)/usr/local/bin/$(MAKE_TARGET)"


install:  install_target  FORCE

uninstall: uninstall_target   FORCE

FORCE:

