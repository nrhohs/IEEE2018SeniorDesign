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
    Libraries/inc/vl53l0x_setup.h \
    Libraries/inc/vl53l0x_tuning.h \
    Libraries/inc/vl53l0x_types.h \
    Libraries/inc/vl53l0x_api_calibration.h \
    Libraries/inc/vl53l0x_api_core.h \
    Libraries/inc/vl53l0x_api_ranging.h \
    Libraries/inc/vl53l0x_api_strings.h \
    Libraries/inc/vl53l0x_def.h \
    Libraries/inc/vl53l0x_device.h \
    Libraries/inc/vl53l0x_i2c_platform.h \
    Libraries/inc/vl53l0x_interrupt_threshold_settings.h \
    Libraries/inc/vl53l0x_platform_log.h 

 

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
    objects/rs232.o \
    objects/SensorLib2.o \
    objects/Adafruit_VL6180X.o \
    objects/vl53l0x_api.o \
    objects/vl53l0x_platform.o \
    objects/vl53l0x_api_calibration.o \
    objects/vl53l0x_api_core.o \
    objects/vl53l0x_api_ranging.o \
    objects/vl53l0x_api_strings.o \
    objects/vl53l0x_setup.o \
    objects/navigation.o 
     

MAKE_TARGET	= Robot
DESTDIR		= Output/
TARGET		= Output/$(MAKE_TARGET)

# Build rules

$(TARGET): $(OBJECTS)
	@$(CHK_DIR_EXISTS) Output/ || $(MKDIR) Output/
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS) -std=c++11 ./Libraries/wiringPi/devLib/lcd.o -lwiringPi -pthread -lrt -lpigpio

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

objects/rs232.o : Libraries/rs232.c Libraries/rs232.h
	test -d objects/ || mkdir -p objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/Adafruit_VL6180X.o : Libraries/Adafruit_VL6180X.cpp Libraries/Adafruit_VL6180X.h
	test -d objects/ || mkdir -p objects/
	$(CXX) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_api.o : Libraries/vl53l0x_api.c Libraries/inc/vl53l0x_api.h $(DEPS)
	test -d objects/ || mkdir -p objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_platform.o : Libraries/vl53l0x_platform.c Libraries/inc/vl53l0x_platform.h $(DEPS)
	test -d objects/ || mkdir -p objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_api_calibration.o : Libraries/vl53l0x_api_calibration.c $(DEPS)
	test -d objects/ || mkdir -p objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_api_core.o : Libraries/vl53l0x_api_core.c $(DEPS)
	test -d objects/ || mkdir -p objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_api_ranging.o : Libraries/vl53l0x_api_ranging.c $(DEPS)
	test -d objects/ || mkdir -p objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_api_strings.o : Libraries/vl53l0x_api_strings.c $(DEPS)
	test -d objects/ || mkdir -p objects/
	$(CC) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/vl53l0x_setup.o : Libraries/vl53l0x_setup.cpp $(DEPS)
	test -d objects/ || mkdir -p objects/
	$(CXX) -c -o $@ $< $(CFLAGS) $(INCPATH)

objects/SensorLib2.o : Libraries/Adafruit_VL6180X.h Libraries/inc/vl53l0x_api.h Libraries/inc/vl53l0x_platform.h Libraries/SensorLib2.h Libraries/SensorLib2.cpp $(DEPS)
	test -d objects/ || mkdir -p objects/
	$(CXX) -c -o $@ $(CFLAGS) -std=c++11 Libraries/SensorLib2.cpp  -lrt -pthread -lpigpio

objects/navigation.o : Libraries/navigation.h Libraries/navigation.cpp Libraries/SensorLib2.h
	test -d objects/ || mkdir -p objects/
	$(CXX) -c -o $@ $(CFLAGS) -std=c++11 Libraries/navigation.cpp

objects/Robot.o : $(OBJECTS) Robot3.cpp $(DEPS) Libraries/navigation.h Libraries/SensorLib2.h Libraries/rs232.h
	test -d objects/ || mkdir -p objects/
	$(CXX) -c -o $@ Robot3.cpp -std=c++11 $(CFLAGS) $(INCPATH) -lm -lwiringPi -lrt -lpigpio -pthread
