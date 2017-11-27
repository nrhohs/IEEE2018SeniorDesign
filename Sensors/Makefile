CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
RM = rm

CFLAGS = -O0 -g -Wall -c

OUTPUT_DIR = bin
OBJ_DIR = obj
 
TARGET_LIB = $(OUTPUT_DIR)/libVL53L0X_Rasp.a

INCLUDES = \
	-I. \
	-I./VL53L0X_API/Api/core/inc \
	-I./TOF \
	-I./VL53L0x/platform/inc 

VPATH = \
	./VL53L0X_API/Api/core/src \
	./TOF \
	./VL53L0x/platform/src/ 

LIB_SRCS = \
	vl53l0x_api_calibration.cpp \
	vl53l0x_api_core.cpp \
	vl53l0x_api_ranging.cpp \
	vl53l0x_api_strings.cpp \
	vl53l0x_setup.cpp \
	vl53l0x_api.cpp \
	Adafruit_VL6180X.cpp \
	vl53l0x_platform.cpp 
  

LIB_OBJS  = $(LIB_SRCS:%.cpp=$(OBJ_DIR)/%.o)

EXAMPLES_SRC = $(wildcard examples/*)
EXAMPLES_BIN = $(EXAMPLES_SRC:examples/%.cpp=$(OUTPUT_DIR)/%)


.PHONY: all
all: ${TARGET_LIB}

$(TARGET_LIB): $(LIB_OBJS)
	mkdir -p $(dir $@)
	$(AR) -rcs $@ $^

$(OBJ_DIR)/%.o:%.cpp
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@

$(EXAMPLES_BIN): bin/%:examples/%.cpp
	mkdir -p $(dir $@)
	$(CC) -L$(OUTPUT_DIR) $^ -lVL53L0X_Rasp  $(INCLUDES) -o $@

examples:${OUTPUT_DIR} ${TARGET_LIB} $(EXAMPLES_BIN)

.PHONY: clean
clean:
	-${RM} -rf ./$(OUTPUT_DIR)/*  ./$(OBJ_DIR)/*


