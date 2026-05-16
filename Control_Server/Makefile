# Makefile

# Compiler options
CC = gcc
CFLAGS = -Wall -Wextra -O1

ROVERLIBPATH ?= /home/ubuntu/SIMBA_C_Driver/rover
ROVERLIBFLAG = -lrover

ARMLIBPATH ?= /home/ubuntu/kinematic-arm
ARMLIBFLAG = -larm

LDFLAGS = $(ROVERLIBFLAG) $(ARMLIBFLAG) -lm

# Source files
SRCS = $(wildcard *.c)
HEADERS = $(wildcard *.h)
BUILD_DIR = build

# Object files
OBJS = $(SRCS:%.c=$(BUILD_DIR)/%.o)

# Executable file
TARGET = server

.PHONY: $(ARMLIBPATH)/lib/libarm.a build_libraries cleanbuild clean_libraries clean

# Targets
all: build_libraries $(TARGET)

$(TARGET): $(OBJS) $(ROVERLIBPATH)/librover.a $(ARMLIBPATH)/lib/libarm.a
	$(CC) $(CFLAGS) -L $(ROVERLIBPATH) -L $(ARMLIBPATH)/lib $(OBJS) -o $(TARGET) $(LDFLAGS)

build_libraries: 
	$(MAKE) -C $(ROVERLIBPATH)
	$(MAKE) -C $(ARMLIBPATH)

# Remake the library if needed

$(BUILD_DIR)/%.o: %.c $(HEADERS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I $(ROVERLIBPATH) -I $(ARMLIBPATH)/include -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

cleanbuild:
	rm -rf build

clean_libraries:
	@echo Cleaning arm lib
	$(MAKE) clean -C $(ARMLIBPATH)
	@echo ""
	@echo Cleaning rover lib
	$(MAKE) clean -C $(ROVERLIBPATH)
	@echo ""

clean: cleanbuild clean_libraries
	rm -f $(TARGET)
