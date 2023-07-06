
CXX := /home/owner/.myenv/gcc-linaro-5.5.0-2017.10-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++
CXX += -fdiagnostics-color=always

CFLAGS := -D_PLATINUM -O0 -c -Wall -fmessage-length=0 -std=c++11 -g -Wno-deprecated -Wno-write-strings 
LDFLAGS := -Wl,-Bstatic -lMMC_APP_LIB -lMMCPPLIB -Wl,-Bdynamic -lpthread -lrt
CMAKE_CURRENT_SOURCE_DIR:= .

INCLUDE_DIRS := \
	 -I$(CMAKE_CURRENT_SOURCE_DIR)/GMAS-elmo-lib/includes \
	 -I$(CMAKE_CURRENT_SOURCE_DIR)/GMAS-elmo-lib/includes/CPP \
	 -I$(CMAKE_CURRENT_SOURCE_DIR)/GMAS-elmo-lib/includes/EMBL \
	 -I$(CMAKE_CURRENT_SOURCE_DIR)/src \
	 -I$(CMAKE_CURRENT_SOURCE_DIR)/external/spdlog/include
LINK_DIRS := \
	 -L$(CMAKE_CURRENT_SOURCE_DIR)/GMAS-elmo-lib/lib/platinum

SRC_DIR := src
OUTPUT_DIR := build
OBJ_DIR := $(OUTPUT_DIR)/obj

SRCS := $(wildcard $(SRC_DIR)/*.cpp)
PROJECTS := $(wildcard ./*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))
EXES := $(patsubst ./%.cpp,$(OUTPUT_DIR)/%.pexe,$(PROJECTS))

# .SILENT:

all: $(EXES)

show: 
	@echo $(SRCS)
	@echo $(PROJECTS)
	@echo $(OBJS)
	@echo $(EXES)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp Makefile
	@echo "build : $< .. $@  ,, $^"
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CFLAGS) $(INCLUDE_DIRS) -c -o $@ $<

$(OBJ_DIR)/%.o: ./%.cpp Makefile
	@echo "build : $< .. $@  ,, $^"
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CFLAGS) $(INCLUDE_DIRS) -c -o $@ $<

$(OUTPUT_DIR)/%.pexe: $(OBJ_DIR)/%.o $(OBJS)
	@echo "build : $< .. $@  ,, $^"
	$(CXX) -o$@ $^ $(LINK_DIRS) $(LDFLAGS) 

.PHONY: write
write: $(EXES)
	scp $(OUTPUT_DIR)/*.pexe user@server:/path/to/destination

clean:
	-rm -f *.o


