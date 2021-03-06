CC = g++
CPPFLAG = -Wall -g -w -fPIC -DWITH_NONAMESPACES -fno-use-cxa-atexit -fexceptions -DWITH_DOM  -DWITH_OPENSSL -std=c++0x -static-libstdc++ `pkg-config --cflags opencv`

BASE_DIR=.
SOURCE=$(BASE_DIR)

INCLUDE +=-I$(SOURCE)/include -I$(BASE_DIR) -I/usr/include/x86_64-linux-gnu
LIB= -lssl -lcrypto -lpthread -lncurses `pkg-config --libs opencv`
PROXYSOURCE=$(BASE_DIR)/proxycpp
ProxyOBJ=$(PROXYSOURCE)/soapDeviceBindingProxy.o $(PROXYSOURCE)/soapMediaBindingProxy.o $(PROXYSOURCE)/soapPTZBindingProxy.o \
		 $(PROXYSOURCE)/soapPullPointSubscriptionBindingProxy.o $(PROXYSOURCE)/soapRemoteDiscoveryBindingProxy.o
PluginSOURCE=$(BASE_DIR)/plugin
PluginOBJ=$(PluginSOURCE)/wsaapi.o $(PluginSOURCE)/wsseapi.o $(PluginSOURCE)/threads.o $(PluginSOURCE)/duration.o \
		  $(PluginSOURCE)/smdevp.o $(PluginSOURCE)/mecevp.o $(PluginSOURCE)/dom.o
SRC= $(SOURCE)/stdsoap2.o  $(SOURCE)/soapC.o $(SOURCE)/soapClient.o $(SOURCE)/Media.o $(SOURCE)/CMT.o $(SOURCE)/common.o \
		  $(SOURCE)/Consensus.o $(SOURCE)/fastcluster.o $(SOURCE)/Fusion.o $(SOURCE)/Matcher.o $(SOURCE)/Tracker.o  $(SOURCE)/main.o $(PluginOBJ) $(ProxyOBJ)
OBJECTS = $(patsubst %.cpp,%.o,$(SRC))
TARGET=ipconvif
all: $(TARGET) 
$(TARGET):$(OBJECTS) 
	$(CC) $(CPPFLAG) $(OBJECTS)  $(INCLUDE)  $(LIB) -o $(TARGET)
$(OBJECTS):%.o : %.cpp
	$(CC) -c $(CPPFLAG) $(INCLUDE) $< -o $@
clean:
	rm -rf  $(OBJECTS) 

