LIB60870_HOME=../..

PROJECT_BINARY_NAME = simple_server
PROJECT_SOURCES = simple_server.c

PROJECT_SOURCES += $(LIB60870_HOME)/../modbus_master/modbus_master.c
INCLUDES = $(LIB60870_HOME)/../modbus_master
LDLIBS = -lmodbus
LDLIBS += -ljansson

include $(LIB60870_HOME)/make/target_system.mk
include $(LIB60870_HOME)/make/stack_includes.mk

all:	$(PROJECT_BINARY_NAME)

include $(LIB60870_HOME)/make/common_targets.mk


$(PROJECT_BINARY_NAME):	$(PROJECT_SOURCES) $(LIB_NAME)
	$(CC) $(CFLAGS) $(LDFLAGS) -g -o $(PROJECT_BINARY_NAME) $(PROJECT_SOURCES) $(INCLUDES) $(LIB_NAME) $(LDLIBS)

clean:
	rm -f $(PROJECT_BINARY_NAME)


