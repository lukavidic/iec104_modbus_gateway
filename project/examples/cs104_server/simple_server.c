#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>

#include "cs104_slave.h"
#include "modbus_master.h"

#include "hal_thread.h"
#include "hal_time.h"

/**
 * Project specific constans
 */
#define MAX_STATIONS 6

#define COIL_ADDRESS_START                  1
#define COIL_ADDRESS_END                10000

#define DISCRETE_INPUT_ADDRESS_START    10001
#define DISCRETE_INPUT_ADDRESS_END      20000

#define INPUT_REGISTER_ADDRESS_START    30001
#define INPUT_REGISTER_ADDRESS_END      40000

#define HOLDING_REGISTER_ADDRESS_START  40001
#define HOLDING_REGISTER_ADDRESS_END    50000

const char* DEVICE_PATHS[SERIAL_PORTS_NUM] = {"/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS6", "/dev/ttyS8"};
const char* CONFIG_FILE_PATH = "config.json";

/**
 * Structure used to hold variables needed for modbus communication.
 * Created in order to enable sending the parameter to IEC104 function handlers.
 */
typedef struct modbus_communication_param
{
    uint8_t num_of_slaves[SERIAL_PORTS_NUM];
    simple_slave_t** slaves;
    modbus_t* ctx[SERIAL_PORTS_NUM];
} modbus_communication_param_t;

static bool running = true;

void sendAllSinglePoints(IMasterConnection connection, interrogation_response_t* resp, simple_slave_t* slave)
{
    CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);
    CS101_ASDU newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION, 0, slave->id, false, false);

    for(uint8_t i = 0; i < resp->num_of_coils; i++)
    {
        InformationObject io = (InformationObject) SinglePointInformation_create(NULL, COIL_ADDRESS_START + slave->coils_addr[i], 
            resp->coils[i], IEC60870_QUALITY_GOOD);
        CS101_ASDU_addInformationObject(newAsdu, io);
        InformationObject_destroy(io);
    }

    for(uint8_t i = 0; i < resp->num_of_discrete_inputs; i++)
    {
        InformationObject io = (InformationObject) SinglePointInformation_create(NULL, DISCRETE_INPUT_ADDRESS_START + slave->discrete_inputs_addr[i], 
            resp->discrete_inputs[i], IEC60870_QUALITY_GOOD);
        CS101_ASDU_addInformationObject(newAsdu, io);
        InformationObject_destroy(io);
    }

    IMasterConnection_sendASDU(connection, newAsdu);
    CS101_ASDU_destroy(newAsdu);
}

void sendAllScaledValues(IMasterConnection connection, interrogation_response_t* resp, simple_slave_t* slave)
{
    CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);
    CS101_ASDU newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION, 0, slave->id, false, false);

    // Create and send input and holding registers for testing

    for(int i = 0; i < resp->num_of_input_registers; i++)
    {
        InformationObject io = (InformationObject) MeasuredValueScaled_create(NULL, INPUT_REGISTER_ADDRESS_START + slave->input_registers_addr[i], 
            resp->input_regs[i], IEC60870_QUALITY_GOOD);
        CS101_ASDU_addInformationObject(newAsdu, io);
        InformationObject_destroy(io);
    }

    for(int i = 0; i < resp->num_of_holding_registers; i++)
    {
        InformationObject io = (InformationObject) MeasuredValueScaled_create(NULL, HOLDING_REGISTER_ADDRESS_START + slave->holding_registers_addr[i], 
            resp->holding_regs[i], IEC60870_QUALITY_GOOD);
        CS101_ASDU_addInformationObject(newAsdu, io);
        InformationObject_destroy(io);
    }

    IMasterConnection_sendASDU(connection, newAsdu);
    CS101_ASDU_destroy(newAsdu);
}

void
sigint_handler(int signalId)
{
    running = false;
}

void
printCP56Time2a(CP56Time2a time)
{
    printf("%02i:%02i:%02i %02i/%02i/%04i\n", 
                            CP56Time2a_getHour(time),
                            CP56Time2a_getMinute(time),
                            CP56Time2a_getSecond(time),
                            CP56Time2a_getDayOfMonth(time),
                            CP56Time2a_getMonth(time),
                            CP56Time2a_getYear(time) + 2000);
}

/* Callback handler to log sent or received messages (optional) */
static void
rawMessageHandler(void* parameter, IMasterConnection conneciton, uint8_t* msg, int msgSize, bool sent)
{
    if (sent)
        printf("SEND: ");
    else
        printf("RCVD: ");

    int i;
    for (i = 0; i < msgSize; i++) {
        printf("%02x ", msg[i]);
    }

    printf("\n");
}

static bool
clockSyncHandler (void* parameter, IMasterConnection connection, CS101_ASDU asdu, CP56Time2a newTime)
{
    printf("Process time sync command with time "); printCP56Time2a(newTime); printf("\n");

    uint64_t newSystemTimeInMs = CP56Time2a_toMsTimestamp(newTime);

    /* Set time for ACT_CON message */
    CP56Time2a_setFromMsTimestamp(newTime, Hal_getTimeInMs());

    /* update system time here */

    return true;
}

static bool
interrogationHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu, uint8_t qoi)
{
    modbus_communication_param_t* mb_param = (modbus_communication_param_t*) (parameter);
    interrogation_response_t* resp = NULL;
    uint8_t idx = 0;
    uint8_t slave_idx = 0;
    uint16_t slave_id = 0;

    printf("Received interrogation for group %i\n", qoi);

    if (qoi == 20) 
    { /* only handle station interrogation */

        CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);

        slave_id = (uint16_t) CS101_ASDU_getCA(asdu);
        idx = slave_id / OFFSET_BY_PORT - 1;
        if(idx < 0 || idx >= SERIAL_PORTS_NUM)
        {
            fprintf(stderr, "Invalid slave ID: %u, index out of bounds.\n", slave_id);
            IMasterConnection_sendACT_CON(connection, asdu, true);
            return true;    
        }

        resp = interrogate_slave(slave_id, mb_param->slaves[idx], mb_param->num_of_slaves[idx], mb_param->ctx[idx]);
        if(resp == NULL)
        {
            fprintf(stderr, "Failed to get interrogation response for slave: %u.\n", slave_id);
            IMasterConnection_sendACT_CON(connection, asdu, true);
            return true;
        } 

        IMasterConnection_sendACT_CON(connection, asdu, false);

        slave_idx = get_slave_idx(slave_id, mb_param->slaves[idx], mb_param->num_of_slaves[idx]);

        /* The CS101 specification only allows information objects without timestamp in GI responses */
        sendAllSinglePoints(connection, resp, &mb_param->slaves[idx][slave_idx]);
        sendAllScaledValues(connection, resp, &mb_param->slaves[idx][slave_idx]);
        
        IMasterConnection_sendACT_TERM(connection, asdu);
    }
    else 
    {
        IMasterConnection_sendACT_CON(connection, asdu, true);
    }
    return true;
}

static bool
readHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu, int ioa)
{
    if(CS101_ASDU_getCOT(asdu) == CS101_COT_REQUEST)
    {
        CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);
        int ca = CS101_ASDU_getCA(asdu);
		CS101_ASDU newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_REQUEST, 0, ca, false, false);
        InformationObject io = NULL;
        uint8_t* state_value = NULL;
        uint16_t* reg_value = NULL;
        modbus_communication_param_t* mb_param = (modbus_communication_param_t*) (parameter);
        uint8_t idx = ca / OFFSET_BY_PORT - 1;
        if(idx < 0 || idx >= SERIAL_PORTS_NUM)
        {
            fprintf(stderr, "Invalid slave ID, index out of bounds.\n");
            ioa = -1; 
        }
        /* 
            IMPORTANT: Might need to further divide address space of MODBUS to fit all of the
            IEC104 information elements ! 
        */
        if(ioa >= COIL_ADDRESS_START && ioa <= COIL_ADDRESS_END)
        {
            state_value = read_coil((uint16_t) ca, (uint8_t) (ioa - COIL_ADDRESS_START), mb_param->slaves[idx], mb_param->num_of_slaves[idx], mb_param->ctx[idx]);
            if(state_value == NULL)
            {
                io = NULL;
                fprintf(stderr, "Failed to read coil status, address: %i.\n", ioa);
            }
            else
            {
                io = (InformationObject) SinglePointInformation_create(NULL, ioa, *state_value, IEC60870_QUALITY_GOOD);
                printf("Reading state of the coil, address: %i\n", ioa);
            }
        }
        else if(ioa >= DISCRETE_INPUT_ADDRESS_START && ioa <= DISCRETE_INPUT_ADDRESS_END)
        {
            state_value = read_discrete_input((uint16_t) ca, (uint8_t) (ioa- DISCRETE_INPUT_ADDRESS_START), mb_param->slaves[idx], 
                mb_param->num_of_slaves[idx], mb_param->ctx[idx]);
            if(state_value == NULL)
            {
                io = NULL;
                fprintf(stderr, "Failed to read discrete input status, address: %i.\n", ioa);
            }
            else
            {
                io = (InformationObject) SinglePointInformation_create(NULL, ioa, *state_value, IEC60870_QUALITY_GOOD);
                printf("Reading state of the discrete input, address: %i\n", ioa);
            }
        }
        else if(ioa >= INPUT_REGISTER_ADDRESS_START && ioa <= INPUT_REGISTER_ADDRESS_END)
        {
            reg_value = read_input_register((uint16_t) ca, (uint8_t) (ioa - INPUT_REGISTER_ADDRESS_START), mb_param->slaves[idx], 
                mb_param->num_of_slaves[idx], mb_param->ctx[idx]);
            if(reg_value == NULL)
            {
                io = NULL;
                fprintf(stderr, "Failed to read input register value, address: %i.\n", ioa);
            }
            else
            {
                io = (InformationObject) MeasuredValueScaled_create(NULL, ioa, *reg_value, IEC60870_QUALITY_GOOD);
                printf("Reading value of the input register, address: %i", ioa);
            }
        }
        else if(ioa >= HOLDING_REGISTER_ADDRESS_START && ioa <= HOLDING_REGISTER_ADDRESS_END)
        {
            reg_value = read_holding_register((uint16_t) ca, (uint8_t) (ioa - HOLDING_REGISTER_ADDRESS_START), mb_param->slaves[idx], 
                mb_param->num_of_slaves[idx], mb_param->ctx[idx]);
            if(reg_value == NULL)
            {
                io = NULL;
                fprintf(stderr, "Failed to read holding register value, address: %i.\n", ioa);
            }
            else
            {
                io = (InformationObject) MeasuredValueScaled_create(NULL, ioa, *reg_value, IEC60870_QUALITY_GOOD);
                printf("Reading value of the holding register, address: %i", ioa);
            }
        }
        else
        {
            io = NULL;
        }
        if(io != NULL)
        {
            CS101_ASDU_addInformationObject(newAsdu, io);
            InformationObject_destroy(io);
            IMasterConnection_sendASDU(connection, newAsdu);
        }
        else
        {
            CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA);
            CS101_ASDU_setNegative(asdu, true);
            IMasterConnection_sendASDU(connection, asdu);
        }
        CS101_ASDU_destroy(newAsdu);
    }
    else
    {
        CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_COT);
        CS101_ASDU_setNegative(asdu, true);
        IMasterConnection_sendASDU(connection, asdu);
    }
    return true;
}

static bool
asduHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu)
{
    modbus_communication_param_t* mb_param = (modbus_communication_param_t*) (parameter);
    uint16_t slave_id = (uint16_t) CS101_ASDU_getCA(asdu);
    uint8_t idx = slave_id / OFFSET_BY_PORT - 1;
    uint8_t target_address = 0;
    uint16_t target_value = 0;

    if(idx < 0 || idx >= SERIAL_PORTS_NUM)
    {
        fprintf(stderr, "Invalid slave ID, index out of bounds.\n");
        CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_CA);
        CS101_ASDU_setNegative(asdu, true);
        IMasterConnection_sendASDU(connection, asdu);
        return true;    
    }

    /* For now implement only responses to single command and set point scaled value command */
    if(CS101_ASDU_getTypeID(asdu) == C_SC_NA_1) 
    {
        printf("Received single command\n");

        if(CS101_ASDU_getCOT(asdu) == CS101_COT_ACTIVATION) 
        {
            InformationObject io = CS101_ASDU_getElement(asdu, 0);
            if(io) 
            {
                if(InformationObject_getObjectAddress(io) >= COIL_ADDRESS_START && InformationObject_getObjectAddress(io) <= COIL_ADDRESS_END) 
                {
                    SingleCommand sc = (SingleCommand) io;
                    target_address = InformationObject_getObjectAddress(io) - COIL_ADDRESS_START;
                    target_value = SingleCommand_getState(sc) == 0 ? COIL_OFF_VALUE : COIL_ON_VALUE;
                    if(write_coil(slave_id, target_address, target_value, mb_param->slaves[idx], mb_param->num_of_slaves[idx], mb_param->ctx[idx]))
                    {
                        printf("IOA: %i switch to %i\n", InformationObject_getObjectAddress(io), SingleCommand_getState(sc));
                        CS101_ASDU_setCOT(asdu, CS101_COT_ACTIVATION_CON);
                    }
                    else
                    {
                        fprintf(stderr, "Failed to set coil status, address: %i.\n", InformationObject_getObjectAddress(io));
                        CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA); 
                        CS101_ASDU_setNegative(asdu, true);
                    }
                }
                else
                {
                    CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA);
                    CS101_ASDU_setNegative(asdu, true);
                }
                InformationObject_destroy(io);
            }
            else 
            {
                printf("ERROR: message has no valid information object\n");
                return true;
            }
        }
        else
        {
            CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_COT);
            CS101_ASDU_setNegative(asdu, true);
        }
        IMasterConnection_sendASDU(connection, asdu);

        return true;
    }
    if(CS101_ASDU_getTypeID(asdu) == C_SC_TA_1)
    {
        printf("Received single command with a time tag\n");

        if(CS101_ASDU_getCOT(asdu) == CS101_COT_ACTIVATION)
        {
            InformationObject io = CS101_ASDU_getElement(asdu, 0);
            if(io)
            {
                if(InformationObject_getObjectAddress(io) >= COIL_ADDRESS_START && InformationObject_getObjectAddress(io) <= COIL_ADDRESS_END) 
                {
                    SingleCommandWithCP56Time2a sc = (SingleCommandWithCP56Time2a) io;
                    target_address = InformationObject_getObjectAddress(io) - COIL_ADDRESS_START;
                    target_value = SingleCommand_getState((SingleCommand) sc) == 0 ? COIL_OFF_VALUE : COIL_ON_VALUE;
                    if(write_coil(slave_id, target_address, target_value, mb_param->slaves[idx], mb_param->num_of_slaves[idx], mb_param->ctx[idx]))
                    {
                        printf("IOA: %i switch to %i\n", InformationObject_getObjectAddress(io), SingleCommand_getState((SingleCommand)sc));
                        printf("Timestamp info: ");
                        printCP56Time2a(SingleCommandWithCP56Time2a_getTimestamp(sc));
                        CS101_ASDU_setCOT(asdu, CS101_COT_ACTIVATION_CON);
                    }
                    else
                    {
                        fprintf(stderr, "Failed to set coil status, address: %i.\n", InformationObject_getObjectAddress(io));
                        CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA); 
                        CS101_ASDU_setNegative(asdu, true);
                    }
                }
                else
                {
                    CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA);
                    CS101_ASDU_setNegative(asdu, true);
                }
                InformationObject_destroy(io);
            }
            else
            {
                printf("ERROR: message has no valid information object\n");
                return true;
            }
        }
        else
        {
            CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_COT);
            CS101_ASDU_setNegative(asdu, true);
        }

        IMasterConnection_sendASDU(connection, asdu);

        return true;
    }
    if(CS101_ASDU_getTypeID(asdu) == C_SE_NB_1)
    {
        printf("Received set-point, scaled value command\n");

        if(CS101_ASDU_getCOT(asdu) == CS101_COT_ACTIVATION) 
        {
            InformationObject io = CS101_ASDU_getElement(asdu, 0);
            if(io) 
            {
                if(InformationObject_getObjectAddress(io) >= HOLDING_REGISTER_ADDRESS_START && 
                   InformationObject_getObjectAddress(io) <= HOLDING_REGISTER_ADDRESS_END) 
                {
                    SetpointCommandScaled spsc = (SetpointCommandScaled) io;
                    target_address = InformationObject_getObjectAddress(io) - HOLDING_REGISTER_ADDRESS_START;
                    target_value = SetpointCommandScaled_getValue(spsc);
                    if(write_holding_register(slave_id, target_address, target_value, mb_param->slaves[idx], mb_param->num_of_slaves[idx], mb_param->ctx[idx]))
                    {
                        printf("IOA: %i set to %i\n", InformationObject_getObjectAddress(io), SetpointCommandScaled_getValue(spsc));
                        CS101_ASDU_setCOT(asdu, CS101_COT_ACTIVATION_CON);
                    }
                    else
                    {
                        fprintf(stderr, "Failed to set holding register value, address: %i.\n", InformationObject_getObjectAddress(io));
                        CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA);  
                        CS101_ASDU_setNegative(asdu, true);
                    }
                }
                else
                {
                    CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA);
                    CS101_ASDU_setNegative(asdu, true);
                }
                InformationObject_destroy(io);
            }
            else 
            {
                printf("ERROR: message has no valid information object\n");
                return true;
            }
        }
        else
        {
            CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_COT);
            CS101_ASDU_setNegative(asdu, true);
        }
        IMasterConnection_sendASDU(connection, asdu);

        return true;
    }
    if(CS101_ASDU_getTypeID(asdu) == C_SE_TB_1)
    {
        printf("Received set-point, scaled value command with a time tag\n");

        if(CS101_ASDU_getCOT(asdu) == CS101_COT_ACTIVATION) 
        {
            InformationObject io = CS101_ASDU_getElement(asdu, 0);
            if(io) 
            {
                if(InformationObject_getObjectAddress(io) >= HOLDING_REGISTER_ADDRESS_START && 
                   InformationObject_getObjectAddress(io) <= HOLDING_REGISTER_ADDRESS_END) 
                {
                    SetpointCommandScaledWithCP56Time2a spsc = (SetpointCommandScaledWithCP56Time2a) io;
                    target_address = InformationObject_getObjectAddress(io) - HOLDING_REGISTER_ADDRESS_START;
                    target_value = SetpointCommandScaled_getValue((SetpointCommandScaled) spsc);
                    if(write_holding_register(slave_id, target_address, target_value, mb_param->slaves[idx], mb_param->num_of_slaves[idx], mb_param->ctx[idx]))
                    {
                        printf("IOA: %i set to %i\n", InformationObject_getObjectAddress(io), SetpointCommandScaled_getValue((SetpointCommandScaled) spsc));
                        printf("Timestamp info: ");
                        printCP56Time2a(SetpointCommandScaledWithCP56Time2a_getTimestamp(spsc));
                        CS101_ASDU_setCOT(asdu, CS101_COT_ACTIVATION_CON);
                    }
                    else
                    {
                        fprintf(stderr, "Failed to set holding register value, address: %i.\n", InformationObject_getObjectAddress(io));
                        CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA);  
                        CS101_ASDU_setNegative(asdu, true);
                    }
                }
                else
                {
                    CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA);
                    CS101_ASDU_setNegative(asdu, true);
                }
                InformationObject_destroy(io);
            }
            else 
            {
                printf("ERROR: message has no valid information object\n");
                return true;
            }
        }
        else
        {
            CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_COT);
            CS101_ASDU_setNegative(asdu, true);
        }
        IMasterConnection_sendASDU(connection, asdu);

        return true;
    }
    return false;
}

static bool
connectionRequestHandler(void* parameter, const char* ipAddress)
{
    printf("New connection request from %s\n", ipAddress);

#if 0
    if (strcmp(ipAddress, "127.0.0.1") == 0) {
        printf("Accept connection\n");
        return true;
    }
    else {
        printf("Deny connection\n");
        return false;
    }
#else
    return true;
#endif
}

static void
connectionEventHandler(void* parameter, IMasterConnection con, CS104_PeerConnectionEvent event)
{
    if (event == CS104_CON_EVENT_CONNECTION_OPENED) {
        printf("Connection opened (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_CONNECTION_CLOSED) {
        printf("Connection closed (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_ACTIVATED) {
        printf("Connection activated (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_DEACTIVATED) {
        printf("Connection deactivated (%p)\n", con);
    }
}



int
main(int argc, char** argv)
{
    /* Prepare variables for modbus master initialization */
    int rc = 0;
    serial_configuration_t cfg[SERIAL_PORTS_NUM];
    modbus_communication_param_t mb_comm_param;

    /* Add Ctrl-C handler */
    signal(SIGINT, sigint_handler);

    /* Initialize modbus slaves and connections */
    mb_comm_param.slaves = init_slaves(CONFIG_FILE_PATH, mb_comm_param.num_of_slaves, cfg);
    if(mb_comm_param.slaves == NULL)
    {
        fprintf(stderr, "Unable to get slave devices configuration.\n");
        return 0;
    }

    for(uint8_t i = 0; i < SERIAL_PORTS_NUM; i++)
    {
        if(mb_comm_param.slaves[i] != NULL)
        {
            mb_comm_param.ctx[i] = init_modbus_connection(DEVICE_PATHS[i], cfg[i].baud_rate, cfg[i].parity, cfg[i].data_bits, cfg[i].stop_bits);
        }
        else
        {
            mb_comm_param.ctx[i] = NULL;
        }
    }

    print_slaves(mb_comm_param.slaves, mb_comm_param.num_of_slaves);

    /* create a new slave/server instance with default connection parameters and
     * default message queue size */
    CS104_Slave slave = CS104_Slave_create(10, 10);

    CS104_Slave_setLocalAddress(slave, "0.0.0.0");

    /* Set mode to a single redundancy group
     * NOTE: library has to be compiled with CONFIG_CS104_SUPPORT_SERVER_MODE_SINGLE_REDUNDANCY_GROUP enabled (=1)
     */
    CS104_Slave_setServerMode(slave, CS104_MODE_SINGLE_REDUNDANCY_GROUP);

    /* get the connection parameters - we need them to create correct ASDUs -
     * you can also modify the parameters here when default parameters are not to be used */
    CS101_AppLayerParameters alParams = CS104_Slave_getAppLayerParameters(slave);

    /* when you have to tweak the APCI parameters (t0-t3, k, w) you can access them here */
    CS104_APCIParameters apciParams = CS104_Slave_getConnectionParameters(slave);

    printf("APCI parameters:\n");
    printf("  t0: %i\n", apciParams->t0);
    printf("  t1: %i\n", apciParams->t1);
    printf("  t2: %i\n", apciParams->t2);
    printf("  t3: %i\n", apciParams->t3);
    printf("  k: %i\n", apciParams->k);
    printf("  w: %i\n", apciParams->w);

    /* set the callback handler for the clock synchronization command */
    CS104_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);

    /* set the callback handler for the interrogation command */
    CS104_Slave_setInterrogationHandler(slave, interrogationHandler, (void*) (&mb_comm_param));

    /* set handler for other message types */
    CS104_Slave_setASDUHandler(slave, asduHandler, (void*) (&mb_comm_param));

    /* set handler to handle connection requests (optional) */
    CS104_Slave_setConnectionRequestHandler(slave, connectionRequestHandler, NULL);

    /* set handler to track connection events (optional) */
    CS104_Slave_setConnectionEventHandler(slave, connectionEventHandler, NULL);

    /* set handler for read command */
    CS104_Slave_setReadHandler(slave, readHandler, (void*) (&mb_comm_param));

    /* uncomment to log messages */
    //CS104_Slave_setRawMessageHandler(slave, rawMessageHandler, NULL);

    CS104_Slave_start(slave);

    if (CS104_Slave_isRunning(slave) == false) {
        printf("Starting server failed!\n");
        goto exit_program;
    }

    int16_t scaledValue = 0;

    while (running) {
        /*
        Thread_sleep(10000);

        CS101_ASDU newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_PERIODIC, 0, 1, false, false);

        InformationObject io = (InformationObject) MeasuredValueScaled_create(NULL, INPUT_REGISTER_ADDRESS_START, scaledValue, IEC60870_QUALITY_GOOD);

        scaledValue++;

        CS101_ASDU_addInformationObject(newAsdu, io);

        InformationObject_destroy(io);

        /* Add ASDU to slave event queue */
        /*
        CS104_Slave_enqueueASDU(slave, newAsdu);

        CS101_ASDU_destroy(newAsdu);
        */
    }

    CS104_Slave_stop(slave);

exit_program:
    CS104_Slave_destroy(slave);
    free_modbus(mb_comm_param.ctx);
    free_slaves(mb_comm_param.slaves, mb_comm_param.num_of_slaves);

    Thread_sleep(500);
}