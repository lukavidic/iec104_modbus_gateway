/**
 * @file modbus_master.h
 * 
 * @brief This file contains declarations of types and functions used to
 * implement a specific modbus RTU master
 */

#ifndef _MODBUS_MASTER_SPECIFIC_H_
#define _MODBUS_MASTER_SPECIFIC_H_

#include <stdint.h>
#include <stdbool.h>
#include <jansson.h>
#include <modbus/modbus.h>

#define MAX_SLAVE_NAME_LEN 64
#define RESPONSE_TIMEOUT 100000
#define MAX_SERIAL_PORTS 6
#define OFFSET_BY_PORT 1000

#define COIL_ON_VALUE  1
#define COIL_OFF_VALUE 0

#define SLAVE_INTERROGATION_CMD    0
#define READ_COIL_CMD              1
#define READ_DISCRETE_INPUT_CMD    2
#define READ_INPUT_REGISTER_CMD    3
#define READ_HOLDING_REGISTER_CMD  4
#define WRITE_COIL_CMD             5
#define WRITE_HOLDING_REGISTER_CMD 6

/**
 * @brief Structure that represents a simple modbus slave used to parse json config file
 */
typedef struct simple_slave 
{
    uint16_t id;
    char name[MAX_SLAVE_NAME_LEN];
    uint8_t num_of_coils;
    uint8_t* coils_addr;
    uint8_t num_of_discrete_inputs;
    uint8_t* discrete_inputs_addr;
    uint8_t num_of_input_registers;
    uint8_t* input_registers_addr;
    uint8_t num_of_holding_registers;
    uint8_t* holding_registers_addr;
} simple_slave_t;

/**
 * @brief Structure that holds interrogation response of certain slave
 */
typedef struct interrogation_response
{
    uint8_t* coils;
    uint8_t num_of_coils;
    uint8_t* discrete_inputs;
    uint8_t num_of_discrete_inputs;
    uint16_t* input_regs;
    uint8_t num_of_input_registers;
    uint16_t* holding_regs;
    uint8_t num_of_holding_registers;
} interrogation_response_t;

/**
 * @brief Structure used to represent a command that modbus master should execute
 * 
 * @details This structure is created in order to simplify usage of modbus master API
 * in a way that all read/write commands can be executed according to the values 
 * inserted in an instance of this structure. Some fields in this structure are not needed 
 * for certain requests (eg. field target_value is not needed in read requests). 
 * In this case, these fields will be ignored, so their values are not a must to set. 
 * The command_type field should be initialized with one of the defined constants at the 
 * beginning this header file.
 */
typedef struct master_command
{
    uint8_t command_type;
    uint8_t target_addr;
    uint8_t target_value;
    uint16_t slave_id;
} master_command_t;

/**
 * @brief Function that parses slave configuration of json config file for slave memory layout
 * 
 * @param json_array JSON object that points to the array of existing coils/registers addresses 
 * @param count Reference to the variable that holds number of elements
 * 
 * @returns Dynamically allocated array of existing slave coils/registers addresses or NULL if failure
 */
uint8_t* parse_address_array(json_t* json_array, uint8_t* count);

/**
 * @brief Function that parses the json config file and searches for slave devices configuration
 * 
 * @param root JSON object representing an opened json config file
 * @param num_of_slaves References to the variables holding the count of created slave objects
 * 
 * @returns Dynamically allocated arrays of parsed slave device objects or NULL if failure
 */
simple_slave_t** parse_slaves(json_t* root, uint8_t* num_of_slaves);

/**
 * @brief Function that releases the memory allocated for slave device objects
 * 
 * @param slaves The array of existing slaves
 * @param num_of_slaves Number of allocated slave objects
 */
void free_slaves(simple_slave_t** slaves, uint8_t* num_of_slaves);

/**
 * @brief Function that releases memory used by modbus context objects and modbus connections
 * 
 * @param ctx Modbus context object used for releasing
 */
void free_modbus(modbus_t** ctx);

/**
 * @brief Function that releases memory used to store interrogation response data
 * 
 * @param resp - Response structure for releasing
 */
void free_interrogation_response(interrogation_response_t* resp);

/**
 * @brief Function that initializes arrays of active slave devices derived from
 * configuration file in json format. Each array is formed according to the serial
 * port configuration corresponding to that array index (see config file).
 * 
 * @param cfg_file Path to the json configuration file
 * @param num_of_slaves Array of variables holding the count of created slave objects
 * 
 * @returns Dynamically allocated arrays of parsed slave device objects or NULL if failure
 */
simple_slave_t** init_slaves(const char* cfg_file, uint8_t* num_of_slaves);

/**
 * @brief Function that initializes modbus connection and creates a modbus context object
 * 
 * @param dev_path Path to the serial line used for communication (/dev/ttyS*)
 * @param baud Baud rate of the serial communication
 * @param parity Serial communication parity, valid values are 'N' (NONE), 'O' (ODD) or 'E' (EVEN)
 * @param data_bits Number of data bits used for the serial communication
 * @param stop_bits Number of stop bits used for the serial communication
 * 
 * @returns Pointer to the initialized modbus context object or NULL if failure
 */
modbus_t* init_modbus_connection(const char* dev_path, uint32_t baud, uint8_t parity, uint8_t data_bits, uint8_t stop_bits);

/**
 * @brief Function to print the information about slave devices found in config file
 * 
 * @param slaves The arrays of existing slaves
 * @param num_of_slaves Number of allocated slave objects for each array
 */
void print_slaves(simple_slave_t** slaves, uint8_t* num_of_slaves);

/**
 * @brief Function that retreives index of a slave with the given ID from the slave array
 * 
 * @param slave_id Address (id) of the slave device
 * @param slaves The array of existing slaves
 * @param num_of_slaves Number of allocated slave objects
 * 
 * @return Index of slave from slave array that matches with given ID or num_of_slaves 
 * value if no matching slave is found 
 */
uint8_t get_slave_idx(uint16_t slave_id, simple_slave_t* slaves, uint8_t num_of_slaves);

/**
 * @brief Function that gathers data of all coils, inputs and registers of the specified slave
 * 
 * @param slave_id Address (id) of the slave device
 * @param slaves The array of existing slaves
 * @param num_of_slaves Number of allocated slave objects
 * @param ctx Initialized modbus context
 * 
 * @returns Dynamically allocated and filled interrogation response structure or NULL if failure
 */
interrogation_response_t* interrogate_slave(uint16_t slave_id, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx);

/**
 * @brief Function that reads status of one coil
 * 
 * @param slave_id Address (id) of the slave device
 * @param coil_addr Address of the coil to be read
 * @param slaves The array of existing slaves
 * @param num_of_slaves Number of allocated slave objects
 * @param ctx Initialized modbus context
 * 
 * @returns Dynamically allocated variable holding the status of a coil or NULL if failure
 */
uint8_t* read_coil(uint16_t slave_id, uint8_t coil_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx);

/**
 * @brief Function that reads status of one discrete input
 * 
 * @param slave_id Address (id) of the slave device
 * @param discrete_input_addr Address of the discrete input to be read
 * @param slaves The array of existing slaves
 * @param num_of_slaves Number of allocated slave objects
 * @param ctx Initialized modbus context
 * 
 * @returns Dynamically allocated variable holding the status of a discrete input or NULL if failure
 */
uint8_t* read_discrete_input(uint16_t slave_id, uint8_t discrete_input_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx);

/**
 * @brief Function that reads value from one input register
 * 
 * @param slave_id Address (id) of the slave device
 * @param input_reg_addr Address of the input register to be read
 * @param slaves The array of existing slaves
 * @param num_of_slaves Number of allocated slave objects
 * @param ctx Initialized modbus context
 * 
 * @returns Dynamically allocated variable holding the value of an input register or NULL if failure
 */
uint16_t* read_input_register(uint16_t slave_id, uint8_t input_reg_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx);

/**
 * @brief Function that reads value from one holding register
 * 
 * @param slave_id Address (id) of the slave device
 * @param holding_reg_addr Address of the holding register to be read
 * @param slaves The array of existing slaves
 * @param num_of_slaves Number of allocated slave objects
 * @param ctx Initialized modbus context
 * 
 * @returns Dynamically allocated variable holding the value of a holding register or NULL if failure
 */
uint16_t* read_holding_register(uint16_t slave_id, uint8_t holding_reg_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx);

/**
 * @brief Function that sets state of one coil
 * 
 * @param slave_id Address (id) of the slave device
 * @param coil_addr Address of the coil to set
 * @param coil_value Value to be written, valid values are defined in the header as COIL_ON_VALUE and COIL_OFF_VALUE
 * @param slaves The array of existing slaves
 * @param num_of_slaves Number of allocated slave objects
 * @param ctx Initialized modbus context
 * 
 * @returns 1 on succes, 0 on failure
 */
uint8_t write_coil(uint16_t slave_id, uint8_t coil_addr, uint8_t coil_value, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx);

/**
 * @brief Function that writes value to one holding register
 * 
 * @param slave_id Address (id) of the slave device
 * @param coil_addr Address of the holding register to be written
 * @param coil_value Value to be written, valid values are defined by the range of uint16_t type
 * @param slaves The array of existing slaves
 * @param num_of_slaves Number of allocated slave objects
 * @param ctx Initialized modbus context
 * 
 * @returns 1 on succes, 0 on failure
 */
uint8_t write_holding_register(uint16_t slave_id, uint8_t holding_reg_addr, uint16_t holding_reg_value, simple_slave_t* slaves, 
                               uint8_t num_of_slaves, modbus_t* ctx);



#endif
/* end of file */