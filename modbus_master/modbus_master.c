/**
 * @file modbus_master.c
 * 
 * @brief This file contains implementation of functions used to
 * implement a specific modbus RTU master
 * 
 * @details This file implements all of the functions available from modbus_master.h 
 * API. Implementation of these API functions also offer debug messages if needed.
 * To enable output of these messages, please define PRINT_DEBUG.
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include "modbus_master.h"

#define PRINT_DEBUG

uint8_t* parse_address_array(json_t* json_array, uint8_t* count) 
{
    uint8_t size = json_array_size(json_array);
    uint8_t* addresses = (uint8_t*)malloc(size * sizeof(uint8_t));

    if (addresses == NULL) 
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to allocate memory for address array !\n");
        #endif
        return NULL;
    }

    for (uint8_t i = 0; i < size; i++) 
    {
        json_t* item = json_array_get(json_array, i);
        json_t* address_obj = json_object_get(item, "address");
        addresses[i] = (uint8_t)json_integer_value(address_obj);
    }

    *count = (uint8_t)size;
    return addresses;
}

simple_slave_t** parse_slaves(json_t* root, uint8_t* num_of_slaves)
{
    uint8_t size = 0;
    uint8_t num_of_ports = 0;
    uint8_t active = 0;
    uint8_t port_value = 0;
    simple_slave_t** slaves = NULL;
    json_t* slaves_array = NULL;
    json_t* slave_obj = NULL;
    json_t* port_obj = NULL;
    json_t* port_array = json_object_get(root, "port");

    if(json_is_array(port_array) == 0)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Invalid JSON format: 'port' is not an array\n");
        #endif
        return NULL;
    }

    num_of_ports = json_array_size(port_array);
    slaves = (simple_slave_t**) malloc(num_of_ports * sizeof(simple_slave_t*));

    for(uint8_t j = 0; j < num_of_ports; j++)
    {
        port_obj = json_array_get(port_array, j);
        active = (uint8_t)json_integer_value(json_object_get(port_obj, "active"));
        port_value = (uint8_t)json_integer_value(json_object_get(port_obj, "value"));
        //fprintf(stdout, "Port value: %u, active: %u\n", port_value, active);
        if(active)
        {
            slaves_array = json_object_get(port_obj, "slaves");
            slave_obj = NULL;
            size = 0;


            if (json_is_array(slaves_array) == 0) 
            {
                #ifdef PRINT_DEBUG
                    fprintf(stderr, "Invalid JSON format: 'slaves' is not an array\n");
                #endif
                return NULL;
            }

            size = json_array_size(slaves_array);
            slaves[j] = (simple_slave_t*) malloc(size * sizeof(simple_slave_t));
            if (slaves[j] == NULL) 
            {
                #ifdef PRINT_DEBUG
                    fprintf(stderr, "Failed to allocate memory for slave device objects.\n");
                #endif
                return NULL;
            }

            for (uint8_t i = 0; i < size; i++) 
            {
                slave_obj = json_array_get(slaves_array, i);

                // Parse ID and description
                slaves[j][i].id = port_value * OFFSET_BY_PORT + (uint8_t)json_integer_value(json_object_get(slave_obj, "id"));
                strncpy(slaves[j][i].name, json_string_value(json_object_get(slave_obj, "description")), MAX_SLAVE_NAME_LEN);

                // Parse coils addresses
                json_t* coils_array = json_object_get(slave_obj, "coils");
                slaves[j][i].coils_addr = parse_address_array(coils_array, &slaves[j][i].num_of_coils);

                // Parse discrete inputs addresses
                json_t* discrete_inputs_array = json_object_get(slave_obj, "discrete_inputs");
                slaves[j][i].discrete_inputs_addr = parse_address_array(discrete_inputs_array, &slaves[j][i].num_of_discrete_inputs);

                // Parse input registers addresses
                json_t* input_registers_array = json_object_get(slave_obj, "input_registers");
                slaves[j][i].input_registers_addr = parse_address_array(input_registers_array, &slaves[j][i].num_of_input_registers);

                // Parse holding registers addresses
                json_t* holding_registers_array = json_object_get(slave_obj, "holding_registers");
                slaves[j][i].holding_registers_addr = parse_address_array(holding_registers_array, &slaves[j][i].num_of_holding_registers);
            }

            num_of_slaves[j] = size;
        }
        else
        {
            slaves[j] = NULL;
            num_of_slaves[j] = 0;
        }
    }
    return slaves;
}

void free_slaves(simple_slave_t** slaves, uint8_t* num_of_slaves) 
{
    for(uint8_t j = 0; j < MAX_SERIAL_PORTS; j++)
    {
        if(slaves[j] != NULL)
            {
            for (uint8_t i = 0; i < num_of_slaves[j]; i++) 
            {
                free(slaves[j][i].coils_addr);
                free(slaves[j][i].discrete_inputs_addr);
                free(slaves[j][i].input_registers_addr);
                free(slaves[j][i].holding_registers_addr);
            }
            free(slaves[j]);
        }
    }
    free(slaves);
}

void free_modbus(modbus_t** ctx)
{
    if(ctx != NULL)
    {
        for(uint8_t i = 0; i < MAX_SERIAL_PORTS; i++)
        {
            if(ctx[i] != NULL)
            {
                modbus_close(ctx[i]);
                modbus_free(ctx[i]);
            }
        }
    }
}

void free_interrogation_response(interrogation_response_t* resp)
{
    free(resp->coils);
    free(resp->discrete_inputs);
    free(resp->input_regs);
    free(resp->holding_regs);
    free(resp);
}

simple_slave_t** init_slaves(const char* cfg_file, uint8_t* num_of_slaves)
{
    simple_slave_t** slaves = NULL;
    json_error_t error;

    json_t* root = json_load_file(cfg_file, 0, &error);
    
    if(root == NULL)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Error parsing JSON: %s (line %d, column %d)\n", error.text, error.line, error.column);
        #endif
        return NULL;
    }

    slaves = parse_slaves(root, num_of_slaves);
    json_decref(root);

    return slaves;
}

modbus_t* init_modbus_connection(const char* dev_path, uint32_t baud, uint8_t parity, uint8_t data_bits, uint8_t stop_bits)
{
    modbus_t* ctx = modbus_new_rtu(dev_path, baud, parity, data_bits, stop_bits); 
    if (ctx == NULL) 
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Unable to create the libmodbus context: %s\n", modbus_strerror(errno));
        #endif
        return NULL;
    }

    /* Set RS485 serial mode */
    /*
    rc = modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);
    if(rc == -1)
    {
        fprintf(stderr, "Failed to set RS485 serial mode: %s\n", modbus_strerror(errno));
        return NULL;
    }
    */

    /* Set timeout */
    modbus_set_byte_timeout(ctx, 0, 0);
    modbus_set_response_timeout(ctx, 0, RESPONSE_TIMEOUT);

    /* Connect to the line */
    if(modbus_connect(ctx) == -1)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Modbus connection failed: %s\n", modbus_strerror(errno));
        #endif
        return NULL;
    }

    return ctx;
}

void print_slaves(simple_slave_t** slaves, uint8_t* num_of_slaves)
{
    uint8_t i, j, k;
    for(k = 0; k < MAX_SERIAL_PORTS; k++)
    {
        fprintf(stdout, "----------------- SERIAL PORT %u -----------------\n\n", k + 1);
        if(slaves[k] != NULL)
        {
            for(i = 0; i < num_of_slaves[k]; i++)
            {
                fprintf(stdout, "Slave description: %s\n", slaves[k][i].name);

                fprintf(stdout, "ID: %u\n", slaves[k][i].id);

                fprintf(stdout, "Coil addresses: [");
                for(j = 0; j < slaves[k][i].num_of_coils; j++)
                {
                    fprintf(stdout, "%u", slaves[k][i].coils_addr[j]);
                    if(j != slaves[k][i].num_of_coils - 1)
                    {
                        fprintf(stdout, ", ");
                    }
                }
                fprintf(stdout, "]\n");

                fprintf(stdout, "Discrete inputs addresses: [");
                for(j = 0; j < slaves[k][i].num_of_discrete_inputs; j++)
                {
                    fprintf(stdout, "%u", slaves[k][i].discrete_inputs_addr[j]);
                    if(j != slaves[k][i].num_of_discrete_inputs - 1)
                    {
                        fprintf(stdout, ", ");
                    }
                }
                fprintf(stdout, "]\n");

                fprintf(stdout, "Input registers addresses: [");
                for(j = 0; j < slaves[k][i].num_of_input_registers; j++)
                {
                    fprintf(stdout, "%u", slaves[k][i].input_registers_addr[j]);
                    if(j != slaves[k][i].num_of_input_registers - 1)
                    {
                        fprintf(stdout, ", ");
                    }
                }
                fprintf(stdout, "]\n");

                fprintf(stdout, "Holding registers addresses: [");
                for(j = 0; j < slaves[k][i].num_of_holding_registers; j++)
                {
                    fprintf(stdout, "%u", slaves[k][i].holding_registers_addr[j]);
                    if(j != slaves[k][i].num_of_holding_registers - 1)
                    {
                        fprintf(stdout, ", ");
                    }
                }
                fprintf(stdout, "]\n\n");
            }
        }
        fprintf(stdout, "--------------------------------------------------\n\n");
    }
}

uint8_t get_slave_idx(uint16_t slave_id, simple_slave_t* slaves, uint8_t num_of_slaves)
{
    uint8_t idx;
    
    for(idx = 0; idx < num_of_slaves; idx++)
    {
        if(slaves[idx].id == slave_id)
        {
            return idx;
        }
    }
    return num_of_slaves;
}

interrogation_response_t* interrogate_slave(uint16_t slave_id, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx)
{
    interrogation_response_t* resp = NULL;
    uint8_t idx = 0;
    uint8_t addr = 0;
    uint8_t real_slave_id = (uint8_t)(slave_id - ((uint16_t)(slave_id / OFFSET_BY_PORT)) * OFFSET_BY_PORT); 

    if(slaves == NULL)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to interrogate slave, slave object is NULL.\n");
        #endif
        return NULL;
    }

    idx = get_slave_idx(slave_id, slaves, num_of_slaves);

    if(idx >= num_of_slaves)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to interrogate slave, invalid slave ID.\n");
        #endif
        return NULL;
    }
    modbus_set_slave(ctx, real_slave_id);

    resp = (interrogation_response_t*) malloc(sizeof(interrogation_response_t));
    if(resp == NULL)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to allocate memory for interrogation response structure.\n");
        #endif
        return NULL;
    }

    resp->coils = (uint8_t*) calloc(slaves[idx].num_of_coils, sizeof(uint8_t));
    resp->discrete_inputs = (uint8_t*) calloc(slaves[idx].num_of_discrete_inputs, sizeof(uint8_t));
    resp->input_regs = (uint16_t*) calloc(slaves[idx].num_of_input_registers, sizeof(uint16_t));
    resp->holding_regs = (uint16_t*) calloc(slaves[idx].num_of_holding_registers, sizeof(uint16_t));

    if(resp->coils == NULL || resp->discrete_inputs == NULL || resp->holding_regs == NULL || resp->input_regs == NULL)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to allocate memory for interrogation response arrays.\n");
        #endif
        return NULL;
    }

    resp->num_of_coils = slaves[idx].num_of_coils;
    resp->num_of_discrete_inputs = slaves[idx].num_of_discrete_inputs;
    resp->num_of_input_registers = slaves[idx].num_of_input_registers;
    resp->num_of_holding_registers = slaves[idx].num_of_holding_registers;

    #ifdef PRINT_DEBUG
        fprintf(stdout, "\n------ Interrogation start -------\n\nSlave id: %u\nSlave description: %s\n", slaves[idx].id, slaves[idx].name);
    #endif

    #ifdef PRINT_DEBUG
        fprintf(stdout, "Reading coils...\n");
    #endif
    for(uint8_t i = 0; i < slaves[idx].num_of_coils; i++)
    {
        addr = slaves[idx].coils_addr[i];
        modbus_read_bits(ctx, addr, 1, (&resp->coils[i]));

        #ifdef PRINT_DEBUG
            fprintf(stdout, "Coil %u status: %s\n", addr, resp->coils[i] ? "ON" : "OFF");
        #endif
    }

    #ifdef PRINT_DEBUG
        fprintf(stdout, "\n");

        fprintf(stdout, "Reading discrete inputs...\n");
    #endif

    for(uint8_t i = 0; i < slaves[idx].num_of_discrete_inputs; i++)
    {
        addr = slaves[idx].discrete_inputs_addr[i];
        modbus_read_input_bits(ctx, addr, 1, (uint8_t*)(&resp->discrete_inputs[i]));
        #ifdef PRINT_DEBUG
            fprintf(stdout, "Discrete input %u status: %s\n", addr, resp->discrete_inputs[i] ? "ON" : "OFF");
        #endif
    }

    #ifdef PRINT_DEBUG
        fprintf(stdout, "\n");

        fprintf(stdout, "Reading input registers...\n");
    #endif
    
    for(uint8_t i = 0; i < slaves[idx].num_of_input_registers; i++)
    {
        addr = slaves[idx].input_registers_addr[i];
        modbus_read_input_registers(ctx, addr, 1, &resp->input_regs[i]);
        
        #ifdef PRINT_DEBUG
            fprintf(stdout, "Input register %u value: %u\n", addr, resp->input_regs[i]);
        #endif
    }

    #ifdef PRINT_DEBUG
        fprintf(stdout, "\n");

        fprintf(stdout, "Reading holding registers...\n");
    #endif

    for(uint8_t i = 0; i < slaves[idx].num_of_holding_registers; i++)
    {
        addr = slaves[idx].holding_registers_addr[i];
        modbus_read_registers(ctx, addr, 1, &resp->holding_regs[i]);
        
        #ifdef PRINT_DEBUG
            fprintf(stdout, "Holding register %u value: %u\n", addr, resp->holding_regs[i]);
        #endif
    }

    #ifdef PRINT_DEBUG
        fprintf(stdout, "\n");
    #endif

    return resp;
}

uint8_t* read_coil(uint16_t slave_id, uint8_t coil_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx)
{
    uint8_t idx = 0;
    uint8_t* res = NULL;
    uint8_t real_slave_id = (uint8_t)(slave_id - ((uint16_t)(slave_id / OFFSET_BY_PORT)) * OFFSET_BY_PORT); 

    if(slaves == NULL)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to read coil status, slave object is NULL.\n");
        #endif
        return NULL;
    }

    idx = get_slave_idx(slave_id, slaves, num_of_slaves);

    if(idx >= num_of_slaves)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to read coil status, invalid slave ID.\n");
        #endif
        return NULL;
    }

    modbus_set_slave(ctx, real_slave_id);
    for(uint8_t i = 0; i < slaves[idx].num_of_coils; i++)
    {
        if(slaves[idx].coils_addr[i] == coil_addr)
        {
            res = (uint8_t*) calloc(1, sizeof(uint8_t));
            modbus_read_bits(ctx, coil_addr, 1, res);

            #ifdef PRINT_DEBUG
                fprintf(stdout, "Coil address: %u, status: %s\n", coil_addr, *res ? "ON" : "OFF");
            #endif
            break;
        }
    }

    #ifdef PRINT_DEBUG
        if(res == NULL)
        {
            fprintf(stderr, "Failed to read coil status, invalid coil address.\n");
        }
    #endif

    return res;
}

uint8_t* read_discrete_input(uint16_t slave_id, uint8_t discrete_input_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx)
{
    uint8_t idx = 0;
    uint8_t* res = NULL;
    uint8_t real_slave_id = (uint8_t)(slave_id - ((uint16_t)(slave_id / OFFSET_BY_PORT)) * OFFSET_BY_PORT); 

    if(slaves == NULL)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to read discrete input status, slave object is NULL.\n");
        #endif
        return NULL;
    }

    idx = get_slave_idx(slave_id, slaves, num_of_slaves);

    if(idx >= num_of_slaves)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to read discrete input status, invalid slave ID.\n");
        #endif
        return NULL;
    }

    modbus_set_slave(ctx, real_slave_id);
    for(uint8_t i = 0; i < slaves[idx].num_of_discrete_inputs; i++)
    {
        if(slaves[idx].discrete_inputs_addr[i] == discrete_input_addr)
        {
            res = (uint8_t*) calloc(1, sizeof(uint8_t));
            modbus_read_input_bits(ctx, discrete_input_addr, 1, res);

            #ifdef PRINT_DEBUG
                fprintf(stdout, "Discrete input address: %u, status: %s\n", discrete_input_addr, *res ? "ON" : "OFF");
            #endif
            break;
        }
    }

    #ifdef PRINT_DEBUG
        if(res == NULL)
        {
            fprintf(stderr, "Failed to read discrete input status, invalid discrete input address.\n");
        }
    #endif

    return res;
}

uint16_t* read_input_register(uint16_t slave_id, uint8_t input_reg_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx)
{
    uint8_t idx = 0;
    uint16_t* res = NULL;
    uint8_t real_slave_id = (uint8_t)(slave_id - ((uint16_t)(slave_id / OFFSET_BY_PORT)) * OFFSET_BY_PORT); 

    if(slaves == NULL)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to read input register value, slave object is NULL.\n");
        #endif
        return NULL;
    }

    idx = get_slave_idx(slave_id, slaves, num_of_slaves);

    if(idx >= num_of_slaves)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to read input register value, invalid slave ID.\n");
        #endif
        return NULL;
    }

    modbus_set_slave(ctx, real_slave_id);
    for(uint8_t i = 0; i < slaves[idx].num_of_input_registers; i++)
    {
        if(slaves[idx].input_registers_addr[i] == input_reg_addr)
        {
            res = (uint16_t*) calloc(1, sizeof(uint16_t));
            modbus_read_input_registers(ctx, input_reg_addr, 1, res);

            #ifdef PRINT_DEBUG
                fprintf(stdout, "Input register address: %u, value: %u\n", input_reg_addr, *res);
            #endif
            break;
        }
    }

    #ifdef PRINT_DEBUG
        if(res == NULL)
        {
            fprintf(stderr, "Failed to read input register value, invalid input register address.\n");
        }
    #endif

    return res;
}

uint16_t* read_holding_register(uint16_t slave_id, uint8_t holding_reg_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx)
{
    uint8_t idx = 0;
    uint16_t* res = NULL;
    uint8_t real_slave_id = (uint8_t)(slave_id - ((uint16_t)(slave_id / OFFSET_BY_PORT)) * OFFSET_BY_PORT); 

    if(slaves == NULL)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to read holding register, slave object is NULL.\n");
        #endif
        return NULL;
    }

    idx = get_slave_idx(slave_id, slaves, num_of_slaves);

    if(idx >= num_of_slaves)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to read holding register value, invalid slave ID.\n");
        #endif
        return NULL;
    }

    modbus_set_slave(ctx, real_slave_id);
    for(uint8_t i = 0; i < slaves[idx].num_of_holding_registers; i++)
    {
        if(slaves[idx].holding_registers_addr[i] == holding_reg_addr)
        {
            res = (uint16_t*) calloc(1, sizeof(uint16_t));
            modbus_read_registers(ctx, holding_reg_addr, 1, res);

            #ifdef PRINT_DEBUG
                fprintf(stdout, "Holding register address: %u, value: %u\n", holding_reg_addr, *res);
            #endif
            break;
        }
    }

    #ifdef PRINT_DEBUG
        if(res == NULL)
        {
            fprintf(stderr, "Failed to read holding register value, invalid holding register address.\n");
        }
    #endif

    return res;
}

uint8_t write_coil(uint16_t slave_id, uint8_t coil_addr, uint8_t coil_value, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx)
{
    uint8_t idx = 0;
    uint8_t real_slave_id = (uint8_t)(slave_id - ((uint16_t)(slave_id / OFFSET_BY_PORT)) * OFFSET_BY_PORT); 

    if(slaves == NULL)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to set coil status, slave object is NULL.\n");
        #endif
        return 0;
    }

    idx = get_slave_idx(slave_id, slaves, num_of_slaves);

    if(idx >= num_of_slaves)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to set the coil state, invalid slave ID.\n");
        #endif
        return 0;
    }

    modbus_set_slave(ctx, real_slave_id);
    for(uint8_t i = 0; i < slaves[idx].num_of_coils; i++)
    {
        if(slaves[idx].coils_addr[i] == coil_addr)
        {
            modbus_write_bit(ctx, coil_addr, coil_value);

            #ifdef PRINT_DEBUG
                fprintf(stdout, "Set status: %s to coil, address: %u\n", coil_value ? "ON" : "OFF", coil_addr);
            #endif
            return 1;
        }
    }
    
    #ifdef PRINT_DEBUG
        fprintf(stderr, "Failed to set coil status, invalid coil address.\n");
    #endif

    return 0;
}

uint8_t write_holding_register(uint16_t slave_id, uint8_t holding_reg_addr, uint16_t holding_reg_value, simple_slave_t* slaves, 
                               uint8_t num_of_slaves, modbus_t* ctx)
{
    uint8_t idx = 0;
    uint8_t real_slave_id = (uint8_t)(slave_id - ((uint16_t)(slave_id / OFFSET_BY_PORT)) * OFFSET_BY_PORT); 

    if(slaves == NULL)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to set holding register value, slave object is NULL.\n");
        #endif
        return 0;
    }

    idx = get_slave_idx(slave_id, slaves, num_of_slaves);

    if(idx >= num_of_slaves)
    {
        #ifdef PRINT_DEBUG
            fprintf(stderr, "Failed to set the holding register value, invalid slave ID.\n");
        #endif
        return 0;
    }

    modbus_set_slave(ctx, real_slave_id);
    for(uint8_t i = 0; i < slaves[idx].num_of_holding_registers; i++)
    {
        if(slaves[idx].holding_registers_addr[i] == holding_reg_addr)
        {
            modbus_write_register(ctx, holding_reg_addr, holding_reg_value);

            #ifdef PRINT_DEBUG
                fprintf(stdout, "Set value: %u to holding register, address: %u\n", holding_reg_value, holding_reg_addr);
            #endif
            return 1;
        }
    }

    #ifdef PRINT_DEBUG
        fprintf(stderr, "Failed to set holding register value, invalid holding register address.\n");
    #endif

    return 0;
}

