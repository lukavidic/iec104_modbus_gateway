/**
 * @file modbus_master.c
 * 
 * @brief This file contains implementation of functions used to
 * implement a specific modbus RTU master
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "modbus_master.h"

uint8_t* parse_address_array(json_t* json_array, uint8_t* count) 
{
    uint8_t size = json_array_size(json_array);
    uint8_t* addresses = (uint8_t*)malloc(size * sizeof(uint8_t));

    if (addresses == NULL) 
    {
        fprintf(stderr, "Failed to allocate memory for address array !\n");
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

simple_slave_t* parse_slaves(json_t* root, uint8_t* num_of_slaves)
{
    json_t* slaves_array = json_object_get(root, "slaves");
    json_t* slave_obj = NULL;
    uint8_t size = 0;
    simple_slave_t* slaves = NULL;

    if (json_is_array(slaves_array) == 0) 
    {
        fprintf(stderr, "Invalid JSON format: 'slaves' is not an array\n");
        return NULL;
    }

    size = json_array_size(slaves_array);
    slaves = (simple_slave_t*)malloc(size * sizeof(simple_slave_t));
    if (slaves == NULL) 
    {
        fprintf(stderr, "Failed to allocate memory for slave device objects.\n");
        return NULL;
    }

    for (uint8_t i = 0; i < size; i++) 
    {
        slave_obj = json_array_get(slaves_array, i);

        // Parse ID and description
        slaves[i].id = (uint8_t)json_integer_value(json_object_get(slave_obj, "id"));
        strncpy(slaves[i].name, json_string_value(json_object_get(slave_obj, "description")), MAX_SLAVE_NAME_LEN);

        // Parse coils addresses
        json_t* coils_array = json_object_get(slave_obj, "coils");
        slaves[i].coils_addr = parse_address_array(coils_array, &slaves[i].num_of_coils);

        // Parse discrete inputs addresses
        json_t* discrete_inputs_array = json_object_get(slave_obj, "discrete_inputs");
        slaves[i].discrete_inputs_addr = parse_address_array(discrete_inputs_array, &slaves[i].num_of_discrete_inputs);

        // Parse input registers addresses
        json_t* input_registers_array = json_object_get(slave_obj, "input_registers");
        slaves[i].input_registers_addr = parse_address_array(input_registers_array, &slaves[i].num_of_input_registers);

        // Parse holding registers addresses
        json_t* holding_registers_array = json_object_get(slave_obj, "holding_registers");
        slaves[i].holding_registers_addr = parse_address_array(holding_registers_array, &slaves[i].num_of_holding_registers);
    }

    *num_of_slaves = size;
    return slaves;
}

void free_slaves(simple_slave_t* slaves, uint8_t num_of_slaves) 
{
    for (uint8_t i = 0; i < num_of_slaves; i++) 
    {
        free(slaves[i].coils_addr);
        free(slaves[i].discrete_inputs_addr);
        free(slaves[i].input_registers_addr);
        free(slaves[i].holding_registers_addr);
    }
    free(slaves);
}

void free_modbus(modbus_t* ctx)
{
    modbus_close(ctx);
    modbus_free(ctx);
}

simple_slave_t* init_slaves(const char* cfg_file, uint8_t* num_of_slaves)
{
    simple_slave_t* slaves = NULL;
    json_error_t error;

    json_t* root = json_load_file(cfg_file, 0, &error);
    
    if(root == NULL)
    {
        fprintf(stderr, "Error parsing JSON: %s (line %d, column %d)\n", error.text, error.line, error.column);
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
        fprintf(stderr, "Unable to create the libmodbus context: %s\n", modbus_strerror(errno));
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
        fprintf(stderr, "Modbus connection failed: %s\n", modbus_strerror(errno));
        return NULL;
    }

    return ctx;
}

void print_slaves(simple_slave_t* slaves, uint8_t num_of_slaves)
{
    uint8_t i, j;
    for(i = 0; i < num_of_slaves; i++)
    {
        fprintf(stdout, "Slave description: %s\n", slaves[i].name);

        fprintf(stdout, "ID: %u\n", slaves[i].id);

        fprintf(stdout, "Coil addresses: [");
        for(j = 0; j < slaves[i].num_of_coils; j++)
        {
            fprintf(stdout, "%u", slaves[i].coils_addr[j]);
            if(j != slaves[i].num_of_coils - 1)
            {
                fprintf(stdout, ", ");
            }
        }
        fprintf(stdout, "]\n");

        fprintf(stdout, "Discrete inputs addresses: [");
        for(j = 0; j < slaves[i].num_of_discrete_inputs; j++)
        {
            fprintf(stdout, "%u", slaves[i].discrete_inputs_addr[j]);
            if(j != slaves[i].num_of_discrete_inputs - 1)
            {
                fprintf(stdout, ", ");
            }
        }
        fprintf(stdout, "]\n");

        fprintf(stdout, "Input registers addresses: [");
        for(j = 0; j < slaves[i].num_of_input_registers; j++)
        {
            fprintf(stdout, "%u", slaves[i].input_registers_addr[j]);
            if(j != slaves[i].num_of_input_registers - 1)
            {
                fprintf(stdout, ", ");
            }
        }
        fprintf(stdout, "]\n");

        fprintf(stdout, "Holding registers addresses: [");
        for(j = 0; j < slaves[i].num_of_holding_registers; j++)
        {
            fprintf(stdout, "%u", slaves[i].holding_registers_addr[j]);
            if(j != slaves[i].num_of_holding_registers - 1)
            {
                fprintf(stdout, ", ");
            }
        }
        fprintf(stdout, "]\n");
        
        fprintf(stdout, "\n\n");
    }
}

interrogation_response_t* interrogate_slave(uint8_t slave_id, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx)
{
    interrogation_response_t* resp = NULL;
    uint8_t idx = 0, addr = 0;

    while(slave_id != slaves[idx].id && idx < num_of_slaves)
    {
        idx++;
    }

    if(idx >= num_of_slaves)
    {
        fprintf(stderr, "Failed to interrogate slave, invalid slave ID.\n");
        return NULL;
    }

    fprintf(stdout, "Interrogation start. Slave id: %u, description: %s\n", slaves[idx].id, slaves[idx].name);
    modbus_set_slave(ctx, slave_id);

    resp = (interrogation_response_t*) malloc(sizeof(interrogation_response_t));
    resp->coils = (uint8_t*) malloc(slaves[idx].num_of_coils * sizeof(uint8_t));
    resp->discrete_inputs = (uint8_t*) malloc(slaves[idx].num_of_discrete_inputs * sizeof(uint8_t));
    resp->input_regs = (uint16_t*) malloc(slaves[idx].num_of_input_registers * sizeof(uint16_t));
    resp->holding_regs = (uint16_t*) malloc(slaves[idx].num_of_holding_registers * sizeof(uint16_t));

    if(resp->coils == NULL || resp->discrete_inputs == NULL || resp->holding_regs == NULL || resp->input_regs == NULL)
    {
        fprintf(stdout, "Failed to allocate memory for interrogation response.\n");
        return NULL;
    }

    resp->num_of_coils = slaves[idx].num_of_coils;
    resp->num_of_discrete_inputs = slaves[idx].num_of_discrete_inputs;
    resp->num_of_input_registers = slaves[idx].num_of_input_registers;
    resp->num_of_holding_registers = slaves[idx].num_of_holding_registers;

    fprintf(stdout, "Reading coils...\n");
    for(uint8_t i = 0; i < slaves[idx].num_of_coils; i++)
    {
        addr = slaves[idx].coils_addr[i];
        modbus_read_bits(ctx, addr, 1, (&resp->coils[i]));
        fprintf(stdout, "Coil %u status: %s\n", addr, resp->coils[i] ? "ON" : "OFF");
    }
    fprintf(stdout, "\n");

    fprintf(stdout, "Reading discrete inputs...\n");
    for(uint8_t i = 0; i < slaves[idx].num_of_discrete_inputs; i++)
    {
        addr = slaves[idx].discrete_inputs_addr[i];
        modbus_read_input_bits(ctx, addr, 1, (uint8_t*)(&resp->discrete_inputs[i]));
        fprintf(stdout, "Discrete input %u status: %s\n", addr, resp->discrete_inputs[i] ? "ON" : "OFF");
    }
    fprintf(stdout, "\n");

    fprintf(stdout, "Reading input registers...\n");
    for(uint8_t i = 0; i < slaves[idx].num_of_input_registers; i++)
    {
        addr = slaves[idx].input_registers_addr[i];
        modbus_read_input_registers(ctx, addr, 1, &resp->input_regs[i]);
        fprintf(stdout, "Input register %u value: %u\n", addr, resp->input_regs[i]);
    }
    fprintf(stdout, "\n");

    fprintf(stdout, "Reading holding registers...\n");
    for(uint8_t i = 0; i < slaves[idx].num_of_holding_registers; i++)
    {
        addr = slaves[idx].holding_registers_addr[i];
        modbus_read_registers(ctx, addr, 1, &resp->holding_regs[i]);
        fprintf(stdout, "Holding register %u value: %u\n", addr, resp->holding_regs[i]);
    }
    fprintf(stdout, "\n");

    return resp;
}

uint8_t* read_coil(uint8_t slave_id, uint8_t coil_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx)
{

}

uint8_t* read_coil(uint8_t slave_id, uint8_t discrete_input_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx)
{
    
}

uint16_t* read_coil(uint8_t slave_id, uint8_t input_reg_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx)
{

}

uint16_t* read_coil(uint8_t slave_id, uint8_t holding_reg_addr, simple_slave_t* slaves, uint8_t num_of_slaves, modbus_t* ctx);