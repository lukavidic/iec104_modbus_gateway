#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include "modbus_master.h"

#define BAUD_RATE 19200
#define PARITY_NONE 'N'
#define DATA_BITS 8 
#define STOP_BITS 2

#define PRINT_DEBUG

const char* DEVICE_PATH      = "/dev/ttyS3";
const char* CONFIG_FILE_PATH = "config.json";

uint8_t running = 1;

/**
 * @brief Basic SIGINT handler function to stop the program and free the resources
 * 
 * @param id The ID of the received signal (SIGINT expected)  
 */
void sigint_handler(int id)
{
    running = 0;
}

int main()
{
    int rc = 0, choice = 0;
    uint8_t num_of_slaves = 0;
    simple_slave_t* slaves = NULL;
    modbus_t* ctx = NULL;

    uint8_t slave_id = 0;
    uint8_t target_address = 0;
    uint16_t target_value = 0;
    uint8_t* recv_value8 = NULL;
    uint16_t* recv_value16 = NULL;
    interrogation_response_t resp = NULL;

    /* Add CTRL-C handler */
    signal(SIGINT, sigint_handler);

    /* Configure the master with information about connected slave devices */
    slaves = init_slaves(CONFIG_FILE_PATH, &num_of_slaves);
    if(slaves == NULL)
    {
        fprintf(stderr, "Unable to get slave devices configuration.\n");
        goto __terminate;
    }

    /* Initialize modbus RTU context */
    ctx = init_modbus_connection(DEVICE_PATH, BAUD_RATE, PARITY_NONE, DATA_BITS, STOP_BITS);

    if(ctx == NULL)
    {
        goto __terminate;
    }

    print_slaves(slaves, num_of_slaves);

    while(running)
    {
        fprintf(stdout, "Enter one of the desired commands.\n");
        fprintf(stdout, "[0] - Slave interrogation\n
                         [1] - Read coil status\n
                         [2] - Read discrete input status\n
                         [3] - Read input register value\n
                         [4] - Read holding register value\n
                         [5] - Set coil status\n
                         [6] - Set holding register value\n\n
                         Choice: ");
        fscanf(stdin, "%d\n", &choice);
        switch (choice)
        {
        case SLAVE_INTERROGATION_CMD:
            fprintf(stdout, "Slave ID: ");
            fscanf(stdin, "%u\n", &slave_id);
            resp = interrogate_slave(slave_id, slaves, num_of_slaves, ctx);
            if(resp == NULL)
            {
                fprintf(stdout, "Slave interrogation failed.\n");
            }
            else
            {
                fprintf(stdout, "Slave interrogation successful.\n");
                free_interrogation_response(resp);
            }
            break;
        case READ_COIL_CMD:
            fprintf(stdout, "Slave ID: ");
            fscanf(stdin, "%u\n", &slave_id);
            fprintf(stdout, "Coil address: ");
            fscanf(stdin, "%u\n", &target_address);
            recv_value8 = read_coil(slave_id, target_address, slaves, num_of_slaves, ctx);
            if(recv_value8 == NULL)
            {
                fprintf(stdout, "Reading coil status failed.\n");
            }
            else
            {
                fprintf(stdout, "Coil status reading successful.\n");
                free(recv_value8);
            }
            break;
        case READ_DISCRETE_INPUT_CMD:
            fprintf(stdout, "Slave ID: ");
            fscanf(stdin, "%u\n", &slave_id);
            fprintf(stdout, "Discrete input address: ");
            fscanf(stdin, "%u\n", &target_address);
            recv_value8 = read_discrete_input(slave_id, target_address, slaves, num_of_slaves, ctx);
            if(recv_value8 == NULL)
            {
                fprintf(stdout, "Reading discrete input status failed.\n");
            }
            else
            {
                fprintf(stdout, "Discrete input status reading successful.\n");
                free(recv_value8);
            }
            break;
        case READ_INPUT_REGISTER_CMD:
            fprintf(stdout, "Slave ID: ");
            fscanf(stdin, "%u\n", &slave_id);
            fprintf(stdout, "Input register address: ");
            fscanf(stdin, "%u\n", &target_address);
            recv_value16 = read_input_register(slave_id, target_address, slaves, num_of_slaves, ctx);
            if(recv_value16 == NULL)
            {
                fprintf(stdout, "Reading input register value failed.\n");
            }
            else
            {
                fprintf(stdout, "Input register value reading successful.\n");
                free(recv_value16);
            }
            break;
        case READ_HOLDING_REGISTER_CMD:
            fprintf(stdout, "Slave ID: ");
            fscanf(stdin, "%u\n", &slave_id);
            fprintf(stdout, "Holding register address: ");
            fscanf(stdin, "%u\n", &target_address);
            recv_value16 = read_holding_register(slave_id, target_address, slaves, num_of_slaves, ctx);
            if(recv_value16 == NULL)
            {
                fprintf(stdout, "Reading holding register value failed.\n");
            }
            else
            {
                fprintf(stdout, "Holding register value reading successful.\n");
                free(recv_value16);
            }
            break;
        case WRITE_COIL_CMD:
            fprintf(stdout, "Slave ID: ");
            fscanf(stdin, "%u\n", &slave_id);
            fprintf(stdout, "Coil address: ");
            fscanf(stdin, "%u\n", &target_address);
            fprintf(stdout, "Coil value (0, 1): ");
            fscanf(stdin, "%u\n", &target_value);
            if(target_value > 0)
            {
                target_value = COIL_ON_VALUE
            }
            else
            {
                target_value = COIL_OFF_VALUE;
            }
            if(write_coil(slave_id, target_address, (uint8_t)target_value, slaves, num_of_slaves, ctx))
            {
                fprintf(stdout, "Setting coil status successful.\n");
            }
            else
            {
                fprintf(stdout, "Setting coil status failed.\n");
            }
            break;
        case WRITE_HOLDING_REGISTER_CMD:
            fprintf(stdout, "Slave ID: ");
            fscanf(stdin, "%u\n", &slave_id);
            fprintf(stdout, "Holding register address: ");
            fscanf(stdin, "%u\n", &target_address);
            fprintf(stdout, "Holding register value (0 - 65535): ");
            fscanf(stdin, "%u\n", &target_value);
            if(write_holding_register(slave_id, target_address, target_value, slaves, num_of_slaves, ctx))
            {
                fprintf(stdout, "Setting holding register value successful.\n");
            }
            else
            {
                fprintf(stdout, "Setting holding register value failed.\n");
            }
            break;
        default:
            fprintf(stdout, "ERROR, non-existent command issued.\n");
            break;
        while(getchar() != '\n');
        system("clear");
        }
    }

    __terminate:
    fprintf(stdout, "Exiting...\n");
    if(slaves != NULL)
    {
        free_slaves(slaves, num_of_slaves);
    }
    free_modbus(ctx);
    return 0;
}