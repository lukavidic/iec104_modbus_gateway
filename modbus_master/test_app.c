#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include "modbus_master.h"

#define PRINT_SLAVES_VAL 7 
#define EXIT_VAL 8

const char* DEVICE_PATHS[SERIAL_PORTS_NUM] = {"/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS5", "/dev/ttyS6"};
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
    int rc = 0;
    uint8_t num_of_slaves[SERIAL_PORTS_NUM] = {0};
    simple_slave_t** slaves = NULL;
    modbus_t* ctx[SERIAL_PORTS_NUM];
    serial_configuration_t cfg[SERIAL_PORTS_NUM];
    int choice = 0;

    uint8_t idx = 0;
    uint16_t slave_id = 0;
    uint8_t target_address = 0;
    uint16_t target_value = 0;
    uint8_t* recv_value8 = NULL;
    uint16_t* recv_value16 = NULL;
    interrogation_response_t* resp = NULL;

    /* Add CTRL-C handler */
    signal(SIGINT, sigint_handler);

    /* Configure the master with information about connected slave devices */
    slaves = init_slaves(CONFIG_FILE_PATH, num_of_slaves, cfg);
    if(slaves == NULL)
    {
        fprintf(stderr, "Unable to get slave devices configuration.\n");
        goto __terminate;
    }

    /* Initialize modbus RTU context */
    for(uint8_t i = 0; i < SERIAL_PORTS_NUM; i++)
    {
        if(slaves[i] != NULL)
        {
            ctx[i] = init_modbus_connection(DEVICE_PATHS[i], cfg[i].baud_rate, cfg[i].parity, cfg[i].data_bits, cfg[i].stop_bits);
        }
        else
        {
            ctx[i] = NULL;
        }
    }

    if(ctx == NULL)
    {
        goto __terminate;
    }

    print_slaves(slaves, num_of_slaves);

    /* Interrogate all slaves */
    /*
    for(uint8_t i = 0; i < num_of_slaves; i++)
    {
        interrogate_slave(slaves[i].id, slaves, num_of_slaves, ctx);
    }
    */

    while(running)
    {
        fprintf(stdout, "Enter one of the desired commands.\n");
        fprintf(stdout, "[0] - Slave interrogation\n"
                        "[1] - Read coil status\n"
                        "[2] - Read discrete input status\n"
                        "[3] - Read input register value\n"
                        "[4] - Read holding register value\n"
                        "[5] - Set coil status\n"
                        "[6] - Set holding register value\n"
                        "[7] - Print information about slaves\n"
                        "[8] - Exit program\n\n"
                        "Choice: ");
        scanf("%d", &choice);
        switch (choice)
        {
        case SLAVE_INTERROGATION_CMD:
            fprintf(stdout, "Slave ID: ");
            scanf("%hu", &slave_id);
            idx = slave_id / OFFSET_BY_PORT - 1;
            if(idx < 0 || idx >= SERIAL_PORTS_NUM)
            {
                fprintf(stderr, "Invalid slave ID, index out of bounds.\n");
                break;
            }
            resp = interrogate_slave(slave_id, slaves[idx], num_of_slaves[idx], ctx[idx]);
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
            scanf("%hu", &slave_id);
            fprintf(stdout, "Coil address: ");
            scanf("%hhu", &target_address);
            idx = slave_id / OFFSET_BY_PORT - 1;
            if(idx < 0 || idx >= SERIAL_PORTS_NUM)
            {
                fprintf(stderr, "Invalid slave ID, index out of bounds.\n");
                break;
            }
            recv_value8 = read_coil(slave_id, target_address, slaves[idx], num_of_slaves[idx], ctx[idx]);
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
            scanf("%hu", &slave_id);
            fprintf(stdout, "Discrete input address: ");
            scanf("%hhu", &target_address);
            idx = slave_id / OFFSET_BY_PORT - 1;
            if(idx < 0 || idx >= SERIAL_PORTS_NUM)
            {
                fprintf(stderr, "Invalid slave ID, index out of bounds.\n");
                break;
            }
            recv_value8 = read_discrete_input(slave_id, target_address, slaves[idx], num_of_slaves[idx], ctx[idx]);
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
            scanf("%hu", &slave_id);
            fprintf(stdout, "Input register address: ");
            scanf("%hhu", &target_address);
            idx = slave_id / OFFSET_BY_PORT - 1;
            if(idx < 0 || idx >= SERIAL_PORTS_NUM)
            {
                fprintf(stderr, "Invalid slave ID, index out of bounds.\n");
                break;
            }
            recv_value16 = read_input_register(slave_id, target_address, slaves[idx], num_of_slaves[idx], ctx[idx]);
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
            scanf("%hu", &slave_id);
            fprintf(stdout, "Holding register address: ");
            scanf("%hhu", &target_address);
            idx = slave_id / OFFSET_BY_PORT - 1;
            if(idx < 0 || idx >= SERIAL_PORTS_NUM)
            {
                fprintf(stderr, "Invalid slave ID, index out of bounds.\n");
                break;
            }
            recv_value16 = read_holding_register(slave_id, target_address, slaves[idx], num_of_slaves[idx], ctx[idx]);
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
            scanf("%hu", &slave_id);
            fprintf(stdout, "Coil address: ");
            scanf("%hhu", &target_address);
            fprintf(stdout, "Coil value (0, 1): ");
            scanf("%hu", &target_value);
            if(target_value > 0)
            {
                target_value = COIL_ON_VALUE;
            }
            else
            {
                target_value = COIL_OFF_VALUE;
            }
            idx = slave_id / OFFSET_BY_PORT - 1;
            if(idx < 0 || idx >= SERIAL_PORTS_NUM)
            {
                fprintf(stderr, "Invalid slave ID, index out of bounds.\n");
                break;
            }
            if(write_coil(slave_id, target_address, (uint8_t)target_value, slaves[idx], num_of_slaves[idx], ctx[idx]))
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
            scanf("%hu", &slave_id);
            fprintf(stdout, "Holding register address: ");
            scanf("%hhu", &target_address);
            fprintf(stdout, "Holding register value (0 - 65535): ");
            scanf("%hu", &target_value);
            idx = slave_id / OFFSET_BY_PORT - 1;
            if(idx < 0 || idx >= SERIAL_PORTS_NUM)
            {
                fprintf(stderr, "Invalid slave ID, index out of bounds.\n");
                break;
            }
            if(write_holding_register(slave_id, target_address, target_value, slaves[idx], num_of_slaves[idx], ctx[idx]))
            {
                fprintf(stdout, "Setting holding register value successful.\n");
            }
            else
            {
                fprintf(stdout, "Setting holding register value failed.\n");
            }
            break;
        case PRINT_SLAVES_VAL:
            print_slaves(slaves, num_of_slaves);
            break;
        case EXIT_VAL:
            goto __terminate;
        default:
            fprintf(stdout, "ERROR, non-existent command issued.\n");
            break;
        }
        fprintf(stdout, "-------------------------------[PRESS ENTER KEY]-------------------------------");
        getchar();
        while(getchar() != '\n');
        system("clear");
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