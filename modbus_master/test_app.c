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
    int rc = 0;
    uint8_t num_of_slaves = 0;
    simple_slave_t* slaves = NULL;
    modbus_t* ctx = NULL;

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

    fprintf(stdout, "\nSending interrogation requests to all slaves...\n\n");

    for(uint8_t i = 0; i < num_of_slaves; i++)
    {
        interrogate_slave(slaves[i].id, slaves, num_of_slaves, ctx);
        fprintf(stdout, "---------------------------------------------------------------------\n\n");
    }

    while(running)
    {
        // TODO: Implement some client specific actions
    }

    __terminate:
    fprintf(stdout, "Exiting...\n");
    free_slaves(slaves, num_of_slaves);
    free_modbus(ctx);
    return 0;
}