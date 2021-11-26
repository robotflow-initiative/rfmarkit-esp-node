#ifndef _SYS_H
#define _SYS_H

/** @brief func_command **/
#define COMMAND_FUNCTION(name) \
        esp_err_t command_func_##name(char* rx_buffer, \
                       int rx_len, \
                       char* tx_buffer, \
                       int tx_len)
                       
#endif