#ifndef _mma7361_H_
#define _mma7361_H_

#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

typedef struct mma7361_t *mma7361_handle_t; 

typedef struct {
    adc_channel_t x_channel;
    adc_channel_t y_channel;
    adc_channel_t z_channel; 
    gpio_num_t zero_g;
    gpio_num_t g_select;
    gpio_num_t self_test;
    gpio_num_t sleep;
   
} mma7361_config_t;


void func(void);

esp_err_t mma7361_new(mma7361_config_t *config, mma7361_handle_t *ret_ma7361);

void mma7361_read_3axes(int *axis_x, int *axis_y, int *axis_z, mma7361_handle_t ret_ma7361);

#endif
