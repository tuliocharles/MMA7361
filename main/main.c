#include <stdio.h>

#include "freertos/FreeRTOS.h"

#include "../components/MMA7361/include/MMA7361.h"

void app_main(void)
{
    //ADC ADC1_0, ADC_3, ADC_6
    //gpio_4 = G-SELECT = 1
    //GPIO1_16 = SELF TEST = 0
    //GPIO_17 = 0G_DETECT = INPUT
    //GPPOI5 = SLEEP = 0

    mma7361_handle_t mma7361_handle;
    mma7361_config_t mma7361_config = {
        .x_channel = ADC_CHANNEL_0,
        .y_channel = ADC_CHANNEL_3,
        .z_channel = ADC_CHANNEL_6,
        .g_select  = GPIO_NUM_4,
        .self_test = GPIO_NUM_16,
        .zero_g = GPIO_NUM_17,
        .sleep = GPIO_NUM_15,
        .x_offset = 1650,
        .y_offset = 1850,
        .z_offset = 1350,
    };
    
    mma7361_new(&mma7361_config, &mma7361_handle);

    int axis_x, axis_y, axis_z;

    int cont = 0;

    while(1){

        if(cont < 20){
            mma7361_read_3axes(&axis_x, &axis_y, &axis_z, mma7361_handle);
        }
        
        vTaskDelay(3000/portTICK_PERIOD_MS);

        cont++;
        
        if(cont == 20){
            mma7361_del(mma7361_handle);
        }
        
        if (cont == 5){
            mma7361_gselect(0,mma7361_handle);

        }
        if (cont == 10){
            mma7361_selftest(1,mma7361_handle);
            
        }
        if (cont == 15){
            mma7361_sleep(0,mma7361_handle);
            
        }
    }
    

}