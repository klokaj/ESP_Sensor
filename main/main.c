/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>

#include "bmp280.h"
#include "dht_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "../config/WifiConfig.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "rom/ets_sys.h"

#include "sdkconfig.h"




#define CONFIG_I2C_MASTER_SCL 19
#define CONFIG_I2C_MASTER_SDA 18


#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000


#define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT        



#define BM280_ADDR                          0x76
//#define BME280_I2C_ADDR_PRIM                0x76
#define BM280_DATA_ADDR_START               0xF7
#define BM280_DATA_ADDR_END                 0xFC
#define BM280_ID_ADDR                       0xD0

#define BM280_CTRL_MEAS_ADDR                0xF4
#define BM280_CONFIG_ADDR                   0xF5
#define BM280_STATUS_ADDR                   0xF3

void delay_ms(uint32_t period);




int calibrated = 0;


// static double compensate_temperature(const struct bme280_uncomp_data *uncomp_data, struct bme280_calib_data *calib_data)
// {
//     double var1;
//     double var2;
//     double temperature;
//     double temperature_min = -40;
//     double temperature_max = 85;

//     var1 = ((double)uncomp_data->temperature) / 16384.0 - ((double)calib_data->dig_t1) / 1024.0;
//     var1 = var1 * ((double)calib_data->dig_t2);
//     var2 = (((double)uncomp_data->temperature) / 131072.0 - ((double)calib_data->dig_t1) / 8192.0);
//     var2 = (var2 * var2) * ((double)calib_data->dig_t3);
//     calib_data->t_fine = (int32_t)(var1 + var2);
//     temperature = (var1 + var2) / 5120.0;

//     if (temperature < temperature_min)
//     {
//         temperature = temperature_min;
//     }
//     else if (temperature > temperature_max)
//     {
//         temperature = temperature_max;
//     }


//     return temperature;
// }
           
/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
// static esp_err_t BM280_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     esp_err_t retv;


//     retv = i2c_master_write_read_device(I2C_MASTER_NUM, BM280_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

//     return retv;

// }


/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t BM280_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BM280_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    

    return ret;
}




int8_t BMP280_i2c_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter intf_ptr can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */


    for(int i = 0; i < len; i++){

        BM280_register_write_byte(reg_addr + i, reg_data[i]);

    }

    //rslt = i2c_master_write_to_device(I2C_MASTER_NUM, i2c_addr, &reg_addr, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    //rslt |= i2c_master_write_to_device(I2C_MASTER_NUM, i2c_addr, reg_data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    return rslt;
}

int8_t BMP280_i2c_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter intf_ptr can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    //printf("Reading from device 0x%x adddr 0x%x  len %d \n", i2c_addr, reg_addr, len);

    delay_ms(10);
    rslt = i2c_master_write_read_device(I2C_MASTER_NUM, i2c_addr, &reg_addr, 1, reg_data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    //i2c_master_read_from_device(I2C_MASTER_NUM, reg_addr, reg_data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    //printf("Read data %x %x %x %x %x %x \n", reg_data[0], reg_data[1], reg_data[2], reg_data[0], reg_data[1], reg_data[2]);

    return rslt;
}




void user_delay_ms(uint32_t period, void *intf_ptr)
{

    TickType_t xDelay = period / portTICK_PERIOD_MS;

    vTaskDelay(xDelay);
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
}


void delay_ms(uint32_t period)
{

    TickType_t xDelay = period / portTICK_PERIOD_MS;

    vTaskDelay(xDelay);
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
}





/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}



#define DTH_DATA_PIN GPIO_NUM_27


void dht_pin_init(dht_gpio_mode_t mode){

    if(mode == 0){
        gpio_set_direction(DTH_DATA_PIN, GPIO_MODE_INPUT);
    }
    else{
        gpio_set_direction(DTH_DATA_PIN, GPIO_MODE_OUTPUT);
    }
    
}

void dht_pin_set(dht_gpio_state_t state){

    if(state == 0){
        gpio_set_level(DTH_DATA_PIN, 0);
    }
    else{
        gpio_set_level(DTH_DATA_PIN, 1);
    }

}


dht_gpio_state_t dht_pin_read(){
    int state = gpio_get_level(DTH_DATA_PIN);


    if(state == 0) return DHT_GPIO_LOW;

    return DHT_GPIO_HIGH;
}


void dht_delay_us(uint32_t period){
    ets_delay_us(period);
}





#define BH1750_ADDR  0x23
#define BH1750_MODE  0b00010001    




void BH1750_ChangeMeasTime(){
    const uint8_t MTreg_High = 0b01000111;       //Change Measurment time - Highg bit, Mtreg[7:5] = 111 
    const uint8_t MTreg_Low = 0b01111110;       //Change measurment time - Low Bit, Mtreg[4:0] = 11110

    gpio_set_level(GPIO_NUM_2, 0);

    delay_ms(10);
    i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_ADDR, &MTreg_High, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    delay_ms(10);
    i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_ADDR, &MTreg_Low, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);


}



void BH1750_Init(){
    const uint8_t buff = BH1750_MODE;
    

    gpio_set_level(GPIO_NUM_2, 0);



    i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_ADDR, &buff, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);


    BH1750_ChangeMeasTime();


    gpio_set_level(GPIO_NUM_2, 0);
    gpio_set_level(GPIO_NUM_2, 1);
}

float BH1750_Read(){
    uint8_t buff[2];


    float lum = 0;



    gpio_set_level(GPIO_NUM_2, 0);
    i2c_master_read_from_device(I2C_MASTER_NUM, BH1750_ADDR, buff, 2, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);




    lum = ((buff[0] << 8) + buff[1]) * 0.113188976;  
    /*

    // lum = read_val * (1/1.2)*(69/254) / 2; 
    
    1/1,2 - default sensitivity
    69/254  - setted time window coeficient. 69 - default time window, 254 - setted time window

    /2 - High resolution mode 2

    */


    return lum;

}




void app_main(void)
{
    uint8_t data[20];
    unsigned int temperature = 1;
    double tmp = 0;
    
    uint8_t settings_sel;

    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    int32_t temp32;
    double temp;

    uint32_t pres32, pres64;
    double pres;
    double humm = 0;
    double lum = 0;


    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000/portTICK_PERIOD_MS;

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);



  
    dht_sensor_t dht = {
        .type = DHT_22,
        .intrf = {
            .pin_init = &dht_pin_init,
            .pin_set = &dht_pin_set,
            .pin_read = &dht_pin_read,
            .delay_us = &dht_delay_us
        }
    };

    struct bmp280_dev dev;
    //struct identifier id;
    int8_t rslt = BMP280_OK;

    

    //id.dev_addr = BME280_I2C_ADDR_PRIM;


    dev.dev_id = BMP280_I2C_ADDR_PRIM;
    dev.intf = BMP280_I2C_INTF;
    dev.read = &BMP280_i2c_read;
    dev.write = &BMP280_i2c_write;
    dev.delay_ms = &delay_ms;

    printf("I2c Initializing \n");
    i2c_master_init();

    printf("Initialized \n");
    // ESP_LOGI(TAG, "I2C initialized successfully");
    
    

    BH1750_Init();


    rslt = bmp280_init(&dev);
    
    
    if (rslt != BMP280_OK)
    {
        printf("Failed to initialize the device (code %+d).\n", rslt);
        exit(1);
    }
    

    delay_ms(500);

    
    rslt = bmp280_get_config(&conf, &dev);
    delay_ms(100);

    if (rslt != BMP280_OK)
    {
        printf("Failed to Get config %+d).\n", rslt);
    }
    
    conf.filter = BMP280_FILTER_OFF;
    conf.os_temp = BMP280_OS_16X;    
    conf.os_pres = BMP280_OS_16X;


    conf.odr = BMP280_ODR_1000_MS;

    bmp280_set_config(&conf, &dev);
    delay_ms(100);
    if (rslt != BMP280_OK)
    {
        printf("Failed to Set config %+d).\n", rslt);
    }


    bmp280_set_power_mode(BMP280_NORMAL_MODE, &dev);
delay_ms(100);
    if (rslt != BMP280_OK)
    {
        printf("Failed to Set Power mode %+d).\n", rslt);
    }

    
    int state = 0;


    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

        gpio_set_level(GPIO_NUM_4, 0);
        dht_read(&dht);
        lum = BH1750_Read();

        /* Reading the raw data from sensor */
        rslt = bmp280_get_uncomp_data(&ucomp_data, &dev);

        /* Getting the 32 bit compensated temperature */
        rslt = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &dev);

        /* Getting the compensated temperature as floating point value */
        rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &dev);




        /* Getting the compensated pressure using 32 bit precision */
        rslt = bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, &dev);

        /* Getting the compensated pressure using 64 bit precision */
        rslt = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &dev);

        /* Getting the compensated pressure as floating point value */
        rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &dev);




        printf("Temperature1: %f Temperature2: %f Pressure: %f Humidity: %f Luminosity: %f \r\n", temp, dht.temperature, pres, dht.humidity, lum);

        gpio_set_level(GPIO_NUM_4, 1);
        /* Sleep time between measurements = BMP280_ODR_1000_MS */
        //dev.delay_ms(5000);
    }

    // dev.settings.osr_h = BMP280_OVERSAMPLING_1X;
    // dev.settings.osr_p = BMP280_OVERSAMPLING_16X;
    // dev.settings.osr_t = BMP280_OVERSAMPLING_16X;
    // dev.settings.filter = BMP280_FILTER_COEFF_16;

    // settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    // rslt = bme280_set_sensor_settings(settings_sel, &dev);

    // if (rslt != BME280_OK)
    // {
    //     printf("Failed to set sensor settings (code %+d).", rslt);

    //     exit(1);
    // }




    // while(1){

    //     rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);

    //     delay_ms(40);

    //     bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);


    //     printf("Temperature: %f, Pressure: %f \n", comp_data.temperature, comp_data.pressure);

    //     delay_ms(1000);
    // }
    



    while(1) {






    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
    //ESP_ERROR_CHECK(mpu9250_register_read(BM280_ID_ADDR, data, 1));
    
    //BM280_print_conf();
   // BM280_print_callib_data();

    // temperature = BM280_read_temp();
    // uncomp_data.temperature = temperature;
    // tmp = BMP280_compensate_T();

    

    
       // printf("Temperature: %f \n ", tmp);
    
   // ESP_LOGI(TAG, "Temperature_Uncompensate = %X", temperature);
   // ESP_LOGI(TAG, "Temperature = %f", tmp);

        vTaskDelay(200);
   // 
    // /* Demonstrate writing by reseting the MPU9250 */
    // ESP_ERROR_CHECK(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

    


    // ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM)); 
    }
}



