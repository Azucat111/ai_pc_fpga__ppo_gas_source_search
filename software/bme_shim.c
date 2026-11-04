#include <stdint.h>
#include <string.h>
#include "bme68x.h"

/* One BME688 per process. Keep vendor compensation and calibration unchanged. */
static struct bme68x_dev dev;
static struct bme68x_conf conf;
static struct bme68x_heatr_conf heater;
struct car_bme_sample { float temperature, humidity, pressure, gas; uint32_t status; };

int car_bme_init(bme68x_read_fptr_t read_cb, bme68x_write_fptr_t write_cb,
                 bme68x_delay_us_fptr_t delay_cb) {
    int8_t rc;
    memset(&dev,0,sizeof(dev));memset(&conf,0,sizeof(conf));memset(&heater,0,sizeof(heater));
    dev.intf=BME68X_I2C_INTF;dev.read=read_cb;dev.write=write_cb;dev.delay_us=delay_cb;
    dev.amb_temp=25;
    rc=bme68x_init(&dev);if(rc!=BME68X_OK)return rc;
    conf.filter=BME68X_FILTER_SIZE_3;conf.odr=BME68X_ODR_NONE;
    conf.os_hum=BME68X_OS_2X;conf.os_pres=BME68X_OS_2X;conf.os_temp=BME68X_OS_2X;
    rc=bme68x_set_conf(&conf,&dev);if(rc!=BME68X_OK)return rc;
    heater.enable=BME68X_ENABLE;heater.heatr_temp=320;heater.heatr_dur=150;
    return bme68x_set_heatr_conf(BME68X_FORCED_MODE,&heater,&dev);
}

static int read_sample(struct car_bme_sample *out) {
    struct bme68x_data sample={0};uint8_t count=0;int8_t rc;
    if(!out)return BME68X_E_NULL_PTR;
    rc=bme68x_get_data(BME68X_FORCED_MODE,&sample,&count,&dev);
    if(rc!=BME68X_OK)return rc;
    if(!count || !(sample.status&BME68X_NEW_DATA_MSK))return BME68X_W_NO_NEW_DATA;
    out->temperature=sample.temperature;out->humidity=sample.humidity;
    out->pressure=sample.pressure;out->gas=sample.gas_resistance;out->status=sample.status;
    return 0;
}

int car_bme_begin_profile(uint16_t temperature, uint16_t duration_ms) {
    int8_t rc;
    if(temperature<200 || temperature>400 || duration_ms<1 || duration_ms>1000)return BME68X_E_INVALID_LENGTH;
    /* Vendor heater changes must be made while asleep; one PS bus owner only. */
    rc=bme68x_set_op_mode(BME68X_SLEEP_MODE,&dev);if(rc!=BME68X_OK)return rc;
    heater.heatr_temp=temperature;heater.heatr_dur=duration_ms;
    rc=bme68x_set_heatr_conf(BME68X_FORCED_MODE,&heater,&dev);if(rc!=BME68X_OK)return rc;
    rc=bme68x_set_op_mode(BME68X_FORCED_MODE,&dev);if(rc!=BME68X_OK)return rc;
    return (int)(bme68x_get_meas_dur(BME68X_FORCED_MODE,&conf,&dev)+(uint32_t)duration_ms*1000+10000);
}
int car_bme_finish_profile(struct car_bme_sample *out) { return read_sample(out); }
int car_bme_sample(struct car_bme_sample *out) {
    int8_t rc=bme68x_set_op_mode(BME68X_FORCED_MODE,&dev);if(rc!=BME68X_OK)return rc;
    dev.delay_us(bme68x_get_meas_dur(BME68X_FORCED_MODE,&conf,&dev)+(uint32_t)heater.heatr_dur*1000+10000,dev.intf_ptr);
    return read_sample(out);
}
