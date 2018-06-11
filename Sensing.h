/* PID HEREDA O ABSTRAE ESTA CLASE */

#ifndef SENSING_H
#define SENSING_H
#endif

#include "XNucleoIKS01A2.h"

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);
 
#define  sens 70
#define G 0.00980665
 
static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGRAccSensor *accelerometer = mems_expansion_board->accelerometer;


#define PULSES_PER_REVOLUTION 624

class MyImu 
{
public:
	MyImu( NodeHandle nh );
	Publish_imu();
	Publish_mag();
	Publish_env();

private:
	sensor_msgs::Imu imu_;
    sensor_msgs::MagneticField imu_mag_;

    uint8_t id;
    int32_t axes[3];
    int32_t off[3];
    float parziale[3];
    float finale[3];
    int32_t k=0;
};

