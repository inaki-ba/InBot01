#include "Sensing.h"

MyImu::MyImu( NodeHandle nh )
{
    imu_publisher_ = new ros::Publisher("imu/data_raw", &imu);
    mag_publisher_ = new ros::Publisher("imu/mag", &imu_mag);

    nh.advertise(imu_publisher_);
    nh.advertise(mag_publisher_);

    for(int i=0;i<3;i++)
        {parziale[i] = 0;}
 
    for(int i=0;i<3;i++)
        {finale[i] = 0;}

	//pc.printf("\r\n--- Starting new run ---\r\n");

    hum_temp->read_id(&temp_id);
    //pc.printf("HTS221  humidity & temperature    = 0x%X\r\n", id);
    press_temp->read_id(&press_id);
    //pc.printf("LPS22HB  pressure & temperature   = 0x%X\r\n", id);    
	magnetometer->read_id(&mag_id);
    //pc.printf("LSM303AGR magnetometer            = 0x%X\r\n", id);
    accelerometer->read_id(&acc_id);
    //pc.printf("LSM303AGR accelerometer           = 0x%X\r\n", id);
    acc_gyro->read_id(&gyro_id);
    //pc.printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);

      /* Enable all sensors */

    hum_temp->enable();
    press_temp->enable();
    magnetometer->enable();
    accelerometer->enable();
    acc_gyro->enable_x();
    acc_gyro->enable_g();

    acc_gyro->get_g_axes(axes);
    //pc.printf("LSM6DSL [gyro/mdps]:   %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
            
    for(int i=0;i<3;i++) {
      off[i]=axes[i];}
}

MyImu::Publish_env()
{
    //pc.printf("\r\n");

    hum_temp->get_temperature(&value1);
    hum_temp->get_humidity(&value2);
    //pc.printf("HTS221: [temp] %7s C,   [hum] %s%%\r\n", print_double(buffer1, value1), print_double(buffer2, value2));
    
    press_temp->get_temperature(&value1);
    press_temp->get_pressure(&value2);
    //pc.printf("LPS22HB: [temp] %7s C, [press] %s mbar\r\n", print_double(buffer1, value1), print_double(buffer2, value2));

    //pc.printf("---\r\n");
}

MyImu::Publish_mag()
{
	magnetometer->get_m_axes(axes);
    imu_mag.magnetic_field.x = axes[0];
    imu_mag.magnetic_field.y = axes[1];
    imu_mag.magnetic_field.z = axes[2];

    //pc.printf("LSM303AGR [magneto/mgauss]:      %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    wait_ms(50);

    imu_mag.header.stamp = current_time;
    imu_mag.header.frame_id = s_imu;
    //imu_mag.covariance =

    mag_publisher.publish( &imu_mag );
}

MyImu::Publish_imu()
{
	acc_gyro->get_x_axes(axes);
    imu.linear_acceleration.x = axes[0]*G;
    imu.linear_acceleration.y = axes[1]*G;
    imu.linear_acceleration.z = axes[2]*G;

    //pc.printf("LSM6DSL [acc/mg]:      %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    wait_ms(50);
    acc_gyro->get_g_axes(axes);                     
    for(int i=0;i<3;i++)
        {axes[i]=axes[i]-off[i];}
    //pc.printf("LSM6DSLfine [gyro/mdps]:   %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
    k=k+1;

    imu.angular_velocity.x = (float)axes[0]/1000000*17.453293;
    imu.angular_velocity.y = (float)axes[1]/1000000*17.453293;
    imu.angular_velocity.z = (float)axes[2]/1000000*17.453293;

    //pc.printf("LSM6DSLraw [gyro/mdps]:   %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    imu.header.stamp = current_time;
    imu.header.frame_id = s_imu;         
    //imu.angular_velocity_covariance =        
    //imu.linear_acceleration_covariance =

    imu_publisher.publish( &imu );

    // ricavo l'parziale dalla velocità angolare
    
    for(int i=0;i<3;i++) {
        parziale[i]=(axes[i]*sens)/1000;
        parziale[i]/= 1000;

        if (axes[i]>150 ||axes[i]<-150)
            finale[i] += parziale[i];
      
    }
    //pc.printf("finale  [gyro/d]:   %6f, %6f, %6f\r\n", finale[0], finale[1], finale[2]);//angolo
}

