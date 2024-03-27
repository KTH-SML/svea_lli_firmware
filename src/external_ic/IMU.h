#include <Arduino.h>
#include <string.h>

// Deps needed to interface with the IMU
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "svea_teensy.h"

#include <AK09918.h>
#include <Arduino.h>
#include <ICM20600.h>
#include <Wire.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "SparkFun_BNO080_Arduino_Library.h"

namespace SVEA
{
    class IMU
    {
    private:
        int32_t mag_x, mag_y, mag_z;
        int16_t acc_x, acc_y, acc_z;
        int16_t gyro_x, gyro_y, gyro_z;
        double roll, pitch;

        float GyroCovariance = 1.0e-2;
        float AccCovariance = 1.0e-2;      
        float MagCovariance; 
        float FakeCovariance = 0;

        SVEA::NodeHandle &nh;
        ros::Publisher imu_pub;
        ros::Publisher imu_mag;
        ros::Publisher imu_temp;

        std_msgs::Header header;
        sensor_msgs::Imu imu_msg;
        sensor_msgs::MagneticField mag_msg;
        sensor_msgs::Temperature temp_msg;
        BNO080 bno080;

    public:
        IMU(SVEA::NodeHandle &nh) : nh(nh),
                                    imu_pub("imu/data_raw", &imu_msg),
                                    imu_mag("imu/mag", &mag_msg),
                                    imu_temp("imu/temp", &temp_msg)
        {
            nh.advertise(imu_pub);
            nh.negotiateTopics();
            nh.advertise(imu_mag);
            nh.negotiateTopics();
            nh.advertise(imu_temp);
            nh.negotiateTopics();

            header.frame_id = "imu";
            header.seq = 0;
        }

        void open(){
            Wire1.begin();
            //The first argument is the I2C address of the sensor, either 0x4B (default) or 0x4A
            //The second is the TwoWire I2C port to use. Wire, Wire1, etc.
            bno080.begin(0x4B, Wire1);

            bno080.enableMagnetometer(2); //Send data update every _ ms
            bno080.enableAccelerometer(2); //Send data update every _ ms
            bno080.enableGyro(2); //Send data update every _ ms
            Serial.println(F("Magnetometer enabled"));
        }

        void update()
        {
            if (bno080.dataAvailable() == true){
                // Header stuff
                header.stamp = nh.now();
                header.seq++;

                imu_msg.header = header;
                mag_msg.header = header;
                temp_msg.header = header;

                imu_msg.orientation.x = bno080.getQuatI();
                imu_msg.orientation.y = bno080.getQuatJ();
                imu_msg.orientation.z = bno080.getQuatK();
                imu_msg.orientation.w = bno080.getQuatReal();

                imu_msg.linear_acceleration.x = bno080.getAccelX(); // m/s^2
                imu_msg.linear_acceleration.y = bno080.getAccelY(); // m/s^2
                imu_msg.linear_acceleration.z = bno080.getAccelZ(); // m/s^2

                imu_msg.angular_velocity.x = bno080.getGyroX(); // radian per second
                imu_msg.angular_velocity.y = bno080.getGyroY(); // radian per second
                imu_msg.angular_velocity.z = bno080.getGyroZ(); // radian per second

                mag_msg.magnetic_field.x = bno080.getMagX(); // uTesla
                mag_msg.magnetic_field.y = bno080.getMagY(); // uTesla
                mag_msg.magnetic_field.z = bno080.getMagZ(); // uTesla
                // Serial.print(bno080.getMagX());
                // Serial.print(" ,");
                // Serial.print(bno080.getMagY());
                // Serial.print(" ,");
                // Serial.print(bno080.getMagZ());
                // Serial.print(" ,");
                // Serial.print(bno080.getAccelX());
                // Serial.print(" ,");
                // Serial.print(bno080.getAccelY());
                // Serial.print(" ,");
                // Serial.print(bno080.getAccelZ());
                // Serial.print(" ,");
                // Serial.print(bno080.getGyroX());
                // Serial.print(" ,");
                // Serial.print(bno080.getGyroY());
                // Serial.print(" ,");
                // Serial.println(bno080.getGyroZ());
            
                MagCovariance = abs(4 - bno080.getMagAccuracy());

                for (int i = 0; i < 9; ++i)
                {
                    imu_msg.orientation_covariance[i] = FakeCovariance;
                    imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? GyroCovariance : 0;
                    imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? AccCovariance : 0;
                    mag_msg.magnetic_field_covariance[i] = (i % 4 == 0) ? MagCovariance : 0;
                }

                imu_pub.publish(&imu_msg);
                imu_mag.publish(&mag_msg);
                imu_temp.publish(&temp_msg);
            }
        }
    };
} // namespace SVEA