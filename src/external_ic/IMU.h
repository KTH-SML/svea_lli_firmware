#include "svea_teensy.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Header.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS 10
#define BNO055_ADDRESS BNO055_ADDRESS_A

namespace IMU {
Adafruit_BNO055 bno(55, BNO055_ADDRESS, &Wire1);
ros::NodeHandle *nh_ptr = nullptr;
std_msgs::Header header;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
sensor_msgs::Temperature temp_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);
ros::Publisher imu_mag("imu/mag", &mag_msg);
ros::Publisher imu_temp("imu/temp", &temp_msg);

static unsigned long last_update = 0;

bool init(ros::NodeHandle &nh) {
    nh_ptr = &nh;
    nh.advertise(imu_pub);
    nh.advertise(imu_mag);
    nh.advertise(imu_temp);
    Wire1.begin();
    header.frame_id = "imu";
    bool success = bno.begin();
    if (success)
        bno.setExtCrystalUse(true);
    else {
        Serial.println("BNO055 not detected");
        header.frame_id = "imu-disconnected";
    }
    return success;
}

void update() {
    unsigned long callTime = millis();
    if (callTime - last_update < BNO055_SAMPLERATE_DELAY_MS)
        return;
    last_update = callTime;

    header.stamp = nh_ptr->now();
    header.seq++;

    imu_msg.header = header;
    mag_msg.header = header;
    temp_msg.header = header;

    // Orientation
    imu::Quaternion q = bno.getQuat();
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    // Linear acceleration
    imu::Vector<3> vec = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu_msg.linear_acceleration.x = vec.x();
    imu_msg.linear_acceleration.y = vec.y();
    imu_msg.linear_acceleration.z = vec.z();

    // Angular velocity
    vec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu_msg.angular_velocity.x = vec.x();
    imu_msg.angular_velocity.y = vec.y();
    imu_msg.angular_velocity.z = vec.z();

    // Magnetic field (using Euler vector as in original code)
    vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    mag_msg.magnetic_field.x = vec.x();
    mag_msg.magnetic_field.y = vec.y();
    mag_msg.magnetic_field.z = vec.z();

    temp_msg.temperature = bno.getTemp();

    // Set covariance (example values)
    int fakeCovariance = 0;
    float orientationCovariance = 0.1;
    for (int i = 0; i < 9; ++i) {
        imu_msg.orientation_covariance[i] = (i % 4 == 0) ? orientationCovariance : fakeCovariance;
        imu_msg.angular_velocity_covariance[i] = fakeCovariance;
        imu_msg.linear_acceleration_covariance[i] = fakeCovariance;
        mag_msg.magnetic_field_covariance[i] = fakeCovariance;
    }

    imu_pub.publish(&imu_msg);
    imu_mag.publish(&mag_msg);
    imu_temp.publish(&temp_msg);
}
} // namespace IMU