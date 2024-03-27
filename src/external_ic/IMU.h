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

namespace SVEA {
class IMU {
private:
    BNO080 bno080;
    int32_t x, y, z;
    int32_t offset_x, offset_y, offset_z;
    // ACCELEROMETER PARAMS (ICM20600)
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    double roll, pitch;

    // Find the magnetic declination at your location
    // http://www.magnetic-declination.com/
    double declination = 0;

    SVEA::NodeHandle &nh;
    ros::Publisher imu_pub;
    ros::Publisher imu_mag;
    ros::Publisher imu_temp;

    std_msgs::Header header;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;
    sensor_msgs::Temperature temp_msg;
    struct euler_angles {
        float roll;
        float pitch;
        float yaw;
    };

    void calibrate(uint32_t timeout, int32_t *offsetx, int32_t *offsety, int32_t *offsetz) {
        int32_t value_x_min = -33;
        int32_t value_x_max = 52;
        int32_t value_y_min = -44;
        int32_t value_y_max = 52;
        int32_t value_z_min = -67;
        int32_t value_z_max = 21;
        uint32_t timeStart = 0;

        // ak09918.getData(&x, &y, &z);

        // value_x_min = x;
        // value_x_max = x;
        // value_y_min = y;
        // value_y_max = y;
        // value_z_min = z;
        // value_z_max = z;
        // delay(100);

        // timeStart = millis();
        // Serial.begin(9600);
        // while ((millis() - timeStart) < timeout) {
        //     ak09918.getData(&x, &y, &z);

        //     /* Update x-Axis max/min value */
        //     if (value_x_min > x) {
        //         value_x_min = x;
        //         Serial.print("Update value_x_min: ");
        //         Serial.println(value_x_min);

        //     } else if (value_x_max < x) {
        //         value_x_max = x;
        //         Serial.print("update value_x_max: ");
        //         Serial.println(value_x_max);
        //     }

        //     /* Update y-Axis max/min value */
        //     if (value_y_min > y) {
        //         value_y_min = y;
        //         Serial.print("Update value_y_min: ");
        //         Serial.println(value_y_min);

        //     } else if (value_y_max < y) {
        //         value_y_max = y;
        //         Serial.print("update value_y_max: ");
        //         Serial.println(value_y_max);
        //     }

        //     /* Update z-Axis max/min value */
        //     if (value_z_min > z) {
        //         value_z_min = z;
        //         Serial.print("Update value_z_min: ");
        //         Serial.println(value_z_min);

        //     } else if (value_z_max < z) {
        //         value_z_max = z;
        //         Serial.print("update value_z_max: ");
        //         Serial.println(value_z_max);
        //     }

        //     Serial.print(".");
        //     delay(100);
        // }

        *offsetx = value_x_min + (value_x_max - value_x_min) / 2;
        *offsety = value_y_min + (value_y_max - value_y_min) / 2;
        *offsetz = value_z_min + (value_z_max - value_z_min) / 2;
    }
    struct euler_angles calculateEuler() {
        struct euler_angles angles;
        angles.roll = 57.3 * atan2((float)acc_y, (float)acc_z);
        angles.pitch = 57.3 * atan2(-(float)acc_x, sqrt((float)acc_y * acc_y + (float)acc_z * acc_z));

        float Xheading = x * cos(angles.pitch) + y * sin(angles.roll) * sin(angles.pitch) + z * cos(angles.roll) * sin(angles.pitch);
        float Yheading = y * cos(angles.roll) - z * sin(angles.pitch);
        angles.yaw = 180 + 57.3 * atan2(Yheading, Xheading) + declination;
        return angles;
    }
    struct Quaternion {
    float w, x, y, z;
    };

    Quaternion eulerToQuaternion(float roll, float pitch, float yaw) {
        Quaternion q;
        float cy = cos(yaw * 0.5);
        float sy = sin(yaw * 0.5);
        float cp = cos(pitch * 0.5);
        float sp = sin(pitch * 0.5);
        float cr = cos(roll * 0.5);
        float sr = sin(roll * 0.5);

        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }
public:
    IMU(SVEA::NodeHandle &nh) : nh(nh),
                                imu_pub("imu/data_raw", &imu_msg),
                                imu_mag("imu/mag", &mag_msg),
                                imu_temp("imu/temp", &temp_msg) {
        nh.advertise(imu_pub);
        nh.negotiateTopics();
        nh.advertise(imu_mag);
        nh.negotiateTopics();
        nh.advertise(imu_temp);
        nh.negotiateTopics();

        header.frame_id = "imu";
        header.seq = 0;
    }

    bool open() {
        icm20600.initialize();
        ak09918.initialize();
        err = ak09918.isDataReady();
        boolean succ = err == AK09918_ERR_OK;
        if (succ) {
            Serial.println("IMU detected");
            // header.frame_id = "imu-calibrating";
            calibrate(10000, &offset_x, &offset_y, &offset_z);
            header.frame_id = "imu";
        } else {
            Serial.println("WHOOPSIE ERROR: #" + String(err));
            // header.frame_id = "imu-error";
        }
        return succ;
    }

    void update() {
        // Header stuff
        header.stamp = nh.now();
        header.seq++;

        imu_msg.header = header;
        mag_msg.header = header;
        temp_msg.header = header;

        acc_x = icm20600.getAccelerationX();
        acc_y = icm20600.getAccelerationY();
        acc_z = icm20600.getAccelerationZ();

        gyro_x = icm20600.getGyroscopeX();
        gyro_y = icm20600.getGyroscopeY();
        gyro_z = icm20600.getGyroscopeZ();

        ak09918.getData(&x, &y, &z);
        x = x - offset_x;
        y = y - offset_y;
        z = z - offset_z;

        // TODO, make more efficient or make sensible covariance, or both
        euler_angles euler;
        euler = calculateEuler();
        euler.roll = euler.roll/57.3;
        euler.pitch = euler.pitch/57.3;
        euler.yaw = euler.yaw/57.3;

        // imu_msg.orientation = tf::createQuaternionFromYaw(euler.yaw);
        
        // Quaternion q = eulerToQuaternion(x, y, z);
        Quaternion q = eulerToQuaternion(euler.roll, euler.pitch, euler.yaw);
        // Serial.begin(9600);
        // Serial.print(euler.roll);
        // Serial.print(", ");
        // Serial.print(euler.pitch);
        // Serial.print(", ");
        // Serial.print(euler.yaw);
        // Serial.print(", ");
        // Serial.print(q.x);
        // Serial.print(", ");
        // Serial.print(q.y);
        // Serial.print(", ");
        // Serial.print(q.z);
        // Serial.print(", ");
        // Serial.println(q.w);

        
        imu_msg.orientation.x = q.x;
        imu_msg.orientation.y = q.y;
        imu_msg.orientation.z = q.z;
        imu_msg.orientation.w = q.w;

        imu_msg.linear_acceleration.x = acc_x * 9.81/1000; // milligal to m/s^2
        imu_msg.linear_acceleration.y = acc_y * 9.81/1000; // milligal to m/s^2
        imu_msg.linear_acceleration.z = acc_z * 9.81/1000; // milligal to m/s^2   

        imu_msg.angular_velocity.x = gyro_x * 0.0174532925199; //from degree per second to radian per second
        imu_msg.angular_velocity.y = gyro_y * 0.0174532925199; //from degree per second to radian per second
        imu_msg.angular_velocity.z = gyro_z * 0.0174532925199; //from degree per second to radian per second

        mag_msg.magnetic_field.x = x;
        mag_msg.magnetic_field.y = y;
        mag_msg.magnetic_field.z = z;

        temp_msg.temperature = icm20600.getTemperature();
        float GyroCovariance = 1.0e-3; //((1/131) * (3.141592653589793 / 180))*((1/131) * (3.141592653589793 / 180)); (from data sheet)
        float AccCovariance = 1.0e-3; //(1/16384*9.81)*(1/16384*9.81); (from data sheet)
        float MagCovariance = 0.15 * 0.15;  //(from data sheet)
        float FakeCovariance = 0;
        for (int i = 0; i < 9; ++i) {
            imu_msg.orientation_covariance[i] = FakeCovariance;
            imu_msg.angular_velocity_covariance[i] = (i%4 == 0) ? GyroCovariance : 0;
            imu_msg.linear_acceleration_covariance[i] = (i%4 == 0) ? AccCovariance : 0;
            mag_msg.magnetic_field_covariance[i] = (i%4 == 0) ? MagCovariance : 0;
        }

        imu_pub.publish(&imu_msg);
        imu_mag.publish(&mag_msg);
        imu_temp.publish(&temp_msg);
    }
};
} // namespace SVEA