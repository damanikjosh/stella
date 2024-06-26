#include "listener.hpp"
#include "MW_serial.hpp"
#include <math.h>

static bool RUN = false;

enum axis
{
  x = 0,
  y = 1,
  z = 2
};

void *MwAhrsRead(void *arg)
{

    while (RUN)
    {
        unsigned char data[8];

        if (AHRS_Read(data))
        {

           switch ((int)(unsigned char)data[1])
            {
            case ACC:
                acc_value[axis::x] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 1000.0;
                acc_value[axis::y] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 1000.0;
                acc_value[axis::z] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 1000.0;
                break;

            case GYO:
                gyr_value[axis::x] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 10.0;
                gyr_value[axis::y] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 10.0;
                gyr_value[axis::z] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 10.0;
                break;

            case DEG:
                deg_value[axis::x] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 100.0;
                deg_value[axis::y] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 100.0;
                deg_value[axis::z] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 100.0;
                break;

            case MAG:
                mag_value[axis::x] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 10.0;
                mag_value[axis::y] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 10.0;
                mag_value[axis::z] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 10.0;
                break;

            }
        }
    }
}


MwAhrsRosDriver::MwAhrsRosDriver()
{

    n.getParam("linear_acceleration_stddev", linear_acceleration_stddev_);
    n.getParam("angular_velocity_stddev", angular_velocity_stddev_);
    n.getParam("magnetic_field_stddev", magnetic_field_stddev_);
    n.getParam("orientation_stddev", orientation_stddev_);

//    imu_pub_ = n.advertise<sensor_msgs::Imu>("imu", 10);
    imu_data_pub_ = n.advertise<sensor_msgs::Imu>("imu", 10);
    imu_data_raw_pub_ = n.advertise<sensor_msgs::Imu>("imu/raw", 10);
    imu_mag_pub_ = n.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
    imu_yaw_pub_ = n.advertise<std_msgs::Float64>("imu/yaw", 10);

    timer = n.createTimer(ros::Duration(0.02),&MwAhrsRosDriver::serial_callback,this);
}

MwAhrsRosDriver::~MwAhrsRosDriver()
{
  MW_Serial_DisConnect();
  run_bool = false;
  sleep(1);
  pthread_join(tid, NULL);
}


void MwAhrsRosDriver::serial_callback(const ros::TimerEvent& event)
{
  if(RUN)
  {

//    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    unsigned char data[8];

    if (AHRS_Read(data))
    {

       switch ((int)(unsigned char)data[1])
        {
        case ACC:
            acc_value[axis::x] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 1000.0;
            acc_value[axis::y] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 1000.0;
            acc_value[axis::z] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 1000.0;
            break;

        case GYO:
            gyr_value[axis::x] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 10.0;
            gyr_value[axis::y] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 10.0;
            gyr_value[axis::z] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 10.0;
            break;

        case DEG:
            deg_value[axis::x] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 100.0;
            deg_value[axis::y] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 100.0;
            deg_value[axis::z] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 100.0;
            break;

        case MAG:
            mag_value[axis::x] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 10.0;
            mag_value[axis::y] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 10.0;
            mag_value[axis::z] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 10.0;
            break;

        }
    }


    auto imu_data_raw_msg = sensor_msgs::Imu();
    auto imu_data_msg = sensor_msgs::Imu();
    auto imu_magnetic_msg = sensor_msgs::MagneticField();
    auto imu_yaw_msg = std_msgs::Float64();

//    double linear_acceleration_cov = linear_acceleration_stddev_ * linear_acceleration_stddev_;
//    double angular_velocity_cov = angular_velocity_stddev_ * angular_velocity_stddev_;
//    double magnetic_field_cov = magnetic_field_stddev_ * magnetic_field_stddev_;
//    double orientation_cov = orientation_stddev_ * orientation_stddev_;

    double linear_acceleration_cov = 0.02 * 0.02;
    double angular_velocity_cov = 0.01 * 0.01;
    double magnetic_field_cov = 0.00000327486 * 0.00000327486;
    double orientation_cov = 0.00125 * 0.00125;

    imu_data_raw_msg.linear_acceleration_covariance[0] =
    imu_data_raw_msg.linear_acceleration_covariance[4] =
    imu_data_raw_msg.linear_acceleration_covariance[8] =
    imu_data_msg.linear_acceleration_covariance[0] =
    imu_data_msg.linear_acceleration_covariance[4] =
    imu_data_msg.linear_acceleration_covariance[8] =
    linear_acceleration_cov;

    imu_data_raw_msg.angular_velocity_covariance[0] =
    imu_data_raw_msg.angular_velocity_covariance[4] =
    imu_data_raw_msg.angular_velocity_covariance[8] =
    imu_data_msg.angular_velocity_covariance[0] =
    imu_data_msg.angular_velocity_covariance[4] =
    imu_data_msg.angular_velocity_covariance[8] =
    angular_velocity_cov;

    imu_data_msg.orientation_covariance[0] =
    imu_data_msg.orientation_covariance[4] =
    imu_data_msg.orientation_covariance[8] =
    orientation_cov;

    imu_magnetic_msg.magnetic_field_covariance[0] =
    imu_magnetic_msg.magnetic_field_covariance[4] =
    imu_magnetic_msg.magnetic_field_covariance[8] =
    magnetic_field_cov;

    double roll, pitch, yaw;

    roll = deg_value[axis::x] * convertor_d2r;
    pitch = mag_value[axis::y] * convertor_d2r;
    yaw = deg_value[axis::z] * convertor_d2r;

//    tf2::Quaternion tf_orientation = Euler2Quaternion(roll, pitch, yaw);
    tf2::Quaternion tf_orientation;

    tf_orientation.setRPY(roll, pitch, yaw);

    current_time = ros::Time::now();

    imu_data_raw_msg.header.stamp = imu_data_msg.header.stamp =
        imu_magnetic_msg.header.stamp = current_time;

    //imu_data_raw_msg.header.frame_id = imu_data_msg.header.frame_id =
        //imu_magnetic_msg.header.frame_id = frame_id_;
    imu_data_raw_msg.header.frame_id = imu_data_msg.header.frame_id =
        imu_magnetic_msg.header.frame_id = imu.header.frame_id = "imu_link";

    // orientation
    imu_data_msg.orientation.x = tf_orientation.x();
    imu_data_msg.orientation.y = tf_orientation.y();
    imu_data_msg.orientation.z = tf_orientation.z();
    imu_data_msg.orientation.w = tf_orientation.w();

    // original data used the g unit, convert to m/s^2
    imu_data_raw_msg.linear_acceleration.x = imu_data_msg.linear_acceleration.x =
        acc_value[axis::x] * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.y = imu_data_msg.linear_acceleration.y =
        acc_value[axis::y] * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.z = imu_data_msg.linear_acceleration.z =
        acc_value[axis::z] * convertor_g2a;

    // original data used the degree/s unit, convert to radian/s
    imu_data_raw_msg.angular_velocity.x = imu_data_msg.angular_velocity.x =
        gyr_value[axis::x] * convertor_d2r;
    imu_data_raw_msg.angular_velocity.y = imu_data_msg.angular_velocity.y =
        gyr_value[axis::y] * convertor_d2r;
    imu_data_raw_msg.angular_velocity.z = imu_data_msg.angular_velocity.z =
        gyr_value[axis::z] * convertor_d2r;

    // original data used the uTesla unit, convert to Tesla
    imu_magnetic_msg.magnetic_field.x = mag_value[axis::x] / convertor_ut2t;
    imu_magnetic_msg.magnetic_field.y = mag_value[axis::y] / convertor_ut2t;
    imu_magnetic_msg.magnetic_field.z = mag_value[axis::z] / convertor_ut2t;

    // original data used the celsius unit
    imu_yaw_msg.data = deg_value[axis::z];


    imu_data_raw_pub_.publish(std::move(imu_data_raw_msg));
    imu_data_pub_.publish(std::move(imu_data_msg));
    imu_mag_pub_.publish(std::move(imu_magnetic_msg));
    imu_yaw_pub_.publish(std::move(imu_yaw_msg));

//    imu.header.frame_id = "imu_link";
//    imu.header.stamp = ros::Time::now();
//    imu_pub_.publish(imu);


    if (publish_tf_)
    {
      geometry_msgs::TransformStamped tf;
      tf.header.stamp = last_time;
      tf.header.frame_id = parent_frame_id_;
      tf.child_frame_id = frame_id_;
      tf.transform.translation.x = 0.0;
      tf.transform.translation.y = 0.0;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = imu_data_msg.orientation;

      odom_broadcaster.sendTransform(tf);
    }

  }
}


  tf2::Quaternion MwAhrsRosDriver::Euler2Quaternion(float roll, float pitch, float yaw)
  {
    float qx = (sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) -
               (cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2));
    float qy = (cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2)) +
               (sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2));
    float qz = (cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2)) -
               (sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2));
    float qw = (cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) +
               (sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2));

    tf2::Quaternion q(qx, qy, qz, qw);
    return q;
  }


int main(int argc, char **argv)
{
    ros::init(argc, argv,"stella_ahrs_node");
    //char *port = "/dev/AHRS";
    char port[] = "/dev/AHRS";
    MW_AHRS_Serial_Connect(port, 115200,3);
    /*
        <ntrex::MwAhrsRosDriver>(port_name, baudrate, sel)

        sel 0 - 기능사용하지 않음
        sel 1 - Z축 캘리브레이션
        sel 2 - 각도리셋
        sel 3 - Z축 캘리브레이션및 각도리셋

    */
    RUN = true;

    MwAhrsRosDriver node;

    ros::spin();

    return 0;
}
