#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iomanip>

class SensorDataProcessor : public rclcpp::Node
{
public:
    SensorDataProcessor() : Node("slo_3_3")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SensorDataProcessor::odomCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&SensorDataProcessor::imuCallback, this, std::placeholders::_1));

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SensorDataProcessor::laserCallback, this, std::placeholders::_1));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        odom_theta_ = getYawFromQuaternion(msg->pose.pose.orientation);

        printSensorData();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Get angular velocity from IMU
        imu_angular_z_ = msg->angular_velocity.z;

        printSensorData();
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Get minimum range value from laser scan
        laser_min_range_ = *std::min_element(msg->ranges.begin(), msg->ranges.end());

        printSensorData();
    }

    void printSensorData()
    {
        // Check if all values are initialized (non-zero) before printing
        if (odom_x_ != 0 && imu_angular_z_ != 0 && laser_min_range_ != 0)
        {
            std::cout << std::fixed << std::setprecision(2)
                      << "Odom: [x: " << odom_x_ << ", y: " << odom_y_ << ", theta: " << odom_theta_ << "] "
                      << "IMU: [angular_z: " << imu_angular_z_ << "] "
                      << "Laser: [min_range: " << laser_min_range_ << "]" << std::endl;
        }
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    // Variables to store sensor data
    double odom_x_ = 0.0, odom_y_ = 0.0, odom_theta_ = 0.0;
    double imu_angular_z_ = 0.0;
    double laser_min_range_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorDataProcessor>());
    rclcpp::shutdown();
    return 0;
}
