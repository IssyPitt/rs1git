#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserScanProcessor3 : public rclcpp::Node
{
public:
    LaserScanProcessor3()
        : Node("tut_3_node")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanProcessor3::scanCallback, this, std::placeholders::_1)); // subscribe to scan topic

        scan_pub_1_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_reduced", 10); // create publisher
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        int n = 5;

        auto subset_scan_1 = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);

        subset_scan_1->ranges.clear();                             // clear ranges
        for (size_t i = 0; i < scan->ranges.size(); i += n) {      //for i = 0, i less than size of ranges, i + n = n (for every nth point)
            subset_scan_1->ranges.push_back(scan->ranges[i]);      // push new ranges into ranges
        }                                                          // Filter the ranges array to include only every n_th point

        subset_scan_1->angle_increment = scan->angle_increment * n;  // Adjust angle_increment for reduced no. of points i * n
        subset_scan_1->angle_min = scan->angle_min;                 // Update other scan stuff that might depend on no. ranges
        subset_scan_1->angle_max = scan->angle_min + (subset_scan_1->ranges.size() - 1) * subset_scan_1->angle_increment; // max is minumum + one less range size * increment)
        subset_scan_1->scan_time = scan->scan_time;                // time is the same
        subset_scan_1->time_increment = scan->time_increment * n;  // time increment is i * n

        scan_pub_1_->publish(*subset_scan_1); // publish
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; // subscrier initialisation
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_1_;  // publisher initialisation
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor3>());
    rclcpp::shutdown();
    return 0;
}
