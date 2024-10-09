#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <thread>

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
    : Node("tutorial_6"), angle_difference_(0.0), relative_orientation_(0.0), map_received_(false)
    {
        // Subscribing to map
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

        // Subscribing to laser scan
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapProcessorNode::scanCallback, this, std::placeholders::_1));

        // Subscribing to odometry
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapProcessorNode::odomCallback, this, std::placeholders::_1));

        // Command velocity publisher
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Scanmatch pose publisher
        scanmatch_pose_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/scanmatch_pose", 10);

        cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "Map Processor Node started.");
    }

private:
    // Callback to handle incoming laser scan messages

    cv::Mat Convert_to_Edge_map(cv::Mat Map){
     // Step 1: Convert the image to grayscale if it isn't already
    cv::Mat edge_map;
    cv::Mat gray_image;
    if (Map.channels() == 3) {
        cv::cvtColor(Map, gray_image, cv::COLOR_BGR2GRAY);
    } else {
        gray_image = Map;  // Image is already grayscale
    }
 
    cv::Mat blurred_image;
    cv::GaussianBlur(gray_image, blurred_image, cv::Size(5, 5), 1.5);
 
    cv::Canny(blurred_image, edge_map, 50, 150);  // Thresholds can be adjusted
 
    cv::waitKey(1);  // Wait for a key press to close the window
    return edge_map;
}

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!map_received_) {
            RCLCPP_WARN(this->get_logger(), "Map not received yet. No images will be generated.");
            return;
        }

        cv::Mat img = laserScanToMat(msg);

        if (!first_image_captured_) {
            first_image_ = img.clone();
            first_image_ = resizeImage(first_image_);
            first_image_ = Convert_to_Edge_map(first_image_);
            first_image_captured_ = true;
            cv::imshow("Image A", first_image_);
            cv::waitKey(1);
            rotateRobot();
        } else if (!second_image_captured_) {
            second_image_ = img.clone();
            second_image_ = resizeImage(second_image_);
            second_image_captured_ = true;
            cv::imshow("Image C", second_image_);
            cv::waitKey(1);
            calculateYawChange();
        } else {
            first_image_ = second_image_.clone();
            second_image_ = img.clone();
            second_image_ = resizeImage(second_image_);
            cv::imshow("Image C", second_image_);
            cv::waitKey(1);
            calculateYawChange();
            relative_orientation_ += angle_difference_;

            // Publish relative orientation as odometry
            auto pose_msg = nav_msgs::msg::Odometry();
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, relative_orientation_ * M_PI / 180.0); // Convert degrees to radians
            pose_msg.pose.pose.orientation.x = quaternion.x();
            pose_msg.pose.pose.orientation.y = quaternion.y();
            pose_msg.pose.pose.orientation.z = quaternion.z();
            pose_msg.pose.pose.orientation.w = quaternion.w();
            scanmatch_pose_publisher_->publish(pose_msg);

            RCLCPP_INFO(this->get_logger(), "Odometry: x: %f, y: %f, z: %f, Relative Orientation: %f, Yaw Change: %f degrees", x, y, z, relative_orientation_, angle_difference_);

        }
    }

    // Callback to handle map messages
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        occupancyGridToImage(mapMsg);
        m_MapColImage = resizeImage(m_MapColImage);
        cv::imshow(WINDOW1, m_MapColImage);
        cv::waitKey(1);
        map_received_ = true;
    }

    // Callback to handle odometry messages
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        z = msg->pose.pose.position.z;
    }

    // Process the occupancy grid data and convert to an OpenCV image
    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        int grid_data;
        unsigned int row, col, val;

        m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        for (row = 0; row < grid->info.height; row++) {
            for (col = 0; col < grid->info.width; col++) {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1) {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                } else {
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }

        map_scale_ = grid->info.resolution;
        origin_x = grid->info.origin.position.x;
        origin_y = grid->info.origin.position.y;
        size_x = grid->info.width;
        size_y = grid->info.height;

        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                    0, 1, 0,
                                    0, 0, 0);
        cv::erode(m_temp_img, m_MapBinImage, kernel);

        m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
        cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);

        std::cout << "Occupancy grid map converted to a binary image\n";
    }

    // Convert laser scan data into an OpenCV image
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        return image;
    }

    // Function to rotate the robot
    void rotateRobot() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 1.0;
        cmd_publisher_->publish(twist_msg);

        rclcpp::sleep_for(std::chrono::seconds(2));

        twist_msg.angular.z = 0.0;
        cmd_publisher_->publish(twist_msg);
    }


    // Function to calculate yaw change between two images
    void calculateYawChange() {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            } else {
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

    // Function to detect and match ORB features between two images
    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::Mat(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::Mat(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        for (auto& match : matches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    // Resize an image for display purposes
    cv::Mat resizeImage(const cv::Mat& img) {
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(500, 500));
        return resized_img;
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr scanmatch_pose_publisher_;   
    rclcpp::TimerBase::SharedPtr forward_timer_;


    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;
    std::string WINDOW1 = "Image B";

    double angle_difference_;
    double relative_orientation_;
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;
    cv::Mat first_image_, second_image_;
    double map_scale_;
    double origin_x, origin_y;
    int size_x, size_y;

    bool map_received_;
    double x = 0.0, y = 0.0, z = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
