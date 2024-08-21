#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// create ROS subscribers and publishers
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, std::bind(&ReactiveFollowGap::scan_callback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "ego_racecar/odom", 10, std::bind(&ReactiveFollowGap::drive_callback, this, std::placeholders::_1));
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        RCLCPP_INFO(this->get_logger(), "Reactive Node has been started");
    }

private:
    double veh_half_width = 0.15; // 296mm wide for Traxxas Slash 4x4 Premium Chassis


    /// create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    

    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        // Proprocessing to average every 5 beams
        int size = ranges.size();

        // Step 1: Convert NaNs and infinity values to 0.0
        for (int i = 0; i < size; ++i) {
            if (std::isnan(ranges[i]) || std::isinf(ranges[i])) {
                ranges[i] = 0.0f;
            }
        }

        // Step 2: Average each 5 beams, ignoring 0.0 values, and replace all 5 beams with the average
        for (int i = 0; i < size; i += 5) {
            float sum = 0.0f;
            int count = 0;
            
            // Calculate the sum and count of non-zero values in the group
            for (int j = i; j < i + 5 && j < size; ++j) {
                if (ranges[j] != 0.0f) {
                    sum += ranges[j];
                    ++count;
                }
            }
            
            // Compute the average, if there are non-zero values
            float average = (count > 0) ? sum / count : 0.0f;
            
            // Replace all values in the current group with the average
            for (size_t j = i; j < i + 5 && j < size; ++j) {
                ranges[j] = average;
            }
        }

        return;
    }

    void find_max_gap(float* ranges, int* indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        return;
    }

    void find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        return;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 

        // Find the best point in the gap 

        // Publish Drive message
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}