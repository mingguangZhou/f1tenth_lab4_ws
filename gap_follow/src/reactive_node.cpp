#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

// to facilitate more intuitive and readable expressions for time durations
using namespace std::chrono_literals;

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// create ROS subscribers and publishers
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, std::bind(&ReactiveFollowGap::scan_callback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "ego_racecar/odom", 10, std::bind(&ReactiveFollowGap::drive_callback, this, std::placeholders::_1));
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

        // Timer for control function at 10Hz
        timer_ = this->create_wall_timer(100ms, std::bind(&ReactiveFollowGap::control_callback, this));

        RCLCPP_INFO(this->get_logger(), "Reactive Node has been started");
    }

private:
    double veh_half_width_ = 0.15; // 296mm wide for Traxxas Slash 4x4 Premium Chassis
    double veh_wheelbase_ = 0.32; //
    double dt = 0.1;
    double speed_curr = 0.0;

    /// create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    // Timer for control callback
    rclcpp::TimerBase::SharedPtr timer_;

    // Mutex to protect access to the latest scan data
    std::mutex scan_mutex_;
    
    // Variable to store the latest scan data
    sensor_msgs::msg::LaserScan latest_scan_;
    

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
            for (int j = i; j < i + 5 && j < size; ++j) {
                ranges[j] = average;
            }
        }
    }

    void find_max_gap(float* ranges, int* indice, float angle_increment_rad)
    {   
        // find the closest point
        float min_value = ranges[0];
        int min_index = 0;
        int size = ranges.size();

        for (int i = 1; i < size; i++) {
            if (ranges[i] < min_value) {
                min_value = ranges[i];
                min_index = i;
            }
        }

        // create the 'bubble' around
        float bubble_ang_rad = atan2f(static_cast<float>(veh_half_width_),min_value);
        int bubble_range_index = static_cast<int>(bubble_ang_rad / angle_increment_rad);

        for (int i = min_index-bubble_range_index; i <= min_index+bubble_range_index && i < size; ++!) {
            ranges[i] = 0.0f;
        }

        // find the longest gap range
        int gap_1_lb = 0;
        int gap_1_rb = min_index - bubble_range_index - 1;
        while (ranges[gap_1_rb]==0.0f && gap_1_rb > gap_1_lb) {
           gap_1_rb--; 
        }
        
        int gap_2_rb = size - 1;
        int gap_2_lb = min_index + bubble_range_index + 1;
        while (ranges[gap_2_lb]==0.0f && gap_1_rb<gap_1_lb) {
           gap_2_lb++; 
        }

        int gap_1_len = gap_1_rb - gap_1_lb;
        int gap_2_len = gap_2_rb - gap_2_lb;

        if (gap_2_len > gap_1_len) {
            indice[0] = gap_2_lb;
            indice[1] = gap_2_rb;
        } else {
            indice[0] = gap_1_lb;
            indice[1] = gap_1_rb;
        }
    }

    int find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        float max_dist = ranges[indice[0]];
        int max_dist_index = indice[0];

        for (int i = indice[0]+1; i <= indice[1]; i++) {
            if (ranges[i] > max_dist) {
                max_dist = ranges[i];
                max_dist_index = i;
            }
        }
        
        return max_dist_index;
    }

    void control_command(float angle_error, double speed, double time_diff)
    {
        // Based on the calculated error, publish vehicle control
        double yaw_rate_desired = static_cast<double>(angle_error) / time_diff;
        double steering_angle = (speed > 0.0) ? atan(yaw_rate_desired * veh_wheelbase_ / speed) : 0.0;
        double angle_deg = steering_angle * (180.0 / M_PI);
        
        // generate desired velocity from steering angle
        double velocity;
        if (fabs(angle_deg)>=0.0 && fabs(angle_deg)<=10.0) {
            velocity = 1.5;
        } else if (fabs(angle_deg)>10.0 && fabs(angle_deg)<=20.0) {
            velocity = 1.0;
        } else {
            velocity = 0.5;
        }

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // fill in drive message and publish
        drive_msg.drive.speed = static_cast<float>(velocity);
        drive_msg.drive.steering_angle = static_cast<float>(steering_angle);
        // drive_msg.drive.steering_angle = 0.0;
        drive_publisher_ ->publish(drive_msg);
        RCLCPP_INFO(this->get_logger(), "steering command: %f [deg]", angle_deg);
    }


    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// update current speed
        speed_curr = msg->twist.twist.linear.x;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Store the latest scan data
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = *scan_msg;
    }


    void control_callback()
    {
        // Lock the mutex to safely access the latest scan data
        std::lock_guard<std::mutex> lock(scan_mutex_);
    
        // With fixed rate
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        // Find closest point to LiDAR

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 

        // Find the best point in the gap 

        // Publish Drive message

        if (!latest_scan_.ranges.empty())
        {
            preprocess_lidar(latest_scan_.ranges);
            int gap_indice[2] = {0, latest_scan_.ranges.size()};
            // Find max length gap 
            find_max_gap(latest_scan_.ranges, gap_indice, latest_scan_.angle_increment);
            // Find the best point in the gap
            int goal_index = find_best_point(latest_scan_.ranges, gap_indice);
            
            // Get the error between vehicle heading and goal direction
            float delta_theta = latest_scan_.ranges[goal_index];
            RCLCPP_INFO(this->get_logger(), "goal angel: %f [deg]", delta_theta * (180 / M_PI));

            // Publish Drive message
            control_command(delta_theta, speed_curr, dt)
        }

    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}