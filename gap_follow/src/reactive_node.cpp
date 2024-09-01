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

        // Declare parameters with default values
        this->declare_parameter<double>("kp", 0.0);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.0);
        this->declare_parameter<double>("ttc_threshold_sec", 0.0);
        this->declare_parameter<double>("side_limit_m", 0.0);


        // Get parameters from the parameter server (YAML file)
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();

        ttc_threshold_sec_ = this->get_parameter("ttc_threshold_sec").as_double();
        side_limit_m_ = this->get_parameter("side_limit_m").as_double();

        RCLCPP_INFO(this->get_logger(), "PID parameters loaded: kp=%f, ki=%f, kd=%f", kp_, ki_, kd_);
        RCLCPP_INFO(this->get_logger(), "Side Protection parameters loaded: ttc_threshold_sec=%f, side_limit_m=%f",
                    ttc_threshold_sec_, side_limit_m_);
    }

private:
    double veh_half_width_ = 0.15; // 296mm wide for Traxxas Slash 4x4 Premium Chassis
    double veh_wheelbase_ = 0.32; //
    double dt = 0.1;
    double speed_curr = 0.0;

    bool AEB_on_ = false; // AEB flag

    // PID CONTROL PARAMS
    double kp_;
    double ki_;
    double kd_;

    // Protection PARAMS
    double ttc_threshold_sec_;
    double side_limit_m_;

    double prev_error = 0.0;
    double integral = 0.0;

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
    

    void preprocess_lidar(std::vector<float>& ranges)
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

    void find_max_gap(std::vector<float>& ranges, int* indice, float angle_increment_rad)
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

        for (int i = min_index-bubble_range_index; i <= min_index+bubble_range_index && i < size; ++i) {
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

    int find_best_point(std::vector<float>& ranges, int* indice)
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

    bool emergency_brake(std::vector<float>& ranges, float angle_min, float angle_increment)
    {
        int size = ranges.size();
        std::vector<float> scan_angles(size);
        for (int i = 0; i < size; ++i) 
        {
            scan_angles[i] = angle_min + i * angle_increment;
        }

        // Get the array of range rates
        std::vector<float> range_rates(size);
        for (int i = 0; i < size; ++i) 
        {
            range_rates[i] = std::max(speed_curr * std::cos(scan_angles[i]), 0.0);
        }

        // Get the TTC array
        std::vector<float> TTC_array(size);
        for (int i = 0; i < size; ++i) {
            if (std::isnan(ranges[i])) {
                TTC_array[i] = std::numeric_limits<float>::infinity();
            } else {
                TTC_array[i] = (range_rates[i] != 0) ? ranges[i] / range_rates[i] : std::numeric_limits<float>::infinity();
            }
        }

        /// publish drive/brake message
        float min_value = TTC_array[0];
        int min_index = 0;

        for (int i = 1; i < size; i++) {
            if (TTC_array[i] < min_value) {
                min_value = TTC_array[i];
                min_index = i;
            }
        }

        if (min_value <= static_cast<float>(ttc_threshold_sec_))
        {
            float AEB_angle = (angle_min + min_index * (angle_increment)) * (180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), "AEB on with TTC:  %f [sec] at the angle %f [deg]", min_value, AEB_angle);
            return true;
        } else {
            return false;
        }
    }

    bool side_protection(std::vector<float>& ranges, float angle_min, float angle_increment)
    {
        // set up a scan angels array
        int size = ranges.size();
        std::vector<float> scan_angles(size);
        for (int i = 0; i < size; ++i) 
        {
            scan_angles[i] = angle_min + i * angle_increment;
        }

        for (int i = 0; scan_angles[i]<(-M_PI/2); i++) 
        {
            if (!std::isnan(ranges[i]) && (ranges[i]>0.0) && (ranges[i] <= side_limit_m_)) 
            {
                RCLCPP_INFO(this->get_logger(), "Too close at %f [deg] with dist %f [m]", 
                            scan_angles[i]*(180.0 / M_PI), ranges[i]);
                return true;
            }
        }

        for (int j = size - 1; scan_angles[j]>(M_PI/2); j--) 
        {
            if (!std::isnan(ranges[j]) && (ranges[j]>0.0) && (ranges[j] <= side_limit_m_))
            {
                RCLCPP_INFO(this->get_logger(), "Too close at %f [deg] with dist %f [m]", 
                            scan_angles[j]*(180.0 / M_PI), ranges[j]);
                return true;
            }
        }

        return false;
    }

    void control_command(float angle_error, double time_diff, bool side_protect_need)
    {   
        // Based on the calculated error, publish vehicle control
        // double yaw_rate_desired = static_cast<double>(angle_error);
        // double steering_angle = (speed > 0.0) ? atan(yaw_rate_desired * veh_wheelbase_ / speed) : 0.0;
        // double steering_angle = yaw_rate_desired;

        /// TODO: add side protection

        // Using PID to generate steering angle command
        double steering_angle = kp_ * angle_error + ki_ * integral + kd_ * (angle_error - prev_error) / time_diff;

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
        if (side_protect_need)
        {
            drive_msg.drive.steering_angle = 0.0;
            RCLCPP_INFO(this->get_logger(), "side protection on, no steering!");
        } else {
            drive_msg.drive.steering_angle = static_cast<float>(steering_angle);
            RCLCPP_INFO(this->get_logger(), "steering command: %f [deg]", angle_deg);
        }
        drive_publisher_ ->publish(drive_msg);
        
        RCLCPP_INFO(this->get_logger(), "speed command: %f m/s", velocity);

        integral += angle_error * dt;
        prev_error = angle_error;
    }


    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// update current speed
        speed_curr = msg->twist.twist.linear.x;
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
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

        bool activate_AEB = emergency_brake(latest_scan_.ranges, latest_scan_.angle_min, latest_scan_.angle_increment);
        if (!latest_scan_.ranges.empty() && !AEB_on_ && !activate_AEB)
        {
            preprocess_lidar(latest_scan_.ranges);
            int gap_indice[2] = {0, static_cast<int>(latest_scan_.ranges.size())};
            // Find max length gap 
            find_max_gap(latest_scan_.ranges, gap_indice, latest_scan_.angle_increment);
            // Find the best point in the gap
            int goal_index = find_best_point(latest_scan_.ranges, gap_indice);
            
            // Get the error between vehicle heading and goal direction
            float delta_theta = latest_scan_.angle_min + goal_index * latest_scan_.angle_increment;
            RCLCPP_INFO(this->get_logger(), "goal angel: %f [deg]", delta_theta * (180 / M_PI));

            // Publish Drive message
            bool activate_side_protect = side_protection(latest_scan_.ranges, latest_scan_.angle_min,
                                                         latest_scan_.angle_increment);
            control_command(delta_theta, dt, activate_side_protect);
        } else {
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.speed = 0.0;
            drive_msg.drive.steering_angle = 0.0;
            drive_publisher_ ->publish(drive_msg);
            AEB_on_ = true;
        }

    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}