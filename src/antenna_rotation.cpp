#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <iostream>
#include <string>
#include <chrono>
#include <numbers>
#include <cmath>
#include <mutex>
#include <atomic>


class AntennaPID : public rclcpp::Node
{
    private:
        /*********************************************************************************************/
        // VARIABLES
        std::mutex data_mutex_;
        
        std::atomic<float> heading{-1.0f};
        std::atomic<long long> lastOverrideReceived{-1};
        
        float lastRoverLon = -9999;
        float lastRoverLat = -9999;
        float lastAntennaLon = -9999;
        float lastAntennaLat = -9999;

        rclcpp::CallbackGroup::SharedPtr cb_group_subscribers_;
        rclcpp::CallbackGroup::SharedPtr cb_group_timer_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_bs;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_antenna_pos;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_rover_pos;
        
        /*********************************************************************************************/
        // FUNCTIONS

        /*
         * bs_callback
         *  the code that is called when the system recieves a new messae from 
         *  the base-station
         * 
         *  @parameter:
         *          std_msgs::msg::Float64::SharedPtr msg - msg - the received message from the base-station
         * 
         *  @return:
         *          none
         */
        void bs_callback(const std_msgs::msg::Float64::SharedPtr msg)
        {
            this->lastOverrideReceived = std::chrono::system_clock::now().time_since_epoch().count();
            this->heading = msg->data * deg_to_rad;
        }
        
        void new_rover_pos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            this->lastRoverLon = msg->longitude;
            this->lastRoverLat = msg->latitude;
            
            if (this->lastAntennaLon == -9999 || this->lastAntennaLat == -9999 || this->lastRoverLon == -9999 || this->lastRoverLat == -9999)
            {
                return;
            }
            
            this->heading.store(rotation_based_on_cord(this->lastAntennaLon, this->lastAntennaLat, this->lastRoverLon, this->lastRoverLat));
        }
        
        void new_antenna_pos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            this->lastAntennaLon = msg->longitude;
            this->lastAntennaLat = msg->latitude;
            
            if (this->lastAntennaLon == -9999 || this->lastAntennaLat == -9999 || this->lastRoverLon == -9999 || this->lastRoverLat == -9999)
            {
                return;
            }
            
            this->heading.store(rotation_based_on_cord(this->lastAntennaLon, this->lastAntennaLat, this->lastRoverLon, this->lastRoverLat));
        }
        
        /*
         *	FUNCTION: ROTATION_BASED_ON_CORD 
         * 	DETERMINES ROATION BASED ON THE DB SIGNAL STRENGTH ROTATE TOWARDS THE STRONGER SIGNAL
         *  ALL VALUES OF LAT AND LON WILL BE CONVERT TO RADIUS AUTOMATICLY	
         *
         * 	@PARAMETER:
         * 			DOUBLE LON2 - ANTENNA LON POSTITION
         * 			DOUBLE LAT2 - ANTENNA LAT POSTITION
         * 			DOUBLE LON1 - ROVER LON POSTITION
         * 			DOUBLE LAT1 - ROVER LAT POSTITION
         * 	@RETURN: DOUBLE - RECOMMENDED ANGLE TO ROTATE TO (USING NORTH AS 0)
         */ 

        //ARRANGED FOR BETTER READABILITY AND PERFORMANCE (i think)
        double rotation_based_on_cord(const double &lon2, const double &lat2, const double &lon1, const double &lat1)
        {
            //CONVERSION FACTOR FROM DEGREES TO RADIANS
            constexpr double deg_to_rad = std::numbers::pi / 180.0;

            //CONVERTING LON AND LAT FROM DEGREES TO RADIANS
            const double Rlon1 = lon1 * deg_to_rad;
            const double Rlat1 = lat1 * deg_to_rad;
            const double Rlon2 = lon2 * deg_to_rad;
            const double Rlat2 = lat2 * deg_to_rad;

	        //CALCULATING THE DIFFERENCE IN LONGITUDE 1 and 2
	        const double diff_lon = (Rlon1 - Rlon2) * deg_to_rad;

            //CALCULATING X AND Y POSTIONS
	        const double y = std::sin(diff_lon) * std::cos(Rlat1);
	        const double x = (std::cos(Rlat2) * std::sin(Rlat1)) - (std::sin(Rlat2) * std::cos(Rlat1) * std::cos(diff_lon));

	        //DEBUG LINE (this line was commented for the original code, but I will leave it here for reference)
		        //std::cout << "X: " << x << " Y: " << y << " DIFF LON: " << diff_lon << std::endl;

	        //RETURNS THE ANGLE IN RADIANS
	        return  std::atan2(y, x);
        }
        
        void send_msg() 
        {
            auto message = std_msgs::msg::Float64();
            message.data = this->heading.load();
            this->pub_motor->publish(message);
        }
       

    public:

        /*
         * AntennaPID
         *  The constructor for the antenna PID class
         * 
         * Builds a new antenna PID and setups the required variables
         */
        AntennaPID() : Node("antenna")
        { 
        
            cb_group_subscribers_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            cb_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            auto sub_options = rclcpp::SubscriptionOptions();
            sub_options.callback_group = cb_group_subscribers_;
            
            sub_bs =  this->create_subscription<std_msgs::msg::Float64>(
                "antenna_override",
                10,
                std::bind(&AntennaPID::bs_callback, this, std::placeholders::_1),
                sub_options
            );
            
            sub_antenna_pos = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/pid",
                10,
                std::bind(&AntennaPID::new_antenna_pos_callback, this, std::placeholders::_1),
                sub_options
            );
            
            sub_rover_pos = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/pid",
                10,
                std::bind(&AntennaPID::new_rover_pos_callback, this, std::placeholders::_1),
                sub_options
            );

            pub_motor =  this->create_publisher<std_msgs::msg::Float64>(
                "/motor_cmd",
                10
            );
            
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(&AntennaPID::send_msg, this),
                cb_group_timer_   
            );
            
        }
};

/*
 * main
 *  The following code starts the Antenna PID
 */

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AntennaPID>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    
    executor.add_node(node);
    executor.spin();
    
    
    rclcpp::shutdown();
    return 0;
}