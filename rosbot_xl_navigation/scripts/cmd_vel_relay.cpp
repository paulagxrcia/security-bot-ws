/**
*@file cmd_vel_relay.cpp
*@brief Relay node that converts Twist messages to TwistStamped messages

*This program subscriber to velocity commands published as Twist messages and
*republishes them as TwistStamped messages. This is useful when interfacing 
*between different ROS 2 nodes that expect different message types for velocity
*commands.
*
*Subscription Topics:
*      /cmd_vel (geometry_msgs/msg/Twist): Raw velocity commands
*
Publishing Topics:
*      /rosbot_xl_base_controller/cmd_vel (geometry_msgs/msg/TwistStamped): 
*      Timestamped velocity commands
*/

//Include necesary header files
#include <memory> 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

/**
* @brief A ROS 2 node that relays velocity commands between diffrent message types

* This class creates a node that subscribes to Twist messages and republishes 
* them as TwistStamped messages, adding a timestamp and frame ID.
*/

class CmdVelRelay : public rclcpp::Node {
public:
    /** 
    *@brief Constructor for the CmdVelRelay node
    *
    *Initialized the node, creates a subscription to the input topic, 
    *and sets up a publisher for the output topic.
    */
    CmdVelRelay() : Node("cmd_vel_relay") {
        // Create subscription to /cmd_vel topic
        //The '10' represents the queue size - how many messages to store if we can't process them fast enough
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, 
            std::bind(&CmdVelRelay::cmd_vel_callback, this, std::placeholders::_1));

        // Create publisher for /rosbot_xl_base_controller/cmd_vel topic
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/rosbot_xl_base_controller/cmd_vel", 10);
        
        // Log a message indicating the node has started
        RCLCPP_INFO(this->get_logger(), "Velocity relay node started.");
    }

private:
    /**
     * @brief Callback function that handles incoming Twist messages
     * 
     * This function is called whenever a new message arrives on the /cmd_vel topic.
     * It creates a new TwistStamped message, copies the velocity data, adds
     * a timestamp and frame ID, and publishes the result.ADJ_ESTERROR
     * 
     * @param msg Shared pointer to the incoming Twist message
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        //Create a new TwistStamped message
        auto stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

        //Set the timestamp to current time
        stamped_msg->header.stamp = this->now();
        //Set the frame ID (coordinate frame this velocity is expressed in)
        stamped_msg->header.frame_id = "base_link";

        //Copy the twist message content (linear and angular velocities)
        stamped_msg->twist = *msg;

        //Publish the stamped message
        publisher_->publish(std::move(stamped_msg));
    }

    //Declare class member variables for the subscriber and publisher
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;

};

/**
 * @brief Main function that initializes and runs the node
 * 
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @returns init Exit status
 */

 int main(int argc, char* argv[]) {
    //Initialize ROS 2
    rclcpp::init(argc, argv);

    //Create an instance of our node
    auto node = std::make_shared<CmdVelRelay>();

    //Spin the node, making it respond to incoming messages
    rclcpp::spin(node);

    //Clean up ROS 2 resources
    rclcpp::shutdown();

    return 0;
 }