#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include "std_msgs/msg/float64.hpp"



class HelloWorldSubNode : public rclcpp::Node
{

public:
    HelloWorldSubNode() : Node("hello_world_sub_node") {

        subscription_=this->create_subscription<std_msgs::msg::Float64>("velocity_topic",10,std::bind(&HelloWorldSubNode::hello_world_topic_callback,this,std::placeholders::_1));

    } ;

private:
    void hello_world_topic_callback(const std_msgs::msg::Float64 &msg) const
    {
        std::cout<<"I heard the velocity: "<<msg.data<<std::endl;
    };

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;



};








int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloWorldSubNode>());   
    rclcpp::shutdown();

    return 0;   

}