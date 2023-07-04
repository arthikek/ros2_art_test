#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "std_msgs/msg/float64.hpp"
#include <functional>



float circum_ference=2*3.14*0.5;

 


class VelocityNode: public rclcpp::Node { 

public:
    VelocityNode():Node("speed_converter_node"){
        publisher_=this->create_publisher<std_msgs::msg::Float64>("velocity_topic",10);
      
        subscription_=this->create_subscription<std_msgs::msg::Float64>("rpm_topic",10,std::bind(&VelocityNode::publish_velocity,this,std::placeholders::_1));
        std::cout<< "Speed node is running"<<std::endl;

    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    


    void publish_velocity(const std_msgs::msg::Float64 &msg){        
        auto message=std_msgs::msg::Float64();
        message.data=circum_ference*msg.data;
        publisher_->publish(message);
    };

};

int main(int argc, char *argv[])

{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityNode>());
    rclcpp::shutdown();

    return 0;
}