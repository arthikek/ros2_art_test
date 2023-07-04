#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "ros2_art_test/srv/corner_camera.hpp"
#include <string>

typedef ros2_art_test::srv::CornerCamera CornerCamera;

class CornerCameraServer : public rclcpp::Node 
{
public:
    CornerCameraServer():Node("corner_camera_service"){
        server_server_ = this->create_service<CornerCamera>("corner_camera",
        std::bind(&CornerCameraServer::check_corner,this,std::placeholders::_1,std::placeholders::_2));
    }
private:
    rclcpp::Service<CornerCamera>::SharedPtr server_server_;
    void check_corner(const CornerCamera::Request::SharedPtr request,
    CornerCamera::Response::SharedPtr response)
    { 
        // We need to grab the number that is send and contruct a path with it
        float number = request->number;
        int number_int = (int)number;
        std::string path = "/home/arthike/Pictures/images/" + std::to_string(number_int) + ".png";
        
        cv::Mat image_1 = cv::imread(path,cv::IMREAD_COLOR);
        
        // Check if the image is loaded correctly
        if (image_1.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the test image file at path: %s", path.c_str());
            return; // Early return
        }
        
        std_msgs::msg::Header header; // empty header
        header.stamp = this->now(); // you need to fill the timestamp here
        cv_bridge::CvImage cv_bridge(header, sensor_msgs::image_encodings::BGR8, image_1);
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge.toImageMsg();
        response-> found_image = *msg;
    }
};

int main (int argc,char * argv[])
{   
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<CornerCameraServer>());
    rclcpp::shutdown();
}
