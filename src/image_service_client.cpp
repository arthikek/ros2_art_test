#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <ros2_art_test/srv/corner_camera.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

typedef ros2_art_test::srv::CornerCamera CornerCamera;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto service_client = rclcpp::Node::make_shared("corner_camera_client");
    auto client = service_client->create_client<CornerCamera>("corner_camera");

    while (rclcpp::ok()) {
        auto request = std::make_shared<CornerCamera::Request>();
        std::cout << "Enter a number (or enter 'q' to quit): ";
        std::string input;
        std::cin >> input;

        if (input == "q") {
            break;
        }

        try {
            request->number = std::stof(input);
        } catch (std::exception& e) {
            std::cout << "Invalid input. Please enter a valid number." << std::endl;
            continue;
        }

        client->wait_for_service();
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(service_client, result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = result.get();
            if (response->found_image.height > 0 && response->found_image.width > 0) {
                try {
                    // Convert ROS image to OpenCV format
                    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(response->found_image, sensor_msgs::image_encodings::BGR8);
                    cv::Mat cv_image = cv_image_ptr->image;

                    // Display the image in a window
                    cv::namedWindow("Image", cv::WINDOW_NORMAL);
                    cv::imshow("Image", cv_image);
                    cv::waitKey(5000); // Use a small delay (1ms) to allow GUI events to be processed
                } catch (cv_bridge::Exception& e) {
                    std::cout << "Error converting ROS image to OpenCV format: " << e.what() << std::endl;
                }
            } else {
                std::cout << "Received an empty image from the service" << std::endl;
            }
        } else {
            std::cout << "Service call failed" << std::endl;
        }
    }

    rclcpp::shutdown();
    return 0;
}