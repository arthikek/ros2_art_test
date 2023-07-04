#include "ros2_art_test/srv/odd_even_check.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
typedef ros2_art_test::srv::OddEvenCheck OddEvenCheck;




int main (int argc,char * argv[])
{
    rclcpp::init(argc,argv);
   
    auto service_client = rclcpp::Node::make_shared("odd_even_check_client");
    auto client = service_client->create_client<OddEvenCheck>("odd_even_check");

    auto request = std::make_shared<OddEvenCheck::Request>();
    std::cout << "Enter a number: " ;
    std::cin >> request->number ;

    client->wait_for_service();
    auto result = client->async_send_request(request);
    if(rclcpp::spin_until_future_complete(service_client,result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cout << "Result: " << result.get()->decision << std::endl;
    }
    else
    {
        std::cout << "Service call failed" << std::endl;
    }
    rclcpp::shutdown();
}





