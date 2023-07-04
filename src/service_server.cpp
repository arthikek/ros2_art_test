#include "ros2_art_test/srv/odd_even_check.hpp"
#include "rclcpp/rclcpp.hpp"

typedef ros2_art_test::srv::OddEvenCheck OddEvenCheck;

class OddEvenCheckServer : public rclcpp::Node
{
public:
    OddEvenCheckServer():Node("odd_even_check_service"){
        server_server_ = this->create_service<OddEvenCheck>("odd_even_check",
        std::bind(&OddEvenCheckServer::check_nom_odd_even,this,std::placeholders::_1,std::placeholders::_2));
    }
private:
    void check_nom_odd_even(const OddEvenCheck::Request::SharedPtr request,
    OddEvenCheck::Response::SharedPtr response)
    {
        int remainder = std::abs(request->number % 2);

        switch(remainder){
            case 0:
                response->decision = "even";
                break;
            case 1:
                response->decision = "odd";
                break;
        }
    }
    rclcpp::Service<OddEvenCheck>::SharedPtr server_server_;
};



int main (int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<OddEvenCheckServer>());
    rclcpp::shutdown();
}




