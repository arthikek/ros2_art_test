#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_art_test/action/function.hpp"
#include "geometry_msgs/msg/point.hpp"
#include  <cstdlib>
#include  <chrono>
#include <string>


typedef ros2_art_test::action::Function Function;
typedef rclcpp_action::ServerGoalHandle<Function> GoalHandle;


class FunctionServer : public rclcpp::Node
{
    public:
        FunctionServer():Node("action_counter"){

        action_server_ = rclcpp_action::create_server<Function>(
            this,
            "function",
            std::bind(&FunctionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FunctionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FunctionServer::handle_accepted, this, std::placeholders::_1));  


    }

    private:

        rclcpp_action::Server<Function>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const Function::Goal> goal)
        {
            
            (void) uuid;
            RCLCPP_INFO(this->get_logger(), "Received goal request with a value %ld", goal->a);
            
            if (goal->a > 0 )
             {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;}
            else{
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
     rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle)
    {
      // RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      std::cout << "Received request to cancel goal" << std::endl;
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&FunctionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::cout << "Executing goal" << std::endl;
        // Generate a random number to estimate the change of being executed between 0 and 11
        
        



        auto feedback =std::make_shared<Function::Feedback>();
        auto result = std::make_shared<Function::Result>();
        const auto goal = goal_handle->get_goal();

        
        
        int moved = 0;
        int goal_to_move = goal ->a;
       

        for (int i=0 ; i<goal_to_move ; i++){

            int chance = rand()%10;

            if (chance > 1){

                moved++;
             
                feedback->percent_complete = (static_cast<float>(moved) / goal_to_move) * 100;
                std::cout<< "Package moved succesfully"<<std::endl;
                goal_handle->publish_feedback(feedback);

            }

            else {
                
                std::cout<< "Package moved unsuccesfully"<<std::endl;
                
            }

       

            



        }

         result-> sum = moved ;

             if(moved==goal_to_move){
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(),"goal succeeded");

            }

            else{
                goal_handle->abort(result);
                RCLCPP_INFO(this->get_logger(), "Goal target was not succeeded. Packages left: %d", goal_to_move - moved);



            }

        
        
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FunctionServer>());
    rclcpp::shutdown();
    return 0;
}