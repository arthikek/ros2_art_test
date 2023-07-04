#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_art_test/action/function.hpp"
#include "geometry_msgs/msg/point.hpp"




typedef ros2_art_test::action::Function Function;
typedef rclcpp_action::ClientGoalHandle<Function> GoalHandle;


class FunctionActionClientNode : public rclcpp::Node
{

    public:
    FunctionActionClientNode():Node("action_counter_client"){
        action_client_ = rclcpp_action::create_client<Function>(
            this,
            "function");
        prompt_user_for_goal();
    }


    private:
        rclcpp_action::Client<Function>::SharedPtr action_client_;
        int64_t sent_goal_a;
        void prompt_user_for_goal(){
        using std::placeholders::_2;
        using std::placeholders::_1; 
        auto goal_msg = Function::Goal();

        std::cout << "Enter amount of packages to move out of the factory: ";
        std::cin >> goal_msg.a;
        sent_goal_a = goal_msg.a;
        this->action_client_->wait_for_action_server();

        std::cout << "Sending goal..." << std::endl;

        auto send_goal_options = rclcpp_action::Client<Function>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&FunctionActionClientNode::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&FunctionActionClientNode::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&FunctionActionClientNode::result_callback, this, _1);

        // Actually send the goal
        this->action_client_->async_send_goal(goal_msg, send_goal_options);
}


        void goal_response_callback(GoalHandle::SharedPtr future){
            auto goal_handle = future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

      void feedback_callback(GoalHandle::SharedPtr,const std::shared_ptr<const Function::Feedback> feedback){
    RCLCPP_INFO(this->get_logger(), "Percentage completed %f", feedback->percent_complete);
}

        // Assuming you have this member variable in your class


void result_callback (const GoalHandle::WrappedResult & result){
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            std::cout << "Result received: " << result.result->sum << std::endl;
            if (result.result->sum == this->sent_goal_a){
                RCLCPP_INFO(this->get_logger(), "Function completed successfully. All packages were moved out of the factory.");
            }
            
            break;

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Function completed unsuccessfully. Not all packages were moved out of the factory. %ld packages are still in the factory", (sent_goal_a-result.result->sum));
            break;

        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;

        default:
            std::cout << "Unknown result code" << std::endl;
            break;
    }

    prompt_user_for_goal();
    
}


};


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FunctionActionClientNode>());
    rclcpp::shutdown();
    return 0;
}