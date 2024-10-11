#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <mutex>


#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "charger_msgs/srv/check_boolean.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class ChargerNode : public rclcpp::Node 
{
    
    rclcpp::Service<charger_msgs::srv::CheckBoolean>::SharedPtr service;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_zk_;
    double goal_x_param;
    double goal_y_param;
    double box_length_param;
    double box_width_param;
    bool inCharger;
    std::mutex charge_state_mutex;

public:ChargerNode():Node("charger_node")
{

    declare_parameter<double>("goal_x", 0.0);
    declare_parameter<double>("goal_y", 1.0);
    declare_parameter<double>("box_length", 0.0);
    declare_parameter<double>("box_width", 1.0);


    get_parameter("goal_x", goal_x_param);
    get_parameter("goal_y", goal_y_param);
    get_parameter("box_length", box_length_param);
    get_parameter("box_width", box_width_param);

    // init service
    service = create_service<charger_msgs::srv::CheckBoolean>("charger_status",  [this](
            const std::shared_ptr<charger_msgs::srv::CheckBoolean::Request> request,
            std::shared_ptr<charger_msgs::srv::CheckBoolean::Response> response
        ) { this->check(request, response); }
    );
    inCharger = false;

    subscriber_zk_ = this->create_subscription<geometry_msgs::msg::Point>(
        "zk/pose", 10,
        std::bind(&ChargerNode::callback, this, std::placeholders::_1)
    );
}

void check(const std::shared_ptr<charger_msgs::srv::CheckBoolean::Request> request,
          std::shared_ptr<charger_msgs::srv::CheckBoolean::Response>      response)
{
    (void)request;
  response->response = inCharger;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", response->response);

    }

void callback(const geometry_msgs::msg::Point::ConstSharedPtr msg)
    {
        double currx = msg->x;
        double curry = msg->y;
        charge_state_mutex.lock();
        if((currx > goal_x_param - (box_width_param/2)) && (currx < goal_x_param + (box_width_param/2))) {
            if((curry > goal_y_param - (box_length_param/2)) && (curry < goal_y_param + (box_length_param/2))) { 
                inCharger = true;
            }
        }
        else {
            inCharger = false;
        }
        charge_state_mutex.unlock();
        
         
    }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChargerNode>());
  rclcpp::shutdown();
}