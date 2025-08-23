#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include <deque>
#include <cmath>
#include <std_msgs/msg/int64.hpp>
#include "using_msgs/msg/counter_status.hpp"
#include "using_msgs/msg/counter_warning.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"



using namespace std::chrono_literals;

class SmartWaringNode : public rclcpp::Node {
public:
    SmartWaringNode()
    : Node("smart_meter_waring_node"),
        waring(false)
        {
            this -> declare_parameter<bool>("waring", false); // 初始警告是否开启
            this -> declare_parameter<int64_t>("waring_count", 100); // 警告阈值/等级1
            this -> declare_parameter<int64_t>("waring_count2", 200); // 警告阈值/等级2
            this -> declare_parameter<int64_t>("waring_count3", 300); // 警告阈值/等级3
            this -> declare_parameter<int64_t>("waring_count4", 400); // 警告阈值/等级4
            this -> declare_parameter<int64_t>("waring_count5", 500); // 警告阈值/等级5
            waring = this -> get_parameter("waring").as_bool();
            waring_count = this -> get_parameter("waring_count").as_int();
            waring_count2 = this -> get_parameter("waring_count2").as_int();
            waring_count3 = this -> get_parameter("waring_count3").as_int();
            waring_count4 = this -> get_parameter("waring_count4").as_int();  
            waring_count5 = this -> get_parameter("waring_count5").as_int(); 
            waring_publisher_ = this -> create_publisher<using_msgs::msg::CounterWarning>("counter_waring", 10);
            counts_subscription_ = this -> create_subscription<std_msgs::msg::Int64>(
                "counts", 10, std::bind(&SmartWaringNode::countsCallback, this, std::placeholders::_1));
            start_waring = this -> create_service<std_srvs::srv::Trigger>(
                "start_waring",
                std::bind(&SmartWaringNode::startWaringCallback, this, 
                         std::placeholders::_1, std::placeholders::_2));
            stop_waring = this -> create_service<std_srvs::srv::Trigger>(
                "stop_waring",
                std::bind(&SmartWaringNode::stopWaringCallback, this,
                         std::placeholders::_1, std::placeholders::_2));
            parameter_callback_handle = this->add_on_set_parameters_callback(
                std::bind(&SmartWaringNode::parametersCallback, this, std::placeholders::_1));        
            RCLCPP_INFO(this->get_logger(), 
                       "智能警告节点启动 - 警告状态: %s, 阈值1: %ld, 阈值2: %ld, 阈值3: %ld, 阈值4: %ld, 阈值5: %ld", 
                       waring ? "true" : "false", waring_count, waring_count2, waring_count3, waring_count4, waring_count5);
        }
private:
    bool waring; // 是否开启警告
    int64_t waring_count; // 警告阈值/等级1
    int64_t waring_count2; // 警告阈值/等级2
    int64_t waring_count3; // 警告阈值/ 等级3
    int64_t waring_count4; // 警告阈值/等级4
    int64_t waring_count5; // 警告阈值/等级5
    int64_t current_count; // 当前计数值
    rclcpp::Publisher<using_msgs::msg::CounterWarning>::SharedPtr waring_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr counts_subscription_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_waring;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_waring;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle;
    void countsCallback(const std_msgs::msg::Int64::SharedPtr msg) {
        int64_t current_count = msg->data;
        auto now = this->get_clock()->now();
        // 检查警告条件
        checkWarnings(current_count);
    }
      rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "成功";
        for (const auto &param : parameters)
        {
            if (param.get_name() == "waring")
            {
                waring = param.as_bool();
                RCLCPP_INFO(this->get_logger(),
                            "警告状态更新为: %s",
                            waring ? "true" : "false");
            }
            else if (param.get_name() == "waring_count")
            {
                waring_count = param.as_int();
                RCLCPP_INFO(this->get_logger(),
                            "警告阈值1更新为: %ld", waring_count);
            }
            else if (param.get_name() == "waring_count2")
            {
                waring_count2 = param.as_int();
                RCLCPP_INFO(this->get_logger(),
                            "警告阈值2更新为: %ld", waring_count2);
            }
            else if (param.get_name() == "waring_count3")
            {
                waring_count3 = param.as_int();
                RCLCPP_INFO(this->get_logger(),
                            "警告阈值3更新为: %ld", waring_count3);
            }
            else if (param.get_name() == "waring_count4")
            {
                waring_count4 = param.as_int();
                RCLCPP_INFO(this->get_logger(),
                            "警告阈值4更新为: %ld", waring_count4);
            }
            else if (param.get_name() == "waring_count5")
            {
                waring_count5 = param.as_int();
                RCLCPP_INFO(this->get_logger(),
                            "警告阈值5更新为: %ld", waring_count5);
            }
        }
        return result;
    }
    void checkWarnings(int64_t count) {
        if (!waring) {
            return; // 警告未开启，直接返回
        }
     auto waring_msg = using_msgs::msg::CounterWarning();
        waring_msg.trigger_value = count;
        if (count >= waring_count5) {
            waring_msg.severity = 5;
            waring_msg.message = "警告等级5: 计数过高!";
            RCLCPP_ERROR(this->get_logger(), "%s 当前计数: %ld", waring_msg.message.c_str(), count);
        } else if (count >= waring_count4) {
            waring_msg.severity = 4;
            waring_msg.message = "警告等级4: 计数过高!";
            RCLCPP_WARN(this->get_logger(), "%s 当前计数: %ld", waring_msg.message.c_str(), count);
        } else if (count >= waring_count3) {
            waring_msg.severity = 3;
            waring_msg.message = "警告等级3: 计数过高!";
            RCLCPP_WARN(this->get_logger(), "%s 当前计数: %ld", waring_msg.message.c_str(), count);
        } else if (count >= waring_count2 ) {
            waring_msg.severity = 2;
            waring_msg.message = "警告等级2: 计数过高!";
            RCLCPP_INFO(this->get_logger(), "%s 当前计数: %ld", waring_msg.message.c_str(), count);
        } else if (count >= waring_count) {
            waring_msg.severity = 1;
            waring_msg.message = "警告等级1: 计数较高!";
            RCLCPP_INFO(this->get_logger(), "%s 当前计数: %ld", waring_msg.message.c_str(), count);
        } else {
            // 低于最低警告阈值，不发布警告
            return;
        }
        waring_publisher_->publish(waring_msg);
    }

    void startWaringCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (waring)
        {
            response->success = false;
            response->message = "警告已在运行中";
            return;
        }
        waring = true;
        response->success = true;
        response->message = "警告已启动"; 
    }
    void stopWaringCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (!waring)
        {
            response->success = false;
            response->message = "警告未在运行中";
            return;
        }
        waring = false;
        response->success = true;
        response->message = "警告已停止";
    }

    };
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartWaringNode>());
    rclcpp::shutdown();
    return 0;
}