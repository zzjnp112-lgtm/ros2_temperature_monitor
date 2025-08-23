#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include <deque>
#include <cmath>
#include <std_msgs/msg/int64.hpp>
#include "using_msgs/msg/counter_status.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;


class CountsNode : public rclcpp::Node {
public:
    CountsNode()
    : Node("counts_node"),
      count_(0),
      frequency_(1.0),
      max_count_(1000),
      reset_on_max_(false),
      step_size_(1),
      is_running_(false) // 默认不运行
    {
        this->declare_parameter<int64_t>("count", 0); // 初始计数值
        this->declare_parameter<double>("frequency", 1.0); // 计数频率
        this->declare_parameter<int64_t>("max_count", 1000); // 最大计数值
        this->declare_parameter<bool>("reset_on_max", false); // 达到最大值后是否重置
        this->declare_parameter<int64_t>("step_size", 1); // 每次计数步长

        count_ = this->get_parameter("count").as_int();
        frequency_ = this->get_parameter("frequency").as_double();
        max_count_ = this->get_parameter("max_count").as_int();
        reset_on_max_ = this->get_parameter("reset_on_max").as_bool();
        step_size_ = this->get_parameter("step_size").as_int();

        status_publisher_ = this->create_publisher<using_msgs::msg::CounterStatus>("counter_status", 10);
        counts_publisher_ = this->create_publisher<std_msgs::msg::Int64>("counts", 10);

        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_counter",
            std::bind(&CountsNode::startCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
                     
        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_counter",
            std::bind(&CountsNode::stopCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
                     
        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_counter",
            std::bind(&CountsNode::resetCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
                     
        pause_service_ = this->create_service<std_srvs::srv::SetBool>(
            "pause_counter",
            std::bind(&CountsNode::pauseCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        auto interval = std::chrono::milliseconds(
        static_cast<int>(1000.0 / frequency_));
        timer_ = this->create_wall_timer(
            interval, std::bind(&CountsNode::timerCallback, this));
            
        // 参数变化监听
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&CountsNode::parametersCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), 
                   "计数器节点启动 - 频率: %.1fHz, 步长: %ld", frequency_, step_size_);
        }
private:
    int64_t count_;
    double frequency_;
    int64_t max_count_;
    bool reset_on_max_;
    int64_t step_size_;
    bool is_running_ = false; // 新增，默认不运行

    rclcpp::Publisher<using_msgs::msg::CounterStatus>::SharedPtr status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr counts_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    void timerCallback()
    {
        if (!is_running_)
            return;
        count_ += step_size_;
        if (count_ >= max_count_)
        {
            if (reset_on_max_)
            {
                count_ = 0;
                RCLCPP_WARN(this->get_logger(), "计数达到最大值，已重置为0");
            }
            else
            {
                count_ = max_count_;
                is_running_ = false;
                RCLCPP_WARN(this->get_logger(), "计数达到最大值，已停止计数");
            }
        }
        publishStatus();
        publishCount();
    }
    void publishStatus()
    {
        auto status_msg = using_msgs::msg::CounterStatus();
        status_msg.count = count_;
        status_msg.frequency = frequency_;
        status_msg.status = is_running_ ? "running" : "paused";
        status_msg.warning_triggered = (count_ >= max_count_);
        status_publisher_->publish(status_msg);
    }
    void publishCount()
    {
        auto count_msg = std_msgs::msg::Int64();
        count_msg.data = count_;
        counts_publisher_->publish(count_msg);
    }
    void startCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (is_running_)
        {
            response->success = false;
            response->message = "计数器已在运行中";
            return;
        }
        is_running_ = true;
        response->success = true;
        response->message = "计数器已启动"; 
    }
    void stopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (!is_running_)
        {
            response->success = false;
            response->message = "计数器未在运行中";
            return;
        }
        is_running_ = false;
        response->success = true;
        response->message = "计数器已停止"; 
    }
    void resetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        count_ = 0;
        response->success = true;
        response->message = "计数器已重置为0";
    }
    void pauseCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            if (!is_running_)
            {
                response->success = false;
                response->message = "计数器未在运行中";     
                return;
            }
            is_running_ = false;
            response->success = true;
            response->message = "计数器已暂停";
        }
        else
        {
            if (is_running_)
            {
                response->success = false;
                response->message = "计数器已在运行中";     
                return;
            }
            is_running_ = true;
            response->success = true;
            response->message = "计数器已恢复运行";
        }
    }
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "成功";
        for (const auto &param : parameters)
        {
            if (param.get_name() == "frequency")
            {
                double new_freq = param.as_double();
                if (new_freq <= 0)
                {
                    result.successful = false;
                    result.reason = "频率必须大于0";
                }
                else
                {
                    frequency_ = new_freq;
                    auto interval = std::chrono::milliseconds(
                        static_cast<int>(1000.0 / frequency_));
                    timer_->cancel();
                    timer_ = this->create_wall_timer(
                        interval, std::bind(&CountsNode::timerCallback, this));
                    RCLCPP_INFO(this->get_logger(),
                                "计数频率更新为: %.1fHz", frequency_);
                }
            }
            else if (param.get_name() == "step_size")
            {
                int new_step = param.as_int();
                if (new_step <= 0)
                {
                    result.successful = false;
                    result.reason = "步长必须大于0";
                }
                else
                {
                    step_size_ = new_step;
                    RCLCPP_INFO(this->get_logger(),
                                "计数步长更新为: %ld", step_size_);
                }
            }
            else if (param.get_name() == "max_count")  
            {
                int64_t new_max = param.as_int();
                if (new_max <= 0)
                {
                    result.successful = false;
                    result.reason = "最大计数值必须大于0";
                }
                else
                {
                    max_count_ = new_max;
                    RCLCPP_INFO(this->get_logger(),
                                "最大计数值更新为: %ld", max_count_);
                }
            }
            else if (param.get_name() == "reset_on_max")
            {
                reset_on_max_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(),
                            "达到最大值后是否重置更新为: %s",
                            reset_on_max_ ? "true" : "false");
            }
        }
        return result;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}