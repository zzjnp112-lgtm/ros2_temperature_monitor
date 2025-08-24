ROS2 智能计数器系统
一个基于ROS2的分布式智能计数系统，展示了发布者/订阅者模式、服务调用、参数管理和自定义消息类型的综合应用。
📋 项目概述
该项目实现了一个完整的智能计数生态系统，包含：

🎯 核心计数器 - 可配置的高精度计数器节点
📊 智能监控 - 实时监控和警告系统
⚙️ 动态配置 - 运行时参数调整
🔧 服务控制 - 完整的生命周期管理
控制命令
# 🎬 开始计数
ros2 service call /start_counter std_srvs/srv/Trigger

# ⏹️ 停止计数
ros2 service call /stop_counter std_srvs/srv/Trigger

# 🔄 重置计数器
ros2 service call /reset_counter std_srvs/srv/Trigger

# ⏸️ 暂停/恢复计数 (true=暂停, false=恢复)
ros2 service call /pause_counter std_srvs/srv/SetBool "{data: true}"
参数配置

# 🎛️ 调整计数频率 (Hz)
ros2 param set /counter_node frequency 2.0

# 📏 修改递增步长
ros2 param set /counter_node step_size 5
