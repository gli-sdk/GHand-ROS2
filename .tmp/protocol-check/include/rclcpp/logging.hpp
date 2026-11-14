#pragma once
namespace rclcpp {
inline int get_logger(const char*) { return 0; }
template<class... T> void log(T&&...) {}
}
#define RCLCPP_INFO(...) rclcpp::log(__VA_ARGS__)
#define RCLCPP_WARN(...) rclcpp::log(__VA_ARGS__)
#define RCLCPP_ERROR(...) rclcpp::log(__VA_ARGS__)
#define RCLCPP_FATAL(...) rclcpp::log(__VA_ARGS__)
#define RCLCPP_DEBUG(...) rclcpp::log(__VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(...) rclcpp::log(__VA_ARGS__)
