#define DEBUG 0  // Set to 1 to enable debug logging; 0 for production (compile-time removal)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>  // Explicit for Header
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>

using std::placeholders::_1;

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("camera_publisher"), count_(0) {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(25), std::bind(&CameraPublisher::timer_callback, this));  // ~30 FPS target
    cap_ = cv::VideoCapture(0);  // Simple V4L2 backend; change to 1 if needed
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open camera!");
      throw std::runtime_error("Camera failed");
    }
    RCLCPP_INFO(this->get_logger(), "Simple camera publisher started at ~30 FPS target");
  }

private:
  void timer_callback() {
    cv::Mat frame;
    if (cap_.read(frame)) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      publisher_->publish(*msg);
#ifdef DEBUG
      RCLCPP_DEBUG(this->get_logger(), "Publishing frame %zu", count_++);
#endif
    } else {
      RCLCPP_WARN(this->get_logger(), "Frame capture failed");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::VideoCapture cap_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
