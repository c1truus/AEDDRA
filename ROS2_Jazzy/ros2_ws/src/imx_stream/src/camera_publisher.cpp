#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>

using std::placeholders::_1;

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("camera_publisher") {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33), std::bind(&CameraPublisher::timer_callback, this));  // ~30 FPS
    // Use GStreamer with libcamerasrc for IMX219 (no V4L2) 
    cap_ = cv::VideoCapture("libcamerasrc ! video/x-raw,width=1280,height=720,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink", cv::CAP_GSTREAMER);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open IMX219 camera with GStreamer!");
      throw std::runtime_error("Camera failed");
    }
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);  // Minimize lag
    RCLCPP_INFO(this->get_logger(), "IMX219 camera publisher started at ~30 FPS");
  }

private:
  void timer_callback() {
    cv::Mat frame;
    if (cap_.read(frame)) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      publisher_->publish(*msg);
    } else {
      RCLCPP_WARN(this->get_logger(), "Frame capture failed");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::VideoCapture cap_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
