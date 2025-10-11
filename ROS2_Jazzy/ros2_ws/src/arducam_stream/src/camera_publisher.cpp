#define DEBUG 0

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("camera_publisher") {
    publisher_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(33), [this]() { timer_callback(); });
    cap_ = cv::VideoCapture(0);
    if (!cap_.isOpened()) {
      cap_ = cv::VideoCapture(0, cv::CAP_V4L2);
      if (!cap_.isOpened()) throw std::runtime_error("Arducam not found on /dev/video0");
    }
    
    // Set resolution for better performance
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FPS, 30);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    
    if(DEBUG) RCLCPP_INFO(get_logger(), "Arducam publisher started");
  }

private:
  void timer_callback() {
    cv::Mat frame;
    if (cap_.read(frame)) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      publisher_->publish(*msg);
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