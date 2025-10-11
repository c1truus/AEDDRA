#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#define DEBUG 0

class ImageSubscriber : public rclcpp::Node {
public:
  ImageSubscriber() : Node("image_subscriber"), latest_detection_(cv::Mat()) {
    camera_sub_ = create_subscription<sensor_msgs::msg::Image>("camera/image_raw", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { camera_callback(msg); });
    
    inference_sub_ = create_subscription<sensor_msgs::msg::Image>("inference_result", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { inference_callback(msg); });
    
    if(DEBUG) RCLCPP_INFO(get_logger(), "Image subscriber started");
  }

private:
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      display_image_ = cv_ptr->image.clone();
      
      if (!latest_detection_.empty()) {
        cv::resize(latest_detection_, latest_detection_, display_image_.size());
        cv::addWeighted(display_image_, 0.7, latest_detection_, 0.3, 0, display_image_);
      }
      
      cv::imshow("Camera Stream + YOLO Detection", display_image_);
      cv::waitKey(1);
    } catch (...) {}
  }

  void inference_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      latest_detection_ = cv_ptr->image.clone();
    } catch (...) {}
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr inference_sub_;
  cv::Mat latest_detection_;
  cv::Mat display_image_;
  std::mutex mutex_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}