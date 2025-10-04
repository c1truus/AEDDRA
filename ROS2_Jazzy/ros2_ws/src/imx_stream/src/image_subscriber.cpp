#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

using std::placeholders::_1;

class ImageSubscriber : public rclcpp::Node {
public:
  ImageSubscriber() : Node("image_subscriber") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 10, std::bind(&ImageSubscriber::image_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "IMX219 image subscriber started");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::imshow("IMX219 Stream", cv_ptr->image);
      cv::waitKey(1);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error displaying image: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}
