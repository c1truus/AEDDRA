#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

#define DEBUG 0
#define INFERENCE_INTERVAL 5

struct Detection {
  float x, y, width, height, confidence;
  int class_id;
};

class YOLOv8Inference : public rclcpp::Node {
public:
  YOLOv8Inference() : Node("yolo_inference"), frame_count_(0) {
    env_ = Ort::Env(ORT_LOGGING_LEVEL_ERROR, "YOLOv8");
    session_options_.SetIntraOpNumThreads(2);
    session_ = Ort::Session(env_, "best_qnt.onnx", session_options_);
    
    input_name_ = session_.GetInputName(0, allocator_);
    output_name_ = session_.GetOutputName(0, allocator_);
    
    camera_sub_ = create_subscription<sensor_msgs::msg::Image>("camera/image_raw", 5, 
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { camera_callback(msg); });
    
    detection_pub_ = create_publisher<sensor_msgs::msg::Image>("inference_result", 5);
    if(DEBUG) RCLCPP_INFO(get_logger(), "YOLO inference started");
  }

private:
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    frame_count_++;
    if (frame_count_ % INFERENCE_INTERVAL != 0) return;
    
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat processed;
      cv::resize(cv_ptr->image, processed, cv::Size(640, 640));
      processed.convertTo(processed, CV_32F, 1.0/255.0);
      
      auto detections = run_inference(processed);
      publish_detections(cv_ptr->image, detections);
    } catch (...) {}
  }

  std::vector<Detection> run_inference(const cv::Mat& image) {
    std::array<int64_t, 4> input_shape = {1, 3, 640, 640};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, (float*)image.data, image.total() * 3, input_shape.data(), 4);

    const char* input_names[] = {input_name_};
    const char* output_names[] = {output_name_};
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

    return parse_detections(outputs);
  }

  std::vector<Detection> parse_detections(const std::vector<Ort::Value>& outputs) {
    std::vector<Detection> detections;
    if (outputs.empty()) return detections;
    
    float* data = outputs[0].GetTensorMutableData<float>();
    auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    int num_boxes = shape[2];
    
    for (int i = 0; i < num_boxes; ++i) {
      float conf = data[4 * num_boxes + i];
      if (conf > 0.5) {
        Detection det;
        det.x = data[i];
        det.y = data[1 * num_boxes + i];
        det.width = data[2 * num_boxes + i];
        det.height = data[3 * num_boxes + i];
        det.confidence = conf;
        
        det.class_id = 0;
        float max_prob = 0;
        for (int c = 0; c < 4; ++c) {
          float prob = data[(5 + c) * num_boxes + i];
          if (prob > max_prob) {
            max_prob = prob;
            det.class_id = c;
          }
        }
        detections.push_back(det);
      }
    }
    return detections;
  }

  void publish_detections(const cv::Mat& image, const std::vector<Detection>& detections) {
    cv::Mat result = image.clone();
    for (const auto& det : detections) {
      int x1 = det.x * image.cols - (det.width * image.cols) / 2;
      int y1 = det.y * image.rows - (det.height * image.rows) / 2;
      int x2 = x1 + det.width * image.cols;
      int y2 = y1 + det.height * image.rows;
      
      cv::rectangle(result, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
      std::string label = std::to_string(det.class_id) + " " + std::to_string(det.confidence).substr(0,4);
      cv::putText(result, label, cv::Point(x1, y1-5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result).toImageMsg();
    detection_pub_->publish(*msg);
  }

  Ort::Env env_;
  Ort::Session session_{nullptr};
  Ort::SessionOptions session_options_;
  Ort::AllocatorWithDefaultOptions allocator_;
  const char* input_name_;
  const char* output_name_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_pub_;
  int frame_count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YOLOv8Inference>());
  rclcpp::shutdown();
  return 0;
}