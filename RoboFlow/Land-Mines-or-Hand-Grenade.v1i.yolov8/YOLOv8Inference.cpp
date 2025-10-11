#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

class YOLOv8Inference : public rclcpp::Node {
public:
  YOLOv8Inference() : Node("yolov8_inference") {
    env_ = Ort::Env(ORT_LOGGING_LEVEL_ERROR, "YOLOv8");
    session_options_.SetIntraOpNumThreads(2);
    session_ = Ort::Session(env_, "best_qnt.onnx", session_options_);
    
    input_name_ = session_.GetInputName(0, allocator_);
    output_name_ = session_.GetOutputName(0, allocator_);
    
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 5, std::bind(&YOLOv8Inference::image_callback, this, std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat processed;
      cv::resize(cv_ptr->image, processed, cv::Size(640, 640));
      processed.convertTo(processed, CV_32F, 1.0/255.0);
      
      auto outputs = run_inference(processed);
      auto output_image = draw_detections(cv_ptr->image, outputs);
      
      cv::imshow("YOLOv8 Detection", output_image);
      cv::waitKey(1);
    } catch (...) {}
  }

  std::vector<Ort::Value> run_inference(const cv::Mat& image) {
    std::array<int64_t, 4> input_shape = {1, 3, 640, 640};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, (float*)image.data, image.total() * 3, input_shape.data(), 4);

    const char* input_names[] = {input_name_};
    const char* output_names[] = {output_name_};
    
    return session_.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
  }

  cv::Mat draw_detections(const cv::Mat& image, const std::vector<Ort::Value>& outputs) {
    cv::Mat output = image.clone();
    if (outputs.empty()) return output;
    
    float* data = outputs[0].GetTensorMutableData<float>();
    auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    int num_boxes = shape[2];
    
    for (int i = 0; i < num_boxes; ++i) {
        float conf = data[4 * num_boxes + i];
        if (conf > 0.5) {
            float cx = data[i] * image.cols;
            float cy = data[1 * num_boxes + i] * image.rows;
            float w = data[2 * num_boxes + i] * image.cols;
            float h = data[3 * num_boxes + i] * image.rows;
            
            int class_id = 0;
            float max_prob = 0;
            for (int c = 0; c < 4; ++c) {
                float prob = data[(5 + c) * num_boxes + i];
                if (prob > max_prob) {
                    max_prob = prob;
                    class_id = c;
                }
            }
            
            int x1 = cx - w/2;
            int y1 = cy - h/2;
            cv::rectangle(output, cv::Point(x1, y1), cv::Point(x1 + w, y1 + h), cv::Scalar(0, 255, 0), 2);
            cv::putText(output, std::to_string(class_id), cv::Point(x1, y1-5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
    }
    return output;
  }

  Ort::Env env_;
  Ort::Session session_{nullptr};
  Ort::SessionOptions session_options_;
  Ort::AllocatorWithDefaultOptions allocator_;
  const char* input_name_;
  const char* output_name_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YOLOv8Inference>());
  rclcpp::shutdown();
  return 0;
}