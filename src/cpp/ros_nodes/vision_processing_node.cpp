#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class VisionProcessingNode : public rclcpp::Node
{
public:
  VisionProcessingNode() : Node("vision_processing_node")
  {
    // Subscriber to camera feed
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera_feed", 10, std::bind(&VisionProcessingNode::camera_callback, this, std::placeholders::_1));

    // Publisher for detected objects
    publisher_ = this->create_publisher<std_msgs::msg::String>("detected_objects", 10);

    RCLCPP_INFO(this->get_logger(), "Vision Processing Node started. Subscribed to /camera_feed, publishing on /detected_objects");
  }

private:
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // TODO: Implement image processing and object detection logic here
    // For now, just acknowledge receiving the image
    RCLCPP_INFO(this->get_logger(), "Received an image: %dx%d", msg->width, msg->height);

    // Example: Publish a dummy detected object
    auto detection_msg = std_msgs::msg::String();
    detection_msg.data = "dummy_object";
    publisher_->publish(detection_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionProcessingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
