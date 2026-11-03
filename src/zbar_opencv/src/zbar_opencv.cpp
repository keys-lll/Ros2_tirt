#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <zbar.h>

class ImageConverter : public rclcpp::Node
{
public:
  ImageConverter()
  : Node("zbar_opencv")
  {
    // 訂閱 USB Camera 原始影像。
    // 使用 SensorData QoS，避免相機端使用 BEST_EFFORT 時無法收到影像。
    image_sub_ = image_transport::create_subscription(
      this,
      "/image_raw",
      std::bind(&ImageConverter::imageCb, this, std::placeholders::_1),
      "raw",
      rmw_qos_profile_sensor_data);

    // 發布畫上 QR Code 綠框後的影像。
    image_pub_ = image_transport::create_publisher(
      this,
      "zbar_opencv",
      rmw_qos_profile_sensor_data);

    // 發布 QR Code 字串內容。
    qr_code_pub_ = this->create_publisher<std_msgs::msg::String>(
      "zbar_opencv_code",
      10);

    // ZBar：先關閉全部格式，再只啟用 QR Code。
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    last_qr_code_data_.clear();

    RCLCPP_INFO(
      this->get_logger(),
      "zbar_opencv node ready. Waiting for new QR codes...");
  }

private:
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(
        msg,
        sensor_msgs::image_encodings::BGR8);
    }
    catch (const cv_bridge::Exception & e)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "cv_bridge exception: %s",
        e.what());
      return;
    }

    // 將 ROS 影像轉為 OpenCV 灰階影像，提供 ZBar 掃描。
    cv::Mat & frame = cv_ptr->image;
    cv::Mat frame_grayscale;

    cv::cvtColor(
      frame,
      frame_grayscale,
      cv::COLOR_BGR2GRAY);

    const int width = frame_grayscale.cols;
    const int height = frame_grayscale.rows;

    auto * raw = frame_grayscale.data;

    zbar::Image image(
      width,
      height,
      "Y800",
      raw,
      static_cast<unsigned long>(width * height));

    scanner_.scan(image);

    for (
      zbar::Image::SymbolIterator symbol = image.symbol_begin();
      symbol != image.symbol_end();
      ++symbol)
    {
      const std::string current_qr_data = symbol->get_data();

      // 與原本 ROS 1 程式相同：
      // 新內容立即發布；相同內容每 500 ms 更新，讓下一次掃描仍能收到。
      const auto now = std::chrono::steady_clock::now();
      if (current_qr_data != last_qr_code_data_ ||
          now - last_qr_publish_ >= std::chrono::milliseconds(500))
      {
        last_qr_code_data_ = current_qr_data;
        last_qr_publish_ = now;

        std_msgs::msg::String msg_out;
        msg_out.data = current_qr_data;

        RCLCPP_INFO(
          this->get_logger(),
          "Found new QR Code: %s",
          msg_out.data.c_str());

        qr_code_pub_->publish(msg_out);
      }

      // 無論 QR Code 是否與上一筆相同，都持續畫出綠色辨識框。
      if (symbol->get_location_size() == 4)
      {
        const cv::Point p0(
          symbol->get_location_x(0),
          symbol->get_location_y(0));

        const cv::Point p1(
          symbol->get_location_x(1),
          symbol->get_location_y(1));

        const cv::Point p2(
          symbol->get_location_x(2),
          symbol->get_location_y(2));

        const cv::Point p3(
          symbol->get_location_x(3),
          symbol->get_location_y(3));

        cv::line(frame, p0, p1, cv::Scalar(0, 255, 0), 2);
        cv::line(frame, p1, p2, cv::Scalar(0, 255, 0), 2);
        cv::line(frame, p2, p3, cv::Scalar(0, 255, 0), 2);
        cv::line(frame, p3, p0, cv::Scalar(0, 255, 0), 2);
      }
    }

    // 發布處理後影像，可使用 rqt_image_view 查看。
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_code_pub_;

  zbar::ImageScanner scanner_;
  std::string last_qr_code_data_;
  std::chrono::steady_clock::time_point last_qr_publish_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ImageConverter>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
