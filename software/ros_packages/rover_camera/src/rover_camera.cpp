#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rover_camera_interface/msg/camera_control_message.hpp"
#include <string>
#include <iostream>
#include <sstream>
#include <ctime>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/qos.hpp>

using std::placeholders::_1;

// Useful info
// RTSP stream is 704x480
// 3 image channels
// Image type 16

class RoverCamera : public rclcpp::Node{

public:
    RoverCamera() : Node("camera") {

	auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                             .best_effort()  // Ensures message delivery
                             .durability_volatile() // Messages are not stored for late subscribers
                             .get_rmw_qos_profile();

        is_rtsp_camera = this->declare_parameter("is_rtsp_camera", false);
        capture_device_path = this->declare_parameter("device_path", std::string("/dev/video0"));
        fps = this->declare_parameter("fps", 20);

        upside_down = this->declare_parameter("upside_down", false);

        large_image_width = this->declare_parameter("large_image_width", 1280);
        large_image_height = this->declare_parameter("large_image_height", 720);
        medium_image_width = this->declare_parameter("medium_image_width", 640);
        medium_image_height = this->declare_parameter("medium_image_height", 360);
        small_image_width = this->declare_parameter("small_image_width", 256);
        small_image_height = this->declare_parameter("small_image_height", 144);

        base_topic = this->declare_parameter("base_topic", "cameras/main_navigation");

        broadcast_large_image = false;
        broadcast_medium_image = false;
        broadcast_small_image = true;

        if (is_rtsp_camera) {
            cap = new cv::VideoCapture(capture_device_path, cv::CAP_FFMPEG);
            RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to RTSP camera with path: " << capture_device_path);
        } else {
            cap = new cv::VideoCapture(capture_device_path, cv::CAP_V4L2);
            RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to USB camera with path: " << capture_device_path);

            cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap->set(cv::CAP_PROP_FRAME_WIDTH, large_image_width);
            cap->set(cv::CAP_PROP_FRAME_HEIGHT, large_image_height);
            cap->set(cv::CAP_PROP_FPS, fps);
        }

        large_image_node_name = base_topic + "/image_" + std::to_string(large_image_width) + "x" + std::to_string(large_image_height);
        medium_image_node_name = base_topic + "/image_" + std::to_string(medium_image_width) + "x" + std::to_string(medium_image_height);
        small_image_node_name = base_topic + "/image_" + std::to_string(small_image_width) + "x" + std::to_string(small_image_height);

        large_image_publisher = image_transport::create_publisher(this, large_image_node_name, qos_profile);
        medium_image_publisher = image_transport::create_publisher(this, medium_image_node_name, qos_profile);
        small_image_publisher = image_transport::create_publisher(this, small_image_node_name, qos_profile);

        control_subscriber = this->create_subscription<rover_camera_interface::msg::CameraControlMessage>(base_topic + "/camera_control", 1, std::bind(&RoverCamera::control_callback, this, _1));

        int period;
        if (is_rtsp_camera) {
            period = 1000/(fps + 10);
        } else {
            period = 1000/(fps + 2);
        }

        image_black = cv::Mat(large_image_height, large_image_width, CV_8UC3, cv::Scalar(0, 0, 0));

        if(!cap->isOpened()) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Failed to open capture for " << capture_device_path);
            return;
        }

        timer = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&RoverCamera::run, this));
    }

    void run(){
        if(!is_rtsp_camera) {
            cap->read(image_large);
        }else{
            cap->read(image_rtsp_raw);
            image_black.copyTo(image_large);

            float image_scalar = float(image_large.rows) / image_rtsp_raw.rows;

            cv::resize(image_rtsp_raw, image_rtsp_scaled, cv::Size(int(image_rtsp_raw.cols * image_scalar), int(image_rtsp_raw.rows * image_scalar)));

            int x = (image_large.cols - image_rtsp_scaled.cols) / 2;

            image_rtsp_scaled.copyTo(image_large(cv::Rect(x , 0, image_rtsp_scaled.cols, image_rtsp_scaled.rows)));
        }

        if(!image_large.empty()){
            if(upside_down){
                cv::flip(image_large, image_large, -1);
            }

            if(broadcast_large_image){
                large_image_message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_large).toImageMsg();
                large_image_publisher.publish(large_image_message);
            } else if (broadcast_medium_image){
                cv::resize(image_large, image_medium, cv::Size(medium_image_width, medium_image_height));
                medium_image_message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_medium).toImageMsg();
                medium_image_publisher.publish(medium_image_message);
            } else if (broadcast_small_image){
                cv::resize(image_large, image_small, cv::Size(small_image_width, small_image_height));
                small_image_message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_small).toImageMsg();
                small_image_publisher.publish(small_image_message);
            }
        }
    }

    void control_callback(const rover_camera_interface::msg::CameraControlMessage::SharedPtr msg) {
        if (msg->take_photo) {
            if (!image_large.empty()) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Taking photo with " << base_topic);
                std::time_t unix_time = std::time(nullptr);
                std::stringstream filepath;
                filepath << "/photos/" << this->get_name() << std::put_time(std::localtime(&unix_time), "_%m_%d_%Y_%H_%M_%S.png");
                cv::imwrite(filepath.str(), image_large);
            }
        } else {
            broadcast_small_image = msg->enable_small_broadcast;
            broadcast_medium_image = msg->enable_medium_broadcast;
            broadcast_large_image = msg->enable_large_broadcast;
        }
    }

    ~RoverCamera(){
        if(cap->isOpened()){
            cap->release();
        }
    }

private:
    cv::VideoCapture *cap;

    bool is_rtsp_camera;

    std::string capture_device_path;
    int fps;

    rclcpp::TimerBase::SharedPtr timer;

    bool upside_down;

    int large_image_width;
    int large_image_height;
    int medium_image_width;
    int medium_image_height;
    int small_image_width;
    int small_image_height;

    bool broadcast_large_image;
    bool broadcast_medium_image;
    bool broadcast_small_image;

    std::string base_topic;

    std::string large_image_node_name;
    std::string medium_image_node_name;
    std::string small_image_node_name;

    image_transport::Publisher large_image_publisher;
    image_transport::Publisher medium_image_publisher;
    image_transport::Publisher small_image_publisher;

    rclcpp::Subscription<rover_camera_interface::msg::CameraControlMessage>::SharedPtr control_subscriber;

    cv::Mat image_black;

    cv::Mat image_rtsp_raw;
    cv::Mat image_rtsp_scaled;
    cv::Mat image_large;
    cv::Mat image_medium;
    cv::Mat image_small;

    sensor_msgs::msg::Image::SharedPtr large_image_message;
    sensor_msgs::msg::Image::SharedPtr medium_image_message;
    sensor_msgs::msg::Image::SharedPtr small_image_message;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverCamera>());
    rclcpp::shutdown();
}
