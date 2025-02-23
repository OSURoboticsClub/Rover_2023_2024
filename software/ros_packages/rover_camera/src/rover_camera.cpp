#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rover_camera_interface/msg/camera_control_message.hpp"
#include <string>
#include <iostream>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/qos.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include "std_msgs/msg/byte_multi_array.hpp"
using std::placeholders::_1;

// Useful info
// RTSP stream is 704x480
// 3 image channels
// Image type 16

class RoverCamera : public rclcpp::Node{

public:
    RoverCamera() : Node("camera") {
        gst_init(nullptr, nullptr);
	auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(30))
                             .best_effort()  // Ensures message delivery
                             .durability_volatile(); // Messages are not stored for late subscribers
                             //.get_rmw_qos_profile();

        is_rtsp_camera = this->declare_parameter("is_rtsp_camera", false);
        capture_device_path = this->declare_parameter("device_path", std::string("/dev/video0"));
        fps = this->declare_parameter("fps", 30);

        upside_down = this->declare_parameter("upside_down", false);

        large_image_width = this->declare_parameter("large_image_width", 800);
        large_image_height = this->declare_parameter("large_image_height", 600);
        small_image_width = this->declare_parameter("small_image_width", 256);
        small_image_height = this->declare_parameter("small_image_height", 144);

        base_topic = this->declare_parameter("base_topic", "cameras/main_navigation");

        broadcast_large_image = true;
        broadcast_small_image = false;
        
        if (is_rtsp_camera) {
            cap = new cv::VideoCapture(capture_device_path, cv::CAP_FFMPEG);
            RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to RTSP camera with path: " << capture_device_path);
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(), "Attempting to open pipeline");
            pipeline_str = "v4l2src device={device_path} ! "
                "videorate ! video/x-raw,framerate=30/1,width=800,height=600 ! "
                "nvvidconv ! nvv4l2h265enc ! h265parse ! "
                "appsink name=sink sync=false";
            size_t pos = pipeline_str.find("{device_path}");
            pipeline_str.replace(pos, std::string("{device_path}").length(), capture_device_path);
            RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to USB camera with path: " << capture_device_path);

        }

        large_image_node_name = base_topic + "/image_" + std::to_string(large_image_width) + "x" + std::to_string(large_image_height);
        small_image_node_name = base_topic + "/image_" + std::to_string(small_image_width) + "x" + std::to_string(small_image_height);

        //rclcpp::Node::SharedPtr node_handle_ = std::shared_ptr<RoverCamera>(this);
        large_image_publisher = this->create_publisher<std_msgs::msg::ByteMultiArray>(large_image_node_name, qos_profile);
        small_image_publisher = this->create_publisher<std_msgs::msg::ByteMultiArray>(small_image_node_name, qos_profile);

        //large_image_publisher = large_image_transport->advertise(large_image_node_name, 1);
        //small_image_publisher = small_image_transport->advertise(small_image_node_name, 1);

        control_subscriber = this->create_subscription<rover_camera_interface::msg::CameraControlMessage>(base_topic + "/camera_control", 1, std::bind(&RoverCamera::control_callback, this, _1));

        int period;
        if (is_rtsp_camera) {
            period = 1000/(fps + 10);
        } else {
            period = 1000/(fps + 2);
        }
        error = nullptr;
        pipeline = gst_parse_launch(pipeline_str.c_str(), &error);

        if(!pipeline) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Failed to open capture for " << capture_device_path);
            g_clear_error(&error);
            return;
        }
        
        // Get appsink element
        appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
        if (!appsink) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Failed to get appsink");	
            gst_object_unref(pipeline);
            return;
        }

        g_object_set(G_OBJECT(appsink), "max-buffers", 1, "drop", TRUE, NULL);

        // Set pipeline state to playing
        ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Failed to set pipeline after getting state for " << capture_device_path);
            gst_object_unref(appsink);
            gst_object_unref(pipeline);
            return;
        }
        timer = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&RoverCamera::run, this));
    }

    void run(){
        GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
        if (!sample) {
	    RCLCPP_INFO_STREAM(this->get_logger(), "Failed to get sample");
            return;
        }
        // Get buffer from sample
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Failed to get buffer");
            gst_sample_unref(sample);
            return;
        }
        // Map buffer
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            // Print first 50 bytes of data
            large_image_message.data.resize(map.size);
            std::memcpy(large_image_message.data.data(), map.data, map.size);
            large_image_publisher -> publish(large_image_message);
            RCLCPP_INFO_STREAM(this->get_logger(), "image published"); 
            // Print buffer size
            // Unmap buffer
            gst_buffer_unmap(buffer, &map);
        }

        // Clean up sample
        gst_sample_unref(sample);
    }

    void control_callback(const rover_camera_interface::msg::CameraControlMessage::SharedPtr msg) {
        broadcast_small_image = msg->enable_small_broadcast;
        broadcast_large_image = msg->enable_large_broadcast;
    }

private:
    GstStateChangeReturn ret;
    GstElement *appsink;
    GError *error;
    GstElement *pipeline;

    cv::VideoCapture *cap;

    bool is_rtsp_camera;

    std::string capture_device_path;
    int fps;

    rclcpp::TimerBase::SharedPtr timer;

    bool upside_down;

    int large_image_width;
    int large_image_height;
    int small_image_width;
    int small_image_height;

    bool broadcast_large_image;
    bool broadcast_small_image;

    std::string base_topic;

    std::string pipeline_str;

    std::string large_image_node_name;
    std::string small_image_node_name;

    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr large_image_publisher;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr small_image_publisher;

    rclcpp::Subscription<rover_camera_interface::msg::CameraControlMessage>::SharedPtr control_subscriber;

    cv::Mat image_black;

    cv::Mat image_rtsp_raw;
    cv::Mat image_rtsp_scaled;
    cv::Mat image_large;
    cv::Mat image_small;

    std_msgs::msg::ByteMultiArray large_image_message;
    std_msgs::msg::ByteMultiArray small_image_message;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverCamera>());
    rclcpp::shutdown();
}
