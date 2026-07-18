// pc_filter.cpp
//
// Osian Leahy
//
// This node removes the extreme points from the point cloud data given for HW7, leaving just the table and stuff on it

// Include the basic ROS 2 stuff.
#include <rclcpp/rclcpp.hpp>

//Include the ROS2 msg for point cloud:
#include <sensor_msgs/msg/point_cloud2.hpp>

//Include the pointcloud stuff
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

//transforms.hpp has a function we can use to apply a tf2 transform to our point cloud without having to convert it back into a ros message
//This lets us do some filtering/truncation in the gripper frame before shifting back over to the rover frame.
#include <pcl_ros/transforms.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

//Placeholders for callback binding:
using std::placeholders::_1;

// The idiom in C++ is the same as in Python; we create a class that
// inherits from the ROS 2 Node class.
class FilterCloud : public rclcpp::Node {
public:
	
	//Simplify the syntax by abstracting this namespace
	using PointCloud2 = sensor_msgs::msg::PointCloud2;

	//This needs to be public because it's used to construct the class
	FilterCloud() : Node("pc_filter"),
		tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
		tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
		target_frame_("rover_arm_base_link") {
		
		//Subscribe to the rosbag stream
		subscriber_ = this->create_subscription<PointCloud2>("raw_point_cloud", 5, std::bind(&FilterCloud::sub_callback, this, _1));
		
		//Republish to:
		publisher_ = this->create_publisher<PointCloud2>("truncated_point_cloud", 10);

		RCLCPP_INFO(this->get_logger(), "pc_filter started");

	}

private:

	//Class variable prototypes, 2 pass compiler blah blah
	rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
	rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	std::string target_frame_;

	//Where the magic is supposed to happen...
	void sub_callback(const PointCloud2::SharedPtr msg) {
		//Log that we've recieved a msg
		// RCLCPP_INFO(this->get_logger(),"Point Cloud Recieved");

		//First, pull in the point cloud, translate from ROS PC2 to PCL:
		//We get RGB data with this camera, as specified in the PC2 message fields
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>); //Create the point cloud object
		pcl::fromROSMsg(*msg, *pcl_cloud_in); //pass it and the msg by reference for conversion
		// RCLCPP_INFO(this->get_logger(),"Unpacked");

		//Second, we want to remove any points that are close enough that it sees the gripper fingers, 
		//or far enough that its significantly bebyond the accuarate range of the camera 
		//Bounding points in the camera frame, Z is the optical axis
		float x_min = -0.5;
		float x_max = 0.5;
		float y_min = -0.5;
		float y_max = 0.5;
		float z_min = 0.2; //Exclude the gripper fingers
		float z_max = 0.7;

		//Create the cropbox filter:
		pcl::CropBox<pcl::PointXYZRGB> box_filter;
		box_filter.setInputCloud(pcl_cloud_in);
		box_filter.setMin(Eigen::Vector4f(x_min,y_min,z_min,1.0f));
		box_filter.setMax(Eigen::Vector4f(x_max,y_max,z_max,1.0f));

		//Create an output object in the gripper frame:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr gripper_cloud_trunc(new pcl::PointCloud<pcl::PointXYZRGB>);
		
		
		//Apply the box filter and save to the output object
		box_filter.filter(*gripper_cloud_trunc);
		// RCLCPP_INFO(this->get_logger(),"filtered gripper");


		//Third, we want to transform the could into the robot frame, and use some bounding boxes to get rid any points related to the wheels:

		//Create an output object in the robot frame:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr rover_cloud_trunc(new pcl::PointCloud<pcl::PointXYZRGB>);
		// RCLCPP_INFO(this->get_logger(),"THis try is soo good");

		//Try and get the transform tree
		try {

			geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp);
			// RCLCPP_INFO(this->get_logger(),"THis try sucks right here");

			if (gripper_cloud_trunc->empty()) {
				RCLCPP_WARN(this->get_logger(), "Input cloud empty — skipping transform");
				return;
			}

			for (const auto &p : gripper_cloud_trunc->points) {
				if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
					RCLCPP_ERROR(this->get_logger(), "NaN point detected — aborting transform");
					return;
				}
			}
			//Transform this pointcloud into the Rover frame;
			pcl_ros::transformPointCloud(*gripper_cloud_trunc,*rover_cloud_trunc,transform_stamped);

			

		} catch (const tf2::TransformException &ex) {
			RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
			
			//If we can't get the tf2, give up on this frame. (Something probably isnt working anyways)
			return;
		
		}
		catch (...) {
			RCLCPP_ERROR(this->get_logger(), "Unknown exception in transformPointCloud");
			return;
		}
		// RCLCPP_INFO(this->get_logger(),"THis try sucks");


		//Make two new box filters in front of the wheels (roughly where we see them show up in the scan)
		pcl::CropBox<pcl::PointXYZRGB> left_wheel;
		pcl::CropBox<pcl::PointXYZRGB> right_wheel;

		//Define the wheel dimensions:
		float wheel_r = 0.4/2; //meters, ~11.8" Wheel diameter
		float wheel_w = 0.3; //meters, this is larger than required but helps obscure the motor drivers/bogies:

		//And offsets relative to base_link:
		//Note: X is robot forward in base_link
		float x_offset = 0.22; //meters, offset from front CF tube
		float y_offset = 0.35; //meters, base_link_center to ~center of width
		float z_offset = -0.09; //meters, base_link to wheel center

		//Set the Crop box min and max dimensions:
		right_wheel.setMin(Eigen::Vector4f(x_offset-wheel_r,-y_offset-(wheel_w/2),z_offset-wheel_r,1.0f));
		right_wheel.setMax(Eigen::Vector4f(x_offset+wheel_r,-y_offset+(wheel_w/2),z_offset+wheel_r,1.0f));
		left_wheel.setMin(Eigen::Vector4f(x_offset-wheel_r,y_offset-(wheel_w/2),z_offset-wheel_r,1.0f));
		left_wheel.setMax(Eigen::Vector4f(x_offset+wheel_r,y_offset+(wheel_w/2),z_offset+wheel_r,1.0f));
		// RCLCPP_INFO(this->get_logger(), "Prefilter length: %zu", rover_cloud_trunc->size());

		//Remove the right wheel:
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_removed(new pcl::PointCloud<pcl::PointXYZRGB>);
		right_wheel.setInputCloud(rover_cloud_trunc);
		right_wheel.setNegative(true);
		right_wheel.filter(*rover_cloud_trunc);
		// RCLCPP_INFO(this->get_logger(), "Post_filter length: %zu", rover_cloud_trunc->size());


		//And the left:
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_removed(new pcl::PointCloud<pcl::PointXYZRGB>);
		left_wheel.setInputCloud(rover_cloud_trunc);
		left_wheel.setNegative(true);
		left_wheel.filter(*rover_cloud_trunc);
		// RCLCPP_INFO(this->get_logger(), "Post_filter length: %zu", rover_cloud_trunc->size());


		//TODO: make these use the transform tree to snap to the wheel locations
		//This needs a better whole-rover URDF

		//Convert back into a ros message and output:
		PointCloud2 msg_out;
		pcl::toROSMsg(*rover_cloud_trunc, msg_out);
		msg_out.header = msg->header;
		msg_out.header.frame_id = target_frame_;

		//Publish and log the result:
		this->publisher_->publish(msg_out);
		// RCLCPP_INFO(this->get_logger(), "point cloud truncated.");

	}
};


// This is the entry point for the executable. 
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node instance and store a shared pointer to it.
	auto node = std::make_shared<FilterCloud>();

	// Give control to ROS via the shared pointer.
	rclcpp::spin(node);

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	// Main always returns 0 on success.
	return 0;
}