// pc_filter.cpp
//
// Osian Leahy
//
// This node removes the table points from the point cloud data given for HW7, leaving just the bottles.

// Include the basic ROS 2 stuff.
#include <rclcpp/rclcpp.hpp>

//Include the ROS2 msg for point cloud:
#include <sensor_msgs/msg/point_cloud2.hpp>

//Include the pointcloud stuff (General)
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

//Include the pointcloud stuff (Specific)
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

//Include geometry messages for visualizing the plane:
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <shape_msgs/msg/plane.hpp>

//Placeholders for callback binding:
using std::placeholders::_1;

const bool vis_pub = true; //TODO: Make this a ros2 param?

// The idiom in C++ is the same as in Python; we create a class that
// inherits from the ROS 2 Node class.
class FilterCloud : public rclcpp::Node {
public:
	
	//Simplify the syntax by abstracting this namespace
	using PointCloud2 = sensor_msgs::msg::PointCloud2;
	using Marker = visualization_msgs::msg::Marker;
	using Point = geometry_msgs::msg::Point;
	using Plane = shape_msgs::msg::Plane;

	//(I Think)This needs to be public because it's used to construct the class
	FilterCloud() : Node("plane_fit") {
		
		//Subscribe to the rosbag stream
		subscriber_ = this->create_subscription<PointCloud2>("truncated_point_cloud", 5, std::bind(&FilterCloud::sub_callback, this, _1));
		
		//Republish to:
		publisher_ = this->create_publisher<PointCloud2>("object_pc", 10);

		//Also publish a polygon representing the plane to:
		plane_publisher_ = this->create_publisher<Marker>("plane_viz",10);

		//Also Also publish an array message of the current planefit parameters:
		plane_param_publisher_ = this->create_publisher<Plane>("ground_plane",10);

		RCLCPP_INFO(this->get_logger(), "plane_fit started");

	}

private:

	//Class variable prototypes, 2 pass compiler blah blah
	rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
	rclcpp::Publisher<Marker>::SharedPtr plane_publisher_;
	rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;
	rclcpp::Publisher<Plane>::SharedPtr plane_param_publisher_;

	//Where the magic is supposed to happen...
	void sub_callback(const PointCloud2::SharedPtr msg) {
		//Log that we've recieved a msg
		RCLCPP_INFO(this->get_logger(),"Point Cloud Recieved");

		//First, pull in the point cloud, translate from ROS PC2 to PCL:
		//We get RGB data with this camera, as specified in the PC2 message fields
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //Create the point cloud object
		pcl::fromROSMsg(*msg, *pcl_cloud); //pass it and the msg by reference for conversion
		
		if (pcl_cloud->size() < 100) {
			RCLCPP_WARN(
				this->get_logger(),
				"Point cloud has fewer than 100 points, skipping processing."
			);
		return;
}

		//Create output object:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_table_removed(new pcl::PointCloud<pcl::PointXYZRGB>);

        //We want to fit a plane and remove everything below (and probably slightly above) The plane.
        //First create the model return objects (plane coefficients and inlying point indicies)
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory, set the model to fit, method, and point->model distance threshold
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.05);

        //Then fit it to the point cloud
        seg.setInputCloud(pcl_cloud);
        seg.segment(*inliers, *coefficients);
		

        //Finally remove the inlier points from the larger point cloud (since we care about the stuff on/above the table.)
        //Using the extract_indices filter class:
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(pcl_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);

        extract.filter(*pcl_cloud_table_removed);

		//Convert back into a ros message and output:
		PointCloud2 msg_out;
		pcl::toROSMsg(*pcl_cloud_table_removed, msg_out);
		msg_out.header = msg->header;

		//Publish and log the resulting point cloud:
		this->publisher_->publish(msg_out);
		RCLCPP_INFO(this->get_logger(), "point cloud truncated.");

		//Also Publish the ground plane:
		Plane plane_params;
		plane_params.coef[0] = coefficients->values[0];
		plane_params.coef[1] = coefficients->values[1];
		plane_params.coef[2] = coefficients->values[2];
		plane_params.coef[3] = coefficients->values[3];
		
		this->plane_param_publisher_->publish(plane_params);

		//Publish the Plane visualization as a marker topic
		if (vis_pub){

			//We also want to visualize the plane:
			//Lazy way: evaluate the plane at 4 points, draw this as a polygon in Rviz:
			//Plane coeffs

			float a = coefficients->values[0];
			float b = coefficients->values[1];
			float c = coefficients->values[2];
			float d = coefficients->values[3];
			
			//Four static xy pairs to evaluate for z at
			std::vector<Point> plane_points = {Point(),Point(),Point(),Point(),Point()};
			plane_points[0].x = -0.5;
			plane_points[0].y = -0.5;
			plane_points[1].x = 0.5;
			plane_points[1].y = -0.5;
			plane_points[2].x = 0.5;
			plane_points[2].y = 0.5;
			plane_points[3].x = -0.5;
			plane_points[3].y = 0.5;
			plane_points[4] = plane_points[0];
			//evaluate each for z
			for(int i=0; i<=4; i++){
				plane_points[i].z = (-d-a*plane_points[i].x-b*plane_points[i].y)/c;
			}

			//TBH, it would probably have been easier to just find the centroid of the table points, determine the proper rotation angle using the 
			//plane normal vector, and then drawing a cube with 0 height and the correct pose, oh well...

			//assign them to a marker and publish
			Marker plane_marker;

			//plane_marker.header.frame_id = "/quori/head_camera_optical"; //Old HW Rosbag Frame
			plane_marker.header.frame_id = "/base_link"; //Rover D405 Frame
			plane_marker.header.stamp = rclcpp::Clock().now();

			plane_marker.pose.position.x = 0;
			plane_marker.pose.position.y = 0;
			plane_marker.pose.position.z = 0;
			plane_marker.pose.orientation.x = 0.0;
			plane_marker.pose.orientation.y = 0.0;
			plane_marker.pose.orientation.z = 0.0;
			plane_marker.pose.orientation.w = 1.0;

			plane_marker.color.r = 0.0f;
			plane_marker.color.g = 1.0f;
			plane_marker.color.b = 0.0f;
			plane_marker.color.a = 1.0;

			plane_marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

			plane_marker.ns = "/";
			plane_marker.id = 0;

			plane_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

			plane_marker.scale.x = 0.05;
			plane_marker.points = plane_points;
			this->plane_publisher_->publish(plane_marker);
		}
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
