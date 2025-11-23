// pc_filter.cpp
//
// Osian Leahy
//
// This Node counts the objects within a pre-processed pointcloud 

// Include the basic ROS 2 stuff.
#include <rclcpp/rclcpp.hpp>

//Include the ROS2 msg for point cloud, which we will recieve and publish:
#include <sensor_msgs/msg/point_cloud2.hpp>
//Also include the geometry message for marker arrays, which we will publish.
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//Service types for reseting our list of objects
#include "pc_processing/srv/reset_objects.hpp"

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
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/distances.h>

//Placeholders for callback binding:
using std::placeholders::_1;
using std::placeholders::_2;

//TODO: This should probably be a lifecycle node...

// The idiom in C++ is the same as in Python; we create a class that
// inherits from the ROS 2 Node class.
class FilterCloud : public rclcpp::Node {
public:
	
	//Simplify the syntax by abstracting this namespace
	using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Marker = visualization_msgs::msg::Marker;
    using ResetObjects = pc_processing::srv::ResetObjects;

	//(I Think)This needs to be public because it's used to construct the class
	FilterCloud() : Node("count_objects") {
		
		//Subscribe to the rosbag stream
		subscriber_ = this->create_subscription<PointCloud2>("object_pc", 5, std::bind(&FilterCloud::sub_callback, this, _1));
		
		//Republish to:
		new_marker_publisher_ = this->create_publisher<MarkerArray>("current_object_markers", 10);
        //and
        all_marker_publisher_ = this->create_publisher<MarkerArray>("all_object_markers",10);
        //and
        basic_points_publisher_ = this->create_publisher<PointCloud2>("obj_centroids",10);

        //Service to reset the list of centroids and start/stop the node
        reset_service_ = this->create_service<ResetObjects>("reset_pc_processing", std::bind(&FilterCloud::service_callback, this, _1, _2));

        //Threshold distance to check if objects in previous scans are the same
        thresh_dist = 0.1;

        //Complementary filter value for weighted sum of centroids 
        filter_val = 0.8;

        //Start with this node disabled, user will enable it with a service call:
        enable = false;

        RCLCPP_INFO(this->get_logger(), "Starting Count");
        
	}

private:

	//Class variable prototypes, 2 pass compiler blah blah
	//Publishers, subscriptions, Service servers
    rclcpp::Publisher<MarkerArray>::SharedPtr new_marker_publisher_;
    rclcpp::Publisher<MarkerArray>::SharedPtr all_marker_publisher_;
    rclcpp::Publisher<PointCloud2>::SharedPtr basic_points_publisher_;
	rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;
    rclcpp::Service<ResetObjects>::SharedPtr reset_service_;

    //point cloud to store the found object centroids in: 
    pcl::PointCloud<pcl::PointXYZ> obj_centroids{};  
    //Threshold distance for consolidating prior and current centroids:
    float thresh_dist; //I think this is in meters?
    
    //Complementary filter value:
    float filter_val; 

    //Enable/Disable the whole loop:
    bool enable;


	//Callback function for when a new pre-processed pointcloud is recieved
	void sub_callback(const PointCloud2::SharedPtr msg) {

        //Don't do anything unless we are enabled
        if (this->enable){

            //Log that we've recieved a msg
            //RCLCPP_INFO(this->get_logger(),"Point Cloud Recieved");
            //First, pull in the point cloud, translate from ROS PC2 to PCL:
            //We get RGB data with this camera, as specified in the PC2 message fields
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); //Create the point cloud object
            pcl::fromROSMsg(*msg, *pcl_cloud); //pass it and the msg by reference for conversion
            //Create output objects:
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_object_centers(new pcl::PointCloud<pcl::PointXYZ>);
            int object_count = 0;
            
            RCLCPP_INFO(this->get_logger(),"Input Width: %d",pcl_cloud->width);
            RCLCPP_INFO(this->get_logger(),"Input Height: %d",pcl_cloud->height);

            //Clustering with a Kd tree and euclidian clustering
            //Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(pcl_cloud);
            //Creating the vector to store groups of cluster indices in: 
            std::vector<pcl::PointIndices> cluster_indices;
            //and the extraction class object.
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.01);
            ec.setMinClusterSize(1000);
            ec.setMaxClusterSize(15000);
            //Running the search and extracting to our indices vector
            ec.setSearchMethod(tree);
            ec.setInputCloud(pcl_cloud);
            ec.extract(cluster_indices);

            //Find the average point of each cluster, and also count the clusters.
            //For cluster within the vector of clusters
            for(pcl::PointIndices indices : cluster_indices){
                //Pointer shennanigans
                pcl::PointIndices::Ptr idxPtr = pcl::make_shared<pcl::PointIndices>(indices);
                
                //Extract the indices to a point cloud:
                pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);

                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(pcl_cloud);
                extract.setIndices(idxPtr);
                extract.setNegative(false);
                extract.filter(*object_cloud);
                
                pcl::PointXYZ average_point;
                average_point.z = -1000000; //Initialize as a very low value
                //Iterate over the points w/in the cluster and get an average point:
                for(auto& point : *object_cloud){
                    average_point.x += point.x;
                    average_point.y += point.y;
                    //Actually, lets use the max height of the object that we saw:
                    average_point.z = std::max(point.z,average_point.z);
                    
                    //average_point.z += point.z;
                    // average_point.r += point.r;
                    // average_point.g += point.g;
                    // average_point.b += point.b;
                }
                //Normalize the avg
                float num_pts = (object_cloud->height * object_cloud->width);
                average_point.x = average_point.x/num_pts;
                average_point.y = average_point.y/num_pts;
                //average_point.z = average_point.z/num_pts;
                // average_point.r = average_point.r/num_pts;
                // average_point.g = average_point.g/num_pts;
                // average_point.b = average_point.b/num_pts;

                //Add it to pcl_object_centers:
                pcl_object_centers->push_back(average_point);

                //Add to the object count
                object_count++;
            }

            
            RCLCPP_INFO(this->get_logger(),"Starting Comparison");

            //Check if there are points in the centroid list
            if (this->obj_centroids.width * this->obj_centroids.height == 0){
                RCLCPP_INFO(this->get_logger(),"Existing cloud is empty");
                //If the list doesn't exist, build it from this frame:
                this->obj_centroids = *pcl_object_centers;
                //This should get run again if pcl_object_centers happens to also be zero length
            }
            else {
                RCLCPP_INFO(this->get_logger(),"Comparing with Existing Cloud");
                //If the list exists, Compare the new centroids against our existing vector of centroids
                //Iterate over the new centroids
                for (pcl::PointXYZ new_center : *pcl_object_centers){
                    
                    //Store best distance between our new point and all old points
                    double best_dist = 0.0;
                    int best_idx = 0;
                    

                    //Iterate over the old centroids:
                    for (unsigned int i = 0; i < this->obj_centroids.height*this->obj_centroids.width; i++){
                        //Calculate and save the closest distance between the current new centroid and old centroid
                        double dist = pcl::euclideanDistance(new_center,this->obj_centroids.points[i]);
                        
                        //Initial Condition, Find shortest distance between new center and all old centers
                        if (best_dist == 0 || dist < best_dist){
                            //Save the distance and index:
                            best_dist = dist;
                            best_idx = i;
                        }
                    }
                                    
                    //Check if that distance is better than our threshold
                    if (best_dist < this->thresh_dist){
                        //If it is, take a weighted average of the current points and update the corresponding point in the pointcloud
                        this->obj_centroids.points[best_idx].x = this->filter_val * this->obj_centroids.points[best_idx].x + (1-this->filter_val) * new_center.x;

                    }
                    else {
                        //If it's not, append the point to the pointcloud of centroids.
                        this->obj_centroids.push_back(new_center);
                    }

                }

            }
            
            //Publish our updated list of object centers:
            PointCloud2 obj_list_msg;
            pcl::toROSMsg(this->obj_centroids,obj_list_msg);
            this->basic_points_publisher_->publish(obj_list_msg);        

            //Publish and log our object centers as markers:
            //New Object centers, published as markers
            float new_colors[] = {0.0,1.0,0.0};
            this->new_marker_publisher_->publish(marker_from_cloud(*pcl_object_centers,new_colors,3));
            //All Object centers, published as markers
            float all_colors[] = {0.0,0.0,1.0};
            this->all_marker_publisher_->publish(marker_from_cloud(obj_centroids,all_colors,50));

            RCLCPP_INFO(this->get_logger(), "%d Points Found",object_count);
        }
        //If the enable variable is off, note that we're skipping this frame:
        else{
            RCLCPP_INFO(this->get_logger(),"this->enable is false, skipping frame");
        }
	}

    //Callback function to handle starting and stopping the monitoring process:
    void service_callback(const std::shared_ptr<pc_processing::srv::ResetObjects::Request> request, std::shared_ptr<pc_processing::srv::ResetObjects::Response> response){

        //Whether to enable or disable the node
        if (request->enable){
            this->enable = true;
        }
        else {
            this->enable = false;
        }
        
        //Whether to reset the object cloud 
        if (request->reset){
            this->obj_centroids.clear();
        }

        //Return a response acknowledging the request:
        response->acknowledge = true;
    }

    //Function which converts a point cloud into a list of markers for slightly easier vizualization in RViz:
    //Inputs: PCL point cloud (not a pointer), float array of colors, # of seconds for markers to last. 
    MarkerArray marker_from_cloud(pcl::PointCloud<pcl::PointXYZ> cloud, float colors[3],int seconds){

        //Create a marker array to be filled in:
        std::vector<Marker> obj_markers;
        
        //Iterate over all our points to create markers of them
        int marker_count = 0; //Probably shoudlve just done this in the same for loop above lmao
        for (pcl::PointXYZ point : cloud){
            Marker marker = Marker();
            
            //Fill in the excessive amount of marker data
            marker.header.frame_id = "base_link";
            marker.header.stamp = rclcpp::Clock().now();

            marker.ns = "/";
            marker.id = marker_count;

            marker.type = visualization_msgs::msg::Marker::SPHERE; //Just gonna do spheres because I don't wanna rotate the cylinders LOL 

            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = point.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.08;
            marker.scale.y = 0.08;
            marker.scale.z = 0.08;

            marker.color.r = colors[0];
            marker.color.g = colors[1];
            marker.color.b = colors[2];
            marker.color.a = 1.0;   // Don't forget to set the alpha!

            marker.lifetime = rclcpp::Duration::from_seconds(seconds);

            //Add it to the list that gets published
            obj_markers.push_back(marker);

            marker_count++;

        }

        //Move the array into the message
        MarkerArray obj_markers_msg;
        obj_markers_msg.markers = obj_markers;

        return obj_markers_msg;

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