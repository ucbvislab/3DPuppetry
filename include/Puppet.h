/*
 * Puppet.h
 *
  *     Created: 2011/2012
  *      Author: Robin Held (begun by Ankit Gupta)
  *
  *      This class is used to represent each tracked puppet and includes
  *      functions that use the iterative closest point (ICP) algorithm
  *
  *      See LICENSE.txt for licensing info.
  */

#ifndef PUPPET_H_
#define PUPPET_H_


#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/publisher.h>
#include "point_cloud_icp/registration/icp_combined.h"

// Old dependencies:
//#include "rgbd_util/ros_utility.h"
//#include <rgbd_msgs/DepthMap.h>
//#include "pcl_rgbd/cloudTofroPLY.h"
//#include "pcl_rgbd/depth_to_cloud_lib.h"
//#include "pcl_rgbd/cloudGeometry.h"
//#include "pcl_rgbd/pointTypes.h"
//#include "pcl_rgbd/cloudUtils.h"
//#include <pcl/ros/conversions.h>
//#include <pcl/point_cloud.h>
//#include <map>

using namespace std;
using namespace Eigen;
using namespace registration;

#define PI 			3.14159

struct LegoBlock
{
	int ID;
	bool rot;
	float trans[3];
	float color[3];
	int voxels[8][3];
};

typedef rgbd::pt PointT;

typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

enum ICP_passes { COARSE_ICP_ONLY, FINE_ICP_ONLY, TWO_STAGE_ICP };

/////////////////////////
// Variables and functions for multi-threading:

struct AlignData
{
	pcl::PointCloud<PointT> curr_cloud;
	pcl::PointCloud<PointT> cloud;
	Eigen::Transform3f 	xform;
	Eigen::Transform3f 	temp_pose;
	vector<float> 		centroid;
	vector<float> 		err_vector;
	int 	num_outliers;
	float	perc_outliers;
	float 	icp_error_local;
	float	puppet_max_distance;
	int		max_ICP_rounds;
	bool	use_outlier_error;
	bool 	useNormals;
	float 	max_normal_difference_deg;
};

void* thread_align_cloud_init(void* threaddata);

float alignCloudsWithInit(pcl::PointCloud<PointT> curr_cloud, pcl::PointCloud<PointT> fullcloud,
							vector<float> &err_vector, int &num_outliers, float &perc_outliers, float &icp_err, Eigen::Transform3f &T,
							vector<float> centroid, float puppet_max_distance, int max_ICP_rounds, bool useNormals,
							float max_normal_difference_deg);


class Puppet {
public:

	Puppet();

	// PLY / OpenGL / 3D-model functions
	void loadPLY(std::string fname);
	void loadDisplayPLY(std::string fname);
	void saveNewPly();
	Eigen::Vector3f calculateNormal(Eigen::Vector3f pt1, Eigen::Vector3f pt2,Eigen::Vector3f pt3 );
	bool parsePLYLine(string line);
	void loadBlock(pcl::PointCloud<PointT> &cloud_out);
	void loadLegoPuppet(std::string filename, std::string filespath);
	void findCentroid();
	void findMaxDistance();
	bool checkVisibility();

	// ICP functions
	void updatePose(pcl::PointCloud<PointT> *curr_cloud, ICP_passes pass_config, bool reset_clouds, float outlier_pct);
	void resetPuppetCloud();
	void transformPuppetCloud();
	void updatePoseInitRobust(pcl::PointCloud<PointT> curr_cloud);
	void updatePoseTrackInverseCoarse(pcl::PointCloud<PointT> *curr_cloud, bool reset_clouds, int max_rounds, float outlier_pct);
	void updatePoseTrackInverseFine(pcl::PointCloud<PointT> *curr_cloud, bool reset_clouds, int max_rounds, float outlier_pct);
	float findOutliersPercentOnly(float max_distance);
	float markOutliers(vector<vector<vector<int> > > corres, pcl::PointCloud<PointT> *cloud);
	void updateCloudPair(pcl::PointCloud<PointT> *curr_cloud);

	// Outdated and may not work properly: ////////////
	void updatePoseInit(pcl::PointCloud<PointT> curr_cloud);
	void updatePoseInitInverse(pcl::PointCloud<PointT> *curr_cloud);
	void updatePoseTrack(pcl::PointCloud<PointT> *curr_cloud);
	///////////////////////////////////////////////////

	// Set functions
	void setTrackingRegion(float h_extent, float v_extent, float nearest);
	void setMultiCore(bool multiCore){use_multi_core = multiCore;};
	void setICPThreshold(float thresh){icp_error_threshold = thresh;};
	void setInverseICP(bool inverse){use_inverse_icp = inverse;};
	void setPuppetMatch(bool puppet_to_cloud){puppet_as_source = puppet_to_cloud;};
	void setThreads(int t){simultaneous_threads = t;};
	void setAngleSpacing(int a){angle_spacing = a;};
	void setMaxICPRounds(int r){max_ICP_rounds = r;};
	void setCurrentPose(vector<float> new_pose);
	void setCurrentPoseTransform(Transform3f new_pose){pose = new_pose;};
	void resetCloudPairStatus(){cloud_pair_created = false;};
	void enableCloudNormals(){useNormals = true;};
	void disableCloudNormals(){useNormals = false;};
	void setMaxNormalDifference(float max_diff){max_normal_difference_deg = max_diff;};
	void setOutlierPercentageThreshold(double p){outlier_thresh = (float) p;};
	void setRotationFilter(bool b){do_filter = b;};
	void setVertexColor(int index, float r, float g, float b);
	void spreadVertexColors();

	// Get functions
	float* 	getDisplayNormals(){return &normal_list_display[0];};
	float* 	getDisplayVertices(){return &vertex_list_display[0];};
	int* 	getDisplayTriangles(){return &triangle_list_display[0];};
	float* 	getDisplayColors(){return &color_list_display[0];};
	int 	getDisplayNumTriangles(){return num_triangles_display;};
	int 	getDisplayNumVertices(){return num_vertices_display;};
	int		getCurrentPoseIndex(){return current_pose_index;};
	int		getPuppetCloudSize(){return cloud.points.size();};
	bool	usingNormals(){return useNormals;};
	vector<float>*	getCurrentPose(){return &poses[current_pose_index];};
	Transform3f 	getCurrentPoseTransform(){return pose;};
	vector<float>*	getTransientPose(){return &transient_pose_vector;};
	void getCorrespondences(vector<int>	&corrs);
	void getCorrsAndErrors(Transform3f transform, vector<int>	&corrs, vector<float>	&errs);
	void getErrors(Transform3f transform, vector<int>	&corrs, vector<float>	&errs);

	// Used to adjust filter parameters. Must be activated in keyboard callbacks in GLManager.cpp
	void increaseAlpha(){if (filter_alpha < 1) filter_alpha+=0.05; ROS_INFO_STREAM("Alpha: " << filter_alpha);};
	void decreaseAlpha(){if (filter_alpha > 0.05) filter_alpha-=0.05; ROS_INFO_STREAM("Alpha: " << filter_alpha);};
	void increaseRotSigma(){if (rotation_sigma < 359) rotation_sigma+=1; ROS_INFO_STREAM("rotation_sigma: " << rotation_sigma);};
	void decreaseRotSigma(){if (rotation_sigma > 1) rotation_sigma-=1; ROS_INFO_STREAM("rotation_sigma: " << rotation_sigma);};
	void increaseTimeSigma(){if (time_sigma < 1) time_sigma+=0.005; ROS_INFO_STREAM("time_sigma: " << time_sigma);};
	void decreaseTimeSigma(){if (time_sigma > 0.005) time_sigma-=0.005; ROS_INFO_STREAM("time_sigma: " << time_sigma);};
	void increaseTransSigma(){translation_sigma+=0.005; ROS_INFO_STREAM("translation_sigma: " << translation_sigma);};
	void decreaseTransSigma(){if (translation_sigma > 0.005) translation_sigma-=0.005; ROS_INFO_STREAM("translation_sigma: " << translation_sigma);};

	//////////////////////////////
	// 3D-model-related variables
	int		puppet_ID;				// Used for tasks like coloring the matched-point cloud
	bool 	added;
	bool 	using_legos;
	bool 	visible;				// Is the puppet in the kinect's field of view?
	int 	block_name_counter;		// For labeling lego blocks
	vector<LegoBlock> blocks;
	vector<float> outliers;

	//////////////////////////////
	// Point clouds and point-cloud pairs for ICP
	pcl::PointCloud<PointT> cloud;					// Comprised of vertices from the puppet's 3D model
	bool					publish_marked_clouds;
	pcl::PointCloud<PointT> markedCameraCloud; 		// For visualization
	pcl::PointCloud<PointT> block_cloud;			// Cloud for one lego block
	boost::shared_ptr<registration::ICPCloudPair> 	cloud_pair;
	registration::ICPCloudPairParams 				cloud_pair_params;
	registration::ICPThreadParams 					cloud_thread_params;
	bool 					cloud_pair_created;
	vector<int>  			cloud_to_original_vertices;

	//////////////////////////////
	// ICP-related variables
	bool 	use_multi_core;
	bool	use_inverse_icp;		// If true, the kinect cloud is transformed during ICP, not the puppet.
									// Leftover from development and testing. This should always be true.
	bool	puppet_as_source;		// Also outdated. This should always be false.
	bool 	use_SIFT;				// Use pose estimates from the rth_SIFT package to initialize ICP?
	bool	show_transient_pose;	// For visualizing pose produced part-way through ICP matching

	//////////////////////////////
	// Pose-related variables
	Eigen::Transform3f pose;					// Current pose for the puppet
	Eigen::Transform3f render_pose;				// Used for rendering while calculating the next pose
	Eigen::Transform3f transient_pose;			// Pose midway through ICP search (used for debugging)
	vector<float> 	   transient_pose_vector;
	Eigen::Transform3f lastT;
	float 	icp_error, icp_error_threshold, inlier_error, inlier_error_threshold;	// Error metrics
	int 	processed_frame;	// Keep track of how many frames from the kinect have been processed
	bool 	lost;
	int 	lost_counter;		// How many frames in a row the puppet has been lost
	unsigned int 				current_pose_index;
	vector< vector<float> >		poses;

	//////////////////////////////
	// debug variables
	vector<float> icp_errors;
	vector<float> inlier_errors;

	// For generating screen caps of ICP progress:
	vector<Transform3f> all_iterations;
	bool	record_iterations;
	int		icp_it;

protected:
	//////////////////////////////
	// PLY-related variables:
	int		vertices_loaded;
	int		triangles_loaded;
	bool	hasNormals;
	bool	hasColors;
	bool	isBinary;
	bool	triMesh;
	bool 	pastHeader;
	int		simultaneous_threads;
	int 	angle_spacing;				// Spacing between initial pose estimates.
	int		max_ICP_rounds;
	bool	useNormals;
	float	max_normal_difference_deg;	// Maximum angle between normals in degrees
	int		num_search_threads;			// Number of llel threads to use for correspondence search if multi-core tracking is enabled.
	float	outlier_thresh;
	float	current_outlier_percentage;

	//////////////////////////////
	// For puppet point clouds from 3D models
	vector<float>	vertex_list;
	vector<float>	normal_list;
	vector<float>	color_list;
	vector<int>  	triangle_list;
	int 			num_triangles;
	int				num_vertices;

	//////////////////////////////
	// For rendering puppets
	vector<float>	vertex_list_display;
	vector<float>	normal_list_display;
	vector<float>	color_list_display;
	vector<int>  	triangle_list_display;
	int 			num_triangles_display;
	int				num_vertices_display;
	std::string		display_ply_name;
	vector<int>     recolored_vertices;

	vector<float> 	centroid;
	float			puppet_max_distance;	// Diagonal of the puppet's bounding box

	vector<int> 	current_correspondences;
	vector<float>  	current_pt_to_pt_errors;

	//////////////////////////////
	// Pose-related variables:
	// Horizontal and vertical tracking regions (within FOV, in angles) and closest trackable distance (in meters)
	float h_tracking_region;
	float v_tracking_region;
	float near_tracking_region;

	// For timing functions
	timeval tracker_time;

	//////////////////////////////
	// Filter variables:
	bool 						do_filter;				// Should the poses be filtered?
	float 						filter_alpha;			// Exponential-decay constant
	bool						use_bilateral_filter;	// Use bilateral filter? (Otherwise exponential filter)
	uint						max_filter_entries;		// How many previous measurements to use in bilateral filter
	vector<Vector3f>			stored_translations;
	vector<double>				stored_times;
	vector<Quaternion<float>> 	stored_rotations;
	// Sigmas used for bilateral filter's Guassian
	float 						translation_sigma; 	// In meters
	float						translation_constant;
	float 						rotation_sigma;		// In degrees
	float						rotation_constant;
	float 						time_sigma;			// In seconds
	float						time_constant;

};

#endif /* PUPPET_H_ */
