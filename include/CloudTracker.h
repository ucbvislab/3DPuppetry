/*
  * CloudTracker.h
  *
  *     Created: 2011/2012
  *      Author: Robin Held (begun by Ankit Gupta)
  *
  *      This class user the iterative-closest-point and the Kinect's data stream
  *      to determine each puppet's pose
  *
  *      See LICENSE.txt for licensing info.
  */


#ifndef BLOCKTRACKER_H_
#define BLOCKTRACKER_H_
#include "Puppet.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <GL/glew.h>
#include <GL/glut.h>    // Header File For The GLUT Library
#include <GL/glext.h>

using namespace std;
using namespace cv;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace message_filters;
using namespace Eigen;
using namespace registration;

using boost::lexical_cast;

typedef rgbd::pt PointT;

/////////////////////////
// Global Variables

// Shared with GLManager:
extern pthread_mutex_t 	amutex;
extern pthread_cond_t 	cond;
extern vector<Puppet*> 	Puppets;
extern bool 			global_capture;
extern bool				raw_capture_started;
extern vector<double>	video_timestamps;
extern bool				clear_animation;
extern bool				suspend_tracking;
extern vector<GLuint>	puppet_lists;
extern vector<GLuint>	puppet_VBOs_created;
extern bool				transfer_colors;
extern bool 			spread_colors;
extern bool 			save_new_ply;
// SIFT data-capture variables
extern std::string		SIFT_save_name_numbered;
extern bool				signal_SIFT_save;
extern bool				playback_only;			// Disable kinect input and only play back saved animations

/////////////////////////
// Structures and functions for parallelizing operations:

struct ICPData
{
	pcl::PointCloud<PointT> *cloud;
	int						pose_index;
	Puppet					*puppet;
	vector<int>				*corrs;
	vector<float>			*errs;
	Transform3f				old_pose;
};

struct UpdateData
{
	pcl::PointCloud<PointT> *cloud;
	Puppet					*puppet;
	vector<int>				*corrs;
	vector<float>			*errs;
	Transform3f				old_pose;
};

void* parallel_ICP(void* threaddata);
void* parallel_correspondences(void* threaddata);

class CloudTracker {
public:

	CloudTracker();
	~CloudTracker();
	void prep();
	void start();

	void imgCallbackRGB(const sensor_msgs::ImageConstPtr& rgb_img);
	void imgCallbackColorDepth(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& depth_msg);
	void poseCallback(const std_msgs::Float32MultiArrayConstPtr &pose_msg);
	std::string setupRawVideoRecorder(std::string f_path, std::string v_name);
	void saveRawVideoFrame();
	void cloudCallback(sensor_msgs::PointCloud2ConstPtr cloud_ptr);
	void sendCloudsToPuppets();
	void transferColorsFromKinect(int puppet_ID,pcl::PointCloud<PointT> *curr_cloud);

	// Functions for manipulating parameters:
	void loadCalibrationData(std::string calib_name);
	void setTrackerFrameSkip(int skip);
	void enableCloudNormals();
	void disableCloudNormals();
	void setFilterCorrespondences(bool filt);
	void enableMatchCloudPublishing();
	void enableSIFTInitiliazer(){use_SIFT=true;};
	void setMinCloudSize(int size){min_cloud_size = size;};
	void enableUsingBag(){using_bag = true;};
	void setQueueSize(int q){queue_size = q;};
	void setMultiCore(bool set_mc){use_multi_core_tracker = set_mc;};
	void setCloudSkip(int cs){cloud_skip = cs;};
	void disableInverseICP(){use_inverse_icp = false;};
	void enablePuppetAsSource(){puppet_as_source = true;};
	void enableSeparateCloudsByColor(){color_cloud_separator = true;};
	void setFilterRatio(float f){filter_ratio = f;};
	void useOldCloudSeparator(){use_old_cloud_separator = true;};
	void enableSIFTPoseSaving(){SIFT_saving = true;};
	void toggleRGBFeed(bool val){use_rgb_feed = val;};

	// Old, deprecated, or debugging functions:
	void imgCallbackBW(const sensor_msgs::ImageConstPtr& bw_img);

protected:
	// ROS-related
	ros::NodeHandle 			nh_global;
	ros::NodeHandle 			nh_local;
	sensor_msgs::CvBridge 		img_bridge_;
	ros::Subscriber 			cloud_subscription;
	ros::Subscriber 			pose_subscription;
	pcl_ros::Publisher<PointT> 	cloud_pub;
	typedef image_transport::SubscriberFilter ImageSubscriber;;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	int 	queue_size;		// Size of ROS message queue
	bool	using_bag;		// Using bag file of recorded data?
	bool 	publish_clouds; // For visualizing the segmented point clouds
	sensor_msgs::CvBridge bridge;
	bool	use_rgb_feed;
	int 	subscription_buffer_size;

	// Puppet-related
	int		num_puppets;
	float 	min_puppet_distance;	// Newly detected puppets must be at least this far away from currently tracked
								// puppets (centroid-to-centroid). Otherwise we consider their detection an error

	// ICP settings and variables
	bool 	useNormals;					// Calculate normals for kinect point cloud and use them for ICP?
	bool 	use_multi_core_tracker;		// Parallelize ICP operations over multiple cores?
	bool	use_inverse_icp;			// Outdated name. Refers to whether the point-cloud is transformed to match the stored puppet
										// durign the ICP search.
	bool 	puppet_as_source;			// Outdated. Tried using the stored puppet points as the source cloud for ICP.
										// Resulted in large slow-down and difficulties with cloud segmentation. Do not enable.
	vector<pcl::PointCloud<PointT>*> clouds;
	pcl::PointCloud<PointT> 	mastercloud;
	pcl::PointCloud<PointT> 	markedCameraCloudCombined;
	int							cloud_skip;					// Used to downsample point cloud (default = 2);
	uint 						min_cloud_size;				// Size threshold for performing ICP on incoming point cloud
	uint						max_cloud_size;				// Excessively large puppet-specific clouds can needlessly slow down track. These should be downsampled.
	bool 						filter_correspondences;		// Check to make sure that each incoming point isn't matched to multiple puppets.
	float 						filter_ratio;				// Threshold ratio between two closest matches to kinect cloud to keep the best
	bool 						use_old_cloud_separator;	// Outdated. Either send the entire cloud to both puppets (only supports two)
															//           or use the color-based separator (see below).
	bool 						color_cloud_separator;		// Outdated. Use red-vs-yellow-based cloud separator?

	// Raw video capture
	bool 			export_raw_video;
	std::string		files_path;
	std::string		video_out_name;
	VideoWriter 	raw_writer;
	unsigned char	*raw_imageData;
	Mat				raw_capturedImg;
	Mat				raw_capturedImg_bw;
	Mat 			raw_rgb_image;
	vector<Mat>		captured_frames;
	std::string		output_raw_video;
	timeval 		get_time;
	double 			frame_time;
	int				capture_fps;
	bool			raw_capture;
	Mat				three_ch_frame;
	double			record_initial_time;		// Time when the current capture session was started.
	double			accumulated_time;			// Total time of captured sessions
	bool			new_capture_session;		// For clearing the recorded animation
	int				tracker_frame_skip;			// How many frames should be skipped by the tracker?
	bool			local_capture_started;
	bool			create_new_video;			// Make a new video file?

	// Timing-related
	timeval tracker_time;
	static const int NUM_FPS_SAMPLES = 64;
	double 	rates_ave[64];
	float 	time_taken_icp, time_taken_publish;
	int 	processed_frame;

	// For capturing SIFT templates:
	vector <vector <float> > SIFT_poses;
	bool SIFT_save_now;
	bool use_SIFT;
	bool SIFT_saving;

};

#endif
