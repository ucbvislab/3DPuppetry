/*
 * Kinect Tracker (3D Puppetry Interface)
 *
 *     Created: 2011/2012
 *      Author: Robin Held
 *
 *      This is the main .cpp for the project.
 *      See README and LICENSE.txt for more info.
 */

#define GLH_EXT_SINGLE_FILE

#include "Puppet.h"
#include "GLManager.h"
#include "CloudTracker.h"

#include <time.h>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <pthread.h>
#include <semaphore.h>
#include <std_msgs/Float32MultiArray.h>


using namespace std;

// For assimp 3D-file loading
#define AI_CONFIG_PP_FD_REMOVE   "PP_FD_REMOVE"


//////////////////////////////////////////////////////
// Variables
//////////////////////////////////////////////////////

GLManager		renderer;

// File and ROS-msg I/O
std::string			files_path;
vector<std::string> short_puppet_names_vector;
vector<std::string> short_scene_names_vector;
vector<std::string> full_puppet_names_vector;
vector<std::string> short_ply_names_vector;
vector<std::string> short_display_ply_names_vector;
vector<std::string> puppet_names;
std::string 		performance_out_name;
std::string 		performance_in_name;
std::string 		video_out_name;
bool 				using_bag;				// Was the input pre-recorded?
bool 				use_rgb_feed;			// Use a color or B&W feed to display raw input?
bool				publish_clouds;			// Publish clouds with outliers labeled?
CvVideoWriter 		*raw_writer;
bool 				export_raw_video;
int 				queue_size;				// Size of ROS message queue (0 = unlimited)


// Puppet variables
vector<Puppet*> 	Puppets;					// (global)
int 			num_puppets;
vector<int> 	puppet_downsampling;
vector<GLuint>	puppet_lists;			// (global)
vector<GLuint>	puppet_VBOs_created;		// (global) (vertex buffer objects)
bool			transfer_colors;		// (global)	(for assigning colors to untextured puppets)
bool 			spread_colors;			// (global) "
bool			save_new_ply;			// (global) "
bool 			using_legos;
bool 			captureSIFT;			// Used to capture data across the CloudTracker and GLManager instances.
bool			use_inverse_icp;		// Before running each ICP pass, transform point cloud to align with puppet
										//   (Rather than vice-versa)
bool			puppet_as_source;		// Perform ICP from puppet to cloud? (or cloud to puppet)
bool			show_transient_pose;	// For visualizing pose produced part-way through ICP matching
										//   (Currently commented out in the rest of the code)

// Background variables
vector<float> 	scene_scalings;			// Adjusts sizes of background puppets
vector<float> 	scene_brightnesses;		// Used in case a background has dim textures/colors

// Point-cloud variables
float 	filter_ratio;				// During point-cloud segmentation, used to determine ambiguity.
int 	cloud_skip;					// Used to downsample incoming point clouds. default = 2;
int 	min_cloud_size;				// Size threshold for performing ICP on incoming point cloud
bool	separate_clouds_by_color;
bool	use_old_cloud_separator;
bool 	filter_corr;				// Make sure each point in the incoming cloud only belongs to one puppet?


// Multithreading variables
pthread_mutex_t amutex;				// (global)
pthread_cond_t cond;				// (global)
sem_t puppet_sem;
bool use_multi_core_tracker;


// Global variables
bool	global_capture;
bool	suspend_tracking;
vector<double> 	video_timestamps;
bool	clear_animation;
bool 	raw_capture_started;
bool	playback_only;					// Disable kinect input and only play back saved animations


// SIFT variables
bool		use_SIFT;					// Use SIFT-based puppet identification?
std::string	SIFT_save_name;				// For saving new SIFT templates
std::string	SIFT_save_name_numbered;	// (global)
bool		SIFT_saving;
std::string SIFT_path;
bool	signal_SIFT_save;				// (global) Indicates that current puppet pose, depth map, and image should be saved


//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


/*
 *  Parse the ROS launch file and set variables
 *
 *  \param tracker	Handles all the ICP-based pose detection
 *  \param nh_load	ROS node for loading user-set parameters
 */
void load_ros_params(CloudTracker tracker,ros::NodeHandle nh_load)
{

	// First deal with variables that are not loaded from the launch file:

	// Setting extent of trackable region
	float nearest_track		= 0.62;			// Nearest distance
	float hor_track			= 40*4/3; // 35 * 4/3;		// Horizontal (angular) extent
	float ver_track			= 37; // 29;			// Vertical		"		"

	// Now for launch file:

	// Video and pose variables
	std::string short_puppet_names;
	std::string short_ply_names;
	std::string	short_display_ply_names;
	std::string	short_scene_names;
	std::string pose_out_name;
	std::string pose_in_name;
	std::string SIFT_save_base;
	std::string lego_string("lego");	// Used for determining whether a puppet is made of legos
	std::string background_image;
	std::string scene_scaling;
	std::string scene_brightness;
	size_t found;

	// Set default values
	use_rgb_feed			= true;
	export_raw_video		= true;
	use_multi_core_tracker	= false;
	double 	icp_thresh		= 0.00001;
	double	outlier_thresh  = 0.5;
	int tracker_frame_skip	= 0;
	min_cloud_size			= 25;
	use_inverse_icp			= true;
	puppet_as_source			= false;
	using_bag				= false;
	queue_size				= 5;
	num_puppets				= 1;
	int angle_spacing		= 90;
	int simultaneous_threads= 1;
	int	max_ICP_rounds		= 30 ;
	cloud_skip				= 1;
	playback_only			= false;

	separate_clouds_by_color= false;
	double filter_ratio_in	= 1.2;		// Threshold ratio between two closest matches to kinect cloud to keep the best
										// Otherwise, the point isn't assigned to any puppet.
	use_old_cloud_separator	= false;
	publish_clouds			= false;
	SIFT_saving				= false;
	show_transient_pose		= false;
	bool useNormals			= true;
	double max_normal_difference_deg = 90;

	bool separate_plys_for_display = false;

	/////////////////////////////
	// Most relevant parameters
    nh_load.param("files_path", files_path, std::string(""));

    /////////////////////////////
	// Puppet variables
    nh_load.param("model_names", short_puppet_names, std::string(""));
    nh_load.param("ply_names", short_ply_names, std::string(""));
    nh_load.param("display_ply_names", short_display_ply_names, std::string(""));

    /////////////////////////////
    // Background settings
    nh_load.param("background_image", background_image, std::string(""));
    nh_load.param("scene_names", short_scene_names, std::string(""));
    nh_load.param("scene_scaling", scene_scaling, std::string(""));
    nh_load.param("scene_brightness", scene_brightness, std::string(""));

    /////////////////////////////
    // Point-cloud variables
    nh_load.param("publish_clouds", publish_clouds, false);
    nh_load.param("icp_outlier_threshold", outlier_thresh, 0.33);			// Percentage of points that need to be outliers to consider tracking lost
    nh_load.param("min_cloud_size", min_cloud_size, 125);
    nh_load.param("max_normal_difference_deg",max_normal_difference_deg,135.0);

    /////////////////////////////
    // For SIFT-template saving mode
    nh_load.param("SIFT_save_base", SIFT_save_base, std::string(""));
    nh_load.param("save_SIFT_poses", SIFT_saving, false);
    nh_load.param("use_SIFT", use_SIFT, false);							// Set to true for colorization mode, too

    /////////////////////////////
    // ICP-matching settings
    nh_load.param("multi_core_tracker", use_multi_core_tracker, true);
    nh_load.param("filter_correspondences", filter_corr, true);
    nh_load.param("filter_distance_ratio",filter_ratio_in,1.2);
    nh_load.param("simultaneous_threads", simultaneous_threads, 3);

    /////////////////////////////
    // Animation loading and saving
    nh_load.param("playback_mode", playback_only, false);
    nh_load.param("load_performance", performance_in_name, std::string(""));
	nh_load.param("save_performance", performance_out_name, std::string(""));

		/////////////////////////////
    // Outdated parameters; Used during development for debugging. Should not be changed.
    nh_load.param("show_transient_pose", show_transient_pose, false);
    nh_load.param("match_puppet_to_cloud", puppet_as_source, false);
    nh_load.param("use_inverse_icp", use_inverse_icp, true);
    nh_load.param("cloud_skip",cloud_skip,1);
    nh_load.param("use_cloud_normals", useNormals, true);
    nh_load.param("export_raw_video", export_raw_video, true);
    nh_load.param("tracker_frame_skip", tracker_frame_skip, 0);				// How many frames should the tracker skip?
    nh_load.param("icp_error_threshold", icp_thresh, 1.0);					// ICP error threshold for considering tracking lost
    nh_load.param("using_bag", using_bag, false);
    nh_load.param("use_rgb_feed", use_rgb_feed, true);
    nh_load.param("use_old_cloud_separator", use_old_cloud_separator, false); 	// Outdated. Either send the entire cloud to both puppets (only supports two)
    																			//           or use the color-based separator (see below).
    nh_load.param("separate_clouds", separate_clouds_by_color, false);			// Outdated. Use red-vs-yellow-based cloud separator?
    nh_load.param("queue_size", queue_size, 3);

	/////////////////////////////
    // Deprecated parameters
    nh_load.param("pose_recording_out", pose_out_name, std::string(""));
	nh_load.param("video_recording_out", video_out_name, std::string(""));
	nh_load.param("pose_recording_in", pose_in_name, std::string(""));
    nh_load.param("max_ICP_rounds", max_ICP_rounds, 20); 					// Value is now hard-coded in Puppet.cpp
    nh_load.param("angle_spacing", angle_spacing, 90);						// Used for non-SIFT-based puppet initialization.

    /////////////////////////////

    Puppets.reserve(50*sizeof(Puppet));

    filter_ratio = (float)filter_ratio_in;

	if (files_path.empty())
		ROS_WARN("File path set incorrecty!");

	// Setup SIFT-saving file names
	if (SIFT_save_base.empty() && SIFT_saving)
		ROS_WARN("Pose saving enabled but no SIFT save base provided");

    if (!SIFT_save_base.empty()){
    	SIFT_path = files_path;
    	SIFT_path.append("SIFT");
    	SIFT_save_name = SIFT_path;
    	SIFT_save_name.append(SIFT_save_base);

    	std::string SIFT_path 	= files_path;
    	SIFT_path.append("SIFT/");
    	SIFT_path.append(SIFT_save_base);
    	SIFT_path.append("/");
		fs::path p(SIFT_path);
		if (!fs::exists(p))
			fs::create_directory(p);
		SIFT_save_name = SIFT_path;
		SIFT_save_name.append("/");
		SIFT_save_name.append(SIFT_save_base);
    }
    renderer.setSIFTSavingName(SIFT_save_name);

	// Load ply file names
	if (short_ply_names.empty())
	{
		ROS_WARN("Ply names incorrectly set!");
	} else {
		std::stringstream ss(short_ply_names);
		char str[200];
		std::string temp_name;
		while (ss.getline(str,50,','))
		{
			temp_name.append(str);
			short_ply_names_vector.push_back(temp_name);
			// A new ply file indicates a new puppet to be tracked
			Puppets.push_back(new Puppet());
			temp_name = "";
		}
	}

	// If separate ply's have been designated for rendering, use them
	if (!short_display_ply_names.empty())
	{
		std::stringstream ss(short_display_ply_names);
		char str[200];
		std::string temp_name;
		while (ss.getline(str,50,','))
		{
			temp_name.append(str);
			short_display_ply_names_vector.push_back(temp_name);
			temp_name = "";
		}
		separate_plys_for_display = true;
	}

	if (separate_plys_for_display && (short_display_ply_names_vector.size() != short_ply_names_vector.size()))
		ROS_WARN("Display and non-display ply lists are incompatible");

	// Load puppet names
	if (!short_puppet_names.empty())
	{
		std::stringstream ss(short_puppet_names);
		char str[200];
		std::string temp_name;
		while (ss.getline(str,50,','))
		{
			temp_name.append(str);
			short_puppet_names_vector.push_back(temp_name);
			temp_name = "";
		}
	}

	num_puppets = Puppets.size();

	// Setup puppet names (either based on ply or puppet names)
	for (int m=0;m<num_puppets;m++)
	{
		found = short_ply_names_vector[m].find(lego_string);
		if (found!=string::npos) {
			Puppets[m]->using_legos = true;
			puppet_names.push_back(short_puppet_names_vector[m]);
		} else {
			puppet_names.push_back(short_ply_names_vector[m]);
		}
	}

	if (show_transient_pose)
		renderer.enableTransientPoses();

	renderer.setPuppetnames(puppet_names);
	renderer.setBaseFilePath(files_path);

	// Load puppets and apply settings
	for (int m=0;m<num_puppets;m++)
	{
		ROS_INFO_STREAM("Loading puppet " << m+1 << " of " << num_puppets);
    	Puppets[m]->setMultiCore(use_multi_core_tracker);
    	Puppets[m]->setICPThreshold(icp_thresh);
    	Puppets[m]->setInverseICP(use_inverse_icp);
    	Puppets[m]->setPuppetMatch(puppet_as_source);
        Puppets[m]->setThreads(simultaneous_threads);
    	Puppets[m]->use_SIFT 				= use_SIFT;
    	Puppets[m]->publish_marked_clouds 	= publish_clouds;
    	Puppets[m]->puppet_ID 				= m;
    	Puppets[m]->show_transient_pose 		= show_transient_pose;
    	Puppets[m]->setOutlierPercentageThreshold(outlier_thresh);

        // Load puppet
		std::string new_puppet_name = files_path;
		new_puppet_name.append("models/");
		new_puppet_name.append(short_puppet_names_vector[m]);
		new_puppet_name.append(".txt");
		full_puppet_names_vector.push_back(new_puppet_name);
		std::string new_ply_name = files_path;
		new_ply_name.append("models/");
        new_ply_name.append(short_ply_names_vector[m]);
		Puppets[m]->loadPLY(new_ply_name);
		if (!useNormals)
			Puppets[m]->disableCloudNormals();

		// Load display ply, if desired
		if (!(Puppets[m]->using_legos) && separate_plys_for_display)
		{
			new_ply_name = files_path;
			new_ply_name.append("models/");
			new_ply_name.append(short_display_ply_names_vector[m]);
			Puppets[m]->loadDisplayPLY(new_ply_name);
		}

		Puppets[m]->setMaxNormalDifference((float)max_normal_difference_deg);
		Puppets[m]->setTrackingRegion(hor_track, ver_track, nearest_track);

		// Deprecated:
//    	Puppets[m]->setMaxICPRounds(max_ICP_rounds);
//   	Puppets[m]->setAngleSpacing(angle_spacing);
	}

	ROS_INFO_STREAM("Initialized puppets");

    tracker.setTrackerFrameSkip(tracker_frame_skip);

    if (!useNormals)
    	tracker.disableCloudNormals();

    // Currently, if bag-recorded data is being used, there is no RGB feed
    if (using_bag)
    	export_raw_video = false;

    // Load background image
    if (!background_image.empty()){
    	std::string background_file;
    	background_file = files_path;
    	background_file.append("backgrounds/");
    	background_file.append(background_image);
    	renderer.setBackgroundImage(background_file);
    }

    // Load virtual scene names
    if (!short_scene_names.empty())
	{
		std::stringstream ss(short_scene_names);
		char str[200];
		std::string temp_name;
		while (ss.getline(str,50,','))
		{
			temp_name.append(str);
			short_scene_names_vector.push_back(temp_name);
			temp_name = "";
		}
	}

    if (!scene_scaling.empty())
   	{
		std::stringstream ss(scene_scaling);
		char str[5];
		while (ss.getline(str,20,','))
		{
			scene_scalings.push_back(atof(str));
		}
   	}

    if (!scene_brightness.empty())
   	{
		std::stringstream ss(scene_brightness);
		char str[5];
		while (ss.getline(str,20,','))
		{
			scene_brightnesses.push_back(atof(str));
		}
   	}

	if (short_scene_names_vector.size() != scene_scalings.size()){
		ROS_WARN("Scene scaling incorrectly set. Include a number for each scene and end each value with comma");
	}

	if (short_scene_names_vector.size() != scene_brightnesses.size()){
		ROS_WARN("Scene lighting incorrectly set. Include a float between 0 and 1 for each scene and end each value with comma");
	}

	for (uint s=0; s<short_scene_names_vector.size(); s++)
	{
		renderer.addScene(short_scene_names_vector[s],scene_scalings[s],scene_brightnesses[s]);
	}

    // Check whether a pose input file was provided. If so, load the poses
	if (!pose_in_name.empty()){
		ROS_WARN("Use load_performance instead of pose_recording_in.");
	}

    if (!pose_out_name.empty()){
    	ROS_WARN("Use save_performance instead of pose_recording_out.");
    }

	// Setup video recorder
    if (!video_out_name.empty()){
    	ROS_WARN("Use save_performance instead of video_recording_out.");
    }

    if (!performance_in_name.empty()){
    	renderer.enablePoseLoading(files_path,performance_in_name);
    }

    // Set size of trackable region so renderer can correctly size the ground plane.
    renderer.setTrackingRegion(hor_track, ver_track, nearest_track);

    global_capture 		= false;
	signal_SIFT_save 	= false;
}

/*
 *  Setup separate thread for Blocktracker (performs ICP)
 */
void *NodeThread(void* tracker)
{
	CloudTracker *tracker_pointer;
	tracker_pointer = (CloudTracker*) tracker;
	tracker_pointer->start();
}


/*
 *  Setup separate thread for Blocktracker (performs ICP)
 */
void *RenderThread(void* renderer)
{
	GLManager *renderer_pointer;
	renderer_pointer = (GLManager*) renderer;
	renderer_pointer->start();
}

/*
 * Main function. Loads user-set parameters, sets variable states, and starts interface
 */
int main (int argc, char** argv)
{

	pthread_mutex_init(&amutex, NULL);
	pthread_cond_init (&cond, NULL);

	sem_init(&puppet_sem,0,0);

	ros::init (argc, argv, "KinectTracker");
	ros::NodeHandle nh_load("~");

	raw_capture_started	= false;
	clear_animation		= false;
	suspend_tracking	= false;
	transfer_colors		= false;
	save_new_ply		= false;
	spread_colors		= false;

	glutInit(&argc, argv);

	CloudTracker 	tracker;

	// Load yaml files
	load_ros_params(tracker,nh_load);

	// Use loaded parameters to set up puppets, tracker, and renderer
    if (publish_clouds){	tracker.enableMatchCloudPublishing();		}
    tracker.setFilterCorrespondences(filter_corr);
    if (use_SIFT){			tracker.enableSIFTInitiliazer();			}
    if (using_bag){			tracker.enableUsingBag();					}
    tracker.setMinCloudSize(min_cloud_size);
    tracker.setQueueSize(queue_size);
    tracker.setMultiCore(use_multi_core_tracker);
    if (!use_inverse_icp){	tracker.disableInverseICP();				}
    if (puppet_as_source){	tracker.enablePuppetAsSource();				}
    if (use_old_cloud_separator){tracker.useOldCloudSeparator();		}
    if (separate_clouds_by_color){tracker.enableSeparateCloudsByColor();}
    tracker.toggleRGBFeed(use_rgb_feed);
    tracker.setCloudSkip(cloud_skip);
    if (SIFT_saving){
    	renderer.enableSIFTPoseSaving();
    	tracker.enableSIFTPoseSaving();
    }
    if (!performance_out_name.empty()){
    	renderer.setupPoseRecorder(files_path,performance_out_name);
    	renderer.setupVideoRecorder(files_path,performance_out_name);
		if (export_raw_video){
			std::string raw_name 			= tracker.setupRawVideoRecorder(files_path,performance_out_name);
			renderer.setRawVideoFileName(raw_name);
		}
    }
    tracker.setFilterRatio(filter_ratio);
	for (int m=0;m<num_puppets;m++)
	{
		if (Puppets[m]->using_legos){
			// The loader only comes here for lego puppets.
			Puppets[m]->loadLegoPuppet(full_puppet_names_vector[m],files_path);
		}
	}

	// Wait for kinect to start publishing
	if (!playback_only)
	{
		if (use_SIFT)
			*ros::topic::waitForMessage<std_msgs::Float32MultiArray>("pose_feed");
	}

	renderer.prep();
	tracker.prep();

	int rc1, rc2;
	pthread_t thread1, thread2;
	pthread_attr_t attr;

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	/* Create independent threads for the renderer and tracker */
	if( (rc1=pthread_create( &thread1, &attr, &NodeThread,(void *) &tracker)) )
	{
		printf("Thread creation failed: %d\n", rc1);
	}

	if( (rc2=pthread_create( &thread2, &attr, &RenderThread, (void *) &renderer)) )
	{
		printf("Thread creation failed: %d\n", rc2);
	}
	pthread_join( thread1, NULL);
	pthread_join( thread2, NULL);
	pthread_attr_destroy(&attr);

	return 0;
}
