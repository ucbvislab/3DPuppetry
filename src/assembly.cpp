/*
 * 3D Puppetry Interface
 *
 *     Created: 2011/2012
 *      Author: Robert Held
 */

#define GLH_EXT_SINGLE_FILE

#include "Model.h"
#include "GLManager.h"
#include "BlockTracker.h"

#include <time.h>
#include <math.h>

#include <unistd.h>     // needed to sleep
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


#define AI_CONFIG_PP_FD_REMOVE   "PP_FD_REMOVE"


//////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////

// File I/O
std::string			files_path;
vector<std::string> short_model_names_vector;
vector<std::string> short_scene_names_vector;
vector<std::string> full_model_names_vector;
vector<std::string> short_ply_names_vector;
vector<std::string> short_display_ply_names_vector;
vector<std::string> puppet_names;
std::string performance_out_name;
std::string performance_in_name;
std::string video_out_name;

vector<int> 	model_downsampling;
vector<float> 	scene_scalings;
vector<float> 	scene_brightnesses;
vector<int>		scene_tilings;
vector<GLuint>	model_lists;
vector<GLuint>	model_VBOs_created;
bool			transfer_colors;
bool 			spread_colors;
bool			save_new_ply;


// Multithreading variables
pthread_mutex_t amutex;
pthread_cond_t cond;
sem_t model_sem;
bool use_multi_core_tracker;


// Key classes
vector<Model*> 	Models;
int 			num_models;
GLManager		renderer;

// Misc
double	global_timer;
bool	global_capture;
bool	suspend_tracking;
vector<double> 	video_timestamps;
bool	clear_animation;
bool 	using_legos;
float 	filter_ratio;
int 	cloud_skip;				// Used to downsample incoming point clouds. default = 2;
int 	min_cloud_size;			// Size threshold for performing ICP on incoming point cloud
bool 	using_bag;				// Was the input pre-recorded?
bool 	use_rgb_feed;			// Use a color or B&W feed?
Eigen::Transform3f render_pose;
bool	publish_clouds;			// Publish clouds with outliers labeled?
bool 	captureSIFT;			// Used to capture data across the BlockTracker and GLManager instances.
bool	use_inverse_icp;
bool	model_as_source;	// Perform ICP from model to cloud? (or cloud to model)
bool	separate_clouds_by_color;
bool	use_old_cloud_separator;
bool	show_transient_pose;	// For visualizing pose produced part-way through ICP matching
bool 	filter_corr;			// Make sure each point in the incoming cloud only belongs to one model?

CvVideoWriter 	*raw_writer;
bool 	export_raw_video;
int 	queue_size;				// Size of ROS message queue (0 = unlimited)
bool 	raw_capture_started;


// SIFT data-capture variables
std::string	SIFT_save_name;
std::string	SIFT_save_name_numbered;
bool		SIFT_saving;
bool		signal_SIFT_save;
std::string SIFT_path;
//// SIFT variables
bool	use_SIFT;

//BlockTracker::siftOptions sift_opts;
// 	std::string SIFT_path;
//std::string	SIFT_load_name_1;
//std::string	SIFT_load_name_2;

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


void load_ros_params(BlockTracker tracker,ros::NodeHandle nh_load)
{
	// Video and pose variables
	std::string short_model_names;
	std::string short_ply_names;
	std::string	short_display_ply_names;
	std::string	short_scene_names;
//	std::string model_downsampling_string;	// No longer used--downsample in Meshlab and save separate PLY file.
	std::string pose_out_name;
	std::string pose_in_name;
	std::string SIFT_save_base;
	std::string lego_string("lego");	// Used for determining whether a puppet is made of legos
	std::string background_image;
	std::string scene_scaling;
	std::string scene_brightness;
	std::string scene_tiling;
	size_t found;

	// Set default values
	use_rgb_feed			= true;
	export_raw_video		= true;
	use_multi_core_tracker	= false;
	double 	icp_thresh		= 0.00001;
	double	outlier_thresh  = 0.5;
	int tracker_frame_skip	= 0;
	int subscription_buffer_size 	= 1;
	min_cloud_size			= 25;
	use_inverse_icp			= true;
	model_as_source			= false;
	using_bag				= false;
	queue_size				= 3;
	num_models				= 1;
	int angle_spacing		= 90;
	int simultaneous_threads= 1;
	int	max_ICP_rounds		= 20;
	cloud_skip				= 1;

	separate_clouds_by_color= false;
	double filter_ratio_in	= 1.2;		// Threshold ratio between two closest matches to kinect cloud to keep the best
										// Otherwise, the point isn't assigned to any model.

	use_old_cloud_separator	= false;
	publish_clouds			= false;
	SIFT_saving				= false;
	show_transient_pose		= false;
	bool useNormals			= true;
	double max_normal_difference_deg = 90;

	bool separate_plys_for_display = false;


	// Setting extent of trackable region
	float nearest_track		= 0.62;			// Nearest distance
	float hor_track			= 40*4/3; // 35 * 4/3;		// Horizontal (angular) extent
	float ver_track			= 37; // 29;			// Vertical		"		"

    nh_load.param("subscription_buffer_size", subscription_buffer_size, 1);
    nh_load.param("model_names", short_model_names, std::string(""));
    nh_load.param("ply_names", short_ply_names, std::string(""));
    nh_load.param("display_ply_names", short_display_ply_names, std::string(""));

    nh_load.param("background_image", background_image, std::string(""));
    nh_load.param("scene_names", short_scene_names, std::string(""));
    nh_load.param("scene_scaling", scene_scaling, std::string(""));
    nh_load.param("scene_brightness", scene_brightness, std::string(""));
    nh_load.param("scene_tiling", scene_tiling, std::string(""));

    nh_load.param("SIFT_save_base", SIFT_save_base, std::string(""));
    nh_load.param("save_SIFT_poses", SIFT_saving, false);
    nh_load.param("files_path", files_path, std::string(""));
    nh_load.param("multi_core_tracker", use_multi_core_tracker, true);
    nh_load.param("publish_clouds", publish_clouds, false);
    nh_load.param("icp_error_threshold", icp_thresh, 1.0);					// ICP error threshold for considering tracking lost
    nh_load.param("icp_outlier_threshold", outlier_thresh, 0.5);			// Percentage of points that need to be outliers to consider tracking lost
//    nh_load.param("model_cloud_downsampling", model_downsampling_string,  std::string(""));
    nh_load.param("tracker_frame_skip", tracker_frame_skip, 1);				// How many frames should the tracker skip?
    nh_load.param("export_raw_video", export_raw_video, true);
    nh_load.param("min_cloud_size", min_cloud_size, 25);
    nh_load.param("use_cloud_normals", useNormals, true);
    nh_load.param("max_normal_difference_deg",max_normal_difference_deg,90.0);
    nh_load.param("cloud_skip",cloud_skip,1);

    // ICP matching settings
    nh_load.param("use_inverse_icp", use_inverse_icp, true);
    nh_load.param("match_model_to_cloud", model_as_source, false);
    nh_load.param("show_transient_pose", show_transient_pose, false);
    nh_load.param("filter_correspondences", filter_corr, false);
    nh_load.param("filter_distance_ratio",filter_ratio_in,2.0);

    // Deprecated /////////
    nh_load.param("pose_recording_out", pose_out_name, std::string(""));
	nh_load.param("video_recording_out", video_out_name, std::string(""));
	nh_load.param("pose_recording_in", pose_in_name, std::string(""));
	///////////////////////

	// Use these instead:
    nh_load.param("load_performance", performance_in_name, std::string(""));
	nh_load.param("save_performance", performance_out_name, std::string(""));

    nh_load.param("using_bag", using_bag, false);
    nh_load.param("queue_size", queue_size, 3);
    nh_load.param("angle_spacing", angle_spacing, 90);
    nh_load.param("simultaneous_threads", simultaneous_threads, 4);
    nh_load.param("max_ICP_rounds", max_ICP_rounds, 20);
    nh_load.param("separate_clouds", separate_clouds_by_color, false);			// TODO: Should use a more descriptive parameter name
    nh_load.param("use_old_cloud_separator", use_old_cloud_separator, false);

    nh_load.param("use_rgb_feed", use_rgb_feed, true);


//    // SIFT variables
    nh_load.param("use_SIFT", use_SIFT, false);
//	nh_load.param("SIFT_load_name_1", SIFT_load_name_1, std::string(""));
//	nh_load.param("SIFT_load_name_2", SIFT_load_name_2, std::string(""));
//	nh_load.param("sift_first_octave",sift_opts.firstOctave,-1);
//	nh_load.param("sift_num_octaves",sift_opts.numOctaves,5);
//	nh_load.param("sift_dog_threshold",sift_opts.dogThreshold ,0.0067);
//	nh_load.param("sift_edge_threshold",sift_opts.edgeThreshold ,10.0);
//	nh_load.param("sift_filter_width_factor",sift_opts.filterWidthFactor ,4.0);
//	nh_load.param("sift_num_feature_orientations",sift_opts.numOrientations ,2);
//	SIFT_path = files_path;
//	SIFT_path.append("SIFT");

    filter_ratio = (float)filter_ratio_in;

	if (files_path.empty())
		ROS_WARN("File path set incorrecty!");

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
//		cout << SIFT_save_name << endl;
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
			// A new ply file indicates a new model to be tracked
			Models.push_back(new Model);
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


//	for (int i=0; i<ply_names.size();i++){
//		cout << ply_names[i] << endl;
//	}

	// Load model names
	if (!short_model_names.empty())
	{
		std::stringstream ss(short_model_names);
		char str[200];
		std::string temp_name;
		while (ss.getline(str,50,','))
		{
			temp_name.append(str);
			short_model_names_vector.push_back(temp_name);
			temp_name = "";
		}
	}

//	for (int i=0; i<model_names.size();i++){
//	 cout << model_names[i] << endl;
//	}

	num_models = Models.size();

	// Setup puppet names (either based on ply or model names)
	for (int m=0;m<num_models;m++)
	{
		found = short_ply_names_vector[m].find(lego_string);
		if (found!=string::npos) {
			Models[m]->using_legos = true;
			puppet_names.push_back(short_model_names_vector[m]);
		} else {
			puppet_names.push_back(short_ply_names_vector[m]);
		}
	}

	if (show_transient_pose){	renderer.enableTransientPoses();}
	renderer.setPuppetnames(puppet_names);
	renderer.setBaseFilePath(files_path);

//	// Load model-cloud downsampling rates
//	std::stringstream ss(model_downsampling_string);
//	char str[3];
//	while (ss.getline(str,3,','))
//	{
//		model_downsampling.push_back(atoi(str));
//	}
//	if (num_models != (int)model_downsampling.size()){
//		ROS_WARN("Model downsampling incorrectly set. Include a number for each model and end each value with comma");
//	}

	for (int m=0;m<num_models;m++)
	{
		ROS_INFO_STREAM("Loading model " << m+1 << " of " << num_models);
//		Models[m]->setModelDownsample(model_downsampling[m]);
		Models[m]->setModelDownsample(1);
    	Models[m]->setMultiCore(use_multi_core_tracker);
    	Models[m]->setICPThreshold(icp_thresh);
    	Models[m]->setInverseICP(use_inverse_icp);
    	Models[m]->setModelMatch(model_as_source);
        Models[m]->setAngleSpacing(angle_spacing);
        Models[m]->setThreads(simultaneous_threads);
    	Models[m]->setMaxICPRounds(max_ICP_rounds);
    	Models[m]->use_SIFT = use_SIFT;
    	Models[m]->publish_marked_clouds = publish_clouds;
    	Models[m]->model_ID =m;
    	Models[m]->show_transient_pose = show_transient_pose;
    	Models[m]->setOutlierPercentageThreshold(outlier_thresh);

        // Load model
		std::string new_model_name = files_path;
		new_model_name.append("models/");
		new_model_name.append(short_model_names_vector[m]);
		new_model_name.append(".txt");
		full_model_names_vector.push_back(new_model_name);
		std::string new_ply_name = files_path;
		new_ply_name.append("models/");
        new_ply_name.append(short_ply_names_vector[m]);
		Models[m]->loadPLY(new_ply_name);
		if (!useNormals)
			Models[m]->disableCloudNormals();

		// Load display ply, if desired
		if (!(Models[m]->using_legos) && separate_plys_for_display)
		{
			new_ply_name = files_path;
			new_ply_name.append("models/");
			new_ply_name.append(short_display_ply_names_vector[m]);
			Models[m]->loadDisplayPLY(new_ply_name);
		}

		Models[m]->setMaxNormalDifference((float)max_normal_difference_deg);
		Models[m]->setTrackingRegion(hor_track, ver_track, nearest_track);
//		cout << "loaded ply" << endl;
	}

	ROS_INFO_STREAM("Initialized models");

    tracker.setTrackerFrameSkip(tracker_frame_skip);
    tracker.setSubscriptionBufferSize(subscription_buffer_size);

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
//
//    if (!scene_tiling.empty())
//   	{
//		std::stringstream ss(scene_tiling);
//		char str[5];
//		while (ss.getline(str,3,','))
//		{
//			scene_tilings.push_back(atoi(str));
//		}
//   	}


	if (short_scene_names_vector.size() != scene_scalings.size()){
		ROS_WARN("Scene scaling incorrectly set. Include a number for each scene and end each value with comma");
	}

	if (short_scene_names_vector.size() != scene_brightnesses.size()){
		ROS_WARN("Scene lighting incorrectly set. Include a float between 0 and 1 for each scene and end each value with comma");
	}
//	if (short_scene_names_vector.size() != scene_tilings.size()){
//		ROS_WARN("Scene-tiling settings incorrectly set. Use 0 or 1 to disable or enable tiling for each scene and end each value with comma");
//	}

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

void *NodeThread(void* tracker)
{
	BlockTracker *tracker_pointer;
	tracker_pointer = (BlockTracker*) tracker;
	tracker_pointer->start();
}

void *RenderThread(void* renderer)
{
	GLManager *renderer_pointer;
	renderer_pointer = (GLManager*) renderer;
	renderer_pointer->start();
}

int main (int argc, char** argv)
{

	pthread_mutex_init(&amutex, NULL);
	pthread_cond_init (&cond, NULL);

	sem_init(&model_sem,0,0);

	ros::init (argc, argv, "KinectTracker");
	ros::NodeHandle nh_load("~");

	raw_capture_started	= false;
	clear_animation		= false;
	suspend_tracking	= false;
	transfer_colors		= false;
	save_new_ply		= false;
	spread_colors		= false;

	glutInit(&argc, argv);

	BlockTracker 	tracker;

	// Load yaml files
	load_ros_params(tracker,nh_load);
//	ROS_INFO_STREAM("Loaded parameters.");

    if (publish_clouds){	tracker.enableMatchCloudPublishing();		}
    if (filter_corr){		tracker.enableFilterCorrespondences();		}
    if (use_SIFT){			tracker.enableSIFTInitiliazer();			}
    if (using_bag){			tracker.enableUsingBag();					}
    tracker.setMinCloudSize(min_cloud_size);
    tracker.setQueueSize(queue_size);
    if (use_multi_core_tracker){tracker.enableMultiCore();				}
    if (!use_inverse_icp){	tracker.disableInverseICP();				}
    if (model_as_source){	tracker.enableModelAsSource();				}
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

	for (int m=0;m<num_models;m++)
	{
		if (Models[m]->using_legos){
			// The loader only comes here for lego models. Look under after the PLY
			// loading for non-lego model loading.
			Models[m]->loadLegoModel(full_model_names_vector[m],files_path);
		}
	}

//	// Load rotations associated with SIFT poses
//	// TODO: HARDCODED
//	Models[0]->loadSIFTPoses("dummy","dummy");


	//	// Setup SIFT
	//	tracker.setupSIFT(sift_opts);
	//	Models[0]->sample_poses.loadPoses(SIFT_path, SIFT_load_name_1, &tracker.sift_extractor);
	//	// TODO: SIFT only supports one model right now

	// Wait for kinect to start publishing
	*ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/rgb/points_seg");
	if (use_SIFT)
		*ros::topic::waitForMessage<std_msgs::Float32MultiArray>("pose_feed");

	renderer.prep();
	tracker.prep();

	int rc1, rc2;
	pthread_t thread1, thread2;
	pthread_attr_t attr;

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	/* Create independent threads each of which will execute functionC */
	if( (rc1=pthread_create( &thread1, &attr, &NodeThread,(void *) &tracker)) )
	{
		printf("Thread creation failed: %d\n", rc1);
	}

	if( (rc2=pthread_create( &thread2, &attr, &RenderThread, (void *) &renderer)) )
	{
		printf("Thread creation failed: %d\n", rc2);
	}

	/* Wait till threads are complete before main continues. Unless we  */
	/* wait we run the risk of executing an exit which will terminate   */
	/* the process and all threads before the threads have completed.   */
//	pthread_mutex_lock(&amutex);
	pthread_join( thread1, NULL);
	pthread_join( thread2, NULL);
	pthread_attr_destroy(&attr);

//	pthread_mutex_unlock(&amutex);

	return 0;
}
