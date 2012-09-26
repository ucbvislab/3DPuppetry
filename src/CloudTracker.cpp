/*
  * CloudTracker.cpp
  *
  *     Created: 2011/2012
  *      Author: Robin Held (begun by Ankit Gupta)
  *
  *      This class user the iterative-closest-point and the Kinect's data stream
  *      to determine each puppet's pose
  *
  *      See LICENSE.txt for licensing info.
  */

#include "CloudTracker.h"

/*
 *  Setup a thread for updating a puppet's poses
 *
 */
void* parallel_ICP(void* threaddata)
{
	struct ICPData *icp_data;
	icp_data = (struct ICPData *) threaddata;
	icp_data->puppet->updatePose((icp_data->cloud),TWO_STAGE_ICP, true, 0.1);
	return (void*) 42;
}

/*
 *  Setup a thread for just finding the correspondences between a puppet's point cloud and
 *  an incoming Kinect point cloud
 *
 */
void* parallel_correspondences(void* threaddata)
{
	struct ICPData *icp_data;
	icp_data = (struct ICPData *) threaddata;
	icp_data->puppet->updateCloudPair(icp_data->cloud);
	icp_data->puppet->getCorrsAndErrors(icp_data->old_pose,*icp_data->corrs, *icp_data->errs);
	return (void*) 42;
}

CloudTracker::~CloudTracker(){
}


/*
 *  CloudTracker Constructor. Assign parameters to default values.
 */
CloudTracker::CloudTracker() : nh_global(), nh_local("~") {
	processed_frame 		= 0;

	time_taken_icp 			= 0;
	time_taken_publish 		= 0;
	tracker_frame_skip 		= 0;
	filter_correspondences	= true;
	filter_ratio			= 1.2;
	useNormals				= true;
	use_SIFT				= false;
	queue_size				= 5;
	using_bag				= false;
	min_cloud_size			= 25;
	use_multi_core_tracker 	= true;
	use_inverse_icp			= true;
	puppet_as_source			= false;
	use_old_cloud_separator = false;
	filter_ratio			= 1.2;
	color_cloud_separator	= false;
	SIFT_saving				= false;
	capture_fps				= 30;
	use_rgb_feed			= true;
	raw_capture				= false;
	new_capture_session     = true;
	cloud_skip				= 1;
	local_capture_started	= false;
	create_new_video		= false;
	min_puppet_distance 		= 0.10;
	max_cloud_size			= 300;
}


/*
 *  Setup OpenCV video writer for saving the raw RGB feed from the Kinect
 *
 *  \param 	f_path				Folder for video file
 *  \param 	v_name				Base name for video file
 *  \return	output_raw_video	Full video filename
 */
std::string CloudTracker::setupRawVideoRecorder(std::string f_path, std::string v_name){
	files_path 		= f_path;
	video_out_name	= v_name;
	export_raw_video = true;
	three_ch_frame  = Mat(480,640,CV_8UC3);
    output_raw_video 	= files_path;
    output_raw_video.append("animations/");
    output_raw_video.append(video_out_name);
    output_raw_video.append("RawVideo.avi");
	return output_raw_video;
}

/*
 *  Save one RGB frame to the raw video file.
 */
void CloudTracker::saveRawVideoFrame(){
	// Store time of capture for combination with rendered video
	gettimeofday(&tracker_time, NULL);
	video_timestamps.push_back(tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0));
	if (new_capture_session)
	{
		// Start new capture session
		record_initial_time = tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
		if (video_timestamps.size() > 0)
		{
			accumulated_time = video_timestamps.back();
		}
		new_capture_session = false;
	}
	if (!use_rgb_feed)
	{
		// Convert RGB image to B&W
		cvtColor(raw_capturedImg_bw, three_ch_frame, CV_GRAY2BGR);
		raw_writer << three_ch_frame;
	} else {
		raw_writer << raw_capturedImg;
	}
	// Inform rest of program that raw-video capture has begun.
	if (!local_capture_started)
		local_capture_started = true;
}


/*
 *  Used during normal operation (not while saving SIFT templates).
 *  Accepts RGB image feed from Kinect. Displays image in its own window and exports
 *  it to a video file if an animation is being recorded.
 *
 *  \param 	msg			Kinect RGB image
 */
void CloudTracker::imgCallbackRGB(const sensor_msgs::ImageConstPtr& rgb_img)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(rgb_img,"bgr8");
		flip(cv_ptr->image,raw_capturedImg,-1);
		imshow("Video Feed",raw_capturedImg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (export_raw_video && raw_capture)
		saveRawVideoFrame();
}


/*
 *  Used while saving SIFT templates.
 *  Accepts image and depth feeds from Kinect. Displays image in its own window. Also
 *  captures image and depths for SIFT pose templates.
 *
 *  \param 	msg			Kinect RGB image
 *  \param 	depth_msg	Corresponding depth map
 */
void CloudTracker::imgCallbackColorDepth(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& depth_msg){
	if (!using_bag){
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		if (SIFT_saving && SIFT_save_now)
		{
			// Output captured RGB image in two formats
			std::string SIFT_img = SIFT_save_name_numbered;
			SIFT_img.append(".png");
			imwrite(SIFT_img.c_str(), cv_ptr->image);
			SIFT_img = SIFT_save_name_numbered;
			SIFT_img.append(".jpg");
			imwrite(SIFT_img.c_str(), cv_ptr->image);

			// Process depth values, removing NaNs. Then save to a yaml file.
			const float* depth_in = reinterpret_cast<const float*>(&depth_msg->data[0]);
			vector<float> depths;
			for (int i = 0; i < (640 * 480); i++) {
				float d = depth_in[i];
				if (d != d) {
					depths.push_back(0.0f);
				} else {
					depths.push_back(d);
				}
			}
			std::string depths_file = SIFT_save_name_numbered;
			depths_file.append(".yml");

			// Alternate saving method (more compressed):
//			CvFileStorage * fileStorage = cvOpenFileStorage(depths_file.c_str(),0,CV_STORAGE_WRITE);
//			cvStartWriteStruct(fileStorage,"depths",CV_NODE_SEQ | CV_NODE_FLOW,NULL);
//			cvWriteRawData(fileStorage,&depths[0], depths.size(), "f");
//			cvEndWriteStruct(fileStorage);
//			cvReleaseFileStorage(&fileStorage);

			FileStorage fs(depths_file.c_str(), FileStorage::WRITE);
			fs << "depths" << "[:";
			for (uint i = 0; i < depths.size(); i++) {
				fs << depths[i];
			}
			fs.release();
			SIFT_save_now = false;
			ROS_INFO_STREAM("RGB and depth grabbed");
		}

		// Display RGB image. Flip first to match user's point of view.
		flip(cv_ptr->image,raw_capturedImg,-1);
		imshow("Video Feed",raw_capturedImg);
	}
}


/*
 *  Set up member variables and OpenCV video functionality
 */
void CloudTracker::prep()
{

	num_puppets = Puppets.size();

	if (!use_old_cloud_separator)
	{
		for (int i=0; i<num_puppets; i++){
			clouds.push_back(new pcl::PointCloud<PointT>);
		}
	}

	if (use_SIFT)
	{
		for (int i=0; i<num_puppets; i++)
		{
			vector<float> temp;
			SIFT_poses.push_back(temp);
		}
	}

	if (!using_bag){
		cvNamedWindow("Video Feed");
		cvStartWindowThread();
		cvResizeWindow("Video Feed",640,480);
	}
	video_timestamps.reserve(60000*sizeof(double));
}

/*
 *  Start running this CloudTracker.
 *
 */
void CloudTracker::start() {

	pthread_mutex_lock(&amutex);

	// Subscribe to the 3D-point-cloud-feed from the Kinect:
	cloud_subscription 	= nh_global.subscribe<sensor_msgs::PointCloud2>("incloud", queue_size, boost::bind(&CloudTracker::cloudCallback, this, _1));
	if (use_SIFT)
	{
		pose_subscription 	= nh_global.subscribe<std_msgs::Float32MultiArray>("pose_feed", queue_size,  boost::bind(&CloudTracker::poseCallback, this, _1));
	}

	// Subscribe to the Kinect's raw RGB and depth feeds:
	image_transport::ImageTransport it(nh_global);
	image_transport::Subscriber bw_only_image_sub;
	image_transport::Subscriber rgb_only_image_sub;
	image_transport::SubscriberFilter rgb_image_sub(it,"rgb_feed",1);
	image_transport::SubscriberFilter depth_image_sub(it,"depth_feed",1);
	message_filters::Synchronizer< MySyncPolicy > sync( MySyncPolicy( 10 ), rgb_image_sub, depth_image_sub );

	if (!playback_only)
	{
		if (SIFT_saving)	{
			// Need to capture both image and depth feeds and ensure they are synced
			ROS_INFO_STREAM("Setting up synced depth and color streams");
			sync.registerCallback( boost::bind( &CloudTracker::imgCallbackColorDepth, this, _1, _2 ) );
		} else {
			// Otherwise we just need the image feed
			if (use_rgb_feed)
			{
				rgb_only_image_sub = it.subscribe("rgb_feed", queue_size,  boost::bind(&CloudTracker::imgCallbackRGB, this, _1));
			} else {
				bw_only_image_sub = it.subscribe("bw_feed", queue_size,  boost::bind(&CloudTracker::imgCallbackBW, this, _1));
			}
		}
		raw_capturedImg  	= Mat(480,640,CV_8UC3);
		raw_capturedImg_bw  = Mat(480,640,CV_8UC1);
	}


	// Start running this service
	ros::AsyncSpinner spinner(0);
	spinner.start();

	pthread_cond_signal(&cond);
	pthread_mutex_unlock(&amutex);

	ros::waitForShutdown();

	return;
}


/*
 *  Takes incoming 3D point cloud from the Kinect and prepare it for use with ICP by
 *  computing normals if necessary and packaging the points into data structures.
 *  Also manages communications (via global variables) with GLManager class.
 *
 *  \param	cloud_ptr	Kinect's 3D point cloud
 */
void CloudTracker::cloudCallback(sensor_msgs::PointCloud2ConstPtr cloud_ptr) {
	// Timing variables
//	double start;
//	double before_pc_conversion;
//	double after_pc_conversion;
//	double pc_conversion_time;
//	double before_icp;
//	double after_icp; // use this also in place of before_publish
//	double icp_time;
//	double after_publish;
//	double publish_time;
//	double mutex_time;
//	double sum_time;
//	double final_time;

//	gettimeofday(&tracker_time, NULL);
//	start 	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);

	if(processed_frame%(tracker_frame_skip+1)==tracker_frame_skip)		// Used to slow down tracking / take up less CPU
	{
//			gettimeofday(&tracker_time, NULL);
//			before_pc_conversion 	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);

		pcl::PointCloud<pcl::PointXYZRGB> kcloud;
		pcl::fromROSMsg(*cloud_ptr, kcloud);

		// For normals calcs
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyzcloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal> cloud_normals;

		// Convert the kcloud into the cloud as required by the point_cloud_icp package
		uint num_points = kcloud.points.size();

		if (num_points >= min_cloud_size)
		{
			if (use_old_cloud_separator)
			{
				clouds.clear();
				for (int i = 0; i < num_puppets; i++){
					clouds.push_back(new pcl::PointCloud<PointT>);
				}
			} else {
				mastercloud.points.resize(num_points/cloud_skip + 1);
				if (!filter_correspondences)
				{
					for (int i = 0; i < num_puppets; i++){
						clouds[i]->points.resize(num_points/cloud_skip + 1);
					}
				}
			}

			if (useNormals)
			{
				xyzcloud->points.resize(num_points/cloud_skip + 1);
				// First calculate the normals for the kinect cloud
				for(uint i=0;i<num_points;i+=cloud_skip)
				{
					pcl::PointXYZ xyzpt;
					pcl::PointXYZRGB kpt = kcloud.points[i];
					xyzpt.x = kpt.x;
					xyzpt.y = kpt.y;
					xyzpt.z = kpt.z;
					for(int j=0;j<4;j++)
						xyzpt.data[j] = kpt.data[j];
					xyzcloud->points[i/cloud_skip] = xyzpt;
				}

				// Create the normal estimation class, and pass the input dataset to it
				pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
				ne.setInputCloud (xyzcloud);

				// Create an empty kdtree representation, and pass it to the normal estimation object.
				// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
				pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
				ne.setSearchMethod (tree);
				ne.setKSearch(8);

				// Compute the features
				ne.compute (cloud_normals);
			}

			// Assemble the point clouds for use by ICP
			for(uint i=0;i<num_points;i+=cloud_skip)
			{
				PointT pt;

				pcl::PointXYZRGB kpt = kcloud.points[i];
				pt.rgb = kpt.rgb;
				pt.x = kpt.x;
				pt.y = kpt.y;
				pt.z = kpt.z;
				for(int j=0;j<4;j++)
				{
					pt.data[j] = kpt.data[j];
				}
				if (useNormals)
				{
					pcl::Normal npt = cloud_normals.points[i/cloud_skip];
					pt.curvature = npt.curvature;	pt.imgX = 0;	pt.imgY = 0;
					pt.normal[0] = npt.normal[0]; pt.normal[1] = npt.normal[1]; pt.normal[2] = npt.normal[2];
					pt.normal_x = npt.normal_x;
					pt.normal_y = npt.normal_y;
					pt.normal_z = npt.normal_z;
				}

				if (num_puppets == 1 || !use_old_cloud_separator) {
					if (filter_correspondences)
					{
						mastercloud.points[i/cloud_skip] = pt;
					} else {
						for (int c=0;c<num_puppets;c++)
						{
							clouds[c]->points[i/cloud_skip] = pt;
						}
					}
				} else if (num_puppets == 2 && use_old_cloud_separator) {
					const uint32_t rgbi = *reinterpret_cast<const uint32_t*>(&(pt.rgb));
													const boost::array<float, 3> fcolor =
																{{(float)((rgbi >> 16) & 0xff) / 255,
																(float)((rgbi >> 8) & 0xff) / 255,
																(float)(rgbi & 0xff) / 255}};
					if (color_cloud_separator)
					{
						// Separate point cloud based on whether each point is yellow or red
						if ((fcolor[0]/fcolor[2] > 1.5) && (fcolor[1]/fcolor[2] > 1.5)){
							clouds[1]->points.push_back(pt);
						} else if ((fcolor[0]/fcolor[1] > 1.5) && (fcolor[0]/fcolor[2] > 1.5)) {
							clouds[0]->points.push_back(pt);
						}
					} else {
						if (fcolor[0] == 0)
						{
							clouds[0]->points.push_back(pt);
						} else {
							clouds[1]->points.push_back(pt);
						}
					}
				} else {
					ROS_WARN("Incompatible number of puppets and choice of cloud separation");
				}
			}

			for (int i = 0; i < num_puppets; i++){
				if (filter_correspondences)
					*clouds[i] = mastercloud;
				clouds[i]->height 	= 1;
				clouds[i]->width 	= clouds[i]->points.size();
			}
//				gettimeofday(&tracker_time, NULL);
//				after_pc_conversion 	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);

			//////////////////////////////////////////////////////////////////////
			pthread_mutex_lock(&amutex);

			if (!suspend_tracking)
				sendCloudsToPuppets();	// Send processed cloud to be segmented and matched to puppets

			// Deal w/ global variables
			if (global_capture && export_raw_video)
			{
				raw_capture 			= true;
			} else {
				// Start recording raw image feed
				raw_capture				= false;
				new_capture_session 	= true;
				local_capture_started 	= false;
				raw_capture_started		= false;
			}
			if (local_capture_started)
				raw_capture_started 	= true;
			if (transfer_colors)
				transfer_colors = false;
			if (spread_colors)
			{
				for (int m=0; m<num_puppets; m++)
				{
					Puppets[m]->spreadVertexColors();
					puppet_lists[m] 	= 0;
					puppet_VBOs_created[m]	= 0;
				}
				spread_colors = false;
			}
			if (clear_animation)
			{
				create_new_video 	= true;
				clear_animation	 	= false;
			}
			if (signal_SIFT_save)
			{
				ROS_INFO_STREAM("Received signal");
				SIFT_save_now = true;
				signal_SIFT_save = false;
			}
			if (save_new_ply)
			{
				for (int m=0; m<num_puppets; m++)
					Puppets[m]->saveNewPly();
				save_new_ply = false;
			}
			pthread_cond_signal(&cond);
			pthread_mutex_unlock(&amutex);
//				cout << "Clouds sent to puppets" << endl;
//				gettimeofday(&tracker_time, NULL);
//				after_publish	=	tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
		} else {
			pthread_mutex_lock(&amutex);

			// Check whether any puppets have left the tracking space
			for (int m=0; m < num_puppets; m++)
				if (!Puppets[m]->checkVisibility())	{
					Puppets[m]->visible 	= false;
					Puppets[m]->lost 		= true;
					Puppets[m]->setRotationFilter(false);
				}

			// (See comments above)
			if (global_capture && export_raw_video)
			{
				raw_capture 			= true;
			} else {
				raw_capture				= false;
				new_capture_session 	= true;
				local_capture_started 	= false;
				raw_capture_started		= false;
			}
			if (local_capture_started)
				raw_capture_started		= true;
			if (transfer_colors)
				transfer_colors = false;
			if (spread_colors)
			{
				for (int m=0; m<num_puppets; m++)
				{
					Puppets[m]->spreadVertexColors();
					puppet_lists[m]	= 0;
					puppet_VBOs_created[m]	= 0;
				}
				spread_colors = false;
			}
			if (clear_animation)
			{
				create_new_video 	= true;
				clear_animation	 	= false;
			}
			if (signal_SIFT_save)
			{
				ROS_INFO_STREAM("Received signal");
				SIFT_save_now = true;
				signal_SIFT_save = false;
			}
			if (save_new_ply)
			{
				for (int m=0; m<num_puppets; m++)
					Puppets[m]->saveNewPly();
				save_new_ply = false;
			}
			pthread_cond_signal(&cond);
			pthread_mutex_unlock(&amutex);
		}
	} else {
		pthread_mutex_lock(&amutex);
		// (See comments above)
		if (global_capture && export_raw_video)
		{
			raw_capture 			= true;
			create_new_video		= true;
		} else {
			raw_capture				= false;
			new_capture_session 	= true;
			local_capture_started 	= false;
			raw_capture_started		= false;
		}
		if (local_capture_started)
			raw_capture_started = true;
		if (transfer_colors)
			transfer_colors = false;
		if (spread_colors)
		{
			for (int m=0; m<num_puppets; m++)
			{
				Puppets[m]->spreadVertexColors();
				puppet_lists[m] 	= 0;
				puppet_VBOs_created[m]	= 0;
			}
			spread_colors = false;
		}
		if (clear_animation)
		{
			create_new_video 	= true;
			clear_animation	 	= false;
		}
		if (signal_SIFT_save)
		{
			ROS_INFO_STREAM("Received signal");
			SIFT_save_now = true;
			signal_SIFT_save = false;
		}
		if (save_new_ply)
		{
			for (int m=0; m<num_puppets; m++)
				Puppets[m]->saveNewPly();
			save_new_ply = false;
		}
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&amutex);
	}


	if (create_new_video){
		// Clear raw images
		video_timestamps.clear();
		captured_frames.clear();
		create_new_video = false;
		raw_writer.open(output_raw_video.c_str(),CV_FOURCC('M', 'P', '4', '2'),capture_fps,cvSize(640,480),1);
	}

	processed_frame++;

//	gettimeofday(&tracker_time, NULL);
//	final_time 	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
//	pc_conversion_time 	= after_pc_conversion - before_pc_conversion;
//	icp_time 			= after_publish - after_pc_conversion;
//	publish_time		= after_publish- after_icp;
//	mutex_time			= final_time - after_publish;
//	sum_time			= pc_conversion_time + icp_time;
//	ROS_INFO_STREAM("Time to convert cloud: "<< pc_conversion_time);
//	ROS_INFO_STREAM("Time to perform ICP: "<< icp_time);
//	ROS_INFO_STREAM("Time to publish cloud: "<< publish_time);
//	ROS_INFO_STREAM("Time to deal with threads: "<< mutex_time);
//	ROS_INFO_STREAM("Sum of times: "<< (after_publish - before_pc_conversion));
//	ROS_INFO_STREAM("Actual time: "<< (final_time - start));
}

/*
 *  Takes in the point cloud prepared by the point-cloud callback and segments it
 *  according to puppet. Then runs ICP to align each puppet's pose with its corresponding
 *  segmented point cloud.
 *
 *  NOTE: This function only works properly when puppet_as_source is set to FALSE. We never
 *  finished implementation for using the puppet as the source point cloud in ICP because it
 *  was slow and incompatible with our point-cloud segmenter.
 */
void CloudTracker::sendCloudsToPuppets()
{
	// Create thread variables
	pthread_attr_t attr;
	void* status;
	struct ICPData icp_data[num_puppets];
//	double before_segmentation,before_icp;
//	double after_icp;
//	gettimeofday(&tracker_time, NULL);
//	before_segmentation	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);

	if (use_SIFT)
	{
		// If we're using the rth_SIFT package to identify puppets, check to see which puppets have been
		// detected.
		for (int i=0;i<num_puppets;i++)
		{
			if (Puppets[i]->lost && !SIFT_poses[i].empty())
			{
				// If we're not already tracking a puppet, check to see whether rth_SIFT has detected it
				// This happens when a puppet is first brought into the Kinect's FOV or after we have lost
				// tracking of it.

				// First make sure that the new puppet doesn't overlap with an existing one
				// This problem may arise from incorrect SIFT matching
				bool register_puppet = true;
				Transform3f temp_pose;
				for (uint s = 0; s < 4; s++) {
					for (uint t = 0; t < 4; t++) {
						temp_pose.matrix()(t, s) = SIFT_poses[i][s * 4 + t];
					}
				}
				for (int m=0; m<num_puppets; m++){
					if (m!=i)
					{
						if ((temp_pose.translation()-(Puppets[m]->getCurrentPoseTransform()).translation()).norm() < min_puppet_distance)
							{
								register_puppet = false;
							}
					}

				}

				// Now retrieve the rough estimate of the puppet's pose from the rth_SIFT message
				if (register_puppet)
				{
					if (!use_inverse_icp)
						Puppets[i]->resetPuppetCloud();
					vector<float> temp_pose;
					Puppets[i]->setCurrentPose(SIFT_poses[i]);
					Puppets[i]->lost 			= false;
					Puppets[i]->setRotationFilter(true);
					Puppets[i]->lost_counter 	= 0;
					if (!use_inverse_icp)
						Puppets[i]->transformPuppetCloud();
				}
			}
		}
	}

	if (!filter_correspondences && use_multi_core_tracker)
	{
		// This is the case where we are not segmenting the point cloud, but still using multiple threads
		// This mode we can only track one puppet, so it should only be enabled if we're capturing SIFT
		// templates or colorizing a model. It should not be enabled for animation capture.

		// Prepare parallel threads
		pthread_t threads[num_puppets];
		for (int m = 0; m < num_puppets; m++){
			icp_data[m].pose_index 	= -1;
			icp_data[m].cloud 			= clouds[m];
			icp_data[m].puppet			= Puppets[m];
		}

		// Start threads
		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		for (int m = 0; m < num_puppets; m++){
			if(clouds[m]->points.size()>min_cloud_size)
			{
				pthread_create(&threads[m], &attr, parallel_ICP, (void *) &icp_data[m]);
			}
			else if (!Puppets[m]->checkVisibility())
			{
				Puppets[m]->visible 	= false;
				Puppets[m]->lost 	= true;
				Puppets[m]->setRotationFilter(false);
			}
		}

		// Wait for the threads to execute
		pthread_attr_destroy(&attr);
		for (int m = 0; m < num_puppets; m++){
			if(clouds[m]->points.size()>min_cloud_size)
				pthread_join(threads[m], &status);
		}

		if (transfer_colors)
		{
			// If we're colorizing a puppet, transfer colors from the Kinect's point cloud to the
			// puppet's model
			for (int m=0; m<num_puppets; m++)
				transferColorsFromKinect(m,clouds[m]);
		}
	} else {
		// We are either segmenting the point cloud, using multiple threads, or both.

		vector <Transform3f>	old_poses(num_puppets);
		vector<bool> 			present(num_puppets,false);
		int num_present = 0;
		if (filter_correspondences && !use_old_cloud_separator)
		{
			// We want to segment the point cloud.

			// Check how many puppets are present
			// Useful for limiting point-cloud segmentation only to relevant puppets.
			for (int m=0; m<num_puppets; m++)
			{
				if (!Puppets[m]->lost)
				{
					old_poses[m] = Puppets[m]->getCurrentPoseTransform();
					present[m] = true;
					num_present++;
				}
			}

			if (!puppet_as_source)
			{
				// Start point-cloud segmentation process
				// Get point-point correspondences for all the present puppets
				vector< vector<int> >	all_corrs(num_puppets);
				vector< vector<float> > all_errs(num_puppets);

				for (int m=0;m<num_puppets;m++)
				{
					if (present[m])
					{
						all_corrs[m].resize(mastercloud.points.size());
						all_errs[m].resize(mastercloud.points.size());
					}
				}

//				gettimeofday(&tracker_time, NULL);
//				double before_corr	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
//				ROS_INFO_STREAM("Time to copy cloud: " << before_corr - before_cloud_copy);

				if (use_multi_core_tracker)
				{
					// Prepare parallel threads
					pthread_t corr_threads[num_present];
					pthread_attr_t corr_attr;
					void* corr_status;



					for (int m = 0; m < num_puppets; m++){
						if (present[m])
						{

							icp_data[m].pose_index 	= -1;
							icp_data[m].cloud 		= clouds[m];
							icp_data[m].puppet		= Puppets[m];
							icp_data[m].corrs		= &all_corrs[m];
							icp_data[m].errs		= &all_errs[m];
							icp_data[m].old_pose	= old_poses[m];
						}
					}

//					gettimeofday(&tracker_time, NULL);
//					double after_setup	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
//					ROS_INFO_STREAM("Time to setup corr: " << after_setup - before_corr);

					// Start threads
					pthread_attr_init(&corr_attr);
					pthread_attr_setdetachstate(&corr_attr, PTHREAD_CREATE_JOINABLE);
					int current_thread = 0;
					for (int m = 0; m < num_puppets; m++){
						if (present[m])
						{
							pthread_create(&corr_threads[current_thread], &corr_attr, parallel_correspondences, (void *) &icp_data[m]);
							current_thread++;
						}
					}

					// Wait for the threads to execute
					pthread_attr_destroy(&corr_attr);
					current_thread = 0;
					for (int m = 0; m < num_puppets; m++){
						if(present[m])
						{
							pthread_join(corr_threads[current_thread], &corr_status);
							current_thread++;
						}
					}

				} else	{
					// Get the puppets' pt-to-pt corresponds in series
					for (int m=0;m<num_puppets; m++)
					{
						if (present[m])
						{
							Puppets[m]->updateCloudPair(clouds[m]);
							Puppets[m]->getCorrsAndErrors(old_poses[m],all_corrs[m], all_errs[m]);
						}
					}
				}

//				gettimeofday(&tracker_time, NULL);
//				double after_corr	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
//				ROS_INFO_STREAM("Time to find correspondences: " << after_corr - before_corr);

				// Compare correspondences
				vector<int> 	closest_corr(mastercloud.points.size(),-1);	// Closest puppet for each point in the cloud
				vector<int> 	second_closest_corr(mastercloud.points.size(),-1);
				vector<float>	smallest_errors(mastercloud.points.size(),0.10);
				vector<float>	second_smallest_errors(mastercloud.points.size(),0.10);

				// For each point in the Kinect's point cloud, determine the nearest puppet
				for (int m=0; m<num_puppets; m++)
				{
					if (present[m])
					{
						vector<int>		*corrs;
						vector<float>	*errs;

						corrs 	= &all_corrs[m];
						errs 	= &all_errs[m];

						for (uint p=0;p<corrs->size();p++)
						{
							int match = corrs->at(p);	// The match is a point in the puppet.
							if (match != -1)
							{
								// Check whether this point has not been assigned to a puppet (-1)
								if ((closest_corr[p] == -1) && (errs->at(p) < smallest_errors[p]))
								{
									closest_corr[p] = m;
									smallest_errors[p] = errs->at(p);
								} else if (errs->at(p) < smallest_errors[p]) // or if the distance is shorter than the existing match.
								{
									// Don't replace the second match if it's the same as the current best
									if (closest_corr[p] != m)
									{
										second_closest_corr[p] = closest_corr[p];
										second_smallest_errors[p] = smallest_errors[p];
									}
									closest_corr[p] = m;
									smallest_errors[p] = errs->at(p);
								} else if ((second_closest_corr[p] == -1) && (m != closest_corr[p]))
								{
									// Only assign a second-closest match if it's from a different puppet.
									second_closest_corr[p] = m;
									second_smallest_errors[p] = errs->at(p);
								} else if ((errs->at(p) < second_smallest_errors[p]) && (m != closest_corr[p]))
								{
									second_closest_corr[p] = m;
									second_smallest_errors[p] = errs->at(p);
								}
							}
						}
					}
					clouds[m]->points.clear();
				}

//				gettimeofday(&tracker_time, NULL);
//				double after_sort	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
//				cout << "Time to sort correspondences: " << after_sort - after_corr << endl;

				// Only used for publishing point clouds:
				pcl::PointCloud<PointT>* unmatched_pts = new pcl::PointCloud<PointT>;

				// Assign each point in the Kinect's point cloud to separate clouds corresponding to
				// each puppet. Assign them based on the proximities computed above.
				for (uint p=0;p<closest_corr.size();p++)
				{
					int match = -1;	// Will store matched puppet
					// First check that a correspondence was found
					if (closest_corr[p] != -1)
					{
						if (second_closest_corr[p] == -1)
						{
							// No second-closest correspondence. Immediately assign to puppet
							match = closest_corr[p];
						} else if (smallest_errors[p] < second_smallest_errors[p]/filter_ratio)
						{
							// Ratio of closest:2nd-closest match passes threshold. Assign to closest puppet.
							match = closest_corr[p];
						}
					}

					if (match > -1)
					{
						clouds[match]->points.push_back(mastercloud.points[p]);

						if (publish_clouds)
						{
							// The following lines are used for debugging. Use rviz to visualize the clouds.
							RGBValue color;
							if (match == 1)
							{
								color.Red   = 200;
								color.Green = 200;
								color.Blue  = 255;
							} else if (match == 2 || match == 0){
								color.Red   = 100;
								color.Green = 100;
								color.Blue  = 255;
							} else if (match == 3 || match == 5){
								color.Red   = 100;
								color.Green = 255;
								color.Blue  = 100;
							} else {
								color.Red   = 150;
								color.Green = 150;
								color.Blue  = 150;
							}
							color.Alpha = 0.5;
							if (match > -1)
								(clouds[match]->points.back()).rgb = color.float_value;
						}
					} else if (match == -1 && publish_clouds){
						RGBValue color;
						color.Red   = 100;
						color.Green = 100;
						color.Blue  = 100;
						unmatched_pts->points.push_back(mastercloud.points[p]);
						unmatched_pts->points.back().rgb = color.float_value;
					}
				}

				if (publish_clouds)
				{
					markedCameraCloudCombined.points.clear();
					markedCameraCloudCombined.header.frame_id	= "/camera_rgb_optical_frame";
					markedCameraCloudCombined.height   			= 0;
					markedCameraCloudCombined.width   			= 0;
				}
				for (int m = 0; m < num_puppets; m++){
					// More debugging/visualization code
					if (publish_clouds && clouds[m]->points.size() > 0)
					{
//						// Add yellow-colored points representing correspondences in stored puppet
//						pcl::PointCloud<PointT> puppet_cloud;
//						puppet_cloud = Puppets[m]->cloud;
//						RGBValue color;
//						color.Red   = 200;
//						color.Green = 0;
//						color.Blue  = 0;
//						color.Alpha = 0.5;
//						for (uint p=0;p<puppet_cloud.points.size();p++)
//							(puppet_cloud.points[p]).rgb = color.float_value;
////						rgbd::transform_point_cloud_in_place(Puppets[m]->getCurrentPoseTransform(), puppet_cloud, true);
////						ROS_INFO_STREAM(Puppets[m]->icp_it);
//						if (Puppets[m]->icp_it >= 0)
//						{
////							rgbd::transform_point_cloud_in_place(Puppets[m]->all_iterations[Puppets[m]->icp_it], puppet_cloud, true);
////							rgbd::mergeInPlace(puppet_cloud, markedCameraCloudCombined); // Merge "combined" cloud with matched points in stored puppet
//							rgbd::mergeInPlace(*clouds[m], markedCameraCloudCombined); // Merge "combined" cloud with estimated pose
//						}
//					}

					rgbd::mergeInPlace(*clouds[m], markedCameraCloudCombined); // Merge "combined" cloud with estimated pose
					}
				}

				if (publish_clouds && markedCameraCloudCombined.points.size() > 0)
				{
					rgbd::mergeInPlace(*unmatched_pts, markedCameraCloudCombined);
					cloud_pub.publish(markedCameraCloudCombined); // Show camera cloud with estimated pose
				}

//				gettimeofday(&tracker_time, NULL);
//				before_icp	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
//				ROS_INFO_STREAM("Time to sort correspondences: " << before_icp - before_segmentation);

				// Time to run ICP on each of the detected puppets
				if (use_multi_core_tracker)
				{
					// Prepare parallel threads
					pthread_t threads[num_present];

					for (int m = 0; m < num_puppets; m++){
						if (present[m])
						{
							icp_data[m].pose_index 	= -1;
							icp_data[m].cloud 		= clouds[m];
							icp_data[m].puppet		= Puppets[m];
//							if (Puppets[m]->all_iterations.size() == 0)
//								Puppets[m]->record_iterations = true;
						}
					}

					// Start threads
					pthread_attr_init(&attr);
					pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
					int current_thread = 0;
					for (int m = 0; m < num_puppets; m++){
						if(clouds[m]->points.size()>min_cloud_size && present[m])
						{
							pthread_create(&threads[current_thread], &attr, parallel_ICP, (void *) &icp_data[m]);
							current_thread++;
						}
						else if (!Puppets[m]->checkVisibility())
						{
							Puppets[m]->visible 	= false;
							Puppets[m]->lost 	= true;
							Puppets[m]->setRotationFilter(false);
						}
					}

					// Wait for the threads to execute
					pthread_attr_destroy(&attr);
					current_thread = 0;
					for (int m = 0; m < num_puppets; m++){
						if(clouds[m]->points.size()>min_cloud_size && present[m])
						{
							pthread_join(threads[current_thread], &status);
							current_thread++;
						}
					}
				} else {
					// Run ICP on each puppet in series.
					for (int m = 0; m < num_puppets; m++)
					{
						if(clouds[m]->points.size()>min_cloud_size && present[m])
						{
							if (!use_SIFT || (use_SIFT && !Puppets[m]->lost))
							{
								Puppets[m]->updatePose(clouds[m], TWO_STAGE_ICP, true,0.1);	// Compute pose of puppet
							}
						} else if (!Puppets[m]->checkVisibility())	{
							Puppets[m]->visible 	= false;
							Puppets[m]->lost 		= true;
							Puppets[m]->setRotationFilter(false);
						}
					}
				}

				// If we're colorizing a puppet, transfer the colors from the Kinect point cloud.
				if (transfer_colors)
				{
					for (int m=0; m<num_puppets; m++)
						transferColorsFromKinect(m,clouds[m]);
				}

			} else {
				// Case where the puppets' cloud is used as the source and the Kinect's cloud
				// is used as the target in ICP.
				// IMPORTANT: This mode is no longer supported. It is slow, likely buggy, and may
				// not work at all. Only left here in case we want to try some other approaches
				// in the future.

				pthread_t threads[num_puppets];
				assert(puppet_as_source);

				pcl::PointCloud<PointT> mastercloud = *clouds[0];

				// First perform the coarse ICP and retrieve the correspondences
				for (int m = 0; m < num_puppets; m++)
				{
					if(clouds[m]->points.size()>m)
					{
						if (!use_SIFT || (use_SIFT && !Puppets[m]->lost))
						{
							Puppets[m]->updateCloudPair(clouds[m]);
						}
					}
				}

				// Compare correspondences
				vector<int> 	closest_corr(mastercloud.points.size(),-1);	// Closest puppet for each point in the cloud
				vector<float>	smallest_errors(mastercloud.points.size(),100);

				vector< vector <int> > all_corrs;

				for (int m=0; m<num_puppets; m++)
				{
					vector<int>		corrs;
					vector<float>	errs;
					Puppets[m]->getCorrsAndErrors(old_poses[m],corrs, errs);

					all_corrs.push_back(corrs);
					for (uint p=0;p<corrs.size();p++)
					{
						int match = corrs[p];
						if (match != -1)
						{
							if ((closest_corr[match] == -1) || (errs[p] < smallest_errors[match]))
							{
								closest_corr[match] = m;
								smallest_errors[match] = errs[p];
							}
						}
					}

					clouds[m]->points.clear();
					Puppets[m]->setCurrentPoseTransform(old_poses[m]);
				}

				for (int m=0; m<num_puppets; m++)
				{
					vector<bool> added(mastercloud.points.size(),false);
					for (uint p=0;p<all_corrs[m].size();p++)
					{
						int match = all_corrs[m][p];
						if (match < 0)
							continue;
						if ((closest_corr[match] == m) && (added[match] == false))
						{
							added[match] = true;
							clouds[m]->points.push_back(mastercloud.points[match]);
							RGBValue color;
							if (m == 0)
							{
								color.Red   = 100;
								color.Green = 0;
								color.Blue  = 0;
							} else {
								color.Red   = 0;
								color.Green = 100;
								color.Blue  = 0;

							}
							color.Alpha = 0.5;
							(clouds[m]->points.back()).rgb = color.float_value;
						}
					}
				}

				if (publish_clouds)
				{
					markedCameraCloudCombined.points.clear();
					markedCameraCloudCombined.header.frame_id	= "/openni_camera";
					markedCameraCloudCombined.height   			= 0;
					markedCameraCloudCombined.width   			= 0;
				}
				for (int m = 0; m < num_puppets; m++){
					if (publish_clouds && clouds[m]->points.size() > 0)
					{
						rgbd::mergeInPlace(*clouds[m], markedCameraCloudCombined); // Merge "combined" cloud with estimated pose
					}
				}

				if (publish_clouds && markedCameraCloudCombined.points.size() > 0)
					cloud_pub.publish(markedCameraCloudCombined); // Show camera cloud with estimated pose


				if (use_multi_core_tracker)
				{
					// Prepare parallel threads
					for (int m = 0; m < num_puppets; m++){
						icp_data[m].pose_index 	= -1;
						icp_data[m].cloud 		= clouds[m];
						icp_data[m].puppet		= Puppets[m];
					}

					// Start threads
					pthread_attr_init(&attr);
					pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
					for (int m = 0; m < num_puppets; m++){
						if(clouds[m]->points.size()>min_cloud_size)
						{
							pthread_create(&threads[m], &attr, parallel_ICP, (void *) &icp_data[m]);
						}
						else if (!Puppets[m]->checkVisibility())
						{
							Puppets[m]->visible 	= false;
							Puppets[m]->lost 	= true;
							Puppets[m]->setRotationFilter(false);
						}
					}

					// Wait for the threads to execute
					pthread_attr_destroy(&attr);
					for (int m = 0; m < num_puppets; m++){
						if(clouds[m]->points.size()>min_cloud_size)
							pthread_join(threads[m], &status);
					}
				} else {

					for (int m = 0; m < num_puppets; m++)
					{
						if(clouds[m]->points.size()>min_cloud_size)
						{
				//			Puppets[m]->visible = true;
				//			gettimeofday(&tracker_time, NULL);
				//			before_icp	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);

							if (!use_SIFT || (use_SIFT && !Puppets[m]->lost))
							{
								Puppets[m]->updatePose(clouds[m],TWO_STAGE_ICP, true,0.1);	// Compute pose of puppet
							}
				//			gettimeofday(&tracker_time, NULL);
				//			after_icp	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
				//			num_pts = clouds[m]->points.size();
						}
						else if (!Puppets[m]->checkVisibility())
						{
							Puppets[m]->visible 	= false;
							Puppets[m]->lost 	= true;
							Puppets[m]->setRotationFilter(false);
						}
					}
				}
			}

			if (transfer_colors)
			{
				for (int m=0; m<num_puppets; m++)
					transferColorsFromKinect(m,clouds[m]);
			}
		} else {
			// To get here, we must be neither segmenting the cloud nor using multiple threads.
			// Only provides support for up to two puppets, assuming use_old_cloud_separator or
			// color_cloud_separator were enabled. This mode is outdated and should never be used.
			for (int m = 0; m < num_puppets; m++)
			{
				if(clouds[m]->points.size()>min_cloud_size)
				{
		//			Puppets[m]->visible = true;
		//			gettimeofday(&tracker_time, NULL);
		//			before_icp	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);

					if (!use_SIFT || (use_SIFT && !Puppets[m]->lost))
					{
						Puppets[m]->updatePose(clouds[m],TWO_STAGE_ICP, true, 0.1);	// Compute pose of puppet
					}
		//			gettimeofday(&tracker_time, NULL);
		//			after_icp	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
		//			num_pts = clouds[m]->points.size();
				}
				else if (!Puppets[m]->checkVisibility())
				{
					Puppets[m]->visible 	= false;
					Puppets[m]->lost 	= true;
					Puppets[m]->setRotationFilter(false);
				}
			}
			if (transfer_colors)
			{
				for (int m=0; m<num_puppets; m++)
					transferColorsFromKinect(m,clouds[m]);
			}
		}
	}
}


/*
 *  Listens for puppet pose estimates published by the rth_SIFT package. Those estimates are
 *  used to initialize ICP-based tracking.
 *
 *  \param 	pose_msg	Float array containing all the puppet poses esimates concatenated
 *  					together.
 */
void CloudTracker::poseCallback(const std_msgs::Float32MultiArrayConstPtr &pose_msg)
{
	vector<float> all_poses_vector = pose_msg->data;
	int vec_size = all_poses_vector.size();
	int it = 0;
	for (int m=0; m<num_puppets;m++)
		SIFT_poses[m].clear();
	// Separate concatenated array into poses for each puppet
	if (all_poses_vector[0] != -1)
	{
		while (it < vec_size)
		{
			int puppet_ID = all_poses_vector[it];
			if (puppet_ID < num_puppets)
			{
				vector<float> temp_pose;
				for (int i=0;i<16;i++)
				{
					it++;
					temp_pose.push_back(all_poses_vector[it]);
				}
				SIFT_poses[puppet_ID] = temp_pose;
			}
			it++;
		}
	}
}

/*
 *  Based on ICP-derived correspondences, transfer colors from the Kinect point cloud
 *  to the puppet's stored model.
 *
 *  \param 	puppet_ID	Index of the puppet.
 *  \param  curr_cloud	The Kinect's colored point cloud
 */
void CloudTracker::transferColorsFromKinect(int puppet_ID, pcl::PointCloud<PointT> *curr_cloud)
{
	// Match puppet to point cloud to more easily set colors
	Puppets[puppet_ID]->puppet_as_source 		= true;
	Puppets[puppet_ID]->cloud_pair_created 	= false;
	Puppets[puppet_ID]->updateCloudPair(curr_cloud);

	// Get correspondences
	vector<int> 	corrs;
	vector<float>	errs;
	Puppets[puppet_ID]->getCorrsAndErrors(Puppets[puppet_ID]->getCurrentPoseTransform(),corrs, errs);

	for (uint p=0; p<Puppets[puppet_ID]->cloud.size();p++)
	{
		// Step through each correspondence and transfer the color to the puppet
		if (corrs[p]>=0 && errs[p] <= 0.0025)
		{
		  PointT pt = curr_cloud->points[corrs[p]];
		  const uint32_t rgbi = *reinterpret_cast<const uint32_t*>(&(pt.rgb));
											const boost::array<float, 3> fcolor =
														{{(float)((rgbi >> 16) & 0xff) / 255,
														(float)((rgbi >> 8) & 0xff) / 255,
														(float)(rgbi & 0xff) / 255}};
		  Puppets[puppet_ID]->setVertexColor(Puppets[puppet_ID]->cloud_to_original_vertices[p],fcolor[0],fcolor[1],fcolor[2]);
		}
	}

	puppet_lists[puppet_ID] 					= 0;
	puppet_VBOs_created[puppet_ID]				= 0;
	Puppets[puppet_ID]->puppet_as_source 		= false;
	Puppets[puppet_ID]->cloud_pair_created 		= false;
	Puppets[puppet_ID]->updateCloudPair(curr_cloud);
}

/////////////////////
// Functions for manipulating parameters:

void CloudTracker::setFilterCorrespondences(bool filt)
{
	filter_correspondences = filt;
}

void CloudTracker::enableMatchCloudPublishing()
{
	cloud_pub.advertise(nh_global, "outcloud", 5);
	publish_clouds = true;
}

void CloudTracker::enableCloudNormals()
{
	useNormals = true;
}

void CloudTracker::disableCloudNormals()
{
	useNormals = false;
}

void CloudTracker::setTrackerFrameSkip(int skip){
	tracker_frame_skip = skip;
}

/////////////////////
// Old, deprecated, or debugging functions:
void CloudTracker::imgCallbackBW(const sensor_msgs::ImageConstPtr& bw_img)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(bw_img,"mono8");
		flip(cv_ptr->image,raw_capturedImg_bw,-1);
		imshow("Video Feed",raw_capturedImg_bw);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (export_raw_video && raw_capture)
		saveRawVideoFrame();
}

