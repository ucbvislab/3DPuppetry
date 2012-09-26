/*
 * Puppet.cpp
 *
  *     Created: 2011/2012
  *      Author: Robin Held (begun by Ankit Gupta)
  *
  *      This class is used to represent each tracked puppet and includes
  *      functions that use the iterative closest point (ICP) algorithm
  *
  *      See LICENSE.txt for licensing info.
  */

#include "Puppet.h"

using namespace std;


// Useful functions:

int roundmine(double d) {return (int) floor(d + 0.5);}
bool orderfunc1(float x, float y) {return (x < y);}

/*
 *  Basic constructor. Sets parameters
 *
 */
Puppet::Puppet() {
	pose.setIdentity();
	render_pose.setIdentity();
	lastT.setIdentity();
	cloud.header.frame_id = "/primesensor_frame";
	using_legos = false;
	processed_frame = 0;
	lost_counter = 1;
	lost = true;
	icp_error = 0;
	icp_error_threshold = 0.0001; // Should be set in launch file
	inlier_error_threshold = 5;
	blocks.clear();
	block_name_counter = 0;
	use_multi_core = false;
	cloud_pair_created = false;
	use_inverse_icp = true;
	visible = false;
	current_pose_index = -1;
	simultaneous_threads = 2;
	angle_spacing = 90;
	max_ICP_rounds = 20;
	publish_marked_clouds = false;
	puppet_ID = 0;
	transient_pose.setIdentity();
	useNormals = true;
	max_normal_difference_deg = 45;
	h_tracking_region = 180;
	v_tracking_region = 180;
	near_tracking_region = 0;
	cloud_thread_params.search_threads = 4;
	outlier_thresh = 0.5;
	current_outlier_percentage = 0;
	do_filter = false;
	filter_alpha     = 0.8;
	icp_it = -1;
	max_filter_entries = 10;

	use_bilateral_filter = true;
	translation_sigma = 0.005; 	// In meters
	rotation_sigma	  = 15;		// In degrees
	time_sigma		  = 0.175;	// In seconds
	time_constant	  		= 1/(time_sigma*sqrt(2*PI));
	translation_constant 	= 1/(translation_sigma*sqrt(2*PI));
	rotation_constant	  	= 1/(rotation_sigma*sqrt(2*PI));
}


///////////////////////////////////////////
// PLY / OpenGL / 3D-model functions
///////////////////////////////////////////

/*
 *  Load the PLY file that contains the puppet's 3D model.
 *  Save the data both as geometric primitives for rendering
 *  and as a point cloud for pose-tracking
 *
 *	\param	fname	The PLY filename
 */
void Puppet::loadPLY(std::string fname) {
	display_ply_name = fname;
	fname.append(".ply");

	// Open the file and parse the data, line by line
	ifstream inFile(fname.c_str(), ifstream::in);
	char line[1023];
	while (inFile.good()) {
		inFile.getline(line, 1023);
		parsePLYLine(string(line));
	}

	inFile.close();

	// Assume that the same vertices will appear both in the puppet's
	// rendered model and in its point cloud.
	// Can explicitly load a set of vertices for rendering only using
	// the loadDisplayPly function.
	vertex_list_display 	= vertex_list;
	normal_list_display		= normal_list;
	color_list_display 		= color_list;
	triangle_list_display 	= triangle_list;
	num_triangles_display 	= num_triangles;
	num_vertices_display 	= num_vertices;
	recolored_vertices		= vector<int>(num_vertices_display,0);

	// Construct point cloud for non-lego puppets
	if (!using_legos) {
		char ur, ug, ub;
		for (int i = 0; i < vertices_loaded; i++) {
			PointT pt;
			pt.x = vertex_list[i * 3];
			pt.y = vertex_list[i * 3 + 1];
			pt.z = vertex_list[i * 3 + 2];
			pt.normal[0] = normal_list[i * 3];
			pt.normal[1] = normal_list[i * 3 + 1];
			pt.normal[2] = normal_list[i * 3 + 2];
			pt.normal_x = normal_list[i * 3];
			pt.normal_y = normal_list[i * 3 + 1];
			pt.normal_z = normal_list[i * 3 + 2];
			pt.curvature = 0;
			pt.imgX = 0;
			pt.imgY = 0;
			ur = (char) (255 * (int) color_list[i * 3]);
			ug = (char) (255 * (int) color_list[i * 3 + 1]);
			ub = (char) (255 * (int) color_list[i * 3 + 2]);
			pt.rgb = rgbd::packRGB(ur, ug, ub);
			cloud.points.push_back(pt);
		}
	}
}

/*
 *  If we want to use separate rendering and tracking models for this puppet,
 *  this function loads the model that will be used for rendering only.
 *  Sometimes this is useful if the rendered model has so many points, it would
 *  slow down ICP
 *
 *	\param	fname	The PLY filename
 */
void Puppet::loadDisplayPLY(std::string fname) {
	display_ply_name = fname;
	fname.append(".ply");
	vertex_list_display.clear();
	normal_list_display.clear();
	color_list_display.clear();
	triangle_list_display.clear();
	num_triangles_display = 0;
	num_vertices_display = 0;
	vertex_list.clear();
	normal_list.clear();
	color_list.clear();
	triangle_list.clear();
	num_triangles = 0;
	num_vertices = 0;

	ifstream inFile(fname.c_str(), ifstream::in);
	char line[1023];
	while (inFile.good()) {
		inFile.getline(line, 1023);
		parsePLYLine(string(line));
	}
	inFile.close();

	vertex_list_display 	= vertex_list;
	normal_list_display 	= normal_list;
	color_list_display 		= color_list;
	triangle_list_display 	= triangle_list;
	num_triangles_display 	= num_triangles;
	num_vertices_display 	= num_vertices;
	recolored_vertices		= vector<int>(num_vertices_display,0);
	ROS_INFO_STREAM("Display PLY loading complete.\n");
}

Vector3f Puppet::calculateNormal(Vector3f mainPt, Vector3f pt2, Vector3f pt3) {
	Vector3f leg1 = pt2 - mainPt;
	Vector3f leg2 = pt3 - mainPt;
	Vector3f norm = leg1.cross(leg2);
	norm.normalize();
	return norm;
}

/*
 *  Load the data from a PLY file, including vertex positions, normals, colors,
 *  and face indices.
 *  NOTE: Currently only supports ASCII PLY files.
 *
 *  \pram line	A single line from a PLY file.
 */
bool Puppet::parsePLYLine(string line) {
	float x, y, z, nx, ny, nz;
	int ix, iy, iz, r, g, b;
	//	char ur, ug, ub;
	string op; // Used to store the first entry in the line
	if (line.empty())
		return true;
	stringstream ss(stringstream::in | stringstream::out);
	ss.str(line);
	ss >> op;

	if ((op[0] == '#') || (op.compare("comment") == 0)) {
		// # indicates a commented line.  No information to parse
		return true;
	} else if (op.compare("element") == 0) {
		ss >> op;
		if (op.compare("vertex") == 0) {
			// Retrieve total number of points
			ss >> num_vertices;
			vertices_loaded = 0;
			//			vertex_list = new float[3*num_vertices];
			//			normal_list = new float[3*num_vertices];
			//			ROS_INFO_STREAM("Total number of vertices: " << num_vertices);
		} else if (op.compare("face") == 0) {
			triMesh = true;
			ss >> num_triangles;
			triangles_loaded = 0;
			//			ROS_INFO_STREAM("Total number of faces/triangles: " << num_triangles);
		}
	} else if (op.compare("property") == 0) {
		ss >> op;
		if (op.compare("float") == 0) {
			ss >> op;
			if (op.compare("nx") == 0) {
				hasNormals = true;
				//				ROS_INFO_STREAM("PLY file includes normals.");
			}
		} else if (op.compare("uchar") == 0) {
			ss >> op;
			if ((op.compare("diffuse_red") == 0) || (op.compare("red") == 0)) {
				hasColors = true;
				//				ROS_INFO_STREAM("PLY file includes colors.");
			}
		}
	} else if (op.compare("ply") == 0) {
		// Do nothing
	} else if (op.compare("format") == 0) {
		ss >> op;
		if (op.compare(0, 6, "binary") == 0) {
			ROS_WARN("Binary PLY files are not currently supported. Resave in ASCII format.");
		}
	} else if (op.compare("end_header") == 0) {
		// Done with the header
		pastHeader = true;
		if (!hasNormals) {
			normal_list = vector<float> (num_vertices * 3, 0);
		}
	} else {
		if (pastHeader) {
			// Interpret triangle-mesh data
			if (vertices_loaded < num_vertices) {
				// Position & normals
				const char * getX = op.c_str();
				x = atof(getX);
				ss >> y >> z;
				vertex_list.push_back(x);
				vertex_list.push_back(y);
				vertex_list.push_back(z);

				if (hasNormals) {
					// Normal
					ss >> nx >> ny >> nz;

					// Ensure the normals are normalized.
					Vector3f temp_norm(nx, ny, nz);
					temp_norm.normalize();
					normal_list.push_back(temp_norm[0]);
					normal_list.push_back(temp_norm[1]);
					normal_list.push_back(temp_norm[2]);
				}
				if (hasColors) {
					// Color
					ss >> r >> g >> b;
					color_list.push_back((float) r / 255);
					color_list.push_back((float) g / 255);
					color_list.push_back((float) b / 255);
				} else {
					color_list.push_back(1.0f);
					color_list.push_back(1.0f);
					color_list.push_back(1.0f);
				}

				vertices_loaded++;
			} else {
				// Triangle faces
				ss >> ix >> iy >> iz;

				triangle_list.push_back(ix);
				triangle_list.push_back(iy);
				triangle_list.push_back(iz);

				if (!hasNormals) {
					// Generate our own normals
					Vector3f pt1(vertex_list[ix * 3], vertex_list[ix * 3 + 1],
							vertex_list[ix * 3 + 2]);
					Vector3f pt2(vertex_list[iy * 3], vertex_list[iy * 3 + 1],
							vertex_list[iy * 3 + 2]);
					Vector3f pt3(vertex_list[iz * 3], vertex_list[iz * 3 + 1],
							vertex_list[iz * 3 + 2]);

					Vector3f temp_normal = calculateNormal(pt1, pt2, pt3);
					normal_list[ix * 3] = temp_normal[0];
					normal_list[ix * 3 + 1] = temp_normal[1];
					normal_list[ix * 3 + 2] = temp_normal[2];

					temp_normal = calculateNormal(pt2, pt3, pt1);
					normal_list[iy * 3] = temp_normal[0];
					normal_list[iy * 3 + 1] = temp_normal[1];
					normal_list[iy * 3 + 2] = temp_normal[2];

					temp_normal = calculateNormal(pt3, pt1, pt2);
					normal_list[iz * 3] = temp_normal[0];
					normal_list[iz * 3 + 1] = temp_normal[1];
					normal_list[iz * 3 + 2] = temp_normal[2];

				}
				triangles_loaded++;
			}

		} else {
			return true;
		}
	}

	if (ss.fail())
		return false;
	return true;
}

/*
 *  Convert the vertices of a model of a lego block to a point cloud
 *
 *  \return	cloud_out	The assembled point cloud *
 */
void Puppet::loadBlock(pcl::PointCloud<PointT> &cloud_out) {
	cloud_out.points.clear();
	//	ROS_INFO_STREAM("Loading block of size " << vertices_loaded);
	for (int i = 0; i < vertices_loaded; i++) {
		PointT pt;
		pt.x = vertex_list[3 * i];
		pt.y = vertex_list[3 * i + 1];
		pt.z = vertex_list[3 * i + 2];
		pt.normal[0] = normal_list[3 * i];
		pt.normal[1] = normal_list[3 * i + 1];
		pt.normal[2] = normal_list[3 * i + 2];
		pt.normal_x = normal_list[3 * i];
		pt.normal_y = normal_list[3 * i + 1];
		pt.normal_z = normal_list[3 * i + 2];
		char ur, ug, ub;
		ur = (char) 255 * 1.0f;
		ug = (char) 255 * 1.0f;
		ub = (char) 255 * 1.0f;
		pt.rgb = rgbd::packRGB(ur, ug, ub);
		pt.curvature = 0;
		pt.imgX = 0;
		pt.imgY = 0;
		cloud_out.points.push_back(pt);
	}
}


/*
 *  Reads a text file that lists the positions of several lego blocks that make up
 *  a puppet. Create a point cloud for the entire model
 *
 *  \param filename		The name of the text file
 *  \param filespath	The folder containing the text file
 */
void Puppet::loadLegoPuppet(std::string filename, std::string filespath) {

	/************ format ************

	 block1_file rot trans colors
	 block2_file rot trans colors

	 ********************************/
	using_legos = true;
	char str[200];
	int temp;
	Transform3f T;
	pcl::PointCloud<PointT> Bcloud;

	// Open the text file
	fstream f;
	f.open(filename.c_str(), ios::in);

	// Read each line and recover the block positions
	while (!f.eof()) {
		f >> str;
		if (strlen(str) < 10)
			continue;

		// Load a non-transformed point cloud for this block
		loadBlock(Bcloud);
		LegoBlock B;
		B.ID = block_name_counter++;
		f >> str;
		temp = atoi(str);
		// Read the rotation
		if (temp == 0)
			B.rot = false;
		else
			B.rot = true;
		// Read the translation
		for (int i = 0; i < 3; i++) {
			f >> str;
			B.trans[i] = (float) atof(str);
		}
		// Read the block color
		for (int i = 0; i < 3; i++) {
			f >> str;
			B.color[i] = float(atof(str));
		}
		// Setup a transformation matrix based on the rotation and translation
		T.setIdentity();
		if (B.rot) {
			T.matrix()(0, 0) = 0;
			T.matrix()(0, 1) = -1;
			T.matrix()(1, 0) = 1;
			T.matrix()(1, 1) = 0;
		}
		for (int k = 0; k < 3; k++) {
			T.matrix()(k, 3) = B.trans[k];
		}

		// Transform the point cloud and add it to the overall point cloud for
		// the whole puppet
		blocks.push_back(B);
		rgbd::transform_point_cloud_in_place(T, Bcloud, true);
		rgbd::mergeInPlace(Bcloud, this->cloud);
		cloud.header.frame_id = "/primesensor_frame";
	}
}

/*
 * 	Find the centroid of the puppet's vertices by averaging its coordinates
 */
void Puppet::findCentroid() {
	int numpts = cloud.points.size();
	centroid.push_back(0);
	centroid.push_back(0);
	centroid.push_back(0);
	PointT pt;
	for (int i = 0; i < numpts; i++) {
		pt = cloud.points[i];
		centroid[0] += pt.x / numpts;
		centroid[1] += pt.y / numpts;
		centroid[2] += pt.z / numpts;
	}
}

/*
 * Find the length of the diagonal spanning the corners of the puppet's bounding box
 */
void Puppet::findMaxDistance() {
	vector<float> vecx, vecy, vecz;
	int numpts = cloud.points.size();

	PointT pt;
	for (int i = 0; i < numpts; i++) {
		pt = cloud.points[i];
		vecx.push_back(pt.x);
		vecy.push_back(pt.y);
		vecz.push_back(pt.z);
	}
	sort(vecx.begin(), vecx.end(), orderfunc1);
	sort(vecy.begin(), vecy.end(), orderfunc1);
	sort(vecz.begin(), vecz.end(), orderfunc1);

	puppet_max_distance = sqrt(
			(vecx.front() - vecx.back()) * (vecx.front() - vecx.back())
					+ (vecy.front() - vecy.back()) * (vecy.front()
							- vecy.back()) + (vecz.front() - vecz.back())
					* (vecz.front() - vecz.back()));
}

/*
 * Check whether the puppet's current position is outside the designated tracking region.
 */
bool Puppet::checkVisibility() {
	// Retrieve translation vector for puppet
	Vector3f position;
	for (int i = 0; i < 3; i++) {
		position[i] = pose.matrix()(i, 3);
	}

//	ROS_INFO_STREAM("Coords: " << position[0] << " " << position[1] << " " << position[2]);
//	ROS_INFO_STREAM("X: " << fabs(atan(position[0] / position[2])) * 180.0 / PI  << " " <<  h_tracking_region/ 2 * 0.8);
//	ROS_INFO_STREAM("Y: " << fabs(atan(position[1] / position[2])) * 180.0 / PI  << " " <<  v_tracking_region/ 2 * 0.8);

	// Build in 20% wiggle room
	if ((fabs(atan(position[0] / position[2])) * 180.0 / PI > h_tracking_region
			/ 2 * 0.8) || (fabs(atan(position[1] / position[2])) * 180.0 / PI
			> v_tracking_region / 2 * 0.8) || (position[2] < near_tracking_region * 1.20)) {
		return false;
	} else {
		return true;
	}
}


/*
 * Use ICP to update the puppet's pose. This is the highest-level function and should always be used
 * for updating poses.
 *
 * \param 	curr_cloud		The point cloud from the kinect
 * \param	pass_config		Run coarse ICP, fine ICP, or both?
 * \param	reset_clouds	Reset the contents of the ICP cloud pairs?
 * \param 	outlier_pct		Maximium percentage of outlier correspondences to consider tracking lost.
 *
 */
void Puppet::updatePose(pcl::PointCloud<PointT> *curr_cloud,ICP_passes pass_config,
						bool reset_clouds,float outlier_pct) {

	Quaternion<float> 	old_rot;
	Vector3f			old_translation;
	if (do_filter)
	{
		// Store last pose for use with exponential filter
		old_rot 		= pose.rotation();
		old_translation = pose.translation();
	}

	if (lost && !use_SIFT) {
		// Case where tracking has not been established and we are not using the rth_SIFT
		// package to obtain an initial pose estimate.

//		gettimeofday(&tracker_time, NULL);
//		start 	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
		if (use_multi_core) {
			updatePoseInitRobust(*curr_cloud);		// This is the only fucntion that should be used
		} else {
			if (use_inverse_icp) {
				updatePoseInitInverse(curr_cloud); 	// WARNING: This function is no longer supported
			} else {
				updatePoseInit(*curr_cloud); 		// WARNING: This function is no longer supported
			}
		}
//		gettimeofday(&tracker_time, NULL);
//		finish	=	tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
//		cout << "Time taken to initialize pose: " << finish - start << " sec" << endl;
	} else {
//		gettimeofday(&tracker_time, NULL);
//		start 	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
		if (!use_SIFT || (use_SIFT && !lost)) {
			// Either tracking has already begun or we are using the pose estimates from
			// rth_SIFT to initialize tracking.

			// We split the ICP-pose-detection process into two steps.
			// The coarse pass roughly aligns the incoming point cloud and the puppet's
			// stored model using a translation-only transform.
			// The fine pass then refines the pose estimate with a full 6D transformation.

			if (use_inverse_icp) {
				int coarse_rounds, fine_rounds;
				if ((pass_config == COARSE_ICP_ONLY) || (pass_config == FINE_ICP_ONLY)) {
					// Debugging mode. Only run one ICP pass
					coarse_rounds = 1;
					fine_rounds = 1;
				} else {
					if (puppet_as_source) {
						coarse_rounds = 10;
						fine_rounds = 25;
					} else {
						// This should be used during normal operation
						coarse_rounds = 20;
						fine_rounds = 50;
					}
				}
				if ((pass_config == COARSE_ICP_ONLY) || (pass_config == TWO_STAGE_ICP)) {
//					gettimeofday(&tracker_time, NULL);
//					start 	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
					updatePoseTrackInverseCoarse(curr_cloud, reset_clouds, coarse_rounds, outlier_pct);
//					gettimeofday(&tracker_time, NULL);
//					coarse_done	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
				}
				if ((pass_config == FINE_ICP_ONLY) || (pass_config
						== TWO_STAGE_ICP)) {
					if (pass_config == TWO_STAGE_ICP)
						reset_clouds = false;
					updatePoseTrackInverseFine(curr_cloud, reset_clouds,
							fine_rounds, outlier_pct);
//					gettimeofday(&tracker_time, NULL);
//					fine_done	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
				}
			} else {
				updatePoseTrack(curr_cloud);	// WARNING: This function is no longer supported
			}
		}
//		cout << "Coarse time: " << coarse_done - start << " fine time: " << fine_done - coarse_done << " Total: " << fine_done - start << endl;
//		gettimeofday(&tracker_time, NULL);
//		finish	=	tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
//		cout << "Time to perform ICP: " << finish - start << " sec" << endl;
	}

	// Evaluate whether the puppet has been lost and perform filtering.
	if ((pass_config == FINE_ICP_ONLY) || (pass_config == TWO_STAGE_ICP)) {
		// Only check the icp error if a fine pass has been made
//		ROS_INFO_STREAM("Error: " << current_outlier_percentage);

		if (current_outlier_percentage > outlier_thresh) {
			// If we've lost tracking for the puppet for more than 20 frames in a row,
			// label it as lost and do not render it
			if (lost == false) {
				lost_counter++;
			}
			if (lost_counter > 10) {
				lost = true;
				visible = false;
				do_filter = false;
				if(use_bilateral_filter)
				{
					stored_translations.clear();
					stored_rotations.clear();
				}
			}
		} else {
			lost_counter -= 1;
			if (lost_counter < 0) {
				visible = true;
				lost_counter = 0;
				lost = false;
				do_filter = true;
			}
		}

		if (!lost)
		{
			if (do_filter)
			{
				if (use_bilateral_filter)
				{
					// Retrieve latest pose estimate
					Quaternion<float> new_rot(pose.rotation());
					Quaternion<float> filtered_rot = new_rot;
					Vector3f new_translation = pose.translation();
					double current_time;
					gettimeofday(&tracker_time, NULL);
					current_time = tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);

					// Used to normalize filtered results
					float time_total 	= time_constant;
					float rot_total 	= rotation_constant*time_constant;
					float trans_total	= translation_constant*time_constant;

					Vector3f output_trans = new_translation * translation_constant*time_constant;

					float time_weight, rot_weight, trans_weight;

					for (uint i=0;i<stored_translations.size();i++)
					{
						time_weight = (float)time_constant*exp(-pow((current_time - stored_times[i]),2)/(2*(double)time_sigma*(double)time_sigma));

						if (new_rot.dot(stored_rotations[i]) < 0)
						{
							stored_rotations[i].w() *= -1;
							stored_rotations[i].x() *= -1;
							stored_rotations[i].y() *= -1;
							stored_rotations[i].z() *= -1;
						}
						float rotation_difference = 2*180.0/PI*acos(abs(new_rot.dot(stored_rotations[i])));
						rot_weight = rotation_constant*exp(-pow(rotation_difference,2)/(2*rotation_sigma*rotation_sigma));
						if (filtered_rot.dot(stored_rotations[i]) < 0)
						{
							stored_rotations[i].w() *= -1;
							stored_rotations[i].x() *= -1;
							stored_rotations[i].y() *= -1;
							stored_rotations[i].z() *= -1;
						}

						float temp_total = rot_total+rot_weight*time_weight;
						if (temp_total == temp_total){
							filtered_rot = filtered_rot.slerp(rot_weight*time_weight/temp_total,stored_rotations[i]);
							rot_total = temp_total;
						}

						float distance = (new_translation - stored_translations[i]).norm();
						trans_weight = translation_constant*exp(-pow(distance,2)/(2*translation_sigma*translation_sigma));
						output_trans += stored_translations[i] * trans_weight * time_weight;
						trans_total += trans_weight*time_weight;
					}

					output_trans = output_trans/trans_total;
					Transform3f new_pose;
					new_pose.setIdentity();
					new_pose.translate(output_trans);
					new_pose.rotate(filtered_rot);
					pose = new_pose;

					// Store the pose in the vector of past poses
					if (stored_translations.size() == max_filter_entries)
					{
						stored_translations.erase(stored_translations.begin());
						stored_rotations.erase(stored_rotations.begin());
						stored_times.erase(stored_times.begin());
					}
					stored_translations.push_back(new_translation);
					stored_rotations.push_back(new_rot);
					stored_times.push_back(current_time);

				} else {
					Transform3f new_pose;
					new_pose.setIdentity();
					Quaternion<float> new_rot(pose.rotation());
					// Check whether the new quaternion needs to be flipped
					if (new_rot.dot(old_rot) < 0)
					{
//						ROS_INFO_STREAM(new_rot.w());
						new_rot.w() *= -1;
						new_rot.x() *= -1;
						new_rot.y() *= -1;
						new_rot.z() *= -1;
//						ROS_INFO_STREAM("Flipped: " << new_rot.w());
					}
					Quaternion<float> filtered_rot = old_rot.slerp(filter_alpha,new_rot);
					Vector3f new_translation		= pose.translation();
					Vector3f filtered_translation 	= filter_alpha*new_translation + (1-filter_alpha) * old_translation;
					new_pose.translate(filtered_translation);
					new_pose.rotate(filtered_rot);
					pose = new_pose;
				}
			}

			// Save pose for rendering by GLManager
			render_pose = this->pose;
			// Store latest valid pose
			vector<float> pose_vector;
			if (show_transient_pose)
				transient_pose_vector.clear();
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					pose_vector.push_back(render_pose.matrix()(j, i));
					if (show_transient_pose)
						transient_pose_vector.push_back(transient_pose.matrix()(j, i));
				}
			}
			poses.push_back(pose_vector);
			current_pose_index++;

			//		ROS_INFO_STREAM("Lost Counter: "<<lost_counter);
		}
		processed_frame++;
	}
}

/*
 *  Refresh the source cloud in the ICP point-cloud pair with a transformed version of
 *  the puppet's stored cloud.
 */
void Puppet::resetPuppetCloud() {
	Transform3f inv_pose = pose.inverse(Affine);
	rgbd::transform_point_cloud_in_place(inv_pose, this->cloud, true);
	pose.setIdentity();
}

/*
 *  Transform the puppet's stored cloud by its current pose.
 */
void Puppet::transformPuppetCloud() {
	rgbd::transform_point_cloud_in_place(pose, this->cloud, true);
}

/*
 *  Performs the first pose estimate for the puppet as it enters the performance
 *  space. This function is called when the rth_SIFT package is not being used to provide
 *  initial pose estiamte. It uses multiple cores to simultaneously test several sample alignments
 *  for the puppet, determines which sample produces the smallest error, and uses it
 *  as the initial pose estimate.
 *
 *  \param 	curr_cloud	The Kinect point cloud
 */
void Puppet::updatePoseInitRobust(pcl::PointCloud<PointT> curr_cloud) {
	//  Create samples poses for the  virtual puppet rotated about each axis, compare each pose to the incoming
	//  point cloud, and use the one with the smallest point-to-point error as the initial pose estimate.
	pose.setIdentity();

	float least_outliers_perc = 100, outliers_perc;
	float smallest_icp_err = 100;
	float temp_icp_err = smallest_icp_err;

	Transform3f outlier_pose;
	Transform3f icp_error_pose;

	int num_threads = 0, count = 0;

	// Determine how many threads will be needed
	for (int theta = -90; theta <= 90; theta += angle_spacing)
		for (int phi = -90; phi <= 90; phi += angle_spacing)
			for (int psi = -180; psi < 180; psi += angle_spacing) {
				float thetad = (1.0f * theta) * M_PI / 180;
				float phid = (1.0f * phi) * M_PI / 180;
				if (cos(phid) * cos(thetad) < -0.001)
					continue;
				num_threads++;
			}

	struct AlignData aligndata[num_threads];
	vector<double> theta_angles;
	vector<double> phi_angles;
	vector<double> psi_angles;
	vector<double> outlier_errors;
	vector<double> ICP_errors;
	vector<double> combined_errors;

	// Create the transforms for each sample pose
	count = 0;
	for (int theta = -90; theta <= 90; theta += angle_spacing)
		for (int phi = -90; phi <= 90; phi += angle_spacing)
			for (int psi = -180; psi < 180; psi += angle_spacing) {
				theta_angles.push_back(theta);
				phi_angles.push_back(phi);
				psi_angles.push_back(psi);
				float thetad = (1.0f * theta) * M_PI / 180;
				float phid = (1.0f * phi) * M_PI / 180;
				float psid = (1.0f * psi) * M_PI / 180;
				Transform3f xform;
				xform.setIdentity();
				xform(0, 0) = cos(thetad) * cos(psid);
				xform(0, 1) = -cos(phid) * sin(psid) + sin(phid) * sin(
						thetad) * cos(psid);
				xform(0, 2) = sin(phid) * sin(psid) + cos(phid) * sin(
						thetad) * cos(psid);
				xform(1, 0) = cos(thetad) * sin(psid);
				xform(1, 1) = cos(phid) * cos(psid) + sin(phid) * sin(
						thetad) * sin(psid);
				xform(1, 2) = -sin(phid) * cos(psid) + cos(phid) * sin(
						thetad) * sin(psid);
				xform(2, 0) = -sin(thetad);
				xform(2, 1) = sin(phid) * cos(thetad);
				xform(2, 2) = cos(phid) * cos(thetad);

				if (xform(2, 2) < -0.001)
					continue;

				rgbd::transform_point_cloud_in_place(xform, this->cloud,true);

				aligndata[count].curr_cloud = curr_cloud;
				aligndata[count].cloud = cloud;
				aligndata[count].xform = xform;
				aligndata[count].centroid = centroid;
				aligndata[count].puppet_max_distance = puppet_max_distance;
				aligndata[count].max_ICP_rounds = max_ICP_rounds;
				aligndata[count].useNormals = useNormals;
				aligndata[count].max_normal_difference_deg = max_normal_difference_deg;
				rgbd::transform_point_cloud_in_place(xform.inverse(Affine),
						this->cloud, true);
				count++;
			}

	// create threads
	pthread_attr_t attr;
	int rc;
	void* status;

	// Run ICP to find the point-to-point error between each sample point and the incoming point cloud.
	int count_thread = 0;
	while (count_thread < num_threads) {
		pthread_t threads[simultaneous_threads];
		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		for (int i = count_thread; i < min(count_thread + simultaneous_threads,	num_threads); i++)
		{
//			ROS_INFO_STREAM("Starting thread " << i << " local value: " << i-count_thread);
			pthread_create(&threads[i - count_thread], &attr, thread_align_cloud_init, (void *) &aligndata[i]);
		}

		pthread_attr_destroy(&attr);
		// wait for the threads to execute
		for (int i = count_thread; i < min(count_thread + simultaneous_threads,	num_threads); i++)
		{
//			ROS_INFO_STREAM("Joining thread " << i << " PID Value: " << threads[i-count_thread]);
			rc = pthread_join(threads[i - count_thread], &status);
		}
		count_thread += simultaneous_threads;
	}

	// Evaluate results
	// Get error metrics from the poses and determine the best-matched pose
	for (int i = 0; i < num_threads; i++) {

		outliers_perc = aligndata[i].perc_outliers;
		outlier_errors.push_back(outliers_perc);
		if (outliers_perc < least_outliers_perc) {
			least_outliers_perc = outliers_perc;
			outlier_pose = aligndata[i].temp_pose * aligndata[i].xform;
			icp_error = aligndata[i].icp_error_local;
		}
	}
//	ROS_INFO_STREAM("Lowest outlier percentage: " << least_outliers_perc << "%");
	current_outlier_percentage = least_outliers_perc/(float)100.0;
	pose = outlier_pose;
	render_pose = pose;

//	// Sort the values (for debugging)
//	vector<double> outlier_errors_sorted = outlier_errors;
//	std::sort(outlier_errors_sorted.begin(), outlier_errors_sorted.end());
//	vector<double> ICP_errors_sorted = ICP_errors;
//	std::sort(ICP_errors_sorted.begin(), ICP_errors_sorted.end());
//
//	// Find difference in errors between top two matches
//	float outlier_diff = (outlier_errors_sorted[1] - outlier_errors_sorted[0])
//			/ outlier_errors_sorted[0];
//	float icp_error_diff = (ICP_errors_sorted[1] - ICP_errors_sorted[0])
//			/ ICP_errors_sorted[0];
//
//	if (outlier_diff >= icp_error_diff) {
//		pose = outlier_pose;
//	} else {
//		pose = icp_error_pose;
//	}
//
//	// Print errors in order
//	vector<double> errors_sorted = errors;
//	std::sort(errors_sorted.begin(), errors_sorted.end());
//	int it;
//	for(int i=0;i<num_threads;i++){
//		it = find(errors.begin(),errors.end(),errors_sorted[i]) - errors.begin();
//		if (use_outlier_error){
//			ROS_INFO_STREAM("Theta: " << *(theta_angles.begin()+it) << ", \tphi: " << *(phi_angles.begin()+it) << ", \tpsi: " << *(psi_angles.begin()+it) << ". \tOutlier %: " << errors_sorted[i]);
//		} else {
//			ROS_INFO_STREAM("Theta: " << *(theta_angles.begin()+it) << ", \tphi: " << *(phi_angles.begin()+it) << ", \tpsi: " << *(psi_angles.begin()+it) << ". \tError: " << errors_sorted[i]);
//		}
//	}
//	ROS_INFO_STREAM(" ");

	if (!use_inverse_icp) {
		rgbd::transform_point_cloud_in_place(pose, this->cloud, true);
	}
}

/*
 *  Initialize one of the threads used by updatePoseInitRobust to sample the error between
 *  multiple puppet poses and the incoming kinect point cloud
 */
void* thread_align_cloud_init(void* threaddata) {
	struct AlignData *aligndata;
	aligndata = (struct AlignData *) threaddata;

	alignCloudsWithInit(aligndata->curr_cloud, aligndata->cloud,
			aligndata->err_vector, aligndata->num_outliers,
			aligndata->perc_outliers, aligndata->icp_error_local,
			aligndata->temp_pose, aligndata->centroid,
			aligndata->puppet_max_distance, aligndata->max_ICP_rounds,
			aligndata->useNormals, aligndata->max_normal_difference_deg);
	return (void*) 42;
}

/*
 *  The function executed by updatePoseInitRobust on each thread. Given an incoming point cloud,
 *  the puppet's stored clouds and and a guess pose for the puppet, runs ICP and reports the
 *  resulting point-to-point error between the clouds
 *
 *  \param 	curr_cloud			The kinect point cloud
 *  \param 	full_cloud			The puppet's point cloud
 *  \return	err_vector			History of point-to-point error for each ICP iteration (not used)
 *  \return num_outliers		The number of correspondences in the cloud pair labeled as outliers
 *  \return	temp_pose			The optimal transform between the point clouds
 *  \param	centroid			The centroid of the puppet point cloud
 *  \param  puppet_max_distance	The longest diagonal of the pupppet's bounding box
 *  \param  max_ICP_rounds		Maximum number of ICP iterations
 *  \param	useNormals			Whether to use normal-normal differences when establishing point-point correspondences
 *  \param	max_normal_difference_deg	Points with normals that differ by more than this amount are not paired
 *
 */
float alignCloudsWithInit(pcl::PointCloud<PointT> curr_cloud,
		pcl::PointCloud<PointT> fullcloud, vector<float> &err_vector,
		int &num_outliers, float &perc_outliers, float &icp_err,
		Transform3f &temp_pose, vector<float> centroid,
		float puppet_max_distance, int max_ICP_rounds, bool useNormals,
		float max_normal_difference_deg) {

	// curr_cloud: 	point cloud from the kinect
	// fullcloud: 	stored pointcloud from puppet

	// Start pose with no translation or rotation
	temp_pose.setIdentity();

	// Compute centroids of the point clouds
	float x1, y1, z1, x2, y2, z2;
	PointT pt;
	int numpts;

	x1 = 0, y1 = 0, z1 = 0, x2 = 0, y2 = 0, z2 = 0;

	vector<float> vecx1, vecy1, vecz1, vecx2, vecy2, vecz2;
	numpts = curr_cloud.points.size();
	for (int i = 0; i < numpts; i++) {
		pt = curr_cloud.points[i];
		vecx1.push_back(pt.x);
		vecy1.push_back(pt.y);
		vecz1.push_back(pt.z);
	}
	sort(vecx1.begin(), vecx1.end(), orderfunc1);
	sort(vecy1.begin(), vecy1.end(), orderfunc1);
	sort(vecz1.begin(), vecz1.end(), orderfunc1);

	x1 = vecx1[(int) (vecx1.size() / 2)];
	y1 = vecy1[(int) (vecy1.size() / 2)];
	z1 = vecz1[(int) (vecz1.size() / 2)];
	x2 = centroid[0];
	y2 = centroid[1];
	z2 = centroid[2];

	// Initpose stores the translation between the two two clouds' centroids
	Transform3f initpose;
	initpose.setIdentity();
	initpose.matrix()(0, 3) = x1 - x2;
	initpose.matrix()(1, 3) = y1 - y2;
	// Extra translation in z added so that culling is not required, saves extra work and coding complexity
	initpose.matrix()(2, 3) = z1 - z2 + 0.025;

	// Translate the puppet pointcloud's centroid to the kinect pointcloud's centroid
	temp_pose = initpose;
	rgbd::transform_point_cloud_in_place(temp_pose, fullcloud, true);

	// Now refine the translation and rotation of the puppet
	Transform3f T, rotZ;

	// Two ICP runs will be executed.
	// First one produces a rough match, ignoring correspondences with large distances.
	// Second refines the match.
	registration::ICPCombined icp;
	registration::ICPCloudPairParams cloud_pair_params;
	cloud_pair_params.outlier_percentage = 0.25; 	// First ICP pass ignores correspondence with large distances (originally 0.25)
	cloud_pair_params.max_distance = puppet_max_distance / 2; // (originally 0.1)
	cloud_pair_params.errType = registration::ICP_ERR_POINT_TO_POINT;
	cloud_pair_params.use_average_point_error = true;

	registration::ICPThreadParams cloud_thread_params;
	cloud_thread_params.search_threads = 1;

	if (useNormals) {
		cloud_pair_params.max_normal_angle = max_normal_difference_deg * PI
				/ 180.0;
		cloud_pair_params.front_can_match_back = false;
	}
	boost::shared_ptr < registration::ICPCloudPair > cloud_pair_multi_core;
	cloud_pair_multi_core = boost::make_shared<registration::ICPCloudPair>(
			cloud_pair_params, curr_cloud, fullcloud);
	cloud_pair_multi_core->setThreadParams(cloud_thread_params);

	icp.addCloudPair(cloud_pair_multi_core);
	registration::ICPCombinedParams icp_params;
	icp_params.max_icp_rounds = max_ICP_rounds;
	icp_params.min_error_frac_to_continue = 0.001;
	icp_params.optimizer = registration::OPTIMIZER_CLOSED_FORM;
	icp.setParams(icp_params);
	T.setIdentity();
	icp.setInitialTransform(T);

	// Run first, coarse ICP pass
	float err;
	T.setIdentity();
	err = icp.runICP(T, false);
	T = T.inverse(Affine);
	temp_pose = T * temp_pose;
	// Adjust the pose of the puppet pointcloud based on the first ICP results
	rgbd::transform_point_cloud_in_place(T, fullcloud, true);

	// Now run a second ICP pass to refine the pose
	cloud_pair_params.outlier_percentage = 0.1; // No longer need to discard outlier correspondences (used to be 0.0)
	cloud_pair_params.max_distance = 0.01; // Original value: 0.01
	cloud_pair_multi_core = boost::make_shared<registration::ICPCloudPair>(
			cloud_pair_params, curr_cloud, fullcloud);
	cloud_pair_multi_core->setThreadParams(cloud_thread_params);

	registration::ICPCombined icp1;
	icp1.addCloudPair(cloud_pair_multi_core);
	icp1.setParams(icp_params);
	T.setIdentity();
	icp1.setInitialTransform(T);
	T.setIdentity();
	err = icp1.runICP(T, false);
	T = T.inverse(Affine);
	temp_pose = T * temp_pose;
	rgbd::transform_point_cloud_in_place(T, fullcloud, true);

	// Generate error metrics
	vector < vector<vector<int> > > corres = icp1.getCorrespondencesHistory();
	numpts = curr_cloud.points.size();

	// Use percentage of point-to-point correspondences that are outliers as error metric
	num_outliers = 0;
	for (int i = 0; i < numpts; i++) {
		pt = curr_cloud.points[i];
		if (((corres.back().back()))[i] == -1) {
			num_outliers++;
		}
	}
	perc_outliers = (float) num_outliers / (float) numpts * 100;
	icp_err = err;
	return err;
}


/*
 *  Coarsely align the incoming kinect point cloud and the stored puppet cloud,
 *  based on translation alone.
 *
 *  \param 	curr_cloud		The kinect point cloud
 *  \param	reset_clodus	Should the source and/or target clouds in the ICP pair be reset?
 *  \param	max_rounds		Maximum number of ICP iterations
 *  \param	outlier_pct		Define what percentage of correspondences should be considered outliers
 *
 */
void Puppet::updatePoseTrackInverseCoarse(pcl::PointCloud<PointT> *curr_cloud,
		bool reset_clouds, int max_rounds, float outlier_pct) {

	// Get transform from cloud to stored puppet
	Transform3f inv_pose = pose.inverse(Affine);
	Transform3f original_pose = pose;

	if (record_iterations)
	{
		all_iterations.push_back(pose);
	}

	// Set up ICP
	cloud_pair_params.max_distance = puppet_max_distance / 4;
	cloud_pair_params.outlier_percentage = outlier_pct;
	cloud_pair_params.use_average_point_error = true;
	cloud_pair_params.errType = registration::ICP_ERR_POINT_TO_POINT;
	if (useNormals) {
		cloud_pair_params.max_normal_angle = max_normal_difference_deg * PI
				/ 180.0;
		cloud_pair_params.front_can_match_back = false;
	}

	if (reset_clouds) {
		if (!cloud_pair_created) {
			// Both clouds needs to be added to the pair structure
			if (puppet_as_source) {
				cloud_pair = boost::make_shared<registration::ICPCloudPair>(
						cloud_pair_params, this->cloud, *curr_cloud);
			} else {
				cloud_pair = boost::make_shared<registration::ICPCloudPair>(
						cloud_pair_params, *curr_cloud, this->cloud);
			}
			cloud_pair_created = true;
			if (use_multi_core)
				cloud_pair->setThreadParams(cloud_thread_params);
		} else {
			// Only replace one of the clouds
			cloud_pair->setParams(cloud_pair_params);
			if (puppet_as_source) {
				cloud_pair->replaceTargetCloud(*curr_cloud);
			} else {
				cloud_pair->replaceSourceCloud(*curr_cloud);
			}
			cloud_pair->setParams(cloud_pair_params);
		}
	}
	cloud_pair->setParams(cloud_pair_params);
	cloud_pair->setScalarWeights();

	registration::ICPCombined icp;
	icp.addCloudPair(cloud_pair);
	registration::ICPCombinedParams icp_params;
	icp_params.max_icp_rounds = max_rounds;
	// If the error reduction between two ICP iterations drops below the following metric, we stop the process:
	icp_params.min_error_frac_to_continue = 0.01;
	icp_params.optimizer = registration::OPTIMIZER_CLOSED_FORM;
	icp.setParams(icp_params);
	if (puppet_as_source) {
		icp.setInitialTransform(pose);
	} else {
		icp.setInitialTransform(inv_pose);
	}

	// Run ICP, initializing it with the puppet's last known pose
	Transform3f T_kinect_cloud;
	T_kinect_cloud = inv_pose;
	icp_error = icp.runICP(T_kinect_cloud, false);

//	// Output the total number of iterations for debugging
//	vector<vector<vector<int> > > corres;
//	corres = icp.getCorrespondencesHistory();
//	cout << "Number of coarse-level ICP iterations: " << corres.size() << endl;

//	vector<vector<vector<int> > > corres;
//	corres = icp1.getCorrespondencesHistory();
//	current_outlier_percentage = findOutliersPercentOnly(corres.back().back(),curr_cloud->points.size());

	// Update the pose
	if (puppet_as_source) {
		inv_pose = T_kinect_cloud.inverse(Affine);
	} else {
		inv_pose = T_kinect_cloud;
	}

	transient_pose = inv_pose.inverse(Affine);
	pose = inv_pose.inverse(Affine);

	// Only use translation from new pose
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			transient_pose.matrix()(j, i) = original_pose.matrix()(j, i);
			pose.matrix()(j, i) = original_pose.matrix()(j, i);
		}
	}

	// For visualization each ICP iteration:
	if (record_iterations)
	{
		std::deque<rgbd::eigen::Affine3f,rgbd::eigen::aligned_allocator<rgbd::eigen::Affine3f> > transform_history = icp.getTransformHistory();
		for (int d=0;d<transform_history.size();d++)
			all_iterations.push_back(transform_history[d].inverse(Affine));
	}
}


/*
 *  Determine the full 6D transform between the incoming kinect point cloud and the stored
 *  puppet cloud. Should follow updatePoseTrackCoarseFine.
 *
 *  \param 	curr_cloud		The kinect point cloud
 *  \param	reset_clodus	Should the source and/or target clouds in the ICP pair be reset?
 *  \param	max_rounds		Maximum number of ICP iterations
 *  \param	outlier_pct		Define what percentage of correspondences should be considered outliers
 *
 */
void Puppet::updatePoseTrackInverseFine(pcl::PointCloud<PointT> *curr_cloud,
		bool reset_clouds, int max_rounds, float outlier_pct) {
	// Get transform from cloud to stored puppet
	Transform3f inv_pose = pose.inverse(Affine);

	cloud_pair_params.use_average_point_error = true;
	cloud_pair_params.errType = registration::ICP_ERR_POINT_TO_POINT;

	cloud_pair_params.outlier_percentage = outlier_pct; //0.1
	cloud_pair_params.max_distance =  0.01; // 0.003
	if (useNormals) {
		cloud_pair_params.max_normal_angle = max_normal_difference_deg * PI
				/ 180.0;
		cloud_pair_params.front_can_match_back = false;
	}

	if (reset_clouds) {
		if (!cloud_pair_created) {
			if (puppet_as_source) {
				cloud_pair = boost::make_shared<registration::ICPCloudPair>(
						cloud_pair_params, this->cloud, *curr_cloud);
			} else {
				cloud_pair = boost::make_shared<registration::ICPCloudPair>(
						cloud_pair_params, *curr_cloud, this->cloud);
			}
			cloud_pair_created = true;
			if (use_multi_core)
				cloud_pair->setThreadParams(cloud_thread_params);
		} else {
			cloud_pair->setParams(cloud_pair_params);
			if (puppet_as_source) {
				cloud_pair->replaceTargetCloud(*curr_cloud);
			} else {
				cloud_pair->replaceSourceCloud(*curr_cloud);
			}
			cloud_pair->setParams(cloud_pair_params);
		}
	} else {
		cloud_pair->setParams(cloud_pair_params);
	}
	cloud_pair->setScalarWeights();

	// setup fine ICP
	registration::ICPCombined icp1;
	registration::ICPCombinedParams icp_params;
	icp1.addCloudPair(cloud_pair);
	icp_params.max_icp_rounds = max_rounds;
	if (puppet_as_source) {
		icp_params.min_error_frac_to_continue = 0.01;
	} else {
		icp_params.min_error_frac_to_continue = 0.001; // This value is finer than in the Coarse function
	}
	icp_params.optimizer = registration::OPTIMIZER_CLOSED_FORM;
	icp1.setParams(icp_params);
	if (puppet_as_source) {
		icp1.setInitialTransform(inv_pose.inverse(Affine));
	} else {
		icp1.setInitialTransform(inv_pose);
	}
	// Run ICP, initializing it with the puppet's last known pose
	Transform3f T_kinect_cloud;
	T_kinect_cloud = inv_pose;
	icp_error = icp1.runICP(T_kinect_cloud, false);

	if (puppet_as_source) {
		inv_pose = T_kinect_cloud.inverse(Affine);
	} else {
		inv_pose = T_kinect_cloud;
	}

	pose = inv_pose.inverse(Affine);
//	render_pose = this->pose; // Commented out because the percentage of outliers should be checked before rendering.

//	// Output the total number of iterations
//	vector<vector<vector<int> > > corres;
//	corres = icp1.getCorrespondencesHistory();
//	cout << "Number of fine-level ICP iterations: " << corres.size() << endl;

	// Update outlier percentage, so we can later decide whether we have lost tracking
	current_outlier_percentage = findOutliersPercentOnly(cloud_pair_params.max_distance);

	if (publish_marked_clouds) {
		vector < vector<vector<int> > > corres;
		corres = icp1.getCorrespondencesHistory();
		current_correspondences = corres.back().back();
		rgbd::transform_point_cloud_in_place(inv_pose, *curr_cloud, true);
		markOutliers(corres, curr_cloud);
	}

	// For visualization each ICP iteration:
	if (record_iterations)
	{
		std::deque<rgbd::eigen::Affine3f,rgbd::eigen::aligned_allocator<rgbd::eigen::Affine3f> > transform_history = icp1.getTransformHistory();
		for (uint d=0;d<transform_history.size();d++)
			all_iterations.push_back(transform_history[d].inverse(Affine));

		record_iterations = false;
	}
}

/*
 *  Given the latest point-to-point correspondences between two clouds,
 *  determine what percentages are considered outliers
 *
 *  \param 	max_distance	The minimum distance between points to consider them outliers
 */

float Puppet::findOutliersPercentOnly(float max_distance) {
	vector<float> errs;
	vector<int> corres;
	getCorrsAndErrors(this->pose, corres, errs);
	float numpts = (float)corres.size();
	int outliers = 0;
	for (int i = 0; i < numpts; i++) {
		if (corres[i] == -1 || errs[i] > max_distance)
			outliers++;
	}
	return (float) outliers / (float) numpts;
}

/*
 *  Given the latest point-to-point correspondences between two clouds,
 *  create a new point cloud based on the kinect cloud, where each point's color
 *  indicates whether it is an outlier. Outliers are labeled in white.
 *
 *  \param 	corres		The list of correspondences
 *  \param	curr_cloud	The kinect point cloud
 */
float Puppet::markOutliers(vector<vector<vector<int> > > corres, pcl::PointCloud<PointT> *curr_cloud) {
	markedCameraCloud.points.clear();
	markedCameraCloud.height = curr_cloud->height;
	markedCameraCloud.width = curr_cloud->width;
	markedCameraCloud.is_dense = curr_cloud->is_dense;
	markedCameraCloud.header = curr_cloud->header;

	int numpts = curr_cloud->points.size();

	float white = (255 << 16) | (255 << 8) | (255);

	outliers.clear();
	PointT pt;
	Transform3f temp_pose;
	temp_pose = pose.inverse();

	// Iterate through each point in the kinect point cloud.
	for (int i = 0; i < numpts; i++) {
		pt = curr_cloud->points[i];
		xformPtPos(pt, temp_pose, pt);
		markedCameraCloud.push_back(curr_cloud->points[i]);
		if (((corres.back().back()))[i] == -1) {
			// This point was labeled as an outlier. Color it white.
			outliers.push_back(pt.x);
			outliers.push_back(pt.y);
			outliers.push_back(pt.z);
			(markedCameraCloud.points.back()).rgb = white;
		}
	}
	return (100.0f * outliers.size() / 3) / numpts;
}

/*
 *  Replace the source, target, or both point clouds in this puppet's ICP pair.
 *
 *  \param curr_cloud	The latest kinect point cloud
 */
void Puppet::updateCloudPair(pcl::PointCloud<PointT> *curr_cloud) {
	cloud_pair_params.max_distance = -1; // 0.01
	cloud_pair_params.outlier_percentage = -1;
	if (useNormals) {
		// This fn is typically used for filtering, so we only care about distances
		// and disable comparisons of normals
		cloud_pair_params.max_normal_angle = -1;
		cloud_pair_params.front_can_match_back = true;
	}

	if (!cloud_pair_created) {
		// Add both the kinect cloud and the puppet's stored cloud
		if (puppet_as_source) {
			cloud_pair = boost::make_shared<registration::ICPCloudPair>(
					cloud_pair_params, this->cloud, *curr_cloud);
		} else {
			cloud_pair = boost::make_shared<registration::ICPCloudPair>(
					cloud_pair_params, *curr_cloud, this->cloud);
		}
		cloud_pair->setScalarWeights();
		if (use_multi_core)
			cloud_pair->setThreadParams(cloud_thread_params);
	} else {
		// Just update the kinect cloud
		cloud_pair->setParams(cloud_pair_params);
		if (puppet_as_source) {
			cloud_pair->replaceTargetCloud(*curr_cloud);
		} else {
			cloud_pair->replaceSourceCloud(*curr_cloud);
		}
		cloud_pair->setParams(cloud_pair_params);
		cloud_pair->setScalarWeights();
	}
}

/*
 *  Outdated and no longer supported. See updatePoseInitRobust for the updated version
 */
void Puppet::updatePoseInit(pcl::PointCloud<PointT> curr_cloud) {

	Transform3f inv_pose = pose.inverse(Affine);
	rgbd::transform_point_cloud_in_place(inv_pose, this->cloud, true);
	pose.setIdentity();

	// compute centroids of the point clouds
	float x1, y1, z1, x2, y2, z2;
	PointT pt;
	int numpts;

	x1 = 0, y1 = 0, z1 = 0, x2 = 0, y2 = 0, z2 = 0;

	// Culled point cloud (from kinect)
	vector<float> vecx1, vecy1, vecz1, vecx2, vecy2, vecz2;
	numpts = curr_cloud.points.size();
	for (int i = 0; i < numpts; i++) {
		pt = curr_cloud.points[i];
		vecx1.push_back(pt.x);
		vecy1.push_back(pt.y);
		vecz1.push_back(pt.z);
	}
	sort(vecx1.begin(), vecx1.end(), orderfunc1);
	sort(vecy1.begin(), vecy1.end(), orderfunc1);
	sort(vecz1.begin(), vecz1.end(), orderfunc1);

	// Input point cloud (from stored puppet)
	numpts = cloud.points.size();
	for (int i = 0; i < numpts; i++) {
		pt = cloud.points[i];
		vecx2.push_back(pt.x);
		vecy2.push_back(pt.y);
		vecz2.push_back(pt.z);
	}
	sort(vecx2.begin(), vecx2.end(), orderfunc1);
	sort(vecy2.begin(), vecy2.end(), orderfunc1);
	sort(vecz2.begin(), vecz2.end(), orderfunc1);

	x1 = vecx1[(int) (vecx1.size() / 2)];
	y1 = vecy1[(int) (vecy1.size() / 2)];
	z1 = vecz1[(int) (vecz1.size() / 2)];
	x2 = vecx2[(int) (vecx2.size() / 2)];
	y2 = vecy2[(int) (vecy2.size() / 2)];
	z2 = vecz2[(int) (vecz2.size() / 2)];

	Transform3f initpose;
	initpose.setIdentity();
	initpose.matrix()(0, 3) = x1 - x2;
	initpose.matrix()(1, 3) = y1 - y2;
	//	initpose.matrix()(2,3) = z1-1.25*z2;
	// By translating in z by this amount, we can remove the use of the culled cloud
	initpose.matrix()(2, 3) = z1 - z2 + 0.01; // Replace 0.01 with half of the depth of an arbitrary puppet

	icp_error = 0;

	this->pose = initpose;
	rgbd::transform_point_cloud_in_place(this->pose, this->cloud, true);

	Transform3f T, rotZ;

	// the following two line parameter settings work for tracking when there is no update
	cloud_pair_params.outlier_percentage = 0.1;
	cloud_pair_params.max_distance = 0.01;
	cloud_pair_params.errType = registration::ICP_ERR_POINT_TO_POINT;
	cloud_pair_params.use_average_point_error = true;
	if (useNormals) {
		cloud_pair_params.max_normal_angle = max_normal_difference_deg * PI
				/ 180.0;
		cloud_pair_params.front_can_match_back = false;
	}
	cloud_pair = boost::make_shared<registration::ICPCloudPair>(cloud_pair_params, curr_cloud, this->cloud);
	if (use_multi_core)
		cloud_pair->setThreadParams(cloud_thread_params);

	registration::ICPCombined icp;
	icp.addCloudPair(cloud_pair);
	registration::ICPCombinedParams icp_params;
	icp_params.max_icp_rounds = 50;
	icp_params.min_error_frac_to_continue = 0.001;
	icp_params.optimizer = registration::OPTIMIZER_CLOSED_FORM;
	icp.setParams(icp_params);
	lastT.setIdentity();
	icp.setInitialTransform(lastT);

	// Run ICP
	T.setIdentity();
	icp_error = icp.runICP(T, false);
	T = T.inverse(Affine);

	rgbd::transform_point_cloud_in_place(T, this->cloud, true);

	this->pose = T * this->pose;

	/************/
	if (publish_marked_clouds) {
		vector < vector<vector<int> > > corres
				= icp.getCorrespondencesHistory();
		markOutliers(corres, &curr_cloud);
	}
	/************/
	render_pose = pose;
}

/*
 *  Outdated and no longer supported. See updatePoseInitRobust for the updated version
 */
void Puppet::updatePoseInitInverse(pcl::PointCloud<PointT> *curr_cloud) {
	// compute centroids of the point clouds
	float x1, y1, z1, x2, y2, z2;
	PointT pt;
	int numpts;

	x1 = 0, y1 = 0, z1 = 0, x2 = 0, y2 = 0, z2 = 0;

	// Begin by translating the source and target point clouds so their
	// centroids are aligned
	vector<float> vecx1, vecy1, vecz1, vecx2, vecy2, vecz2;
	numpts = curr_cloud->points.size();
	for (int i = 0; i < numpts; i++) {
		pt = curr_cloud->points[i];
		vecx1.push_back(pt.x);
		vecy1.push_back(pt.y);
		vecz1.push_back(pt.z);
	}
	sort(vecx1.begin(), vecx1.end(), orderfunc1);
	sort(vecy1.begin(), vecy1.end(), orderfunc1);
	sort(vecz1.begin(), vecz1.end(), orderfunc1);

	x1 = vecx1[(int) (vecx1.size() / 2)];
	y1 = vecy1[(int) (vecy1.size() / 2)];
	z1 = vecz1[(int) (vecz1.size() / 2)];
	x2 = centroid[0];
	y2 = centroid[1];
	z2 = centroid[2];

	Transform3f initpose_kinect_cloud;
	initpose_kinect_cloud.setIdentity();
	initpose_kinect_cloud.matrix()(0, 3) = x2 - x1;
	initpose_kinect_cloud.matrix()(1, 3) = y2 - y1;
	// By translating in z by this amount, we can remove the use of the culled cloud
	initpose_kinect_cloud.matrix()(2, 3) = z2 - z1 - 0.01; // Replace 0.01 with half of the depth of an arbitrary puppet
	icp_error = 0;

	rgbd::transform_point_cloud_in_place(initpose_kinect_cloud, *curr_cloud,
			true);

	Transform3f T_puppet, T_kinect_cloud, rotZ;

	//registration::ICPCloudPairParams cloud_pair_params;
	// the following two line parameter settings work for tracking when there is no update
	cloud_pair_params.outlier_percentage = 0.10;
	cloud_pair_params.max_distance = puppet_max_distance / 2;
	cloud_pair_params.errType = registration::ICP_ERR_POINT_TO_PLANE;
	cloud_pair_params.use_average_point_error = true;
	if (useNormals) {
		cloud_pair_params.max_normal_angle = max_normal_difference_deg * PI
				/ 180.0;
		cloud_pair_params.front_can_match_back = false;
	}
	if (!cloud_pair_created) {
		if (puppet_as_source) {
			cloud_pair = boost::make_shared<registration::ICPCloudPair>(
					cloud_pair_params, this->cloud, *curr_cloud);
		} else {
			cloud_pair = boost::make_shared<registration::ICPCloudPair>(
					cloud_pair_params, *curr_cloud, this->cloud);
		}
		cloud_pair_created = true;
		if (use_multi_core)
			cloud_pair->setThreadParams(cloud_thread_params);
	} else {
		if (puppet_as_source) {
			cloud_pair->replaceTargetCloud(*curr_cloud);
		} else {
			cloud_pair->replaceSourceCloud(*curr_cloud);
		}
		cloud_pair->setScalarWeights();
		cloud_pair->setParams(cloud_pair_params);
	}

	registration::ICPCombined icp;
	icp.addCloudPair(cloud_pair);
	registration::ICPCombinedParams icp_params;
	icp_params.max_icp_rounds = 50;
	icp_params.min_error_frac_to_continue = 0.001;
	icp_params.optimizer = registration::OPTIMIZER_CLOSED_FORM;
	icp.setParams(icp_params);
	lastT.setIdentity();
	icp.setInitialTransform(lastT);

	// Run ICP
	T_kinect_cloud.setIdentity();
	icp_error = icp.runICP(T_kinect_cloud, false);

	if (puppet_as_source) {
		initpose_kinect_cloud = T_kinect_cloud.inverse(Affine)
				* initpose_kinect_cloud;
	} else {
		initpose_kinect_cloud = T_kinect_cloud * initpose_kinect_cloud;
	}
	this->pose = initpose_kinect_cloud.inverse(Affine);

	/************/
	if (publish_marked_clouds) {
		vector < vector<vector<int> > > corres
				= icp.getCorrespondencesHistory();
		markOutliers(corres, curr_cloud);
	}
	/************/
	render_pose = pose;
	current_outlier_percentage = findOutliersPercentOnly(cloud_pair_params.max_distance);
}

/*
 *  Outdated and no longer supported. See updatePoseTrackInverseCoarse and updatePoseTrackInverseFine for the updated versions
 */
void Puppet::updatePoseTrack(pcl::PointCloud<PointT> *curr_cloud) {

	// Set up ICP
	cloud_pair_params.max_distance = puppet_max_distance / 2;
	cloud_pair_params.outlier_percentage = 0.1;
	cloud_pair_params.use_average_point_error = true;
	if (useNormals) {
		cloud_pair_params.max_normal_angle = max_normal_difference_deg * PI
				/ 180.0;
		cloud_pair_params.front_can_match_back = false;
	}

	clock_t start, finish;
	start = clock();
	//	cloud_pair = boost::make_shared<registration::ICPCloudPair>(cloud_pair_params, *curr_cloud, this->cloud);
	cloud_pair = boost::make_shared<registration::ICPCloudPair>(
			cloud_pair_params, this->cloud, *curr_cloud);
	if (use_multi_core)
		cloud_pair->setThreadParams(cloud_thread_params);
	finish = clock();

	registration::ICPCombined icp;
	icp.addCloudPair(cloud_pair);
	registration::ICPCombinedParams icp_params;
	icp_params.max_icp_rounds = 25;
	icp_params.min_error_frac_to_continue = 0.001;
	icp_params.optimizer = registration::OPTIMIZER_CLOSED_FORM;
	icp.setParams(icp_params);
	lastT.setIdentity();
	icp.setInitialTransform(lastT);

	// Run ICP
	Transform3f T;
	T.setIdentity();
	icp_error = icp.runICP(T, false);
	if (!puppet_as_source)
		T = T.inverse(Affine);

	// Update the pose
	lastT = T;
	this->pose = T * this->pose;
	rgbd::transform_point_cloud_in_place(T, this->cloud, true);

	transient_pose = this->pose;

	cloud_pair_params.outlier_percentage = 0.0;
	cloud_pair_params.max_distance = 0.01;
	if (puppet_as_source) {
		cloud_pair = boost::make_shared<registration::ICPCloudPair>(
				cloud_pair_params, this->cloud, *curr_cloud);
	} else {
		cloud_pair = boost::make_shared<registration::ICPCloudPair>(
				cloud_pair_params, *curr_cloud, this->cloud);
	}
	if (use_multi_core)
		cloud_pair->setThreadParams(cloud_thread_params);

	registration::ICPCombined icp1;
	icp1.addCloudPair(cloud_pair);
	icp1.setParams(icp_params);
	lastT.setIdentity();
	icp1.setInitialTransform(lastT);

	// Run ICP
	icp_error = icp1.runICP(T, false);
	if (!puppet_as_source)
		T = T.inverse(Affine);

	this->pose = T * this->pose;

	// Transform the current point cloud by that transformation
	rgbd::transform_point_cloud_in_place(T, this->cloud, true);
	if (publish_marked_clouds) {
		vector < vector<vector<int> > > corres
				= icp1.getCorrespondencesHistory();
		markOutliers(corres, curr_cloud);
	}
	render_pose = this->pose;
	if (publish_marked_clouds) {
		vector < vector<vector<int> > > corres
				= icp.getCorrespondencesHistory();
		markOutliers(corres, curr_cloud);
	}

	current_outlier_percentage = findOutliersPercentOnly(cloud_pair_params.max_distance);
}

/*
 * Set the vertical, horizontal, and distance extents of the tracking region.
 *
 * \param 	h_extent	Horizontal extend in degrees
 * \param 	v_extent	Vertical extend in degrees
 * \param 	nearest	Closest trackable distance from the kinect sensor
 */
void Puppet::setTrackingRegion(float h_extent, float v_extent, float nearest) {
	h_tracking_region = h_extent;
	v_tracking_region = v_extent;
	near_tracking_region = nearest;
}

/*
 * Manually define the puppet's current pose
 *
 * \param	new_pose	The pose to be set
 */
void Puppet::setCurrentPose(vector<float> new_pose) {
	current_pose_index++;
	visible = true;
	lost = false;
	poses.push_back(new_pose);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			render_pose.matrix()(j, i) = new_pose[i * 4 + j];
		}
	}
	pose = render_pose;

	if(!use_inverse_icp){
		rgbd::transform_point_cloud_in_place(pose, this->cloud, true); // Outdated; may not function correctly
	}
}

/*
 * Set the RGB value associated with a vertex of the puppet's display model
 *
 * \param	r,g,b	The components of the color
 */
void Puppet::setVertexColor(int index, float r, float g, float b)
{
	if (index*3+2 < color_list_display.size())
	{
		color_list_display[index*3] 	= r;
		color_list_display[index*3+1] 	= g;
		color_list_display[index*3+2] 	= b;
	}
	recolored_vertices[index] = 1;
}


/*
 * Used for scanning colors. Sometimes vertices can be difficult to colorize
 * due to visibility. This function spreads colors from labeled vertices to
 * unlabeled vertices.
 *
 */
void Puppet::spreadVertexColors()
{
	ROS_INFO_STREAM("Spreading colors. May take several seconds");
	// Make sure all vertices have colors
	for (uint v=0; v<num_vertices_display; v++)
	{
		if (recolored_vertices[v] == 0)
		{
			// Found vertex that has not been colored;
			int closest_index 		= -1;
			float closest_distance 	= 1000;
			// Find closest vertex using greedy search
			for (uint s=0; s<num_vertices_display; s++)
			{
				if (v != s && recolored_vertices[s])
				{
					float new_distance = sqrt(pow(vertex_list_display[v*3] - vertex_list_display[s*3],2)
										  + pow(vertex_list_display[v*3+1] - vertex_list_display[s*3+1],2)
										  + pow(vertex_list_display[v*3+2] - vertex_list_display[s*3+2],2));

					if (new_distance < closest_distance)
					{
						closest_distance = new_distance;
						closest_index = s;
					}
				}
			}
			if (closest_index != -1)
			{
				recolored_vertices[v] 		= 1;
				color_list_display[v*3] 	= color_list_display[closest_index*3];
				color_list_display[v*3+1] 	= color_list_display[closest_index*3+1];
				color_list_display[v*3+2]	= color_list_display[closest_index*3+2];
			}
		}
	}
}


/*
 * Used for scanning colors. Save a new version of the puppet's display model
 * with its newly colored vertices.
 */
void Puppet::saveNewPly(){
	ofstream outfile;
	std::string output_name = display_ply_name;
	output_name.append("_recolored.ply");
	ROS_INFO_STREAM("Saving recolored ply to " << output_name);
	outfile.open (output_name.c_str());

	// Header
	outfile << "ply" << endl;
	outfile << "format ascii 1.0" << endl;
	outfile << "element vertex " <<  num_vertices << endl;
	outfile << "property float x" << endl;
	outfile << "property float y" << endl;
	outfile << "property float z" << endl;
	outfile << "property float nx" << endl;
	outfile << "property float ny" << endl;
	outfile << "property float nz" << endl;
	outfile << "property uchar red" << endl;
	outfile << "property uchar green" << endl;
	outfile << "property uchar blue" << endl;
	outfile << "property uchar alpha" << endl;
	outfile << "element face " <<  num_triangles << endl;
	outfile << "property list uchar int vertex_indices" << endl;
	outfile << "end_header" << endl;

	// Vertices
	for (uint v=0; v<num_vertices_display; v++)
	{
		outfile << vertex_list_display[v*3] << " " << vertex_list_display[v*3+1] << " " << vertex_list_display[v*3+2] << " ";
		outfile << normal_list_display[v*3] << " " << normal_list_display[v*3+1] << " " << normal_list_display[v*3+2] << " ";
		outfile << round(color_list_display[v*3]*255) << " " << round(color_list_display[v*3+1]*255) << " " << round(color_list_display[v*3+2]*255) << " ";
		outfile << 255 << endl;
	}

	// Faces
	for (uint t=0; t<num_triangles_display; t++)
	{
		outfile << "3 " << triangle_list_display[t*3] << " " << triangle_list_display[t*3+1] << " " << triangle_list_display[t*3+2] << endl;
	}
	outfile.close();
}

/*
 * Retrieve the latest list of corresponding points in the target and source clouds
 *
 * \return	corrs	The indices of the corresponding points
 */
void Puppet::getCorrespondences(vector<int> &corrs) {
	if (puppet_as_source) {
		cloud_pair->getCorrespondenceIndices(pose, corrs);
	} else {
		cloud_pair->getCorrespondenceIndices(pose.inverse(Affine), corrs);
	}
}

/*
 * Retrieve the latest list of corresponding points in the target and source clouds,
 * as well as their distances
 *
 * \param	transform	The transform that aligns the source points with the target points
 * \return	corrs		The indices of the corresponding points
 * \return	errs		The distances between the corresponding points
 */
void Puppet::getCorrsAndErrors(Transform3f transform, vector<int> &corrs,vector<float> &errs) {

//	double start, transformed,correspondences, transformed2;
//	gettimeofday(&tracker_time, NULL);
//	start 	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);

	Transform3f xform;
	if (puppet_as_source) {
		xform = transform;
	} else {
		xform = transform.inverse(Affine);
	}

//	gettimeofday(&tracker_time, NULL);
//	transformed 	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
//	cout << "1st transform: " << transformed - start << endl;

	cloud_pair->getCorrespondenceIndices(xform, corrs);

//	gettimeofday(&tracker_time, NULL);
//	correspondences 	= tracker_time.tv_sec+(tracker_time.tv_usec/1000000.0);
//	cout << "corr: " << correspondences - start << endl;

	vector < rgbd::eigen::Vector3f > source_pts;
	vector < rgbd::eigen::Vector3f > target_pts;
	cloud_pair->getPoints(source_pts, target_pts);

	// Transform the source points, then find the distances to their corresponding
	// target points
	errs.resize(corrs.size());
	int matches = 0;
	for (uint i = 0; i < corrs.size(); i++) {
		if (corrs[i] == -1) {
			errs[i] = 100;
		} else {
			errs[i]	= (float) (xform * source_pts[i] - target_pts[corrs[i]]).norm();
		}
	}
}

/*
 * Retrieve the distances between the corresponding points in the target and source clouds,
 * as well as their distances
 *
 * \param	transform	The transform that aligns the source points with the target points
 * \param	corrs		The indices of the corresponding points
 * \return	errs		The distances between the corresponding points
 */
void Puppet::getErrors(Transform3f transform, vector<int> &corrs,vector<float> &errs) {

	Transform3f xform;
	if (puppet_as_source) {
		xform = transform;
	} else {
		xform = transform.inverse(Affine);
	}

	vector < rgbd::eigen::Vector3f > source_pts;
	vector < rgbd::eigen::Vector3f > target_pts;
	cloud_pair->getPoints(source_pts, target_pts);

	// Transform the source points, then find the distances to their corresponding
	// target points
	errs.resize(corrs.size());
	for (uint i = 0; i < corrs.size(); i++) {
		if (corrs[i] == -1) {
			errs[i] = 100;
		} else {
			errs[i]	= (float) (xform * source_pts[i] - target_pts[corrs[i]]).norm();
		}
	}
}
