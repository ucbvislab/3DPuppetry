/*
  * GLManager.cpp
  *
  *     Created: 2011/2012
  *      Author: Robin Held (begun by Ankit Gupta)
  *
  *      This class provides rendering and animation loading/saving functionality.
  *      See LICENSE.txt for licensing info.
  */


#include "GLManager.h"
extern GLManager renderer;


///////////////////////////////////////////////////////////////////////////////
// Wrapper functions to call GLManager OpenGL commands
///////////////////////////////////////////////////////////////////////////////

void ReSizeGLSceneWrapper(int Width, int Height) {
	renderer.ReSizeGLScene(Width, Height);
}
void myKeyboardWrapper(unsigned char key, int x, int y) {
	renderer.myKeyboard(key, x, y);
}
void mySpecialKeyboardWrapper(int key, int x, int y) {
	renderer.mySpecialKeyboard(key, x, y);
}

void myMouseButtonWrapper(int button, int state, int x, int y) {
	renderer.myMouseButton(button, state, x, y);
}
void myMouseMoveWrapper(int x, int y) {
	renderer.myMouseMove(x, y);
}
void DrawGLSceneWrapper() {
	renderer.DrawGLScene();
}
void emptyWrapper() {
	renderer.empty();
}


///////////////////////////////////////////////////////////////////////////////
// Functions used by ASSIMP-based rendering functions
///////////////////////////////////////////////////////////////////////////////
void color4_to_float4(const struct aiColor4D *c, float f[4]) {
	f[0] = c->r;
	f[1] = c->g;
	f[2] = c->b;
	f[3] = c->a;
}

void set_float4(float f[4], float a, float b, float c, float d) {
	f[0] = a;
	f[1] = b;
	f[2] = c;
	f[3] = d;
}

///////////////////////////////////////////////////////////////////////////////
// Prep and initialization functions
///////////////////////////////////////////////////////////////////////////////

/*
 *  Class constructor. Set default parameter values.
 */
GLManager::GLManager() {
	render_width = 1024;
	render_height = 768;
	arcball_radius = 768 / 2;
	first_pass = true;
	record_pose = false;
	playNow = false;
	play_all_layers = false;
	current_puppet = 0;
	use_puppet_colors = true;
	near_clip = 0.1;
	far_clip = 5;
	v_FOV = 45.0f;
	bkgd_loaded = false;
	load_poses = false;
	h_tracking_region = v_FOV * 4.0 / 3.0;
	v_tracking_region = v_FOV;
	tracking_asymmetry = 0.9;
	near_tracking_region = 0;
	tracking_region_visible = 1;
	accumulated_time = 0;
	accumulated_time_two = 0;
	show_transient_pose = false;
	SIFT_saving = false;
	SIFT_it = 0;
	SIFT_save_now = false;
	capture_fps = 30;
	frame_period = 1.0 / ((double) capture_fps);
	captureNow = false;
	captureTwoNow = false;
	export_to_video = false;
	export_raw_video = false;
	table_shim = 0.005;
	started_capture = false;
	clear_poses = false;
	current_scene = -1;
	num_scenes = 0;
	transparency_fall_off_angle = 5;
	transparency_fall_off_distance = 0.05;
	local_capture = false;
	layer_two_synced = false;
	background_info_loaded = 0;
	control_mode = CAMERA_CONTROL;
	back_adj = Vector3f(0, 0, 0);
	show_light = false;
	light_quaternion = AngleAxis<float> (0, Vector3f(0, 0, 1));
	directional_light = false;
	show_light_frames = 45;
	light_frames_shown = 0;
	shadowClipThickness = 1.0;
	render_shadows = false;
	use_VBOs = false;
	apply_extra_light = true;
}

/*
 *  Class constructor. Set default parameter values.
 */
void GLManager::prep() {
	// Setup OpenGL
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(0, 0);
	window_occ = glutCreateWindow("Find Visible Pts");
	GLenum err = glewInit();
	if (GLEW_OK != err) {
		/* Problem: glewInit failed, something is seriously wrong. */
		ROS_INFO_STREAM("Error: " << glewGetErrorString(err));
	}
	ROS_INFO_STREAM("Status: Using GLEW " << glewGetString(GLEW_VERSION));
//	glutDisplayFunc(DrawGLSceneWrapper);
//	glutIdleFunc(emptyWrapper);
//	glutReshapeFunc(ReSizeGLSceneWrapper);
//	glutKeyboardFunc(myKeyboardWrapper); // function for when a key is pressed
//	glutSpecialFunc(mySpecialKeyboardWrapper);
//	glutMouseFunc(myMouseButtonWrapper);
//	glutMotionFunc(myMouseMoveWrapper);

	// Setup cameras, lights, and colors
	camera_pos[0] = 0.0;
	camera_pos[1] = 0.0;
	camera_pos[2] = 0.0;
	light_pos0[0] = 3.0;
	light_pos0[1] = 0.5;
	light_pos0[2] = 0.0;
	light_pos0[3] = 1.0;
	white[0] = 1.0;
	white[1] = 1.0;
	white[2] = 1.0;
	white[3] = 1.0;
	black[0] = 0.0;
	black[1] = 0.0;
	black[2] = 0.0;
	black[3] = 1.0;

	// Initialize OpenGL context
	InitGL(640, 480);

	num_available_puppets = Puppets.size();
	num_puppets = num_available_puppets;

	ROS_INFO_STREAM("GL INITIALIZED");

	//	int* maxTextureUnits = new int[1];
	//	glGetIntegerv(GL_MAX_TEXTURE_UNITS_ARB, maxTextureUnits);
	//	int nbTextureUnits = maxTextureUnits[0];
	//	ROS_INFO_STREAM("Texture Units available = " << nbTextureUnits);

	GLint texSize;
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &texSize);
	ROS_INFO_STREAM("Max texture size = " << texSize);

	// Prep each puppet
	for (int m = 0; m < num_puppets; m++) {
		// Preprocess the puppet's point cloud to remove the occluded points
		// Remove the invisible points from the virtual puppet

		removeOccludedPoints(m);

		// Create a point cloud out of the puppet's vertices
		pcl::PointCloud<PointT> ptcloud = Puppets[m]->cloud;
		int num_pts = Puppets[m]->cloud.points.size();
		Puppets[m]->cloud.points.clear();
		int counter = 0;
		for (int i = 0; i < num_pts; i++) {
			if (visible[i]) {
				Puppets[m]->cloud.points.push_back(ptcloud.points[i]);
				Puppets[m]->cloud_to_original_vertices.push_back(i);
				counter++;
			}
		}
		Puppets[m]->cloud.width		= Puppets[m]->cloud.points.size();
		Puppets[m]->cloud.height 	= 1;
		visible.clear();

		ROS_INFO_STREAM("Size of the old point cloud" << ptcloud.points.size());
		ROS_INFO_STREAM("Size of the new point cloud" << Puppets[m]->cloud.points.size());
		Puppets[m]->findCentroid();
		Puppets[m]->findMaxDistance();

		// Setup pose-recording variables
		timestamps.reserve(60000 * sizeof(double));
		absolute_timestamps.reserve(60000 * sizeof(double));
		timestamps_two.reserve(60000 * sizeof(double));
		absolute_timestamps_two.reserve(60000 * sizeof(double));
		vector<int> temp_visible_states;
		visible_states.push_back(temp_visible_states);
		visible_states[m].reserve(60000 * sizeof(int));
		visible_states_two.push_back(temp_visible_states);
		visible_states_two[m].reserve(60000 * sizeof(int));
		vector < vector<float> > temp_transforms;
		saved_poses.push_back(temp_transforms);
		saved_poses[m].reserve(60000 * sizeof(vector<float> (16)));
		saved_poses_two.push_back(temp_transforms);
		saved_poses_two[m].reserve(
				60000 * sizeof(vector<float> (16)));
		vector<float> temp_pose(16, 0);
		transient_puppet_poses.push_back(temp_pose);
		puppet_poses.push_back(temp_pose);
		puppet_visibility.push_back(0);
		in_first_layer.push_back(0);
		translation_components.push_back(Vector3f(0, 0, 0));
		puppets_to_draw.push_back(0);
		GLfloat *temp_float = new GLfloat[16];
		temp_puppet_poses.push_back(temp_float);
	}

	// Load animation?
	if (load_poses)
		loadPoses();

	current_puppet = 0;
	new_time = 0;
	current_pose_index = 0;
	need_to_save = false;

	// Setup performance space
	if (background_captured == false) {
		if (!playback_only)
		{
			// First detect position and orientation of tabletop
			ROS_INFO_STREAM("Waiting for background detection.");
			std_msgs::Float32MultiArray background_plane =
					*ros::topic::waitForMessage<std_msgs::Float32MultiArray>(
							"camera/background_plane");
			vector<float> background_plane_data = background_plane.data;

			captured_back_normal[0] = background_plane_data[0];
			captured_back_normal[1] = background_plane_data[1];
			captured_back_normal[2] = background_plane_data[2];
			captured_back_base[0] = background_plane_data[3];
			captured_back_base[1] = background_plane_data[4];
			captured_back_base[2] = background_plane_data[5];

			ROS_INFO_STREAM(" Captured live background info:");
			ROS_INFO_STREAM(
					"   Base: (" << captured_back_base[0] << ", "
							<< captured_back_base[1] << ", "
							<< captured_back_base[2] << ")");
			ROS_INFO_STREAM(
					"   Normal: (" << captured_back_normal[0] << ", "
							<< captured_back_normal[1] << ", "
							<< captured_back_normal[2] << ")");
			float normal_angle = 180 / PI * acos(
					back_normal.dot(
							Vector3f(0, 0, -1)
									/ (sqrt(back_normal.dot(back_normal)))));
			ROS_INFO_STREAM("   Angle with optical axis: " << normal_angle);

			background_captured = true;
		}

		if (background_info_loaded == 3) {
			ROS_INFO_STREAM(" Using background settings from loaded animation file.");
		} else {
			ROS_INFO_STREAM(" Using captured background settings.");
			back_normal = captured_back_normal;
			back_base = captured_back_base;
		}

		// Setup 3D tracking volume
		setTrackingVolume();
		setGroundPlane();
	}

	// Create draw lists for each puppet
	for (int m = 0; m < num_puppets; m++) {
		puppet_lists.push_back(0);
		puppet_VBOs_created.push_back(0);
		//		GLuint temp_uint[4];
		puppet_VBOs.push_back(new GLuint[4]);
		all_num_triangles.push_back(0);
		all_num_vertices.push_back(0);
	}
}

/*
 *  Initialize an OpenGL context
 *
 *  \param Width	Window width
 *  \param Height	Window height
 */
void GLManager::InitGL(int Width, int Height)
{
	// Load identity puppetview
	glMatrixMode( GL_MODELVIEW);
	glLoadIdentity();

	// Shading states
	glShadeModel( GL_SMOOTH);
	glClearColor(0.9, 0.9, 0.9, 0.0);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	// Depth states
	glClearDepth(1.0f);
	glDepthFunc( GL_LEQUAL);
	glEnable( GL_DEPTH_TEST);

	glEnable( GL_CULL_FACE);

	// We use glScale when drawing the scene
	glEnable( GL_NORMALIZE);

	// Set up lighting
	light_direction = Vector4f(0, 0, 1, 1);
	light_position = Vector4f(0, 0, 0, 1);
	ambient_term = 0.5;
	diffuse_term = 0.5;

	for (uint i = 0; i < 3; i++) {
		light_direction0[i] = light_direction[i];
		light_position0[i] = light_position[i];
		light_ambient0[i] = ambient_term;
		light_diffuse0[i] = diffuse_term;
	}
	light_direction0[3] = 0.0;
	light_position0[3] = 1.0;
	light_diffuse0[3] = 1.0;
	light_ambient0[3] = 1.0;
	light_diffuse0_dim[3] = 1.0;
	light_ambient0_dim[3] = 1.0;

	// Set up default camera position
	cam_direction[2] = 1;
	cam_up_vector[1] = 1;
	cam_quaternion = AngleAxis<float> (0, Vector3f(0, 0, 1));
	// Initialize mouse-related variables
	button_states[0] = false;
	button_states[1] = false;
	button_states[2] = false;
	button_states[3] = false;
	button_states[4] = false;

	glClearDepth(1.0); // Enables Clearing Of The Depth Buffer
	glDepthFunc(GL_LEQUAL);

	//Create the shadow map texture
	glActiveTextureARB( GL_TEXTURE1_ARB);
	glGenTextures(1, &shadowMapTexture);
	glBindTexture(GL_TEXTURE_2D, shadowMapTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, shadowMapSize,
			shadowMapSize, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glActiveTextureARB( GL_TEXTURE0_ARB);

	// Set up frame buffer for shadow rendering (see http://fabiensanglard.net/shadowmapping/index.php )
	GLenum FBOstatus;
	// create a framebuffer object
	glGenFramebuffersEXT(1, &fboId);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fboId);
	// Instruct openGL that we won't bind a color texture with the currently binded FBO
	glDrawBuffer( GL_NONE);
	glReadBuffer(GL_NONE);
	// Attach the texture to FBO depth attachment point
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
			GL_TEXTURE_2D, shadowMapTexture, 0);
	// check FBO status
	FBOstatus = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	if (FBOstatus != GL_FRAMEBUFFER_COMPLETE_EXT)
		ROS_INFO_STREAM("GL_FRAMEBUFFER_COMPLETE_EXT failed, CANNOT use FBO");
	// switch back to window-system-provided framebuffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

/*
 *  Start the renderer
 */
void GLManager::start() {
	if (!playback_only)
	{
		pthread_mutex_lock(&amutex);
		pthread_cond_wait(&cond, &amutex);
	}
	glutDestroyWindow(window_occ);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(render_width, render_height);
	glutInitWindowPosition(0, 0);
	window = glutCreateWindow("Puppet Show");
	glutSetWindow(window);
	//	glutSetWindow(window);
	glutDisplayFunc(DrawGLSceneWrapper);
	glutIdleFunc(emptyWrapper);
	glutReshapeFunc(ReSizeGLSceneWrapper);
	glutKeyboardFunc(myKeyboardWrapper); // function for when a key is pressed
	glutSpecialFunc(mySpecialKeyboardWrapper);
	glutMouseFunc(myMouseButtonWrapper);
	glutMotionFunc(myMouseMoveWrapper);

	camera_pos[0] = 0.0;
	camera_pos[1] = 0.0;
	camera_pos[2] = 0.0;
	//	light_pos0 	= {3.0, 0.5, 0.0, 1.0};
	light_pos0[0] = 3.0;
	light_pos0[1] = 0.5;
	light_pos0[2] = 0.0;
	light_pos0[3] = 1.0;
	white[0] = 1.0;
	white[1] = 1.0;
	white[2] = 1.0;
	white[3] = 1.0;
	black[0] = 0.0;
	black[1] = 0.0;
	black[2] = 0.0;
	black[3] = 1.0;

	InitGL(render_width, render_height);

	// Load scenes using ASSIMP library
	/////////////////////////
	// Messages from ASSIMP:
	// get a handle to the predefined STDOUT log stream and attach
	// it to the logging system. It remains active for all further
	// calls to aiImportFile(Ex) and aiApplyPostProcessing.
	//	struct aiLogStream 			stream;
	//	stream = aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT,NULL);
	//	aiAttachLogStream(&stream);
	/////////////////////////

	for (uint i = 0; i < scene_names.size(); i++) {
		glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
		glMatrixMode( GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode ( GL_MODELVIEW);
		glLoadIdentity();

		ROS_INFO_STREAM("Loading scene " << scene_names[i]);
		const aiScene* temp_scene = scene_importers[i].ReadFile(
				scene_names[i],
				aiProcess_GenNormals | aiProcess_FindDegenerates
						| aiProcess_SortByPType
						| aiProcess_ValidateDataStructure
						| aiProcess_JoinIdenticalVertices
						| aiProcess_FindInstances | aiProcess_GenUVCoords
						| aiProcess_TransformUVCoords
						| aiProcess_ImproveCacheLocality
						| aiProcess_RemoveRedundantMaterials
						| aiProcess_OptimizeMeshes
				);

		if (!temp_scene) {
			ROS_WARN(scene_importers[i].GetErrorString());
		} else {
			ROS_INFO_STREAM("  ...loading complete.");
		}

		glActiveTextureARB( GL_TEXTURE0_ARB);
		recursiveTextureLoad(temp_scene, temp_scene->mRootNode, i);

		// Create draw list
		if (scene_lists[i] == 0) {
			scene_lists[i] = glGenLists(1);
			glNewList(scene_lists[i], GL_COMPILE);
			recursiveAIRender(temp_scene, temp_scene->mRootNode, false, i);
			glEndList();
			ROS_INFO_STREAM("  ...draw list complete.");
		}
	}

	if (bkgd_loaded)
		loadBackgroundImage();
	if (!playback_only)
		pthread_mutex_unlock(&amutex);
	glutMainLoop();
	return;
}

///////////////////////////////////////////////////////////////////////////////
// Rendering functions
///////////////////////////////////////////////////////////////////////////////

/*
 *  Reset the rendering window
 *
 *  \param Width	New window width
 *  \param Height	New window height
 */
void GLManager::ReSizeGLScene(int Width, int Height) {

	if (Height == 0) // Prevent A Divide By Zero If The Window Is Too Small
		Height = 1;

	// Maintain 3:4 aspect ratio
	int new_width, new_height;
	new_width = Width;
	new_height = Height;
	if (new_width > new_height * 4 / 3) {
		new_width = new_height * 4 / 3;
	} else if (new_height > new_width * 3 / 4) {
		new_height = new_width * 3 / 4;
	}

	glutReshapeWindow(new_width, new_height);
	glViewport(0, 0, new_width, new_height);
	glMatrixMode( GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(v_FOV, (GLfloat) new_width / (GLfloat) new_height,near_clip, far_clip);

	render_width = new_width;
	render_height = new_height;
	arcball_radius = render_height / 2;
}

/*
 *  Calculate the equation for a plane that includes three vertices.
 *
 *  \param vertex_1	3D point
 *  \param vertex_2	3D point
 *  \param vertex_3	3D point
 */
GLdouble* GLManager::calculateClippingPlane(Vector3f vertex_1,
		Vector3f vertex_2, Vector3f vertex_3) {
	GLdouble *plane_equation = new GLdouble[4];

	double x1 = vertex_1[0];
	double y1 = vertex_1[1];
	double z1 = vertex_1[2];
	double x2 = vertex_2[0];
	double y2 = vertex_2[1];
	double z2 = vertex_2[2];
	double x3 = vertex_3[0];
	double y3 = vertex_3[1];
	double z3 = vertex_3[2];

	plane_equation[0] = y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2);
	plane_equation[1] = z1 * (x2 - x3) + z2 * (x3 - x1) + z3 * (x1 - x2);
	plane_equation[2] = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
	plane_equation[3] = -(x1 * (y2 * z3 - y3 * z2) + x2 * (y3 * z1 - y1 * z3)
			+ x3 * (y1 * z2 - y2 * z1));

	return plane_equation;
}


/*
 *  Main renering function. Also handles animation recording and playback,
 *  as well as global-variable-based communication with CloudTracker.
 */
void GLManager::DrawGLScene() {

	///////////////////////////////////////////////////////////////////////////////////////////////////
	// Deal with animation and puppet data first
	///////////////////////////////////////////////////////////////////////////////////////////////////

	//////////////////
	// Deal with timing data:

	timeval tim_framerate;
	gettimeofday(&tim_framerate, NULL);
	if (new_time == 0) {
		new_time = tim_framerate.tv_sec + (tim_framerate.tv_usec / 1000000.0);
		for (int i = 0; i < NUM_FPS_SAMPLES; i++) {
			rates_ave[i] = 0;
		}
	} else {
		old_time = new_time;
		new_time = tim_framerate.tv_sec + (tim_framerate.tv_usec / 1000000.0);
	}

	// Determine whether a new frame of a pre-recorded animation needs to be displayed
	// Here, we're only concerned with the first layer of animation
	if (playNow || captureTwoNow) {
		int restart_all_layers = 0;

		if (play_frame == 0) {
			play_start_time = new_time;
		}

		if (play_all_layers && play_frame_two == 0) {
			play_start_time_two = new_time;
		}

		// First make sure there is any data to play
		if (timestamps.size() == 0) {
			playNow = false;
		} else {
			new_frame_accessed = false;
			// Check whether we should move to the next frame in the stored data
			if ((int) timestamps.size() > play_frame + 1) {
				if (play_frame != 0) {
					if (timestamps[play_frame + 1] <= (new_time
							- play_start_time)) {
						if (export_to_video)
						{
							// Only proceed by one frame to avoid dropping frames
							if ((timestamps[play_frame + 1] <= (new_time
									- play_start_time)) && ((int) timestamps.size()
									> play_frame + 1))
								play_frame++;
						} else {
							// Proceed by as many frames as necessary to keep pace
							while ((timestamps[play_frame + 1] <= (new_time
												- play_start_time)) && ((int) timestamps.size()
												> play_frame + 1))
											play_frame++;

						}
						new_frame_accessed = true;
					} else {
						new_frame_accessed = false;
					}
				} else {
					new_frame_accessed = true;
					play_frame++;
				}
			} else {
				// Restart at beginning of data
				if (!play_all_layers && !captureTwoNow) {
					play_frame = 0;
				} else {
					restart_all_layers++;
				}
			}
		}

		// Determine whether a new frame of a pre-recorded animation needs to be displayed
		// Here, we're concerned with both animation layers
		if (play_all_layers) {
			// First make sure there is any data to play
			if (timestamps_two.size() == 0) {
				play_all_layers = false;
			} else {
				new_frame_accessed_two = false;
				// Check whether we should move to the next frame in the stored data
				if ((int) timestamps_two.size() > play_frame_two + 1) {
					if (play_frame_two != 0) {
						if (timestamps_two[play_frame_two + 1] <= (new_time
								- play_start_time_two)) {
							if (export_to_video)
							{
								// Only proceed by one frame to avoid dropping frames
								if ((timestamps_two[play_frame_two + 1] <= (new_time - play_start_time_two))
										&& ((int) timestamps_two.size()
												> play_frame_two + 1))
									play_frame_two++;
							} else {
								// Proceed by as many frames as necessary to keep pace
								while ((timestamps_two[play_frame_two + 1] <= (new_time - play_start_time_two))
										&& ((int) timestamps_two.size()
												> play_frame_two + 1))
									play_frame_two++;
							}
							new_frame_accessed_two = true;
						} else {
							new_frame_accessed_two = false;
						}
					} else {
						new_frame_accessed_two = true;
						play_frame_two++;
					}
				} else {
					restart_all_layers++;
				}
			}

			// Make sure the two animation layers are in sync
			if (timestamps_two[play_frame_two] < timestamps[play_frame])
			{
				while ((timestamps_two[play_frame_two] < timestamps[play_frame])
						&& (play_frame_two < (timestamps_two.size()-1)))
				{
						play_frame_two++;
						new_frame_accessed_two = true;
				}
			} else {
				while ((timestamps[play_frame] < timestamps_two[play_frame_two])
						&& (play_frame < (timestamps.size()-1)))
				{
						play_frame++;
						new_frame_accessed = true;
				}
			}

			if ((restart_all_layers == 2) && !export_to_video) {
				play_frame = 0;
				play_frame_two = 0;
			}
		}
	}

	// TAKE OWNERSHIP OF MUTEX LOCK
	gettimeofday(&tp_cond, NULL);
	ts_cond.tv_sec = tp_cond.tv_sec;
	ts_cond.tv_nsec = tp_cond.tv_usec * 1000;
	ts_cond.tv_sec += 1;
	if (!playNow && !playback_only) {
		pthread_mutex_lock(&amutex);
		pthread_cond_timedwait(&cond, &amutex, &ts_cond);
	}

	// Determine if vertex buffer objects or draw lists need to be generated for
	// each puppet. This process only needs to be done once.
	for (uint m = 0; m < num_puppets; m++) {
		if (use_VBOs) {
			// Check vertex buffer objects
			if (puppet_VBOs_created[m] == 0) {
				float* normals = Puppets[m]->getDisplayNormals();
				float* vertices = Puppets[m]->getDisplayVertices();
				float* colors = Puppets[m]->getDisplayColors();
				int* triangles = Puppets[m]->getDisplayTriangles();
				int num_triangles = Puppets[m]->getDisplayNumTriangles();
				int num_vertices = Puppets[m]->getDisplayNumVertices();

				if (use_puppet_colors) {
					glGenBuffers(1, &puppet_VBOs[m][0]);
					glBindBuffer(GL_ARRAY_BUFFER, puppet_VBOs[m][0]);
					glBufferData(GL_ARRAY_BUFFER,
							num_vertices * 3 * sizeof(float), colors,
							GL_STATIC_DRAW);
					glColorPointer(3, GL_FLOAT, 0, 0);
				}
				glGenBuffers(1, &puppet_VBOs[m][1]);
				glBindBuffer(GL_ARRAY_BUFFER, puppet_VBOs[m][1]);
				glBufferData(GL_ARRAY_BUFFER, num_vertices * 3 * sizeof(float),
						normals, GL_STATIC_DRAW);
				glNormalPointer(GL_FLOAT, 0, 0);

				glGenBuffers(1, &puppet_VBOs[m][2]);
				glBindBuffer(GL_ARRAY_BUFFER, puppet_VBOs[m][2]);
				glBufferData(GL_ARRAY_BUFFER, num_vertices * 3 * sizeof(float),
						vertices, GL_STATIC_DRAW);
				glVertexPointer(3, GL_FLOAT, 0, 0);

				glGenBuffers(1, &puppet_VBOs[m][3]);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, puppet_VBOs[m][3]);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER,
						num_triangles * 3 * sizeof(int), triangles,
						GL_STATIC_DRAW);

				all_num_vertices[m] = num_vertices;
				all_num_triangles[m] = num_triangles;
				puppet_VBOs_created[m] = 1;
			}
		} else {
			// Check draw lists
			if (puppet_lists[m] == 0) {
				puppet_lists[m] = glGenLists(1);
				glNewList(puppet_lists[m], GL_COMPILE);
				if (Puppets[m]->using_legos) {
					int numblocks = Puppets[m]->blocks.size();
					for (int i = 0; i < numblocks; i++) {
						glPushMatrix();
						glTranslatef(Puppets[m]->blocks[i].trans[0],
								Puppets[m]->blocks[i].trans[1],
								Puppets[m]->blocks[i].trans[2]);
						if (Puppets[m]->blocks[i].rot)
							glRotatef(90, 0.0f, 0.0f, 1.0f);
						renderBlock(Puppets[m]->blocks[i].color[0],
								Puppets[m]->blocks[i].color[1],
								Puppets[m]->blocks[i].color[2],m);
						glPopMatrix();
					}
				} else {
					renderPuppet(m);
				}
				glEndList();
			}
		}
	}

	// Retrieve the visibility and pose data for each puppet
	grabPuppetData();

	// If we're playing back an animation, suspend tracking.
	if (playNow) {
		suspend_tracking = true;
	} else {
		suspend_tracking = false;
	}

	//	gettimeofday(&tim_framerate, NULL);
	//	double puppet_rendering_start = tim_framerate.tv_sec+(tim_framerate.tv_usec/1000000.0);

	//	// Outdated. During debugging, we rendered the pose of each puppet after the coarse ICP pass
	//	// using a different color
	//	// Recover poses, if available
	//	if (show_transient_pose && !playNow)
	//	{
	//		for (int m = 0; m < num_puppets; m++)
	//		{
	//			if (Puppets[m]->visible)
	//			{
	//				puppet_visibility[m] = 1;
	//				if (show_transient_pose)
	//					transient_puppet_poses[m] = *Puppets[m]->getTransientPose();
	//				puppet_poses[m] = *Puppets[m]->getCurrentPose();
	//
	//			} else {
	//				puppet_visibility[m] = 0;
	//			}
	//		}
	//	}

	// Clear all the poses? (read: clear the stored animations)
	if (clear_poses) {
		clear_animation = true; // Send signal to blocktracker
		clear_poses = false;
	}

	// Capture animation?
	if (captureNow && !playNow) {
		global_capture = true;
		local_capture = true;
		if (!started_capture) {
			//			gettimeofday(&time_grabber, NULL);
			//			record_initial_time = time_grabber.tv_sec+(time_grabber.tv_usec/1000000.0);]
			record_initial_time = new_time;
			started_capture = true;
		}
	} else if (captureTwoNow && !playNow) {
		local_capture = true;
		if (!started_capture) {
			//			gettimeofday(&time_grabber, NULL);
			//			record_initial_time = time_grabber.tv_sec+(time_grabber.tv_usec/1000000.0);]
			record_initial_time_two = new_time;
			started_capture = true;
		} else {
			layer_two_synced = true;
		}
	} else {
		global_capture = false;
		local_capture = false;
	}

	//	gettimeofday(&tim_framerate, NULL);
	//	double puppet_rendering_stop = tim_framerate.tv_sec+(tim_framerate.tv_usec/1000000.0);
	//	cout << "Time to render under mutex lock: " << (puppet_rendering_stop - puppet_rendering_start) << endl;

	if (local_capture && (captureNow || captureTwoNow) && !playNow	&& started_capture) {
		if (record_pose && ((raw_capture_started && captureNow)	|| (layer_two_synced && captureTwoNow))) {
			savePoses();
		}
	}

	// Capture SIFT pose?
	if (SIFT_save_now) {
		getSIFTSet();
		signal_SIFT_save = true; // This is read by CloudTracker to trigger an image capture.
		SIFT_save_now = false;
	}

	// Check whether we've reached the end of the rendering phase of movie export
	if (playNow && export_to_video && new_frame_accessed) {
		if ((!play_all_layers && play_frame == (int) timestamps.size() - 1)
				|| (play_all_layers && (play_frame == (int) timestamps.size()
						- 1) && (play_frame_two == (int) timestamps_two.size()
						- 1))) {
			// If so, stop rendering the animation...
			export_to_video = false;
			playNow = false;
			// ...and combine the rendered movie with the raw-video movie
			if (export_raw_video)
				combineVideos();
		}
	}

	// RELEASE OWNERSHIP OF MUTEX LOCK
	if (!playNow && !playback_only)
		pthread_mutex_unlock(&amutex);
	//////////////

	///////////////////////////////////////////////////////////////////////////////////////////////////
	// Now execute rendering calls
	///////////////////////////////////////////////////////////////////////////////////////////////////

	use_puppet_colors = true; // Use the stored vertex colors to render each puppet.

	if (current_scene == 4) {
		glClearColor(0.0, 0.0, 0.0, 0.0);
	} else {
		glClearColor(0.6, 0.6, 0.9, 0.0);
	}

	// Set up lighting
	Transform3f light_rot;
	light_rot.setIdentity();
	light_rot.rotate(light_quaternion);

	light_direction = Vector4f(0, 0, -1, 1);
	light_direction = light_rot * light_direction;

	for (uint i = 0; i < 3; i++) {
		light_direction0[i] = light_direction[i];
		light_position0[i] = light_position[i];
		light_ambient0[i] = ambient_term;
		light_ambient0_dim[i] = ambient_term * 0.75f;
		light_diffuse0[i] = diffuse_term;
		light_diffuse0_dim[i] = diffuse_term * 0.75f;
	}

	// Calculate rotation for camera
	Transform3f rot;
	rot.setIdentity();
	rot.rotate(cam_quaternion);

	// Calculate & save projection matrices
	// Uses shadow-mapping approach, which requires rendering from the light source's
	// point of view, as well as the camera's point of view
	glMatrixMode( GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluPerspective(v_FOV, (GLfloat) render_width / (GLfloat) render_height,
			near_clip, far_clip);
	glGetFloatv(GL_PROJECTION_MATRIX, cameraProjectionMatrix);
	glPopMatrix();

	glPushMatrix();
	glLoadIdentity();
	Vector3f light_to_base(light_position[0] - back_base[0],
			light_position[1] - back_base[1], light_position[2] - back_base[2]);
	gluPerspective(135.0f, 1.0f,
			max(0.1, light_to_base.norm() - (shadowClipThickness - 0.1)),
			light_to_base.norm() + 0.1);
	glGetFloatv(GL_PROJECTION_MATRIX, lightProjectionMatrix);
	glLoadIdentity();
	glPopMatrix();

	glMatrixMode( GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	gluLookAt(cam_position[0], cam_position[1], cam_position[2],
			cam_position[0] + cam_direction[0],
			cam_position[1] + cam_direction[1],
			cam_position[2] + cam_direction[2], cam_up_vector[0],
			cam_up_vector[1], cam_up_vector[2]);

	glTranslatef(0, 0, back_distance);
	glMultMatrixf(rot.data());
	glTranslatef(0, 0, -back_distance);
	glGetFloatv(GL_MODELVIEW_MATRIX, cameraViewMatrix);
	glPopMatrix();

	glPushMatrix();
	glLoadIdentity();
	gluLookAt(light_position0[0], light_position0[1], light_position0[2], 0, 0,
			1000, 0.0f, 1.0f, 0.0f);
	glGetFloatv(GL_MODELVIEW_MATRIX, lightViewMatrix);
	glLoadIdentity();
	glPopMatrix();

	if (render_shadows) {
		apply_extra_light = false;

		// First pass - from light's point of view - saved to FBO
		// (again, see http://fabiensanglard.net/shadowmapping/index.php)
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fboId); //Rendering offscreen

		// Using the fixed pipeline to render to the depthbuffer
		glUseProgramObjectARB(0);

		// In the case we render the shadowmap to a higher resolution, the viewport must be modified accordingly.
		glViewport(0, 0, shadowMapSize, shadowMapSize);

		// Clear previous frame values
		glClear( GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(lightProjectionMatrix);

		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(lightViewMatrix);

		// Use viewport the same size as the shadow map
		glViewport(0, 0, shadowMapSize, shadowMapSize);

		// Draw back faces into the shadow map
		glCullFace( GL_FRONT);

		//	glDisable(GL_CULL_FACE);
		glPolygonOffset(3, 1.0);
		glEnable( GL_POLYGON_OFFSET_FILL);

		// Disable color writes, and use flat shading for speed
		glShadeModel( GL_FLAT);
		glColorMask(0, 0, 0, 0);

		// Draw the scene
		glPushMatrix();
		// Draw the virtual set
		drawBackground();
		// Draw puppets
		drawAllPuppets();
		glPopMatrix();
		glActiveTextureARB( GL_TEXTURE1_ARB);
		//Read the depth buffer into the shadow map texture
		glBindTexture(GL_TEXTURE_2D, shadowMapTexture);
		glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, shadowMapSize,
				shadowMapSize);
		glActiveTextureARB( GL_TEXTURE0_ARB);

		glColorMask(1, 1, 1, 1);

		// Now rendering from the camera POV, using the FBO to generate shadows
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

		//restore states
		glCullFace( GL_BACK);
		glShadeModel( GL_SMOOTH);

		glDisable(GL_POLYGON_OFFSET_FILL);

		// 2nd pass - Draw from camera's point of view with dim light
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(cameraProjectionMatrix);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadMatrixf(cameraViewMatrix);

		glViewport(0, 0, render_width, render_height);

		// Use dim light to represent shadowed areas
		glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
		glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient0_dim);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse0_dim);
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
		glEnable( GL_LIGHTING);
		glEnable( GL_LIGHT0);

		// Draw scene
		glEnable( GL_DEPTH_TEST);
		glEnable ( GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glShadeModel(GL_SMOOTH);

		// Draw the virtual set
		drawBackground();

		glColor3f(1.0f, 1.0f, 1.0f);
		glDisable( GL_CULL_FACE);
		glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);

		// Draw puppets
		drawAllPuppets();

		glEnable(GL_CULL_FACE);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		// 3rd pass - draw with bright light
		apply_extra_light = true;

		//Calculate texture matrix for projection
		//This matrix takes us from eye space to the light's clip space
		//It is postmultiplied by the inverse of the current view matrix when specifying texgen
		glActiveTextureARB(GL_TEXTURE1_ARB);
		static MATRIX4X4 biasMatrix(0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f,
				0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.5f, 0.5f, 0.5f, 1.0f); //bias from [-1, 1] to [0, 1]
		MATRIX4X4 textureMatrix = biasMatrix * lightProjectionMatrix
				* lightViewMatrix;

		//Set up texture coordinate generation.
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
		glTexGenfv(GL_S, GL_EYE_PLANE, textureMatrix.GetRow(0));
		glEnable( GL_TEXTURE_GEN_S);

		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
		glTexGenfv(GL_T, GL_EYE_PLANE, textureMatrix.GetRow(1));
		glEnable( GL_TEXTURE_GEN_T);

		glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
		glTexGenfv(GL_R, GL_EYE_PLANE, textureMatrix.GetRow(2));
		glEnable( GL_TEXTURE_GEN_R);

		glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
		glTexGenfv(GL_Q, GL_EYE_PLANE, textureMatrix.GetRow(3));
		glEnable( GL_TEXTURE_GEN_Q);

		glBindTexture(GL_TEXTURE_2D, shadowMapTexture);
		glEnable( GL_TEXTURE_2D);

		// Enable shadow comparison
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE,
				GL_COMPARE_R_TO_TEXTURE);

		// Shadow comparison should be true (ie not in shadow) if r<=texture
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

		// Shadow comparison should generate an INTENSITY result
		glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);

		//Set alpha test to discard false comparisons
		glAlphaFunc(GL_GEQUAL, 0.99f);
		glEnable( GL_ALPHA_TEST);

		glActiveTextureARB(GL_TEXTURE0_ARB);

		glEnable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse0);
		glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient0);
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);

		glShadeModel(GL_SMOOTH);

		// Draw the virtual set
		drawBackground();

		glColor3f(1.0f, 1.0f, 1.0f);
		glDisable(GL_CULL_FACE);
		glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);

		// Draw puppets
		drawAllPuppets();

		//Disable textures and texgen
		glDisable(GL_TEXTURE_2D);
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glDisable(GL_ALPHA_TEST);
		glDisable(GL_TEXTURE_GEN_S);
		glDisable(GL_TEXTURE_GEN_T);
		glDisable(GL_TEXTURE_GEN_R);
		glDisable(GL_TEXTURE_GEN_Q);
		glActiveTextureARB(GL_TEXTURE0_ARB);

		glEnable(GL_CULL_FACE);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		// Arrays to hold matrix information for arcball rotations
		glGetDoublev(GL_MODELVIEW_MATRIX, arc_rotation_puppet_view);
		glGetDoublev(GL_PROJECTION_MATRIX, arc_rotation_projection);
		glGetIntegerv(GL_VIEWPORT, arc_rotation_viewport);

		// Show edges of trackable region?
		if (tracking_region_visible > 0)
			drawTrackingBase();
		if (tracking_region_visible == 2)
			drawTrackingVolume();

		// If the user is changing the lighting direction, display a vector showing its direction
		if ((directional_light && show_light) || (!directional_light
				&& light_frames_shown < show_light_frames))
			drawLightingVector();

		//Restore other states
		glDisable(GL_LIGHTING);
		glPopMatrix();
	} else {
		// Render without shadows (faster)
		glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

		glViewport(0, 0, render_width, render_height);
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(cameraProjectionMatrix);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadMatrixf(cameraViewMatrix);

		glEnable( GL_DEPTH_TEST);
		glEnable ( GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glShadeModel ( GL_SMOOTH);

		glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
		glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
		glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
		if (directional_light)
			glLightfv(GL_LIGHT0, GL_POSITION, light_direction0);
		else
			glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

		glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse0);
		glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient0);
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

		glEnable( GL_LIGHTING);
		glEnable( GL_LIGHT0);

		// Arrays to hold matrix information for arcball rotation
		glGetDoublev(GL_MODELVIEW_MATRIX, arc_rotation_puppet_view);
		glGetDoublev(GL_PROJECTION_MATRIX, arc_rotation_projection);
		glGetIntegerv(GL_VIEWPORT, arc_rotation_viewport);

		// Draw the virtual set
		drawBackground();

		glColor3f(1.0f, 1.0f, 1.0f);
		glDisable( GL_CULL_FACE);
		glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);

		// Draw puppets
		drawAllPuppets();

		glEnable(GL_CULL_FACE);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		// Show edges of trackable region?
		if (tracking_region_visible > 0)
			drawTrackingBase();

		if (tracking_region_visible == 2)
			drawTrackingVolume();

		// If the user is changing the lighting direction, display a vector showing its direction
		if ((directional_light && show_light) || (!directional_light
				&& light_frames_shown < show_light_frames))
			drawLightingVector();
		glPopMatrix();
	}

	if (!export_to_video)
		drawTextOnScreen();	// Draw info like frame rate, recording/playback state, etc.

	// Swap the buffers to display, since double buffering is used.
	glutSwapBuffers();

	glDisable( GL_LIGHTING);
	glDisable( GL_LIGHT0);

	///////////////////////////////////////////////////////////////////////////////////////////////////
	// Video-capture functionality
	///////////////////////////////////////////////////////////////////////////////////////////////////

	// Save more frames to video, if requested
	if (playNow && export_to_video && (new_frame_accessed || new_frame_accessed_two)) {
		getScreenShot(capturedImg);
		// Make sure we're not ahead of the clock
		if (!play_all_layers) {
			while ((double) total_exported_frames * frame_period
					< timestamps[play_frame]) {
				//			ROS_INFO_STREAM(timestamps[play_frame] << " " << (double)total_exported_frames*frame_period);
				writer << *capturedImg;
				total_exported_frames++;
				absolute_exported_timestamps.push_back(absolute_timestamps[play_frame]);
			}
		} else {
			while (((double) total_exported_frames * frame_period
					< timestamps[play_frame])
					|| ((double) total_exported_frames * frame_period
							< timestamps_two[play_frame_two])) {
				writer << *capturedImg;
				total_exported_frames++;
				if (new_frame_accessed) {
					absolute_exported_timestamps.push_back(
							absolute_timestamps[play_frame]);
				} else {
					absolute_exported_timestamps.push_back(
							absolute_timestamps_two[play_frame_two]
									- (absolute_timestamps_two[0]
											- absolute_timestamps[0]));
				}
			}
		}

		if ((!play_all_layers && play_frame == (int) timestamps.size() - 1)
				|| (play_all_layers && (play_frame == (int) timestamps.size() - 1)
						&& (play_frame_two == (int) timestamps_two.size() - 1))) {
			export_to_video = false;
			playNow = false;
			if (export_raw_video)
				combineVideos();
		}
	}
}


/*
 *	Render the virtual set
 */
void GLManager::drawBackground() {
	glPushMatrix();
	glTranslatef(back_adj[0], back_adj[1], back_adj[2]);

	if (current_scene >= 0) {
		// 3D virtual set
		glColor3f(1.0f, 1.0f, 1.0f);
		if (apply_extra_light) {
			GLfloat temp_brightness[4];
			for (uint i = 0; i < 4; i++)
				temp_brightness[i] = scene_brightnesses[current_scene];

			glLightfv(GL_LIGHT1, GL_AMBIENT, temp_brightness);
			glEnable( GL_LIGHT1);
		}
		glDisable( GL_CULL_FACE);
		glPushMatrix();
		Transform3f scene_rot;
		glTranslatef(0, 0, back_distance);
		scene_rot.setIdentity();
		scene_rot.rotate(bkgd_quaternion);
		scene_rot.rotate(AngleAxis<float> (PI, Vector3f(1, 0, 0)));
		glMultMatrixf(scene_rot.data());
		glScalef(scene_scales[current_scene], scene_scales[current_scene],
				scene_scales[current_scene]);
		glEnable( GL_TEXTURE_2D);
		glCallList(scene_lists[current_scene]);
		if (current_scene == 3) {
			glRotatef(90, 0, 0, 1);
			glTranslatef(1.6, -6.8, 0);
			glCallList(scene_lists[current_scene]);
		}
		glDisable(GL_TEXTURE_2D);
		glPopMatrix();
		glEnable(GL_CULL_FACE);
		if (apply_extra_light)
			glDisable( GL_LIGHT1);
	} else {
		// Simple ground plane
		if (bkgd_loaded) {
			// Use loaded image
			glColor3f(1.0f, 1.0f, 1.0f);
			glEnable( GL_COLOR_MATERIAL);
			glEnable( GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, bkgd_texture);
			glBegin( GL_QUADS);
			glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0, 0.0);
			glVertex3f(ground_plane_edges[0].x(), ground_plane_edges[0].y(),
					ground_plane_edges[0].z() - table_shim);
			glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 1.0, 0.0);
			glVertex3f(ground_plane_edges[1].x(), ground_plane_edges[1].y(),
					ground_plane_edges[1].z() - table_shim);
			glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 1.0, 1.0);
			glVertex3f(ground_plane_edges[2].x(), ground_plane_edges[2].y(),
					ground_plane_edges[2].z() - table_shim);
			glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0, 1.0);
			glVertex3f(ground_plane_edges[3].x(), ground_plane_edges[3].y(),
					ground_plane_edges[3].z() - table_shim);
			glEnd();
			// Back face
			glBegin(GL_QUADS);
			glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0, 1.0);
			glVertex3f(ground_plane_edges[3].x(), ground_plane_edges[3].y(),
					ground_plane_edges[3].z() - table_shim);
			glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 1.0, 1.0);
			glVertex3f(ground_plane_edges[2].x(), ground_plane_edges[2].y(),
					ground_plane_edges[2].z() - table_shim);
			glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 1.0, 0.0);
			glVertex3f(ground_plane_edges[1].x(), ground_plane_edges[1].y(),
					ground_plane_edges[1].z() - table_shim);
			glMultiTexCoord2fARB(GL_TEXTURE0_ARB, 0.0, 0.0);
			glVertex3f(ground_plane_edges[0].x(), ground_plane_edges[0].y(),
					ground_plane_edges[0].z() - table_shim);
			glEnd();
			glDisable(GL_TEXTURE_2D);
		} else {
			// Use green rectangle
			glColor3f(0.1f, 0.75f, 0.1f);
			glBegin( GL_QUADS);
			for (uint g = 0; g < ground_plane_edges.size(); g++) {
				glVertex3f(ground_plane_edges[g].x(),
						ground_plane_edges[g].y(),
						ground_plane_edges[g].z() - table_shim);
			}
			for (int g = ground_plane_edges.size() - 1; g >= 0; g--) {
				glVertex3f(ground_plane_edges[g].x(),
						ground_plane_edges[g].y(),
						ground_plane_edges[g].z() - table_shim);
			}
			glEnd();
		}
	}
	glPopMatrix();
}

/*
 *  Draw sphere to represent position of light source
 */
void GLManager::drawLightingVector() {
	glDisable( GL_LIGHTING);
	glDisable( GL_CULL_FACE);
	glColor3f(0, 1.0, 0);
//  // We allowed directional lights in a previous version. This code is included in
	// case we want to put them back in.
//	if (directional_light) {
//		glDisable(GL_LIGHTING);
//		glDisable(GL_CULL_FACE);
//		Vector3f base = captured_back_base + back_normal * 0.25;
//
//		//	ROS_INFO_STREAM("Base: " << base[0] << " , " << base[1] << " , " <<  base[2]);
//		Vector3f head;
//		for (uint i = 0; i < 3; i++)
//			head[i] = base[i] - light_direction[i] * 0.10;
//
//		glPushMatrix();
//		Transform3f light_rot;
//		light_rot.setIdentity();
//		light_rot.rotate(light_quaternion);
//		light_rot.pretranslate(base);
//		//		glTranslatef(0, 0, back_distance);
//		glMultMatrixf(light_rot.data());
//		//		glTranslatef(0, 0, -back_distance);
//		gluDisk(gluNewQuadric(), 0.025, 0.03, 32, 32);
//		glPopMatrix();
//
//		glColor3f(1.0, 0.0, 1.0);
//
//		glPointSize(8);
//		glBegin( GL_POINTS);
//		glVertex3f(base[0], base[1], base[2]);
//		glEnd();
//		glPointSize(1);
//
//		glLineWidth(4);
//		glBegin( GL_LINES);
//		glVertex3f(base[0], base[1], base[2]);
//		glVertex3f(head[0], head[1], head[2]);
//		glEnd();
//		glLineWidth(2);
//
//	} else {
		glPointSize(50);
		glBegin( GL_POINTS);
		glVertex3f(light_position[0], light_position[1], light_position[2]);
		glEnd();
		glPointSize(2);
//	}
	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glColor3f(1.0, 1.0, 1.0);
	light_frames_shown++;

}

/*
 *	Display the trackable region as a flat polygon on the virtual performance surface
 */
void GLManager::drawTrackingBase() {
	glColor3f(0.0, 1.0, 0.0);
	glDisable( GL_COLOR_MATERIAL);
	glDisableClientState( GL_COLOR_ARRAY);
	mat_specular[0] = 0.0;
	mat_specular[1] = 0.0;
	mat_specular[2] = 0.0;
	mat_specular[3] = 0.0;
	mat_shininess[0] = 0;
	mat_ambient[0] = 0.1;
	mat_ambient[1] = 0.8;
	mat_ambient[2] = 0.1;
	mat_ambient[3] = 0.1;
	GLfloat mat_diffuse[] = { 0, 0, 0, 0.25 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glBegin( GL_QUADS);
	for (uint g = 0; g < far_track_plane_edges.size(); g++) {
		glVertex3f(far_track_plane_edges[g].x(), far_track_plane_edges[g].y(),
				far_track_plane_edges[g].z() - table_shim - 0.0025);
	}
	glEnd();
	glColor3f(0.0, 1.0, 0.0);
}

/*
 *	Display the edges of the entire, 3D trackable volume
 */
void GLManager::drawTrackingVolume() {

	mat_specular[0] = 0.0;
	mat_specular[1] = 0.0;
	mat_specular[2] = 0.0;
	mat_specular[3] = 0.0;
	mat_shininess[0] = 0;
	mat_ambient[0] = 0.1;
	mat_ambient[1] = 0.8;
	mat_ambient[2] = 0.1;
	mat_ambient[3] = 0.1;
	GLfloat mat_diffuse[] = { 0, 0, 0, 0.25 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

	glDisable( GL_LIGHTING);
	glLineWidth(2);
	glBegin( GL_LINES);
	glColor3f(0.0, 1.0, 0.0);
	glVertex3f(close_track_plane_edges[0].x(), close_track_plane_edges[0].y(),
			close_track_plane_edges[0].z());
	glVertex3f(close_track_plane_edges[1].x(), close_track_plane_edges[1].y(),
			close_track_plane_edges[1].z());
	glVertex3f(close_track_plane_edges[1].x(), close_track_plane_edges[1].y(),
			close_track_plane_edges[1].z());
	glVertex3f(close_track_plane_edges[2].x(), close_track_plane_edges[2].y(),
			close_track_plane_edges[2].z());
	glVertex3f(close_track_plane_edges[2].x(), close_track_plane_edges[2].y(),
			close_track_plane_edges[2].z());
	glVertex3f(close_track_plane_edges[3].x(), close_track_plane_edges[3].y(),
			close_track_plane_edges[3].z());
	glVertex3f(close_track_plane_edges[3].x(), close_track_plane_edges[3].y(),
			close_track_plane_edges[3].z());
	glVertex3f(close_track_plane_edges[0].x(), close_track_plane_edges[0].y(),
			close_track_plane_edges[0].z());

	glVertex3f(close_track_plane_edges[0].x(), close_track_plane_edges[0].y(),
			close_track_plane_edges[0].z());
	glVertex3f(far_track_plane_edges[0].x(), far_track_plane_edges[0].y(),
			far_track_plane_edges[0].z());

	glVertex3f(close_track_plane_edges[1].x(), close_track_plane_edges[1].y(),
			close_track_plane_edges[1].z());
	glVertex3f(far_track_plane_edges[1].x(), far_track_plane_edges[1].y(),
			far_track_plane_edges[1].z());

	glVertex3f(close_track_plane_edges[2].x(), close_track_plane_edges[2].y(),
			close_track_plane_edges[2].z());
	glVertex3f(far_track_plane_edges[2].x(), far_track_plane_edges[2].y(),
			far_track_plane_edges[2].z());

	glVertex3f(close_track_plane_edges[3].x(), close_track_plane_edges[3].y(),
			close_track_plane_edges[3].z());
	glVertex3f(far_track_plane_edges[3].x(), far_track_plane_edges[3].y(),
			far_track_plane_edges[3].z());
	glEnd();
	glEnable(GL_LIGHTING);

	glBegin( GL_QUADS);
	// Top
	for (uint g = 0; g < close_track_plane_edges.size(); g++) {
		glVertex3f(close_track_plane_edges[g].x(),
				close_track_plane_edges[g].y(), close_track_plane_edges[g].z());
	}
	for (int g = close_track_plane_edges.size() - 1; g >= 0; g--) {
		glVertex3f(close_track_plane_edges[g].x(),
				close_track_plane_edges[g].y(), close_track_plane_edges[g].z());
	}
	// Side 1
	glVertex3f(close_track_plane_edges[0].x(), close_track_plane_edges[0].y(),
			close_track_plane_edges[0].z());
	glVertex3f(close_track_plane_edges[1].x(), close_track_plane_edges[1].y(),
			close_track_plane_edges[1].z());
	glVertex3f(far_track_plane_edges[1].x(), far_track_plane_edges[1].y(),
			far_track_plane_edges[1].z());
	glVertex3f(far_track_plane_edges[0].x(), far_track_plane_edges[0].y(),
			far_track_plane_edges[0].z());
	glVertex3f(close_track_plane_edges[1].x(), close_track_plane_edges[1].y(),
			close_track_plane_edges[1].z());
	glVertex3f(close_track_plane_edges[0].x(), close_track_plane_edges[0].y(),
			close_track_plane_edges[0].z());
	glVertex3f(far_track_plane_edges[0].x(), far_track_plane_edges[0].y(),
			far_track_plane_edges[0].z());
	glVertex3f(far_track_plane_edges[1].x(), far_track_plane_edges[1].y(),
			far_track_plane_edges[1].z());
	// Side 2
	glVertex3f(close_track_plane_edges[1].x(), close_track_plane_edges[1].y(),
			close_track_plane_edges[1].z());
	glVertex3f(close_track_plane_edges[2].x(), close_track_plane_edges[2].y(),
			close_track_plane_edges[2].z());
	glVertex3f(far_track_plane_edges[2].x(), far_track_plane_edges[2].y(),
			far_track_plane_edges[2].z());
	glVertex3f(far_track_plane_edges[1].x(), far_track_plane_edges[1].y(),
			far_track_plane_edges[1].z());
	glVertex3f(close_track_plane_edges[2].x(), close_track_plane_edges[2].y(),
			close_track_plane_edges[2].z());
	glVertex3f(close_track_plane_edges[1].x(), close_track_plane_edges[1].y(),
			close_track_plane_edges[1].z());
	glVertex3f(far_track_plane_edges[1].x(), far_track_plane_edges[1].y(),
			far_track_plane_edges[1].z());
	glVertex3f(far_track_plane_edges[2].x(), far_track_plane_edges[2].y(),
			far_track_plane_edges[2].z());
	// Side 3
	glVertex3f(close_track_plane_edges[2].x(), close_track_plane_edges[2].y(),
			close_track_plane_edges[2].z());
	glVertex3f(close_track_plane_edges[3].x(), close_track_plane_edges[3].y(),
			close_track_plane_edges[3].z());
	glVertex3f(far_track_plane_edges[3].x(), far_track_plane_edges[3].y(),
			far_track_plane_edges[3].z());
	glVertex3f(far_track_plane_edges[2].x(), far_track_plane_edges[2].y(),
			far_track_plane_edges[2].z());
	glVertex3f(close_track_plane_edges[3].x(), close_track_plane_edges[3].y(),
			close_track_plane_edges[3].z());
	glVertex3f(close_track_plane_edges[2].x(), close_track_plane_edges[2].y(),
			close_track_plane_edges[2].z());
	glVertex3f(far_track_plane_edges[2].x(), far_track_plane_edges[2].y(),
			far_track_plane_edges[2].z());
	glVertex3f(far_track_plane_edges[3].x(), far_track_plane_edges[3].y(),
			far_track_plane_edges[3].z());
	// Side 4
	glVertex3f(close_track_plane_edges[3].x(), close_track_plane_edges[3].y(),
			close_track_plane_edges[3].z());
	glVertex3f(close_track_plane_edges[0].x(), close_track_plane_edges[0].y(),
			close_track_plane_edges[0].z());
	glVertex3f(far_track_plane_edges[0].x(), far_track_plane_edges[0].y(),
			far_track_plane_edges[0].z());
	glVertex3f(far_track_plane_edges[3].x(), far_track_plane_edges[3].y(),
			far_track_plane_edges[3].z());
	glVertex3f(close_track_plane_edges[0].x(), close_track_plane_edges[0].y(),
			close_track_plane_edges[0].z());
	glVertex3f(close_track_plane_edges[3].x(), close_track_plane_edges[3].y(),
			close_track_plane_edges[3].z());
	glVertex3f(far_track_plane_edges[3].x(), far_track_plane_edges[3].y(),
			far_track_plane_edges[3].z());
	glVertex3f(far_track_plane_edges[0].x(), far_track_plane_edges[0].y(),
			far_track_plane_edges[0].z());
	glEnd();
}

/*
 *	Display info like frame rate, recording/playback status, etc.
 */
void GLManager::drawTextOnScreen() {
	if (new_time > 0) {
		double rate = 1 / (round(new_time * 1000) / 1000.0 - round(
				old_time * 1000) / 1000.0);
		double aved_rate = 0;
		for (int i = 0; i < NUM_FPS_SAMPLES - 1; i++) {
			rates_ave[i] = rates_ave[i + 1];
			aved_rate += rates_ave[i] / NUM_FPS_SAMPLES;
		}
		rates_ave[NUM_FPS_SAMPLES - 1] = rate;
		aved_rate += rate / NUM_FPS_SAMPLES;
		//		ROS_INFO_STREAM("time difference: "<<rate);

		glMatrixMode( GL_PROJECTION);
		glPushMatrix();
		glDisable( GL_LIGHTING);
		glLoadIdentity();
		glOrtho(0, render_width, 0, render_height, -100, 100);
		if (playNow) {
			glColor3f(0.9f, 0.9f, 1.0f);
			glRasterPos2f(0.0f, 0.0f);
			string s = "Time: ";
			if ((uint) play_frame < timestamps.size() - 1) {
				s.append(
						boost::lexical_cast<std::string>(timestamps[play_frame]));
			} else if (play_all_layers) {
				s.append(
						boost::lexical_cast<std::string>(
								timestamps_two[play_frame_two]));
			}
			const char *str = s.c_str();
			for (int i = 0; i < 12; i++)
				glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str[i]);
		} else {
			if (!playback_only)
			{
				glColor3f(0.9f, 0.9f, 1.0f);
				glRasterPos2f(0.0f, 0.0f);
				string s = "FPS: ";
				s.append(boost::lexical_cast<std::string>(aved_rate));
				const char *str = s.c_str();
				for (int i = 0; i < 10; i++)
					glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str[i]);
			}
		}
		if ((captureNow || captureTwoNow) && !playNow) {
			glColor3f(1.0f, 0.5f, 0.5f);
			string r = "*REC*";
			glRasterPos2f(0.0f, 20.0f);
			const char *str_r = r.c_str();
			for (int i = 0; i < 5; i++)
				glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str_r[i]);
		} else if (playNow && !export_to_video) {
			glColor3f(0.5f, 1.0f, 0.5f);
			string r = "*PLAY*";
			glRasterPos2f(0.0f, 20.0f);
			const char *str_r = r.c_str();
			for (int i = 0; i < 6; i++)
				glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str_r[i]);
		}
		glPopMatrix();
	}
	glColor3f(1.0f, 1.0f, 1.0f);
}

/*
 *	Obtain the latest pose estimates for each puppet
 *	Stores the translation component of each puppet so we can render the puppets with
 *	partial transparency if they're near an edge of the tracking volume.
 *
 */
void GLManager::grabPuppetData() {
	for (int m = 0; m < num_puppets; m++) {
		//		ROS_INFO_STREAM(current_puppet << " " << in_first_layer[current_puppet] << " " << visible_states[current_puppet][play_frame]);
		//		if (Puppets[current_puppet]->visible){
		if ((!playNow && (Puppets[m]->getCurrentPoseIndex() > -1)
				&& (Puppets[m]->visible)) || (playNow
				&& visible_states[m][play_frame]) || (playNow
				&& play_all_layers
				&& visible_states_two[m][play_frame_two])
				|| (captureTwoNow && visible_states[m][play_frame])) {

			puppets_to_draw[m] = true;
			Vector3f translation_component;

			if (!playNow) {
				// Currently recording the second layer of animation.
				if (!captureTwoNow || (captureTwoNow && !in_first_layer[m])) {

					vector<float>* current_pose = Puppets[m]->getCurrentPose();
					for (int i = 0; i < 16; i++) {
						temp_puppet_poses[m][i]	= current_pose->at(i);
					}
					translation_component = Puppets[m]->getCurrentPoseTransform().translation();
				}

			} else {
				if (in_first_layer[m]) {
					// Get pose from stored data
					for (int i = 0; i < 16; i++) {
						temp_puppet_poses[m][i]
								= saved_poses[m][play_frame][i];
					}
					translation_component[0]	= temp_puppet_poses[m][12];
					translation_component[1]	= temp_puppet_poses[m][13];
					translation_component[2]	= temp_puppet_poses[m][14];
				} else if (play_all_layers) {
					// Get pose from stored data
					for (int i = 0; i < 16; i++) {
						temp_puppet_poses[m][i]
								= saved_poses_two[m][play_frame_two][i];
					}
					translation_component[0]	= temp_puppet_poses[m][12];
					translation_component[1]	= temp_puppet_poses[m][13];
					translation_component[2]	= temp_puppet_poses[m][14];
				}
			}

			if (captureTwoNow && in_first_layer[m]) {
				// Get pose from stored data
				for (int i = 0; i < 16; i++) {
					temp_puppet_poses[m][i]
							= saved_poses[m][play_frame][i];
				}
				translation_component[0] = temp_puppet_poses[m][12];
				translation_component[1] = temp_puppet_poses[m][13];
				translation_component[2] = temp_puppet_poses[m][14];
			}
			translation_components[m] = translation_component;
		} else {
			puppets_to_draw[m] = false;
		}
	}
}

/*
 *	Go through each puppet, and if it's labeled as visible, render it onscreen *
 */
void GLManager::drawAllPuppets() {
	// Draw final poses
	for (int m = 0; m < num_puppets; m++) {
		if (puppets_to_draw[m]) {
			// Apply partial transparency if the puppet is near an edge of the tracking volume.
			float apply_alpha = adjustTransparency(translation_components[m]);

			if (apply_extra_light || (!apply_extra_light && apply_alpha > 0.5)) {
				// Global transform
				glPushMatrix();
				glMultMatrixf(temp_puppet_poses[m]);
				if (use_VBOs)
					renderPuppetVBO(m);
				else
					glCallList(puppet_lists[m]);
				glPopMatrix();
			}
		}
	}
}

/*
 *	Render a duplo block
 *
 *	\param r,g,b			Red, blue, and green components of the block's color
 *	\param current_puppet	The index of the block (stored with the puppets) to be rendered;
 */
void GLManager::renderBlock(float r, float g, float b, int current_puppet) {
	if (use_puppet_colors) {
		GLfloat mat_diffuse[] = { 0, 0, 0, 1 };
		mat_diffuse[0] = r;
		mat_diffuse[1] = g;
		mat_diffuse[2] = b;
		glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
		glMaterialfv(GL_FRONT, GL_AMBIENT, mat_diffuse);
	}

	glEnableClientState( GL_VERTEX_ARRAY);
	glEnableClientState( GL_NORMAL_ARRAY);

	float* normals 		= Puppets[current_puppet]->getDisplayNormals();
	float* vertices 	= Puppets[current_puppet]->getDisplayVertices();
	int* triangles 		= Puppets[current_puppet]->getDisplayTriangles();
	int num_triangles 	= Puppets[current_puppet]->getDisplayNumTriangles();

	glNormalPointer(GL_FLOAT, 0, normals);
	glVertexPointer(3, GL_FLOAT, 0, vertices);
	glDrawElements(GL_TRIANGLES, num_triangles * 3, GL_UNSIGNED_INT, triangles);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}

/*
 *	Render a puppet using its stored 3D model and element arrays
 *
 *	\param current_puppet	The index of the puppet to be rendered;
 */
void GLManager::renderPuppet(int current_puppet) {
	glEnableClientState( GL_VERTEX_ARRAY);
	glEnableClientState( GL_NORMAL_ARRAY);
	glEnable( GL_CULL_FACE);
	glCullFace( GL_BACK);
	if (use_puppet_colors) {
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glEnable( GL_COLOR_MATERIAL);
		glEnableClientState( GL_COLOR_ARRAY);
	}

	float* normals = Puppets[current_puppet]->getDisplayNormals();
	float* vertices = Puppets[current_puppet]->getDisplayVertices();
	float* colors = Puppets[current_puppet]->getDisplayColors();
	int* triangles = Puppets[current_puppet]->getDisplayTriangles();
	int num_triangles = Puppets[current_puppet]->getDisplayNumTriangles();

	glNormalPointer(GL_FLOAT, 0, normals);
	glVertexPointer(3, GL_FLOAT, 0, vertices);
	if (use_puppet_colors)
		glColorPointer(3, GL_FLOAT, 0, colors);
	glDrawElements(GL_TRIANGLES, num_triangles * 3, GL_UNSIGNED_INT, triangles);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	if (use_puppet_colors) {
		glDisable( GL_COLOR_MATERIAL);
		glDisableClientState( GL_COLOR_ARRAY);
	}
}

/*
 *	Render a puppet using its stored 3D model and a vertex buffer object
 *
 *	\param current_puppet	The index of the puppet to be rendered;
 */
void GLManager::renderPuppetVBO(int current_puppet) {
	//	gettimeofday(&time_grabber, NULL);
	//	double init_time = time_grabber.tv_sec + (time_grabber.tv_usec / 1000000.0);
	glEnableClientState( GL_VERTEX_ARRAY);
	glEnableClientState( GL_NORMAL_ARRAY);
	glDisable( GL_CULL_FACE);
	int num_triangles = all_num_triangles[current_puppet];
	if (use_puppet_colors) {
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glEnable( GL_COLOR_MATERIAL);
		glEnableClientState( GL_COLOR_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, puppet_VBOs[current_puppet][0]);
		glColorPointer(3, GL_FLOAT, 0, 0);
	}
	glBindBuffer(GL_ARRAY_BUFFER, puppet_VBOs[current_puppet][1]);
	glNormalPointer(GL_FLOAT, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, puppet_VBOs[current_puppet][2]);
	glVertexPointer(3, GL_FLOAT, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, puppet_VBOs[current_puppet][3]);
	glDrawElements(GL_TRIANGLES, num_triangles * 3, GL_UNSIGNED_INT, 0);

	glDisableClientState(GL_VERTEX_ARRAY); // disable vertex arrays
	glDisableClientState(GL_NORMAL_ARRAY);
	if (use_puppet_colors) {
		glDisable( GL_COLOR_MATERIAL);
		glDisableClientState( GL_COLOR_ARRAY);
	}
	//	gettimeofday(&time_grabber, NULL);
	//	double final_time = time_grabber.tv_sec + (time_grabber.tv_usec / 1000000.0);
	//	ROS_INFO_STREAM("Time to render: " << final_time - init_time);
}


/*
 *	Renders a puppet from several points of view and removes from the puppet's point cloud
 *	any vertices that are never visible. This saves time when running ICP.
 *
 *	Only performed once per puppet and saved to a file for future use.
 *
 *	\param current_puppet	Index of the puppet to be processed
 */
void GLManager::removeOccludedPoints(int current_puppet) {
	int num_pts = Puppets[current_puppet]->cloud.points.size();
	int visible_pts = 0;
	vector<int> input_visible;

	// Setup file names
	std::string new_folder = files_path;
	std::string name;
	std::string occluded_pts_saver;
	bool find_occluded_pts = false;
	name = puppet_names[current_puppet];
	// Check whether the occluded points have already been saved
	occluded_pts_saver = new_folder;
	occluded_pts_saver.append("models/");
	occluded_pts_saver.append(name);
	occluded_pts_saver.append(".yml");

	pcl::PointCloud<PointT> temp_cloud; // Used for checking direction of normals

	if (!fs::exists(occluded_pts_saver.c_str())) {
		ROS_INFO_STREAM("No visible-points file for puppet " << current_puppet + 1);
		find_occluded_pts = true;
	} else {
		ROS_INFO_STREAM("Found visible-points file for puppet " << current_puppet + 1);
		cv::FileStorage	input(occluded_pts_saver.c_str(), cv::FileStorage::READ);
		input["visible"] >> input_visible;
		input.release();
		// Make sure the loaded vector is the correct size
		if ((int) input_visible.size() != num_pts) {
			find_occluded_pts = true;
		} else {
			visible.resize(num_pts, false);
			for (int i = 0; i < num_pts; i++) {
				if (input_visible[i]) {
					visible[i] = true;
					visible_pts++;
				}
			}
		}
	}

	// If a file with point visibility wasn't found, rotate the puppet about the
	// x and y axes and check which points are visible from at least one point of view
	// Any points are never visible should be removed from the point cloud used for tracking
	if (find_occluded_pts) {
		ROS_INFO_STREAM("Finding occluded points.");
		//        if( ! glh_init_extensions("GL_ARB_occlusion_query ") )
		//            {
		//                    cerr << "Necessary extensions were not supported:" << endl
		//                             << glh_get_unsupported_extensions() << endl << endl
		//                             << "Press <enter> to quit." << endl;
		//            }
		GLint bitsSupported;
		glGetQueryiv(GL_SAMPLES_PASSED, GL_QUERY_COUNTER_BITS, &bitsSupported);
		ROS_INFO_STREAM("Number of counter bits = " << bitsSupported);
		GLuint oquery, numsamples;
		glGenQueries(1, &oquery);

		visible.resize(num_pts, false);
		PointT pt;//, ptn;

		// Rotations about x-axis
		for (int j = -180; j <= 180; j += 15) {
			ROS_INFO_STREAM(j);
			glViewport(0, 0, 640, 480); // Reset The Current Viewport And Perspective Transformation
			glMatrixMode( GL_PROJECTION);
			glLoadIdentity(); // Reset The Projection Matrix

			gluPerspective(v_FOV, (GLfloat) 640 / (GLfloat) 480, near_clip,
					far_clip);

			glMatrixMode ( GL_MODELVIEW);
			glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); // Clear The Screen And The Depth Buffer

			glLoadIdentity();

			gluLookAt(0, -0.05, -0.3, 0, 0, 1, 0, -1, 0);

			GLfloat mat_specular[4];
			mat_specular[0] = 0.0;
			mat_specular[1] = 0.0;
			mat_specular[2] = 0.0;
			mat_specular[3] = 1.0;
			GLfloat mat_shininess[1];
			mat_shininess[0] = 50;
			GLfloat light_positionA[] = { -1.0, 1.0, -1.0, 1.0 };
			GLfloat light_positionB[] = { 1.0, 1.0, -1.0, 1.0 };
			GLfloat light_positionC[] = { -1.0, 1.0, 1.0, 1.0 };
			GLfloat light_positionD[] = { 1.0, 1.0, 1.0, 1.0 };
			GLfloat light_diffuse[] = { 0.5, 0.5, 0.5, 1.0 };
			glClearColor(0.0, 0.0, 0.0, 0.0);
			glShadeModel ( GL_SMOOTH);

			glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
			glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
			glLightfv(GL_LIGHT0, GL_POSITION, light_positionA);
			glLightfv(GL_LIGHT1, GL_POSITION, light_positionB);
			glLightfv(GL_LIGHT2, GL_POSITION, light_positionC);
			glLightfv(GL_LIGHT3, GL_POSITION, light_positionD);
			glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
			glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
			glLightfv(GL_LIGHT2, GL_DIFFUSE, light_diffuse);
			glLightfv(GL_LIGHT3, GL_DIFFUSE, light_diffuse);

			glEnable( GL_LIGHTING);
			glEnable( GL_LIGHT0);
			glEnable( GL_LIGHT1);
			glEnable( GL_LIGHT2);
			glEnable( GL_LIGHT3);
			glEnable( GL_DEPTH_TEST);
			glDepthFunc( GL_LEQUAL);

			glRotatef(j, 1, 0, 0);
			// Load point cloud and transform it if we want to verify that reasonable normals are being used.
			if (Puppets[current_puppet]->usingNormals()) {
				temp_cloud = Puppets[current_puppet]->cloud;
				Transform3f temp_transform;
				temp_transform.setIdentity();
				AngleAxis<float> x_rot((float) j * PI / 180, Vector3f(1, 0, 0));
				temp_transform.rotate(x_rot);
				rgbd::transform_point_cloud_in_place(temp_transform,
						temp_cloud, true);
			}
			glPushMatrix();
			if (Puppets[current_puppet]->using_legos) {
				int numblocks = Puppets[current_puppet]->blocks.size();
				for (int i = 0; i < numblocks; i++) {
					glPushMatrix();
					glTranslatef(Puppets[current_puppet]->blocks[i].trans[0],
							Puppets[current_puppet]->blocks[i].trans[1],
							Puppets[current_puppet]->blocks[i].trans[2]);
					if (Puppets[current_puppet]->blocks[i].rot)
						glRotatef(90, 0.0f, 0.0f, 1.0f);
					renderBlock(Puppets[current_puppet]->blocks[i].color[0],
							Puppets[current_puppet]->blocks[i].color[1],
							Puppets[current_puppet]->blocks[i].color[2],current_puppet);
					glPopMatrix();
				}
			} else {
				renderPuppet(current_puppet);
			}
			glPopMatrix();

			int bad_normals = 0;

			// Draw tiny spheres at each vertex
			for (int i = 0; i < num_pts; i++) {
				// Draw sphere
				pt = Puppets[current_puppet]->cloud.points[i];
				glColor3f(0, 1, 0);
				// Prepare to check occlusions/visibility
				// and re-render all the spheres
				glBeginQuery(GL_SAMPLES_PASSED, oquery);
				glPushMatrix();
				glTranslatef(pt.x, pt.y, pt.z);
				glutSolidSphere(.002, SPHERE_SAMPLES, SPHERE_SAMPLES);
				glPopMatrix();
				glEndQuery( GL_SAMPLES_PASSED);

				// Store visibility of the spheres in numsamples
				glGetQueryObjectuiv(oquery, GL_QUERY_RESULT, &numsamples);

				// Store whether the current point is visible.
				if (numsamples > 0) {
					bool check_point = true;
					if (Puppets[current_puppet]->usingNormals()) {
						// Check whether the normal is facing the camera
						pt = temp_cloud.points[i];
						if (pt.normal_z >= 0) {
							bad_normals++;
							check_point = false;
						}
					}

					if (!visible[i] && check_point) {
						visible_pts++;
						visible[i] = true;
					}
				}
			}
			glutSwapBuffers();
			printf("Number of points: %d, %d\n", visible_pts, num_pts);
			printf("Number of points removed due to bad normals: %d\n",
					bad_normals);
			printf("Percentage of points visible: %f\n",
					(100.0f * visible_pts) / num_pts);
		}

		// Rotations about Y-axis
		for (int j = -180; j <= 180; j += 15) {
			ROS_INFO_STREAM(j);
			glViewport(0, 0, 640, 480); // Reset The Current Viewport And Perspective Transformation
			glMatrixMode( GL_PROJECTION);
			glLoadIdentity(); // Reset The Projection Matrix

			gluPerspective(v_FOV, (GLfloat) 640 / (GLfloat) 480, near_clip,
					far_clip);

			glMatrixMode ( GL_MODELVIEW);
			glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); // Clear The Screen And The Depth Buffer

			glLoadIdentity();
			gluLookAt(0, -0.05, -0.3, 0, 0, 1, 0, -1, 0);
			GLfloat mat_specular[4];
			mat_specular[0] = 0.0;
			mat_specular[1] = 0.0;
			mat_specular[2] = 0.0;
			mat_specular[3] = 1.0;
			GLfloat mat_shininess[1];
			mat_shininess[0] = 50;
			GLfloat light_positionA[] = { -1.0, 1.0, -1.0, 1.0 };
			GLfloat light_positionB[] = { 1.0, 1.0, -1.0, 1.0 };
			GLfloat light_positionC[] = { -1.0, 1.0, 1.0, 1.0 };
			GLfloat light_positionD[] = { 1.0, 1.0, 1.0, 1.0 };
			GLfloat light_diffuse[] = { 0.5, 0.5, 0.5, 1.0 };
			glClearColor(0.0, 0.0, 0.0, 0.0);
			glShadeModel ( GL_SMOOTH);

			glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
			glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
			glLightfv(GL_LIGHT0, GL_POSITION, light_positionA);
			glLightfv(GL_LIGHT1, GL_POSITION, light_positionB);
			glLightfv(GL_LIGHT2, GL_POSITION, light_positionC);
			glLightfv(GL_LIGHT3, GL_POSITION, light_positionD);
			glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
			glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
			glLightfv(GL_LIGHT2, GL_DIFFUSE, light_diffuse);
			glLightfv(GL_LIGHT3, GL_DIFFUSE, light_diffuse);

			glEnable( GL_LIGHTING);
			glEnable( GL_LIGHT0);
			glEnable( GL_LIGHT1);
			glEnable( GL_LIGHT2);
			glEnable( GL_LIGHT3);
			glEnable( GL_DEPTH_TEST);
			glDepthFunc( GL_LEQUAL);

			glRotatef(j, 0, 1, 0);

			if (Puppets[current_puppet]->usingNormals()) {
				temp_cloud = Puppets[current_puppet]->cloud;
				Transform3f temp_transform;
				temp_transform.setIdentity();
				AngleAxis<float> y_rot((float) j * PI / 180, Vector3f(0, 1, 0));
				temp_transform.rotate(y_rot);
				rgbd::transform_point_cloud_in_place(temp_transform,
						temp_cloud, true);
			}

			glPushMatrix();
			if (Puppets[current_puppet]->using_legos) {
				int numblocks = Puppets[current_puppet]->blocks.size();
				for (int i = 0; i < numblocks; i++) {
					glPushMatrix();
					glTranslatef(Puppets[current_puppet]->blocks[i].trans[0],
							Puppets[current_puppet]->blocks[i].trans[1],
							Puppets[current_puppet]->blocks[i].trans[2]);
					if (Puppets[current_puppet]->blocks[i].rot)
						glRotatef(90, 0.0f, 0.0f, 1.0f);
					renderBlock(Puppets[current_puppet]->blocks[i].color[0],
							Puppets[current_puppet]->blocks[i].color[1],
							Puppets[current_puppet]->blocks[i].color[2],current_puppet);
					glPopMatrix();
				}
			} else {
				renderPuppet(current_puppet);
			}
			glPopMatrix();

			int bad_normals = 0;
			for (int i = 0; i < num_pts; i++) {
				// draw sphere
				pt = Puppets[current_puppet]->cloud.points[i];
				glColor3f(0, 1, 0);
				glBeginQuery(GL_SAMPLES_PASSED, oquery);
				glPushMatrix();
				glTranslatef(pt.x, pt.y, pt.z);
				glutSolidSphere(.002, SPHERE_SAMPLES, SPHERE_SAMPLES);
				glPopMatrix();
				glEndQuery( GL_SAMPLES_PASSED);
				glGetQueryObjectuiv(oquery, GL_QUERY_RESULT, &numsamples);
				if (numsamples > 0) {
					bool check_point = true;
					if (Puppets[current_puppet]->usingNormals()) {
						pt = temp_cloud.points[i];
						if (pt.normal_z >= 0) {
							bad_normals++;
							check_point = false;
						}
					}
					if (!visible[i] && check_point) {
						visible_pts++;
						visible[i] = true;
					}
				}
			}
			glutSwapBuffers();
			printf("Number of points: %d, %d\n", visible_pts, num_pts);
			printf("Number of points removed due to bad normals: %d\n",
					bad_normals);
			printf("Percentage of points visible: %f\n",
					(100.0f * visible_pts) / num_pts);
		}

		cv::FileStorage output(occluded_pts_saver.c_str(),
				cv::FileStorage::WRITE);
		output << "visible" << "[:";
		for (int i = 0; i < num_pts; i++) {
			output << visible[i];
		}
		output.release();
	}
}

/*
 *	Add a virtual scene to the system and set up all the necessary variables.
 */
void GLManager::addScene(std::string scene_name, float scale_factor, float brightness) {
	std::string temp_scene_name = files_path;
	temp_scene_name.append("backgrounds/");
	temp_scene_name.append(scene_name);
	scene_names.push_back(temp_scene_name);
	scene_lists.push_back(0);
	scene_importers.push_back(Assimp::Importer());
	vector<struct MyMesh> temp_mesh;
	scene_meshes.push_back(temp_mesh);
	vector<struct aiMatrix4x4> temp_trans;
	scene_transforms.push_back(temp_trans);
	scene_scales.push_back(scale_factor);
	scene_brightnesses.push_back(brightness);
	vector<TextureAndPath> texture_paths_temp;
	texturesAndPaths.push_back(texture_paths_temp);
	num_scenes++;
}

/*
 *	Load the textures associated with a virtual set.
 *
 *	\param 	sc			The ASSIMP scene object
 *	\param 	nd			Node within the scene
 *	\param	scene_num	Index of the scene object
 *
 *	Note: Code adapted from http://assimp.svn.sourceforge.net/viewvc/assimp/trunk/samples/SimpleOpenGL/
 */
void GLManager::recursiveTextureLoad(const struct aiScene *sc, const struct aiNode* nd, int scene_num) {
	int i;
	unsigned int n = 0, t;

	// draw all meshes assigned to this node
	for (; n < nd->mNumMeshes; ++n) {
		const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];
		unsigned int cont = aiGetMaterialTextureCount(
				sc->mMaterials[mesh->mMaterialIndex], aiTextureType_DIFFUSE);
		struct aiString* str = (aiString*) malloc(sizeof(struct aiString));

		if (cont > 0) {
			aiTextureMapMode *mapU;
			aiGetMaterialTexture(sc->mMaterials[mesh->mMaterialIndex],
					aiTextureType_DIFFUSE, 0, str, 0, 0, 0, 0, mapU, 0);
			// See if another mesh is already using this texture, if so, just copy GLuint instead of remaking entire texture
			bool newTextureToBeLoaded = true;
			for (int x = 0; x < texturesAndPaths[scene_num].size(); x++) {
				if (texturesAndPaths[scene_num][x].pathName == *str) {
					TextureAndPath reusedTexture;
					reusedTexture.hTexture
							= texturesAndPaths[scene_num][x].hTexture;
					reusedTexture.pathName = *str;
					texturesAndPaths[scene_num].push_back(reusedTexture);
					newTextureToBeLoaded = false;
					break;
				}
			}

			int modeS;
			switch (*mapU) {
			case aiTextureMapMode_Wrap:
				modeS = GL_REPEAT;
				break;
			case aiTextureMapMode_Clamp:
				modeS = GL_CLAMP;
				break;
			case aiTextureMapMode_Mirror:
				modeS = GL_MIRRORED_REPEAT;
				break;
			default:
				modeS = GL_REPEAT;
			}

			if (newTextureToBeLoaded) {
				// Get material name
				string tex_name;
				// Get scene name;
				size_t found_folder =
						scene_names[scene_num].find_last_of("/\\");
				string scene_folder = scene_names[scene_num].substr(0,
						found_folder);
				scene_folder.append("/");
				tex_name = scene_folder;
				tex_name.append(str->data);
				Mat tex = imread(tex_name);

				if (tex.data == NULL)
					ROS_ERROR("Could not load texture.");

				//Now generate the OpenGL texture object
				TextureAndPath newTexture;
				newTexture.pathName = *str;
				glGenTextures(1, &newTexture.hTexture);

				glBindTexture(GL_TEXTURE_2D, newTexture.hTexture);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
						GL_LINEAR_MIPMAP_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, modeS);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, modeS);
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex.cols, tex.rows, 0,
						GL_BGR_EXT, GL_UNSIGNED_BYTE, tex.data);
				glGenerateMipmap( GL_TEXTURE_2D);
				GLenum huboError = glGetError();

				if (huboError)
					ROS_ERROR("There was an error loading the texture.");

				texturesAndPaths[scene_num].push_back(newTexture);
			}
		}
	}

	// Get textures from all children
	for (n = 0; n < nd->mNumChildren; ++n)
		recursiveTextureLoad(sc, nd->mChildren[n], scene_num);
}


/*
 *	Activate a material to be applied to a mesh within a virtual scene
 *
 *	\param 	mtl		The ASSIMP material to be applied
 *
 *	Note: Code adapted from http://assimp.svn.sourceforge.net/viewvc/assimp/trunk/samples/SimpleOpenGL/
 *	Note: We disabled several of the material properties due to issues with our Sketchup-
 *		  exported dae files.
 */
void GLManager::apply_material(const struct aiMaterial *mtl) {
	float c[4];

	struct aiColor4D diffuse;
	struct aiColor4D specular;
	struct aiColor4D ambient;
	struct aiColor4D emission;
	int two_sided;
	uint max;

	set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
	if (AI_SUCCESS	== aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse)) {
		color4_to_float4(&diffuse, c);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, c);
	}

	//	set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	//	if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR,
	//			&specular)) {
	//		color4_to_float4(&specular, c);
	//		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
	//		//		ROS_INFO_STREAM("specular");
	//	}

	//	set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
	//	if (AI_SUCCESS
	//			== aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient)) {
	//		color4_to_float4(&ambient, c);
	//		//		ROS_INFO_STREAM("ambient");
	//		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);
	//	}

	//	set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
	//	if (AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE,
	//			&emission)) {
	//		color4_to_float4(&emission, c);
	//		//		ROS_INFO_STREAM("emission");
	//		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);
	//	}

	//	max = 1;
	//	ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
	//	if (ret1 == AI_SUCCESS) {
	//				ROS_INFO_STREAM("s array");
	//		max = 1;
	//		ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH,
	//				&strength, &max);
	//		if (ret2 == AI_SUCCESS)
	//			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
	//		else
	//			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
	//	} else
	//
	//  // NOTE: Disabling emission and shininess due to funky sketchup models
	//	{
	//		//		ROS_INFO_STREAM("no shininess");
	//		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
	//		set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
	//		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
	//	}

	//	max = 1;
	//	if (AI_SUCCESS == aiGetMaterialIntegerArray(mtl,
	//			AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
	//		fill_mode = wireframe ? GL_LINE : GL_FILL;
	//	else
	//		fill_mode = GL_FILL;
	//	glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

	max = 1;
	if ((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED,
			&two_sided, &max)) && two_sided) {
		glDisable( GL_CULL_FACE);
	} else {
		glEnable( GL_CULL_FACE);
	}
}

/*
 *	Recursively render a 3D virtual scene
 *
 *	\param 	sc					The ASSIMP scene object
 *	\param 	nd					Node within the scene
 *	\param 	apply_transform		Whether a transform should be applied before rendering this node
 *	\param	scene_num			Index of the scene object
 *
 *	Note: Code adapted from http://assimp.svn.sourceforge.net/viewvc/assimp/trunk/samples/SimpleOpenGL/
 */
void GLManager::recursiveAIRender(const struct aiScene *sc,
		const struct aiNode* nd, bool apply_transform, int scene_num) {
	int i;
	unsigned int n = 0, t;
	//
	if (apply_transform) {
		// Update transform
		struct aiMatrix4x4 m = nd->mTransformation;
		aiTransposeMatrix4(&m);
		glPushMatrix();
		glMultMatrixf((float*) &m);
	}

	// Draw all meshes assigned to this node
	for (; n < nd->mNumMeshes; ++n) {
		const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

		if (n < texturesAndPaths[scene_num].size()) {
			glEnable( GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D,
					texturesAndPaths[scene_num][n].hTexture);
		} else {
			glDisable( GL_TEXTURE_2D);
		}

		apply_material(sc->mMaterials[mesh->mMaterialIndex]);

		if (mesh->mColors[0] != NULL) {
			glEnable( GL_COLOR_MATERIAL);
		} else {
			glDisable( GL_COLOR_MATERIAL);
		}

		for (t = 0; t < mesh->mNumFaces; ++t) {
			const struct aiFace* face = &mesh->mFaces[t];
			GLenum face_mode;
			switch (face->mNumIndices) {
			case 1:
				face_mode = GL_POINTS;
				break;
			case 2:
				face_mode = GL_LINES;
				break;
			case 3:
				face_mode = GL_TRIANGLES;
				break;
			default:
				face_mode = GL_POLYGON;
				break;
			}

			// Draw the mesh
			glBegin(face_mode);
			for (i = 0; i < face->mNumIndices; i++) {
				int index = face->mIndices[i];
				if (mesh->mColors[0] != NULL)
					glColor4fv((GLfloat*) &mesh->mColors[0][index]);
				if (mesh->mNormals != NULL)
					glNormal3fv(&mesh->mNormals[index].x);
				if (mesh->HasTextureCoords(0)) {
					glMultiTexCoord2fARB(GL_TEXTURE0_ARB,
							mesh->mTextureCoords[0][index].x,
							mesh->mTextureCoords[0][index].y);
					glTexCoord2f(mesh->mTextureCoords[0][index].x,
							mesh->mTextureCoords[0][index].y);
				}
				glVertex3fv(&mesh->mVertices[index].x);
			}
			glEnd();
		}
	}

	// Draw all children of this node
	for (n = 0; n < nd->mNumChildren; ++n) {
		recursiveAIRender(sc, nd->mChildren[n], true, scene_num);
	}

	if (apply_transform)
		glPopMatrix();
}



/*
 *	Adjusts a puppet's opacity so it fades away as it leaves the tracking area.
 *
 *	\param 	position			Translation component of a puppet's pose
 */
float GLManager::adjustTransparency(Vector3f position) {
	// Recover horizontal and vertical distances to the edges of the tracking volume
	// Use metric distances from the top and bottom and angular differences from the other sides
	float h_angle = 180 / PI * atan(position[0] / position[2]);
	float v_angle = 180 / PI * fabs(atan(position[1] / position[2]));
	float apply_alpha = 1;

	if (position[2] < near_tracking_region + transparency_fall_off_distance / 2) {
		apply_alpha *= max(
				0.0f,
				(1 - (near_tracking_region + transparency_fall_off_distance / 2
						- position[2]) / (transparency_fall_off_distance)));
	}

	float h_extent = h_tracking_region / 2;
	if (h_angle > 0)
		h_extent *= tracking_asymmetry;
	h_angle = fabs(h_angle);
	if (h_angle > (h_extent - transparency_fall_off_angle / 2)) {
		apply_alpha *= max(
				0.0f,
				(1 - (h_angle - (h_extent - transparency_fall_off_angle / 2))
						/ (transparency_fall_off_angle)));
	}

	if (v_angle > (v_tracking_region / 2 - transparency_fall_off_angle / 2)) {
		apply_alpha *= max(
				0.0f,
				(1 - (v_angle - (v_tracking_region / 2
						- transparency_fall_off_angle / 2))
						/ (transparency_fall_off_angle)));
	}

	glBlendColor(1.0f, 1.0f, 1.0f, apply_alpha);
	return apply_alpha;
}

/*
 *	Read the image that will be painted on the virtual performance surface.
 */
void GLManager::setBackgroundImage(std::string background_image) {
	bkgd_img = imread(background_image.c_str(), 1);
	if (bkgd_img.data == NULL) {
		ROS_WARN("Cannot load file background-image file.");
	}
	bkgd_loaded = true;
}

/*
 *	Bind the background image to a texture.
 */
void GLManager::loadBackgroundImage() {
	glMatrixMode( GL_MODELVIEW);
	glGenTextures(1, &bkgd_texture);
	glBindTexture(GL_TEXTURE_2D, bkgd_texture);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, bkgd_img.cols, bkgd_img.rows, 0,
		GL_BGR_EXT, GL_UNSIGNED_BYTE, bkgd_img.data);
}


/*
 *	Define the physical volume within which physical puppets can be tracked
 */
void GLManager::setTrackingVolume() {
	// Calculate distance from center of camera to center of ground plane
	Vector3f base_to_cam = -back_base;
	//		back_distance = base_to_cam.norm()*base_to_cam.dot(back_normal)/(base_to_cam.norm()*base_to_cam.norm());
	back_distance = -base_to_cam.dot(back_normal.normalized()) / (Vector3f(0,
			0, 1).dot(back_normal.normalized()));

	// Calculate vertices of ground plane, give defined field of view.
	Vector3f upper_left(-tan(v_FOV * 640.0 / 480.0 / 2.0f * PI / 180),
			tan(v_FOV / 2.0f * PI / 180), 1);
	Vector3f upper_right(tan(v_FOV * 640.0 / 480.0 / 2.0f * PI / 180),
			tan(v_FOV / 2.0f * PI / 180), 1);
	Vector3f lower_right(tan(v_FOV * 640.0 / 480.0 / 2.0f * PI / 180),
			-tan(v_FOV / 2.0f * PI / 180), 1);
	Vector3f lower_left(-tan(v_FOV * 640.0 / 480.0 / 2.0f * PI / 180),
			-tan(v_FOV / 2.0f * PI / 180), 1);
	Vector3f upper_left_track(-tan(h_tracking_region / 2.0f * PI / 180),
			tan(v_tracking_region / 2.0f * PI / 180), 1);
	Vector3f upper_right_track(
			tan(h_tracking_region * tracking_asymmetry / 2.0f * PI / 180),
			tan(v_tracking_region / 2.0f * PI / 180), 1);
	Vector3f lower_right_track(
			tan(h_tracking_region * tracking_asymmetry / 2.0f * PI / 180),
			-tan(v_tracking_region / 2.0f * PI / 180), 1);
	Vector3f lower_left_track(-tan(h_tracking_region / 2.0f * PI / 180),
			-tan(v_tracking_region / 2.0f * PI / 180), 1);

	Vector3f ul_close_track(
			-near_tracking_region * tan(h_tracking_region / 2.0f * PI / 180),
			near_tracking_region * tan(v_tracking_region / 2.0f * PI / 180),
			near_tracking_region);
	Vector3f ur_close_track(
			near_tracking_region * tan(
					h_tracking_region * tracking_asymmetry / 2.0f * PI / 180),
			near_tracking_region * tan(v_tracking_region / 2.0f * PI / 180),
			near_tracking_region);
	Vector3f lr_close_track(
			near_tracking_region * tan(
					h_tracking_region * tracking_asymmetry / 2.0f * PI / 180),
			-near_tracking_region * tan(v_tracking_region / 2.0f * PI / 180),
			near_tracking_region);
	Vector3f ll_close_track(
			-near_tracking_region * tan(h_tracking_region / 2.0f * PI / 180),
			-near_tracking_region * tan(v_tracking_region / 2.0f * PI / 180),
			near_tracking_region);

	far_track_plane_edges.clear();
	close_track_plane_edges.clear();
	extra_clipping_planes.clear();

	far_track_plane_edges.push_back(
			(back_base).dot(back_normal) / (lower_left_track.dot(back_normal))
					* lower_left_track);
	far_track_plane_edges.push_back(
			(back_base).dot(back_normal) / (upper_left_track.dot(back_normal))
					* upper_left_track);
	far_track_plane_edges.push_back(
			(back_base).dot(back_normal) / (upper_right_track.dot(back_normal))
					* upper_right_track);
	far_track_plane_edges.push_back(
			(back_base).dot(back_normal) / (lower_right_track.dot(back_normal))
					* lower_right_track);

	close_track_plane_edges.push_back(ll_close_track);
	close_track_plane_edges.push_back(ul_close_track);
	close_track_plane_edges.push_back(ur_close_track);
	close_track_plane_edges.push_back(lr_close_track);

	// Find equations for clipping planes
	// (Outdated. Used before we implemented semi-transparency near the edges)
	GLdouble* clip_plane_0 = calculateClippingPlane(far_track_plane_edges[1],
			close_track_plane_edges[1], close_track_plane_edges[0]);
	GLdouble* clip_plane_1 = calculateClippingPlane(far_track_plane_edges[2],
			close_track_plane_edges[2], close_track_plane_edges[1]);
	GLdouble* clip_plane_2 = calculateClippingPlane(far_track_plane_edges[3],
			close_track_plane_edges[3], close_track_plane_edges[2]);
	GLdouble* clip_plane_3 = calculateClippingPlane(far_track_plane_edges[0],
			close_track_plane_edges[0], close_track_plane_edges[3]);
	GLdouble* clip_plane_4 = calculateClippingPlane(
			-close_track_plane_edges[0], -close_track_plane_edges[1],
			-close_track_plane_edges[2]);
	extra_clipping_planes.push_back(clip_plane_0);
	extra_clipping_planes.push_back(clip_plane_1);
	extra_clipping_planes.push_back(clip_plane_2);
	extra_clipping_planes.push_back(clip_plane_3);
	extra_clipping_planes.push_back(clip_plane_4);
}

/*
 *	Define the position and orientation of the performance surface.
 *	Based on data from the Kinect.
 */
void GLManager::setGroundPlane() {
	// Calculate distance from center of camera to center of ground plane
	Vector3f base_to_cam = -back_base;
	back_distance = -base_to_cam.dot(back_normal.normalized()) / (Vector3f(0,
			0, 1).dot(back_normal.normalized()));

	// Calculate vertices of ground plane, give defined field of view.
	Vector3f upper_left(-tan(v_FOV * 640.0 / 480.0 / 2.0f * PI / 180),
			tan(v_FOV / 2.0f * PI / 180), 1);
	Vector3f upper_right(tan(v_FOV * 640.0 / 480.0 / 2.0f * PI / 180),
			tan(v_FOV / 2.0f * PI / 180), 1);
	Vector3f lower_right(tan(v_FOV * 640.0 / 480.0 / 2.0f * PI / 180),
			-tan(v_FOV / 2.0f * PI / 180), 1);
	Vector3f lower_left(-tan(v_FOV * 640.0 / 480.0 / 2.0f * PI / 180),
			-tan(v_FOV / 2.0f * PI / 180), 1);

	ground_plane_edges.clear();
	ground_plane_edges.push_back(
			(back_base).dot(back_normal) / (lower_left.dot(back_normal))
					* lower_left + (Vector3f(0, 0, back_distance) - back_base));
	ground_plane_edges.push_back(
			(back_base).dot(back_normal) / (lower_right.dot(back_normal))
					* lower_right + (Vector3f(0, 0, back_distance) - back_base));

	Vector3f bottom_vector = ground_plane_edges[1] - ground_plane_edges[0];
	Vector3f edge = back_normal.cross(bottom_vector);
	edge = -edge.normalized();
	edge *= sqrt(bottom_vector.dot(bottom_vector)) * 3.0 / 4.0;
	ground_plane_edges.push_back(ground_plane_edges[1] + edge);
	ground_plane_edges.push_back(ground_plane_edges[0] + edge);

	// Find rotation to align virtual scenes with ground plane
	Vector3f rot_axis = back_normal.cross(Vector3f(0, 0, -1));
	rot_axis = rot_axis.normalized();
	float angle = -acos(back_normal.normalized().dot(Vector3f(0, 0, -1)));
	bkgd_quaternion = AngleAxis<float> (angle, rot_axis);

	// Find vectors to be used for moving background
	if (back_normal.normalized() == Vector3f(0, 0, -1)) {
		right_vector = Vector3f(1, 0, 0);
		up_vector = Vector3f(0, 1, 0);
	} else {
		right_vector = back_normal.normalized().cross(Vector3f(0, 0, -1));
		up_vector = back_normal.normalized().cross(right_vector);
	}

	ROS_INFO_STREAM("   Right vector: (" << right_vector[0] << ", " << right_vector[1]
					<< ", " << right_vector[2] << ")");
	ROS_INFO_STREAM("   Up vector: (" << up_vector[0] << ", " << up_vector[1] << ", "
					<< up_vector[2] << ")");
}

///////////////////////////////////////////////////////////////////////////////
// Animation / video functions
///////////////////////////////////////////////////////////////////////////////

/*
 *	Set up video file to store frames of rendered animation
 *
 *	\param 	files_path	Folder for saving the video
 *	\param 	v_name		Base name of the animation
 */
void GLManager::setupVideoRecorder(std::string files_path, std::string v_name) {
	video_out_name = v_name;
	record_video = true;
	output_video = files_path;
	output_video.append("animations/");
	output_video.append(video_out_name);
	output_video.append("Rendered.avi");
}

/*
 *	Grab a screenshot from the OpenGL window
 *
 *	\param 	img0	Data structure for storing the image
 */
void GLManager::getScreenShot(Mat *imgO) {
	//	ROS_INFO_STREAM("Grabbin'");
	int width = render_width;
	int height = render_height;
	int hIndex = 0;
	int wIndex = 0;
	int iout;

	glReadBuffer( GL_FRONT);
	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, imageData);

	for (int i = 0; i < width * height * 3; i += 3, wIndex++) {
		if (wIndex >= width) {
			wIndex = 0;
			hIndex++;
		}

		iout = -hIndex + height - 1;

		((uchar *) (imgO->data))[iout * imgO->step + wIndex * 3 + 0]
				= imageData[i + 2]; // B
		((uchar *) (imgO->data))[iout * imgO->step + wIndex * 3 + 1]
				= imageData[i + 1]; // G
		((uchar *) (imgO->data))[iout * imgO->step + wIndex * 3 + 2]
				= imageData[i]; // R
	}
}


/*
 *	Set up text file that will store the frame-by-frame poses of each puppet in
 *	the animation.
 *
 *	\param 	files_path		Folder for saving the animation
 *	\param 	pose_out_name	Base name of the animation
 */
void GLManager::setupPoseRecorder(std::string files_path,std::string pose_out_name) {
	record_pose = true;
	output_pose = files_path;
	output_pose.append("animations/");
	output_pose.append(pose_out_name);
	output_pose.append(".txt");
}

/*
 * Save the visibility and pose states of each puppet from the latest frame
 */
void GLManager::savePoses() {
	//	ROS_INFO_STREAM("POSE CAPTURED");
	gettimeofday(&time_grabber, NULL);
	double save_time = time_grabber.tv_sec + (time_grabber.tv_usec / 1000000.0);
	if (captureNow) {
		timestamps.push_back(save_time - record_initial_time + accumulated_time);
		//				ROS_INFO_STREAM(new_time - record_initial_time + accumulated_time);
		absolute_timestamps.push_back(save_time);
	} else if (captureTwoNow) {
		timestamps_two.push_back(
				save_time - record_initial_time_two + accumulated_time_two);
		//				ROS_INFO_STREAM(new_time - record_initial_time_two + accumulated_time_two);
		absolute_timestamps_two.push_back(save_time);
	}

	//	ROS_INFO_STREAM("New time: " << new_time << " difference: " << new_time - record_initial_time);
	if (captureNow) {
		for (int m = 0; m < num_puppets; m++) {
			visible_states[m].push_back(Puppets[m]->visible);
			if (Puppets[m]->visible)
				in_first_layer[m] = 1;
			if (Puppets[m]->getCurrentPoseIndex() > -1) {

				saved_poses[m].push_back(*(Puppets[m]->getCurrentPose()));
			} else {
				vector<float> dummy_pose(16, 0);
				saved_poses[m].push_back(dummy_pose);
			}
		}
	} else if (captureTwoNow) {
		for (int m = 0; m < num_puppets; m++) {
			//			ROS_INFO_STREAM(in_first_layer[m]);
			if (!in_first_layer[m]) {
				visible_states_two[m].push_back(Puppets[m]->visible);
				//				ROS_INFO_STREAM(Puppets[m]->visible);
				if (Puppets[m]->getCurrentPoseIndex() > -1) {
					//					ROS_INFO_STREAM("FOR SECOND LAYER");
					saved_poses_two[m].push_back(*(Puppets[m]->getCurrentPose()));
				} else {
					vector<float> dummy_pose(16, 0);
					saved_poses_two[m].push_back(dummy_pose);
				}
			}
		}
	}
}

/*
 * Assuming an animation has been completed, print out all the puppets' visibility
 * and pose states, frame by frame.
 */
void GLManager::printPoses() {
	ROS_INFO_STREAM("Saving poses.");
	pose_stream_out.open(output_pose.c_str());
	int total_frames = timestamps.size();
	int total_frames_two = timestamps_two.size();
	pose_stream_out << "frames\t" << total_frames << endl;
	pose_stream_out << "frames_layer_two\t" << total_frames_two << endl;
	pose_stream_out << "models\t" << num_puppets << endl;
	pose_stream_out << "back_base\t" << back_base[0] << "\t" << back_base[1]
			<< "\t" << back_base[2] << "\t" << endl;
	pose_stream_out << "back_normal\t" << back_normal[0] << "\t"
			<< back_normal[1] << "\t" << back_normal[2] << "\t" << endl;
	pose_stream_out << "back_adj\t" << back_adj[0] << "\t" << back_adj[1]
			<< "\t" << back_adj[2] << "\t" << endl;
	pose_stream_out << "in_first_layer";
	for (int m = 0; m < num_puppets; m++) {
		pose_stream_out << "\t" << in_first_layer[m];
	}
	pose_stream_out << endl;

	pose_stream_out << "light_quaternion\t";
	pose_stream_out << light_quaternion.w() << "\t";
	pose_stream_out << light_quaternion.x() << "\t";
	pose_stream_out << light_quaternion.y() << "\t";
	pose_stream_out << light_quaternion.z() << endl;

	pose_stream_out << "end_header" << endl;

	for (int f = 0; f < total_frames; f++) {
		pose_stream_out << timestamps[f] << endl;
	}
	for (int f = 0; f < total_frames_two; f++) {
		pose_stream_out << timestamps_two[f] << endl;
	}
	for (int m = 0; m < num_puppets; m++) {
		//		pose_stream_out << "visible:" << endl;
		if (in_first_layer[m]) {
			for (int f = 0; f < total_frames; f++) {
				pose_stream_out << visible_states[m][f] << endl;
			}
			//		pose_stream_out << "poses:" << endl;
			for (int f = 0; f < total_frames; f++) {
				for (int c = 0; c < 16; c++) {
					pose_stream_out << saved_poses[m][f][c] << "\t";
				}
				pose_stream_out << endl;
			}
		} else {
			for (int f = 0; f < total_frames_two; f++) {
				pose_stream_out << visible_states_two[m][f] << endl;
			}
			//		pose_stream_out << "poses:" << endl;
			for (int f = 0; f < total_frames_two; f++) {
				for (int c = 0; c < 16; c++) {
					pose_stream_out << saved_poses_two[m][f][c] << "\t";
				}
				pose_stream_out << endl;
			}
		}
	}
	pose_stream_out.close();
}


/*
 *	Prepare to load a previously saved animation.
 *
 *	\param 	files_path		Folder for saving the animation
 *	\param 	pose_in_name	Base name of the animation
 */
void GLManager::enablePoseLoading(std::string files_path,
		std::string pose_in_name) {
	performance_in_name = pose_in_name;
	input_pose = files_path;
	input_pose.append("animations/");
	input_pose.append(pose_in_name);
	input_pose.append(".txt");
	if (fs::exists(input_pose.c_str())) {
		load_poses = true;
	} else {
		ROS_ERROR("Animation file could not be found");
		clear_poses = true;
	}
}

/*
 *	Parse a text file to load all the pose and visibility data for
 *	each puppet from a previously saved animation.
 */
void GLManager::loadPoses() {
	ifstream inFile(input_pose.c_str(), ifstream::in);

	ROS_INFO_STREAM("Loading poses from " << input_pose);

	timestamps_loaded = 0;
	timestamps_loaded_layer_two = 0;
	poses_loaded = 0;
	puppets_loaded = 0;
	visible_states_loaded = 0;
	frames_to_load_layer_two = 0;
	frames_to_load = 0;
	past_header = false;
	loaded_layer_states = false;

	// Parse each line in order
	char line[1024];
	while (inFile.good()) {
		inFile.getline(line, 1024);
		parsePoseFileLine(string(line));
	}

	// Make sure visibility vectors are appropriately filled
	for (uint m = 0; m < num_puppets; m++) {
		if (fill_empty_visibility[m]) {
			for (uint i = 0; i < frames_to_load; i++)
				visible_states[m].push_back(0);
		}
		if (fill_empty_visibility_two[m]) {
			for (uint i = 0; i < frames_to_load_layer_two; i++)
				visible_states_two[m].push_back(0);
		}
	}
	inFile.close();
}

/*
 * Parse the information from a single line in a saved animation text file
 */
bool GLManager::parsePoseFileLine(string line) {
	double temp_stamp;
	float temp_float;
	float pose_entry;
	int visibility;

	string op; // Used to store the first entry in the line
	if (line.empty())
		return true;
	stringstream ss(stringstream::in | stringstream::out);
	ss.str(line);
	ss >> op;

	if ((op[0] == '#') || (op.compare("comment") == 0)) {
		// # indicates a commented line.  No information to parse
		return true;
	} else if (op.compare("frames") == 0) {
		ss >> frames_to_load;
	} else if (op.compare("frames_layer_two") == 0) {
		ss >> frames_to_load_layer_two;
	} else if (op.compare("models") == 0) {
		ss >> puppets_to_load;
		// Default is for all puppets to be in the first layer
		for (int m = 0; m < puppets_to_load; m++) {
			in_first_layer[m] = 1;
			fill_empty_visibility.push_back(1);
			fill_empty_visibility_two.push_back(1);
		}
	} else if (op.compare("back_base") == 0) {
		for (uint i = 0; i < 3; i++) {
			ss >> op;
			const char * get_float = op.c_str();
			temp_float = atof(get_float);
			back_base[i] = temp_float;
		}
		ROS_INFO_STREAM(
				"Loaded background base: (" << back_base[0] << ", "
						<< back_base[1] << ", " << back_base[2] << ")");
		background_info_loaded++;

	} else if (op.compare("back_normal") == 0) {
		for (uint i = 0; i < 3; i++) {
			ss >> op;
			const char * get_float = op.c_str();
			temp_float = atof(get_float);
			back_normal[i] = temp_float;
		}
		ROS_INFO_STREAM(
				"Loaded background normal: (" << back_normal[0] << ", "
						<< back_normal[1] << ", " << back_normal[2] << ")");
		background_info_loaded++;
	} else if (op.compare("back_adj") == 0) {
		for (uint i = 0; i < 3; i++) {
			ss >> op;
			const char * get_float = op.c_str();
			temp_float = atof(get_float);
			back_adj[i] = temp_float;
		}
		ROS_INFO_STREAM(
				"Loaded background adjustment: (" << back_adj[0] << ", "
						<< back_adj[1] << ", " << back_adj[2] << ")");
		background_info_loaded++;
	} else if (op.compare("in_first_layer") == 0) {
		for (int m = 0; m < puppets_to_load; m++) {
			ss >> in_first_layer[m];
//			ROS_INFO_STREAM("In first layer: " << in_first_layer[m]);
			loaded_layer_states = true;
		}
	} else if (op.compare("light_quaternion") == 0) {
		vector<float> quat_temp;
		for (uint i = 0; i < 4; i++) {
			ss >> op;
			const char * get_float = op.c_str();
			temp_float = atof(get_float);
			quat_temp.push_back(temp_float);
		}
		light_quaternion = Quaternion<float> (quat_temp[0], quat_temp[1],
				quat_temp[2], quat_temp[3]);

		ROS_INFO_STREAM(
				"Loaded light quaternion: (" << light_quaternion.w() << ", "
						<< light_quaternion.x() << ", " << light_quaternion.y()
						<< ", " << light_quaternion.z() << ")");
	} else if (op.compare("end_header") == 0) {
		// Done with the header
		past_header = true;
	} else {
		if (past_header) {
			if (timestamps_loaded < frames_to_load) {
				// Load timestamp
				const char * get_time = op.c_str();
				temp_stamp = atof(get_time);
				timestamps.push_back(temp_stamp);
				timestamps_loaded++;
			} else if (timestamps_loaded_layer_two < frames_to_load_layer_two) {
				// Load timestamp
				const char * get_time = op.c_str();
				temp_stamp = atof(get_time);
				timestamps_two.push_back(temp_stamp);
				timestamps_loaded_layer_two++;
			} else if (puppets_loaded < puppets_to_load) {
				if ((poses_loaded == 0) && (puppets_loaded == 0)) {
					while ((puppet_frames_to_load == 0) && (puppets_loaded
							< puppets_to_load)) {
						if (in_first_layer[puppets_loaded]) {
							puppet_frames_to_load = frames_to_load;
							if (puppet_frames_to_load != 0)
								fill_empty_visibility[puppets_loaded] = 0;
						} else {
							puppet_frames_to_load = frames_to_load_layer_two;
							if (puppet_frames_to_load != 0)
								fill_empty_visibility_two[puppets_loaded] = 0;
						}
						if (puppet_frames_to_load == 0)
							puppets_loaded++;
					}
					//					ROS_INFO_STREAM("Frames to load: " << frames_to_load);
				}
				if (visible_states_loaded < puppet_frames_to_load) {
					// Load puppet's visibility state
					const char * get_visibility = op.c_str();
					visibility = atoi(get_visibility);
					if (in_first_layer[puppets_loaded]) {
						visible_states[puppets_loaded].push_back(visibility);
					} else {
						visible_states_two[puppets_loaded].push_back(visibility);
					}
					visible_states_loaded++;
					//					saved_poses[m].push_back(*(Puppets[m]->getCurrentPose()));
				} else if (poses_loaded < puppet_frames_to_load) {
					// Load puppet's pose
					vector<float> temp_pose;
					const char * get_entry = op.c_str();
					pose_entry = atof(get_entry);
					temp_pose.push_back(pose_entry);
					//					cout << temp_pose[0] << " ";
					for (int i = 1; i < 16; i++) {
						ss >> pose_entry;
						temp_pose.push_back(pose_entry);
						//						cout << temp_pose[i] << " ";
					}
					//					cout << endl;
					if (in_first_layer[puppets_loaded]) {
						saved_poses[puppets_loaded].push_back(temp_pose);
					} else {
						saved_poses_two[puppets_loaded].push_back(temp_pose);
					}
					poses_loaded++;

					if (poses_loaded == puppet_frames_to_load) {
						ROS_INFO_STREAM(
								"Puppet: " << puppets_loaded
										<< " loaded frames: " << poses_loaded);
						visible_states_loaded = 0;
						poses_loaded = 0;
						puppet_frames_to_load = 0;
						puppets_loaded++;
						while ((puppet_frames_to_load == 0) && (puppets_loaded
								< puppets_to_load)) {
							if (in_first_layer[puppets_loaded]) {
								puppet_frames_to_load = frames_to_load;
								if (puppet_frames_to_load != 0) {
//									ROS_INFO_STREAM("Puppet: " << puppets_loaded << " layer 1 " << puppet_frames_to_load);
									fill_empty_visibility[puppets_loaded] = 0;
								}
							} else {
								puppet_frames_to_load = frames_to_load_layer_two;
								if (puppet_frames_to_load != 0) {
									//									ROS_INFO_STREAM("Puppet: " << puppets_loaded << " layer 2 " << puppet_frames_to_load);
									fill_empty_visibility_two[puppets_loaded]
											= 0;
								}
							}
							if (puppet_frames_to_load == 0) {
								puppets_loaded++;
//								ROS_INFO_STREAM("Puppet: " << puppets_loaded << " " << puppet_frames_to_load);
							}
						}
					}
				}
			}
		}
	}

	num_playback_puppets = puppets_to_load;
	if (ss.fail())
		return false;
	return true;
}

/*
 * Combine rendered and video feeds
 */
void GLManager::combineVideos() {
	// Determine name of raw video file
	record_video = true;
	int frameW = render_width; // 744 for firewire cameras
	int frameH = render_height; // 480 for firewire cameras
	std::string output_raw_video;
	output_raw_video = files_path;
	output_raw_video.append("animations/");
	output_raw_video.append(video_out_name);
	output_raw_video.append("RawVideo.avi");

	// Need to re-save rendered video to make sure the frame rate is correct
	// Create a new version of the rendered video
	std::string output_paced_video;
	output_paced_video = files_path;
	output_paced_video.append("animations/");
	output_paced_video.append(video_out_name);
	//	output_raw_video.append("ProcessedVideo.avi");
	output_paced_video.append("RenderedFinal.avi");

	int pip_width	= render_width / 6;
	int pip_height 	= render_height / 6;
	int pip_border 	= render_width / 50;
	ROS_INFO_STREAM("sizes: " << pip_width << " " << pip_height << " " << render_width);

	if (load_poses) {
		// Need to RE-save raw video
		std::string old_video = files_path;
		old_video.append(performance_in_name);
		old_video.append("RawVideo.avi");
		if (old_video.compare(output_raw_video) != 0) {
			ROS_WARN("Using loaded raw video file.");
			output_raw_video = old_video;
		}
	}

	// Create name for video that combines the rendered video and the raw input
	std::string combo_out;
	combo_out = files_path;
	combo_out.append("animations/");
	combo_out.append(video_out_name);
	combo_out.append("Combined.avi");

	// Create an image structure for combining the renderings and raw frames
	Mat combo_image(frameH, frameW, CV_8UC3);
	Mat img_video(480, 640, CV_8UC3);
	Mat img_video_downsized(pip_height, pip_width, CV_8UC3);
	Mat img_render(render_height, render_width, CV_8UC3);
	namedWindow("Combining Videos");

	// Create OpenCV video-capture and -writing structures
	VideoCapture capture_video(output_raw_video.c_str());
	VideoCapture capture_render(output_video.c_str());
	VideoWriter final_render_writer(output_paced_video.c_str(), CV_FOURCC('M', 'P', '4', '2'),
				capture_fps, cvSize(render_width, frameH), 1);
	VideoWriter combo_writer(combo_out.c_str(), CV_FOURCC('M', 'P', '4', '2'),
			capture_fps, cvSize(frameW, frameH), 1);

	uint output_frames = 0;
	uint render_frame = 0;
	uint raw_frame = 0;
	double time_difference;

	// Process video one frame at a time, ensuring that the rendered and raw frames
	// are in sync.
	while (capture_video.grab() && capture_render.grab()) {
		time_difference = absolute_exported_timestamps[render_frame]
				- video_timestamps[raw_frame];
		//				ROS_INFO_STREAM("Difference: " << time_difference);

		if (time_difference < -0.05) {
			render_frame++;
			while (capture_render.grab() && (time_difference < -0.05)
					&& (time_difference > -50) && (render_frame
					< absolute_exported_timestamps.size())) {
				capture_render.grab();
				render_frame++;
				time_difference = absolute_exported_timestamps[render_frame]
						- video_timestamps[raw_frame];
			}
		} else if (time_difference > 0.05) {
			raw_frame++;
			while (capture_video.grab() && (time_difference > 0.05)
					&& (time_difference < 50) && (raw_frame
					< video_timestamps.size())) {
				capture_video.grab();
				raw_frame++;
				time_difference = absolute_exported_timestamps[render_frame]
						- video_timestamps[raw_frame];
			}
		}
		// Combine images
		capture_video.retrieve(img_video);
		resize(img_video, img_video_downsized, img_video_downsized.size(), 0,
				0, INTER_CUBIC);
		capture_render.retrieve(img_render);
		Mat render_roi = combo_image(Rect(0, 0, render_width, render_height));
		img_render.copyTo(render_roi);
		Mat video_roi = combo_image(Rect(render_width - pip_width - pip_border, pip_border,
						pip_width, pip_height));
		//			Mat video_roi = combo_image(Rect(render_width-pip_width-pip_border, render_height-pip_height-pip_border, pip_width, pip_height));
		//			Mat video_roi = combo_image(Rect(pip_border, pip_border, pip_width, pip_height));
		img_video_downsized.copyTo(video_roi);

		// Make sure the timing is still correct
		while ((output_frames <= render_frame)) {
			imshow("Combining Videos", combo_image);
			combo_writer << combo_image;
			final_render_writer << img_render;
			output_frames++;
		}
		render_frame++;
		raw_frame++;
	}
	destroyWindow("Combining Videos");
}


/*
 * Grab the current pose of the tracked puppet and export it as part of
 * a SIFT-matching template
 */
void GLManager::getSIFTSet() {
	char num[50];
	sprintf(num, "%d", SIFT_it);
	SIFT_save_name_numbered = SIFT_save_name;
	SIFT_save_name_numbered.append(num);

	std::string SIFT_pose = SIFT_save_name_numbered;
	SIFT_pose.append("_pose.yml");

	vector<float>* current_pose = Puppets[current_puppet]->getCurrentPose();

	cv::FileStorage fs(SIFT_pose.c_str(), cv::FileStorage::WRITE);
	fs << "pose" << "[:";
	for (uint i = 0; i < 16; i++) {
		fs << current_pose->at(i);
	}
	fs.release();
	SIFT_it++;
}


///////////////////////////////////////////////////////////////////////////////
// User input
///////////////////////////////////////////////////////////////////////////////

/*
 * Handle standard key presses
 */
void GLManager::myKeyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 98:
		// (b) change to next background
		if (current_scene == num_scenes - 1) {
			current_scene = -1; // Checkerboard background
		} else {
			current_scene++;
		}

		break;

	case 99:
		// (c)
		break;

	case 100:
		// (d) Toggle rendering of shadows
		render_shadows = !render_shadows;
		//		if (control_mode == LIGHTING_CONTROL)
		//		{
		//			directional_light = !directional_light;
		//			if (directional_light)
		//				ROS_INFO_STREAM("Switched to directional light.");
		//			else
		//				ROS_INFO_STREAM("Switched to position light.");
		//		}
		break;

	case 67:
		// (C) (shift-c) change control mode to camera
		control_mode = CAMERA_CONTROL;
		ROS_INFO_STREAM("Control switched to virtual camera.");
		break;

	case 66:
		// (B) (shift-b) change control mode to background
		control_mode = BACKGROUND_CONTROL;
		ROS_INFO_STREAM("Control switched to background positioning.");
		break;

	case 76:
		// (L) (shift-l) change control mode to lighting
		control_mode = LIGHTING_CONTROL;
		ROS_INFO_STREAM("Control switched to lighting.");
		break;

	case 112:
		// (p) start / stop playing recorded poses
		if (!captureNow) {
			playNow = !playNow;
			if (playNow) {
				num_puppets = num_playback_puppets;
				play_frame = 0;
				if (timestamps_two.size() > 0) {
					play_frame_two = 0;
					play_all_layers = true;
				} else {
					play_all_layers = false;
				}
			} else {
				num_puppets = num_available_puppets;
			}
		}
		break;

	case 80:
		// (P) (shift-p)
		break;

	case 114:
		if (!playback_only)
		{
			// (r) Start / stop recording frames
			if (!load_poses) {
				if (!playNow && !captureTwoNow) {
					captureNow = !captureNow;
					if (captureNow) {
						num_playback_puppets = num_available_puppets;
						need_to_save = true;
						started_capture = false;
						if (timestamps.size() > 0) {
							accumulated_time = timestamps.back();
						}
						if (!record_pose) {
							ROS_INFO_STREAM("No pose recorded - no name provided.");
						}
					}
				}
			} else {
				ROS_WARN("You must clear the loaded performance before recording a new one.");
			}
		} else {
			ROS_WARN("You are in playback mode.");
		}
		break;

	case 82:
		if (!playback_only)
		{
			// (R) Start / stop recording 2nd-layer frames
			if (!load_poses) {
				if (!playNow && !captureNow) {
					captureTwoNow = !captureTwoNow;

					if (captureTwoNow) {
						//					ROS_INFO_STREAM("Start capture two");
						play_frame = 0;
						num_puppets = num_playback_puppets;
						num_playback_puppets = num_available_puppets;
						need_to_save = true;
						started_capture = false;
						layer_two_synced = false;
						if (timestamps_two.size() > 0) {
							accumulated_time_two = timestamps_two.back();
						}
						if (!record_pose) {
							ROS_INFO_STREAM("No pose recorded - no name provided.");
						}
					}
				}
			} else {
				ROS_WARN("You must clear the loaded performance before recording a new one.");
			}
		} else {
			ROS_WARN("You are in playback mode.");
		}
		break;

	case 101:
		// (e) Export a video
		if (!captureNow) {
			// Setup video recorder
			absolute_exported_timestamps.clear();
			imageData = (unsigned char*) malloc(render_width * render_height * 3);
			capturedImg = new Mat(render_height, render_width, CV_8UC3);
			img_mat = new Mat(render_height, render_width, CV_8UC3);
			writer = VideoWriter(output_video.c_str(),
					CV_FOURCC('M', 'P', '4', '2'), capture_fps,
					cvSize(render_width, render_height), 1);
			// ...and export opengl images to video
			export_to_video = true;
			play_all_layers = true;
			total_exported_frames = 0;
			new_frame_accessed = true;
			new_frame_accessed_two = true;
			playNow = true;
			play_frame = 0;
			play_frame_two = 0;
		}
		break;

	case 75:
		// (K (shift-k)) If some of the tracked puppet's vertices do not have assigned colors,
		// assign them based on the nearest colored vertex.
		if (SIFT_saving) {
			if (!playback_only)
			{
				pthread_mutex_trylock(&amutex);
				pthread_cond_wait(&cond, &amutex);
			}
			spread_colors = true;
			if (!playback_only)
				pthread_mutex_unlock(&amutex);
		} else {
			ROS_WARN("Color spread only available in SIFT-scanning mode.");
		}
		break;

	case 107:
		// (k) Transfer colors from kinect cloud to the vertices of the tracked puppet
		if (SIFT_saving) {
			ROS_INFO_STREAM("Transferring colors from kinect cloud to stored puppet.");
			if (!playback_only)
			{
				pthread_mutex_trylock(&amutex);
				pthread_cond_wait(&cond, &amutex);
			}
			transfer_colors = true;
			if (!playback_only)
				pthread_mutex_unlock(&amutex);
		} else {
			ROS_WARN("Color transfer only available in SIFT-scanning mode.");
		}
		break;

	case 108:
		// (l) Save a version of the tracked puppet with its new coloring
		if (SIFT_saving) {
			if (!playback_only)
			{
				pthread_mutex_trylock(&amutex);
				pthread_cond_wait(&cond, &amutex);
			}
			save_new_ply = true;
			if (!playback_only)
				pthread_mutex_unlock(&amutex);
		} else {
			ROS_WARN("Color saving only available in SIFT-scanning mode.");
			ROS_WARN("Use L (shift-l) to enable lighting control.");
		}
		break;

	case 115:
		// (s) capture SIFT image/depth/pose set
		if (SIFT_saving) {
			SIFT_save_now = true;
			ROS_INFO_STREAM("Saving SIFT set.");
		}
		break;

	case 120:
		// (x) Manually restart puppet tracking
		if (!playback_only)
		{
			pthread_mutex_trylock(&amutex);
			pthread_cond_wait(&cond, &amutex);
		}
		for (int i = 0; i < num_puppets; i++) {
			Puppets[i]->lost = true;
			Puppets[i]->visible = false;
			Puppets[i]->lost_counter = 1;
			Puppets[i]->all_iterations.clear();
			Puppets[i]->icp_it = -1;
		}
		if (!playback_only)
			pthread_mutex_unlock(&amutex);
		break;

	case 116:
		// (t) Toggle tracking-region visualization
		if (tracking_region_visible < 2) {
			tracking_region_visible++;
		} else {
			tracking_region_visible = 0;
		}
		break;

	case 95:
		if (!playback_only)
		{
			// (Shift-(-)) Delete saved poses and videos
			if (!captureNow && !playNow && !captureTwoNow) {
				timestamps.clear();
				timestamps_two.clear();
				absolute_timestamps.clear();
				absolute_timestamps_two.clear();
				accumulated_time = 0;
				accumulated_time_two = 0;
				for (int m = 0; m < num_available_puppets; m++) {
					visible_states[m].clear();
					visible_states_two[m].clear();
					saved_poses[m].clear();
					saved_poses_two[m].clear();
				}
				clear_poses = true;
				load_poses = false;
				for (int m = 0; m < num_available_puppets; m++) {
					in_first_layer[m] = 0;
					Puppets[m]->lost = true;
					Puppets[m]->visible = false;
					Puppets[m]->lost_counter = 1;
					Puppets[m]->all_iterations.clear();
					Puppets[m]->icp_it = -1;
				}

				// If we were using loaded background info, clear it and use the newly captured data
				if (background_info_loaded == 2) {
					back_normal = captured_back_normal;
					back_base = captured_back_base;
					setGroundPlane();
					background_info_loaded = 0;
				}
			}
		}
		break;

	case 119:
		// (w)	Write the current animation to a text file.
		if (record_pose && need_to_save) {
			printPoses();
		}
		break;

	case 27:
		// (esc) Exit the program
		if (!playback_only)
		{
			pthread_mutex_trylock(&amutex);
			pthread_cond_wait(&cond, &amutex);
		}
		cleanup();
		ROS_INFO_STREAM("Exiting OpenGL.");
		exit(0);
		if (!playback_only)
			pthread_mutex_unlock(&amutex);
		break;
	default:
		// Don't do anything
		break;
	}
}

/*
 * Handle special key presses
 */
void GLManager::mySpecialKeyboard(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_UP:
		if (!playback_only)
		{
			pthread_mutex_trylock(&amutex);
			pthread_cond_wait(&cond, &amutex);
		}
		//			for (uint m=0;m<num_puppets;m++)
		//				Puppets[m]->increaseAlpha();
		//		for (uint m = 0; m < num_puppets; m++)
		//			Puppets[m]->increaseRotSigma();
		//		if (Puppets[0]->all_iterations.size() > 0) {
		//			if (Puppets[0]->all_iterations.size() - 1 > Puppets[0]->icp_it) {
		//				Puppets[0]->icp_it++;
		//			} else {
		//				Puppets[0]->icp_it = 0;
		//			}
		//		}
		//		ROS_INFO_STREAM(Puppets[0]->icp_it);

		switch (control_mode) {
		case BACKGROUND_CONTROL:
			back_adj -= up_vector * 0.025;
			setGroundPlane();
			need_to_save = true;
			break;

		case LIGHTING_CONTROL:
//			if (!directional_light) {
			for (uint i=0; i<3; i++)
			{
				light_position[i] -= up_vector[i] * 0.05;
			}
				light_frames_shown = 0;
//			}
			break;

		default:
			// Do nothing
			break;
		}
		if (!playback_only)
			pthread_mutex_unlock(&amutex);

		break;

	case GLUT_KEY_DOWN:
		if (!playback_only)
		{
			pthread_mutex_trylock(&amutex);
			pthread_cond_wait(&cond, &amutex);
		}
		//			for (uint m=0;m<num_puppets;m++)
		//				Puppets[m]->decreaseAlpha();
		//		for (uint m = 0; m < num_puppets; m++)
		//			Puppets[m]->decreaseRotSigma();
		//		if (Puppets[0]->all_iterations.size() > 0) {
		//			if (Puppets[0]->icp_it > 0) {
		//				Puppets[0]->icp_it--;
		//			} else {
		//				Puppets[0]->icp_it = Puppets[0]->all_iterations.size() - 1;
		//			}
		//		}
		//		ROS_INFO_STREAM(Puppets[0]->icp_it);
		switch (control_mode) {
		case BACKGROUND_CONTROL:
			back_adj += up_vector * 0.025;
			setGroundPlane();
			need_to_save = true;
			break;

		case LIGHTING_CONTROL:
//			if (!directional_light) {
			for (uint i=0; i<3; i++)
			{
				light_position[i] += up_vector[i] * 0.05;
			}
				light_frames_shown = 0;
//			}
			break;

		default:
			// Do nothing
			break;
		}
		if (!playback_only)
			pthread_mutex_unlock(&amutex);
		break;

	case GLUT_KEY_LEFT:
		if (!playback_only)
		{
			pthread_mutex_trylock(&amutex);
			pthread_cond_wait(&cond, &amutex);
		}
		//		for (uint m = 0; m < num_puppets; m++)
		//			Puppets[m]->decreaseTimeSigma();
		switch (control_mode) {
		case BACKGROUND_CONTROL:
			back_adj += right_vector * 0.025;
			setGroundPlane();
			need_to_save = true;
			break;

		case LIGHTING_CONTROL:
//			if (!directional_light) {
				for (uint i=0; i<3; i++)
				{
					light_position[i] += right_vector[i] * 0.05;
				}
				light_frames_shown = 0;
//			}
			break;

		default:
			// Do nothing
			break;
		}

		if (!playback_only)
			pthread_mutex_unlock(&amutex);
		break;

	case GLUT_KEY_RIGHT:
		if (!playback_only)
		{
			pthread_mutex_trylock(&amutex);
			pthread_cond_wait(&cond, &amutex);
		}
		//		for (uint m = 0; m < num_puppets; m++)
		//			Puppets[m]->increaseTimeSigma();
		switch (control_mode) {
		case BACKGROUND_CONTROL:
			back_adj -= right_vector * 0.025;
			setGroundPlane();
			need_to_save = true;
			break;

		case LIGHTING_CONTROL:
//			if (!directional_light) {
				for (uint i=0; i<3; i++)
				{
					light_position[i] -= right_vector[i] * 0.05;
				}
				light_frames_shown = 0;
//			}
			break;

		default:
			// Do nothing
			break;
		}
		if (!playback_only)
			pthread_mutex_unlock(&amutex);
		break;

	case GLUT_KEY_PAGE_DOWN:
		if (!playback_only)
		{
			pthread_mutex_trylock(&amutex);
			pthread_cond_wait(&cond, &amutex);
		}
		//		for (uint m = 0; m < num_puppets; m++)
		//			Puppets[m]->decreaseTransSigma();
		switch (control_mode) {
			case BACKGROUND_CONTROL:
				back_adj -= back_normal.normalized() * 0.005;
				setGroundPlane();
				need_to_save = true;
				break;

			case LIGHTING_CONTROL:
	//			if (!directional_light) {
				for (uint i=0; i<3; i++)
				{
					light_position[i] -= (back_normal.normalized())[i] * 0.025;
				}
				light_frames_shown = 0;
	//			}
				break;

			default:
				// Do nothing
				break;
			}
		if (!playback_only)
			pthread_mutex_unlock(&amutex);
		break;

	case GLUT_KEY_PAGE_UP:
		if (!playback_only)
		{
			pthread_mutex_trylock(&amutex);
			pthread_cond_wait(&cond, &amutex);
		}
		//		for (uint m = 0; m < num_puppets; m++)
		//			Puppets[m]->increaseTransSigma();
		switch (control_mode) {
			case BACKGROUND_CONTROL:
				back_adj += back_normal.normalized() * 0.005;
				setGroundPlane();
				need_to_save = true;
				break;

			case LIGHTING_CONTROL:
	//			if (!directional_light) {
				for (uint i=0; i<3; i++)
				{
					light_position[i] += (back_normal.normalized())[i] *0.025;
				}
				light_frames_shown = 0;
	//			}
				break;

			default:
				// Do nothing
				break;

		}
		if (!playback_only)
			pthread_mutex_unlock(&amutex);
		break;

	case GLUT_KEY_F4:
		if (diffuse_term < 1.0)
			diffuse_term += 0.1;
		break;

	case GLUT_KEY_F3:
		if (diffuse_term > 0.0)
			diffuse_term -= 0.1;
		break;

	case GLUT_KEY_F2:
		if (ambient_term < 1.0)
			ambient_term += 0.1;
		break;

	case GLUT_KEY_F1:
		if (ambient_term > 0.0)
			ambient_term -= 0.1;
		break;

	default:
		// Do nothing
		break;
	}
}

/*
 * Handle mouse button presses
 */
void GLManager::myMouseButton(int button, int state, int x, int y) {
	if (button == MOUSE_LEFT_BUTTON)
		button_states[0] = (state == GLUT_DOWN);

	if (button == MOUSE_MIDDLE_BUTTON)
		button_states[1] = (state == GLUT_DOWN);

	if (button == MOUSE_RIGHT_BUTTON)
		button_states[2] = (state == GLUT_DOWN);

	if (button == MOUSE_SCROLL_UP)
		cam_position += z_gain * cam_direction;

	if (button == MOUSE_SCROLL_DOWN)
		cam_position -= z_gain * cam_direction;

	if (state == GLUT_DOWN) {
		new_coordinates[0] = x;
		new_coordinates[1] = y;
	}

	if (state == GLUT_UP) {
		new_coordinates[0] = x;
		new_coordinates[1] = y;
	}

}

/*
 * Handle mouse motions while a button is pressed
 *
 * \param 	x,y		Current coordinate of mouse cursor
 */
void GLManager::myMouseMove(int x, int y) {

	if (button_states[0]) {
		// Left button = camera rotation
		memcpy(last_coordinates, new_coordinates, 2 * sizeof(int));
		new_coordinates[0] = x;
		new_coordinates[1] = y;
		//		if (control_mode == CAMERA_CONTROL)
		//		{
		arcRotation(cam_quaternion, back_base);
		//		} else if ((control_mode == LIGHTING_CONTROL) && (directional_light))
		//		{
		//			Vector3f base 	= captured_back_base + back_normal*0.25;
		//			arcRotation(light_quaternion,base);
		//		}
	} else if (button_states[2]) {
		// Right button = Camera pan in x & y
		memcpy(last_coordinates, new_coordinates, 2 * sizeof(int));
		new_coordinates[0] = x;
		new_coordinates[1] = y;

		cam_position -= (new_coordinates[0] - last_coordinates[0])
				* (cam_direction.cross(cam_up_vector)) * translation_gain;
		cam_position += (new_coordinates[1] - last_coordinates[1])
				* cam_up_vector * translation_gain;
	}
}

/*
 * Interpret mouse motions as trackball rotations
 */
void GLManager::arcRotation(Quaternion<float> &quat, Vector3f object_center) {
	Vector3f new_coord_centered;
	Vector3f last_coord_centered;

	////////////////////////
	// Adapted from http://www.gamedev.net/topic/65558-gluproject-glunproject-use-examples/
	GLdouble pos3D_x, pos3D_y, pos3D_z, pos2D_x, pos2D_y, pos2D_z;
	pos3D_x = object_center[0];
	pos3D_y = object_center[1];
	pos3D_z = object_center[2];

	//	if (control_mode == LIGHTING_CONTROL)
	//	{
	//		glMatrixMode ( GL_MODELVIEW);
	//		glPushMatrix();
	//		Transform3f rot;
	//		//	Transform3f rot;
	//		rot.setIdentity();
	//		rot.rotate(cam_quaternion);
	//		glTranslatef(0, 0, back_distance);
	//		glMultMatrixf(rot.data());
	//		glTranslatef(0, 0, -back_distance);
	//	}

	// Get 2D coordinates based on window coordinates
	gluProject(pos3D_x, pos3D_y, pos3D_z, arc_rotation_puppet_view, arc_rotation_projection, arc_rotation_viewport,
			&pos2D_x, &pos2D_y, &pos2D_z);
	pos2D_y = render_height - pos2D_y;

	//	if (control_mode == LIGHTING_CONTROL)
	//		glPopMatrix();

	////////////////////////
	// Center mouse coordinates

	new_coord_centered[0] = new_coordinates[0] - pos2D_x;
	last_coord_centered[0] = last_coordinates[0] - pos2D_x;
	new_coord_centered[1] = new_coordinates[1] - pos2D_y;
	last_coord_centered[1] = last_coordinates[1] - pos2D_y;

	// Get z-values for mouse coordinates as projected onto trackball sphere (accounting for points outside sphere)
	if (pow(new_coord_centered[0], 2) + pow(new_coord_centered[1], 2) <= pow(
			arcball_radius, 2) / 2) {
		new_coord_centered[2] = sqrt(
				pow(arcball_radius, 2) - (pow(new_coord_centered[0], 2) + pow(
						new_coord_centered[1], 2)));
	} else {
		new_coord_centered[2] = pow((float) arcball_radius, 2) / 2.0f / sqrt(
				pow(new_coord_centered[0], 2) + pow(new_coord_centered[1], 2));
	}

	if (pow(last_coord_centered[0], 2) + pow(last_coord_centered[1], 2) < pow(
			arcball_radius, 2) / 2) {
		last_coord_centered[2] = sqrt(
				pow(arcball_radius, 2) - (pow(last_coord_centered[0], 2) + pow(
						last_coord_centered[1], 2)));
	} else {
		last_coord_centered[2] = pow((float) arcball_radius, 2) / 2.0f
				/ sqrt(
						pow(last_coord_centered[0], 2) + pow(
								last_coord_centered[1], 2));
	}

	new_coord_centered.normalize();
	last_coord_centered.normalize();

	Vector3f coord_product = last_coord_centered.cross(new_coord_centered);
	float new_angle = acos(last_coord_centered.dot(new_coord_centered));

	coord_product.normalize();

	//	if (control_mode == LIGHTING_CONTROL)
	//	{
	//		Transform3f apply_cam_quat;
	//		apply_cam_quat.setIdentity();
	//		apply_cam_quat.rotate(cam_quaternion.inverse());
	//		coord_product = apply_cam_quat * coord_product;
	//	}

	// Update quaternion
	if (coord_product[0] == coord_product[0]) {
		Quaternion<float> new_rotation;
		new_rotation = AngleAxis<float> (new_angle, coord_product);

		if (new_rotation.w() == new_rotation.w()) {
			quat = new_rotation * quat;
//			ROS_INFO_STREAM("New rotation: " << new_rotation.w() << " " << new_rotation.x() << " " << new_rotation.y() << " " << new_rotation.z());
//			ROS_INFO_STREAM("New quat: " << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z());
		}
	}
}


//****************************************************
// Simple parameter manipulation
//****************************************************

/*
 * Set the vertical, horizontal, and distance extents of the tracking region.
 *
 * \param 	h_extent	Horizontal extent of tracking region in degrees
 * \param 	v_extent	Vertical extent of tracking region in degrees
 * \param 	nearest		Closest trackable distance from the Kinect
 *
 */
void GLManager::setTrackingRegion(float h_extent, float v_extent, float nearest) {
	h_tracking_region = h_extent;
	v_tracking_region = v_extent;
	near_tracking_region = nearest;
}


//****************************************************
// Exit functions
//****************************************************

void GLManager::empty() {
	glutPostRedisplay();
}

void GLManager::cleanup() {
	if (record_pose && need_to_save) {
		printPoses();
	}
	ROS_INFO_STREAM("Cleaned up.");
}
