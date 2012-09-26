/*
  * GLManager.h
  *
  *     Created: 2011/2012
  *      Author: Robin Held (begun by Ankit Gupta)
  *
  *      This class provides rendering and animation loading/saving functionality.
  *      See LICENSE.txt for licensing info.
  */


#ifndef GLMANAGER_H_
#define GLMANAGER_H_

#include <Maths/Maths.h>
#include <cmath>
#include "Puppet.h"
#include <boost/filesystem.hpp>
#include <opencv/cv.h>
#include <stdlib.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include <opencv/highgui.h>
#include <cstdio>
#include <assimp/assimp.h>
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/aiVersion.h>
#include <std_msgs/Float32MultiArray.h>
#include <pthread.h>
#include <semaphore.h>

namespace fs = boost::filesystem;

using namespace fs;
using namespace std;
using namespace cv;
using namespace cv_bridge;

using boost::lexical_cast;

#define PI 			3.14159
#define glewExperimental TRUE


// For recovering visible points of puppet puppets
#define SPHERE_SAMPLES	5

// For arcball controls
#define translation_gain 	0.001
#define z_gain 			 	0.01
#define rotation_gain 	 	0.001

enum
{
	MOUSE_LEFT_BUTTON = 0,
	MOUSE_MIDDLE_BUTTON = 1,
	MOUSE_RIGHT_BUTTON = 2,
	MOUSE_SCROLL_UP = 3,
	MOUSE_SCROLL_DOWN = 4
};

/////////////////////////
// Global Variables

//// Shared with BlockTracker:
extern pthread_mutex_t 	amutex;
extern pthread_cond_t 	cond;
extern vector<Puppet*> 	Puppets;
extern bool 			global_capture;
extern bool				raw_capture_started;
extern vector<double> 	video_timestamps;
extern bool				clear_animation;
extern bool				suspend_tracking;
extern vector<GLuint>	puppet_lists;
extern vector<GLuint>	puppet_VBOs_created;
extern bool				transfer_colors;
extern bool 			spread_colors;
extern bool				save_new_ply;
// SIFT data-capture variables
extern std::string		SIFT_save_name_numbered;
extern bool				signal_SIFT_save;
extern bool				playback_only;			// Disable kinect input and only play back saved animations

/////////////////////////
// Assimp functions, variables, and structs for scene loading:

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)

struct TextureAndPath
{
	GLuint hTexture;
	aiString pathName;
};

// This is for a shader uniform block
struct MyMaterial{

	float diffuse[4];
	float ambient[4];
	float specular[4];
	float emissive[4];
	float shininess;
	int texCount;
};

// Scene-VBO variables:
// Information to render each assimp node
struct MyMesh{
	GLuint vao;
	GLuint texIndex;
	GLuint uniformBlockIndex;
	int numFaces;
};


/////////////////////////
// For shadow rendering:
// Uniform Buffer for Matrices
// this buffer will contain 3 matrices: projection, view and puppet
// each matrix is a float array with 16 components
extern GLuint matricesUniBuffer;
#define MatricesUniBufferSize sizeof(float) * 16 * 3
#define ProjMatrixOffset 0
#define ViewMatrixOffset sizeof(float) * 16
#define PuppetMatrixOffset sizeof(float) * 16 * 2
#define MatrixSize sizeof(float) * 16


// Control modes
enum { CAMERA_CONTROL, BACKGROUND_CONTROL, LIGHTING_CONTROL };


class GLManager
{
public:
	GLManager();
	// Prep and initialization functions.
	void prep();
	void InitGL(int Width, int Height);
	void start();

	// Rendering functions
	void ReSizeGLScene(int Width, int Height);
	GLdouble* calculateClippingPlane(Vector3f vertex_1, Vector3f vertex_2, Vector3f vertex_3);
	void DrawGLScene();
	void drawBackground();
	void drawLightingVector();
	void drawTrackingBase();
	void drawTrackingVolume();
	void drawTextOnScreen();
	void grabPuppetData();
	void drawAllPuppets();
	void renderBlock(float r, float g, float b, int current_puppet);
	void renderPuppet(int current_puppet);
	void renderPuppetVBO(int current_puppet);
	void removeOccludedPoints(int current_puppet);
	void addScene(std::string scene_name, float scale_factor, float scene_brightness);
	void recursiveTextureLoad(const struct aiScene *sc, const struct aiNode* nd, int scene_num);
	void recursiveAIRender (const struct aiScene *sc, const struct aiNode* nd, bool apply_transform, int scene_num);
	void apply_material(const struct aiMaterial *mtl);
	float adjustTransparency(Vector3f position);
	void setBackgroundImage(std::string background_image);
	void loadBackgroundImage();
	void setTrackingVolume();
	void setGroundPlane();


	// Animation / video functions
	void setupVideoRecorder(std::string files_path, std::string video_out_name);
	void getScreenShot( Mat *imgO );
	void setupPoseRecorder(std::string files_path, std::string pose_out_name);
	void savePoses();
	void printPoses();
	void enablePoseLoading(std::string files_path, std::string pose_in_name);
	void loadPoses();
	bool parsePoseFileLine(string line);
	void combineVideos();
	void getSIFTSet();

	// User input
	void myKeyboard(unsigned char key, int x, int y);
	void mySpecialKeyboard(int key, int x, int y);
	void myMouseButton(int button, int state, int x, int y);
	void myMouseMove(int x, int y);
	void arcRotation(Quaternion<float> &quat, Vector3f object_center);

	// Simple parameter manipulation
	void setTrackingRegion(float h_extent, float v_extent, float nearest);
	void enableTransientPoses(){show_transient_pose = true;};
	void setPuppetnames(vector<std::string> names){puppet_names = names;};
	void setBaseFilePath(std::string path){files_path = path;};
	void enableSIFTPoseSaving(){SIFT_saving = true;};
	void setSIFTSavingName(std::string name){SIFT_save_name = name;};
	void setRawVideoFileName(std::string name){output_raw_video = name; export_raw_video = true;};

	// Exit functions
	void empty();
	void cleanup();

protected:

	/////////////////////////
	// Rendering variables
	int 	window_occ;
	int 	window;
	bool 	first_pass;
	int 	render_width;
	int		render_height;
	bool	use_VBOs;		// Use vertex buffer objects for rendering the puppets? Otherwise, use draw lists.

	/////////////////////////
	// Puppet variables
	vector<bool> 			visible;				// Used for resampling puppet models
	int						current_puppet;
	int						num_puppets;
	int						num_playback_puppets;
	int						num_available_puppets;
	vector<vector<float> > 	transient_puppet_poses;
	vector<vector<float> > 	puppet_poses;
	vector<int>				puppet_visibility;
	vector<std::string> 	puppet_names;
	bool					show_transient_pose;
	vector<GLfloat*>		temp_puppet_poses;
	vector<bool>			puppets_to_draw;
	vector<Vector3f>		translation_components;
	vector<GLuint*>			puppet_VBOs;
	vector<int>				all_num_triangles;
	vector<int>				all_num_vertices;

	/////////////////////////
	// Video-recording variables
	bool 			record_video;
	VideoWriter 	writer;
	unsigned char	*imageData;
	Mat				*capturedImg;
	std::string		output_video;
	std::string		video_out_name;
	std::string		performance_in_name;
	bool			export_raw_video;
	std::string		files_path;
	Mat 			*img_mat;
	bool			local_capture;
	bool			layer_two_synced;

	/////////////////////////
	// Animation recording, loading, and playing variable
	timeval 	time_grabber;
	bool		captureNow;
	bool 		captureTwoNow;
	bool		record_pose;
	vector<int> in_first_layer;
	double		accumulated_time;						// Used to correctly store time frames when recording is repeatedly started and stopped.
	double		accumulated_time_two;					// Used to correctly store time frames when recording is repeatedly started and stopped.
	vector<float> 				timestamps;				// Time of each captured pose-set
	vector<float> 				timestamps_two;				// Time of each captured pose-set
	vector<double> 				absolute_timestamps;	// Absolute time of each captured pose-set
	vector<double>				absolute_timestamps_two;
	vector<double> 				absolute_exported_timestamps;	// Absolute time of each captured pose-set that has been exported as a video frame
	vector<vector<int> >		visible_states;			// Whether each puppet was detected
	vector<vector<int> >		visible_states_two;			// Whether each puppet was detected
	vector<vector<vector <float> > > saved_poses;			// Each puppet's pose
	vector<vector<vector <float> > > saved_poses_two;		// Each puppet's pose in the second frame
	bool		playNow;
	bool		play_all_layers;
	int			play_frame;								// Current playback frame
	int			play_frame_two;							// Current playback frame for second layer
	double		play_start_time;						// When the user initiated playback
	double		play_start_time_two;					// ditto for second layer
	std::string input_pose;
	ofstream 	pose_stream_out;
	std::string output_pose;
	vector< vector<float> >		poses;
	unsigned int 				current_pose_index;
	bool		need_to_save;
	bool 		load_poses;
	bool		past_header;
	int			frames_to_load;
	int			frames_to_load_layer_two;
	int			puppet_frames_to_load;
	int			absolute_frames_to_load;
	int			video_frames_to_load;
	double		record_initial_time;		// Time when the current capture session was started.
	double		record_initial_time_two;	// Time when the current (2nd-layer) gocapture session was started.
	int			timestamps_loaded;
	int			timestamps_loaded_layer_two;
	int			video_timestamps_loaded;
	bool		loaded_layer_states;
	int			puppets_to_load;
	int			puppets_loaded;
	vector<int>	fill_empty_visibility;
	vector<int>	fill_empty_visibility_two;
	int			poses_loaded;
	int			visible_states_loaded;
	bool		SIFT_saving;
	std::string SIFT_save_name;
	int 		SIFT_it;
	bool		SIFT_save_now;
	int			capture_fps;
	double		frame_period;
	std::string output_raw_video;
	bool		process_video_local;
	bool		export_to_video;
	int			total_exported_frames;
	bool		new_frame_accessed;
	bool		new_frame_accessed_two;
	bool		started_capture;
	bool		clear_poses;

	/////////////////////////
	// Timing variables
	int               rc;
	struct timespec   ts;
	struct timeval    tp;
	int               rc_cond;
	struct timespec   ts_cond;
	struct timeval    tp_cond;
	double	old_time;
	double	new_time;
	static const int NUM_FPS_SAMPLES = 64;
	double 	rates_ave[64];

	/////////////////////////
	// Mouse variables
	bool 	button_states[5];
	int		last_coordinates[2];
	int		new_coordinates[2];

	/////////////////////////
	// Camera variables
	Vector3f 			cam_position;
	Vector3f 			cam_direction;
	Vector3f 			cam_up_vector;
	Quaternion<float> 	cam_quaternion;
	// Clipping planes
	float near_clip;
	float far_clip;
	// Horizontal field of view
	float v_FOV;
	// Horizontal and vertical tracking regions (within FOV, in angles) and closest trackable distance (in meters)
	float h_tracking_region;
	float v_tracking_region;
	float tracking_asymmetry;				// The horizontal tracking region extends less to the right. This represents
											// the ratio of the left and right tracking extents.
	float transparency_fall_off_angle;		// Rate at which a puppet fades away as it leaves the tracking area.
											// In degrees. This far / 2 within the tracking region, the puppet is opaque.
											// This far / 2 outside the tracking region, the puppet is transparent
	float transparency_fall_off_distance;	// Same as above, but for puppets close to the kinect
	float near_tracking_region;
	float table_shim;

	/////////////////////////
	// Lighting and material variables
	GLfloat light_position0[4];
	GLfloat light_direction0[4];
	GLfloat light_position1[4];
	GLfloat light_diffuse0[4];
	GLfloat light_diffuse0_dim[4];
	GLfloat light_ambient0[4];
	GLfloat light_ambient0_dim[4];
	GLfloat light_ambient3[4];
	GLfloat mat_specular[4];
	GLfloat mat_shininess[1];
	GLfloat mat_ambient[4];
	bool	use_puppet_colors;	// Use the puppets' colors for rendering? (Don't use them when creating masks)
	float	ambient_term;
	float	diffuse_term;
	bool	directional_light;
	Vector4f 		  	light_direction;
	Vector4f 		  	light_position;
	Quaternion<float> 	light_quaternion;
	bool	show_light;
	int		show_light_frames;	// For how many frames should the light position be rendered?
	int		light_frames_shown;
	bool	render_shadows;
	GLuint	fboId;


	/////////////////////////
	// Background / scene variables
	bool 		background_captured;
	int			background_info_loaded;		// 0 = none, 1/2 = combo of base, normal, or distance, 3 = all values (desired)
	Vector3f 	captured_back_normal;
	Vector3f	captured_back_base;
	Vector3f 	back_normal;
	Vector3f 	right_vector;
	Vector3f	up_vector;
	Vector3f	back_base;
	Vector3f	back_adj;
	float	 	back_distance;
	vector<Vector3f> 	ground_plane_edges;
	vector<Vector3f> 	far_track_plane_edges;
	vector<Vector3f> 	close_track_plane_edges;
	vector<GLdouble*> 	extra_clipping_planes;
	int 		tracking_region_visible;			// 0 = no tracking info, 1 = tracking plane only, 2 = tracking volume

	// 3D backgrounds
	// Create an instance of the Importer class
	vector<std::string> 			scene_names;
	vector<GLuint> 					scene_lists;
	vector<vector< struct aiMatrix4x4 > > scene_transforms;
	vector<vector< bool> >			apply_scene_transforms;
	vector<Assimp::Importer> 		scene_importers;
	vector<const aiScene*> 		 	scene_instances;
	vector<vector<struct MyMesh> > 	scene_meshes;

	vector<GLuint>			scene_VBOs_created;
	vector<float>			scene_scales;
	vector<float>			scene_brightnesses;		// For adding extra lighting to dim scenes.
	bool					apply_extra_light;
	Quaternion<float> 		bkgd_quaternion;
	vector< vector<TextureAndPath> >	texturesAndPaths;
	int						num_scenes;
	int 					current_scene;

	/////////////////////////
	// Control variables
	int control_mode;

	/////////////////////////
	// Shadow-casting Variables (From http://www.paulsprojects.net/tutorials/smt/smt.html)
	GLfloat camera_pos[3];
	GLfloat light_pos0[4];
	//Size of shadow map
	static const int 	shadowMapSize=4112; //2056;
	float 	shadowClipThickness;	// Thickness of shadowmap frustum. extends 10cm below performance surface.
	GLfloat white[4];
	GLfloat black[4];

	/////////////////////////
	//Textures
	GLuint 	bkgd_texture;
	Mat 	bkgd_img;
	bool	bkgd_loaded;
	GLuint 	shadowMapTexture;

	/////////////////////////
	// OpenGL matrices
	MATRIX4X4 	lightProjectionMatrix, lightViewMatrix;
	MATRIX4X4 	cameraProjectionMatrix, cameraViewMatrix;

	/////////////////////////
	// Variables for arcball rotation
	GLdouble 	arc_rotation_puppet_view[16];
	GLdouble 	arc_rotation_projection[16];
	GLint 		arc_rotation_viewport[4];
	float 		arcball_radius;

};
#endif
