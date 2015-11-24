#include <iostream>
#include <fstream>
#include <chrono>
#include <sys/stat.h>
#include <math.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "OpenNI.h"

#include "ini.hpp"

enum Color
{
	Red,
	Green,
	Yellow,
	Blue
};

#include "tracker/Sphere.h"
#include "tracker/Pose.h"
#include "tracker/SphereDetector.h"
#include "tracker/SphereFilters.h"
#include "tracker/PoseEstimator.h"



cv::Size size;

float sphere_radius; // [m]
float camera_x_angle; // [rad], 0 rad -> horizontal, pi/2 -> vertical down
float camera_f; // [a.u.]

std::string model_path;
std::string background_image_dir;
std::string depth_background_image_dir;
std::string save_image_dir;
std::string tracking_data_dir;

bool calculate_pose;


// Get current time in [s]
double getTime()
{
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> ( std::chrono::system_clock::now().time_since_epoch() );
    return ((double)ms.count()) / 1000;
}

// Save pose and time data in txt file
void saveTrackingData(std::vector<Pose> pose_history)
{
    std::ofstream myfile;
    myfile.open(tracking_data_dir);
    
    myfile << "t[s],x[m],y[m],z[m],roll[rad],pitch[rad],yaw[rad]" << std::endl;
    for (auto pose : pose_history)
    {
        myfile << pose.time << ","
            << pose.position(X) << ","
            << pose.position(Y) << ","
            << pose.position(Z) << ","
            << pose.orientation(ROLL) << ","
            << pose.orientation(PITCH) << ","
            << pose.orientation(YAW) << std::endl;
    }
    myfile.close();
    
    std::cout << "File saved. It will be overwritten if it is not renamed." << std::endl;
}




using namespace openni;

int main(int argc, char *argv[])
{
    // Open INI config file
    std::ifstream file("../config.ini");
    std::stringstream buffer;
    buffer << file.rdbuf();
    
    // INI parser
    INI::Parser p(buffer);
    std::stringstream out;
    p.dump(out);
    auto config = p.top();
    
    int width = std::stoi(config["width"]);
    int height = std::stoi(config["height"]);
    size = cv::Size(width, height);
    
    sphere_radius = std::stof(config["sphere_radius"]);
    camera_x_angle = std::stof(config["camera_x_angle"]);
    camera_f = std::stof(config["camera_f"]);

    model_path = config["model_path"];
    background_image_dir = config["background_image_dir"];
    depth_background_image_dir = config["depth_background_image_dir"];
    save_image_dir = config["save_image_dir"];
    tracking_data_dir = config["tracking_data_dir"];
	
	calculate_pose = ("true" == config["calculate_pose"]) ? true : false;
    
    
    // Open INI model file
    std::ifstream model_file(model_path);
    std::stringstream model_buffer;
    model_buffer << model_file.rdbuf();
    
    // INI parser
    INI::Parser model_p(model_buffer);
    std::stringstream model_out;
    model_p.dump(model_out);
    auto model_config = model_p.top();
    
    int number_spheres = std::stoi(model_config["number_spheres"]);
    
    std::map<std::string, Color> names_color;
    names_color["green"] = Green;
    names_color["red"] = Red;
    names_color["yellow"] = Yellow;
    names_color["blue"] = Blue;
    
    std::vector<Sphere> copter;
    for (int i = 0; i < number_spheres; i++)
    {
        std::string raw = "sphere_" + std::to_string(i) + "_";
        std::string color = model_config[raw + "color"];

        float x = std::stof(model_config[raw + "x"]);
        float y = std::stof(model_config[raw + "y"]);
        float z = std::stof(model_config[raw + "z"]);
        
        copter.push_back( Sphere(x, y, z, names_color[color]) );
    }
    
    
    
	// Initialize OpenNI
    if (STATUS_OK != OpenNI::initialize())
	{
        std::cerr << "OpenNI Initial Error: "  << OpenNI::getExtendedError() << std::endl;
    	return -1;
	}
  
	// Open Device
    Device device;
    if (STATUS_OK != device.open(ANY_DEVICE))
	{
        std::cerr << "Can't Open Device: "  << OpenNI::getExtendedError() << std::endl;
    	return -1;
	}
  
	// Create depth stream
	VideoStream depth_stream;
    if (STATUS_OK == depth_stream.create(device, SENSOR_DEPTH))
    {
	    // 3a. set video mode
    	VideoMode mode;
    	mode.setResolution(size.width, size.height);
    	mode.setFps(30);
    	mode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);

    	if (STATUS_OK != depth_stream.setVideoMode(mode))
    		std::cout << "Can't apply VideoMode: " << OpenNI::getExtendedError() << std::endl;
    }
    else
    {
    	std::cerr << "Can't create depth stream on device: " << OpenNI::getExtendedError() << std::endl;
    	return -1;
    }
  
	// Create color stream
	VideoStream color_stream;
	if (device.hasSensor(SENSOR_COLOR))
	{
    	if (STATUS_OK == color_stream.create(device, SENSOR_COLOR))
    	{
    		// 4a. set video mode
		    VideoMode mode;
    		mode.setResolution(size.width, size.height);
    		mode.setFps(30);
    		mode.setPixelFormat(PIXEL_FORMAT_RGB888);
            
    		if (STATUS_OK != color_stream.setVideoMode(mode))
        		std::cout << "Can't apply VideoMode: " << OpenNI::getExtendedError() << std::endl;
                
    		// 4b. image registration
    		if (device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
        		device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    	}
    	else
    	{
    		std::cerr << "Can't create color stream on device: " << OpenNI::getExtendedError() << std::endl;
    		return -1;
    	}
	}
	
	
    // Print help
	std::cout << "c - calibrate pose and time \n"
        << "p - subtract background \n"
        << "t - save tracking data \n"
        << "s - save frame \n"
        << "r / b / g / y - red / blue / green / yellow color analysis \n"
        << "q - quit" << std::endl;
	

    // Init color and depth frames, start streames
	VideoFrameRef color_frame;
	VideoFrameRef depth_frame;
	depth_stream.start();
	color_stream.start();
	int max_depth = depth_stream.getMaxPixelValue();	
    
    // Read first frame
    if (STATUS_OK != color_stream.readFrame(&color_frame))
    {
        std::cerr << "Can't read color frame." << std::endl;
        return -1;
    }
    if (STATUS_OK != depth_stream.readFrame(&depth_frame))
    {
        std::cerr << "Can't read depth frame." << std::endl;
        return -1;
    }
    const cv::Mat color_frame_temp( color_frame.getHeight(), color_frame.getWidth(), CV_8UC3, (void*)color_frame.getData() );
    cv::Mat frame_color_temp;
    cv::cvtColor(color_frame_temp, frame_color_temp, CV_RGB2BGR);
    
    const cv::Mat depth_frame_temp( depth_frame.getHeight(), depth_frame.getWidth(), CV_16UC1, (void*)depth_frame.getData() );
    cv::Mat frame_depth_temp;
    depth_frame_temp.convertTo(frame_depth_temp, CV_8U, 255. / max_depth);
    
    
    // Init sphere detector
    SphereDetector sphere_detector = SphereDetector(size, frame_color_temp, frame_depth_temp, sphere_radius, camera_x_angle, camera_f, background_image_dir, depth_background_image_dir);
    SphereFilters sphere_filters = SphereFilters();
    PoseEstimator pose_estimator = PoseEstimator();

    // Init calibration pose and time
    Pose calibration_pose;
    calibration_pose.position = Eigen::Vector3f::Zero();
    calibration_pose.orientation = Eigen::Vector3f::Zero();
    calibration_pose.time = getTime();
    
    
    // Prepare while loop
    std::vector<Pose> pose_history;
    int current_time = getTime();
    std::vector<Sphere> current_spheres;
	while (true)
	{
    	// Check possible errors
    	if (!color_stream.isValid())
    	{
    		std::cerr << "Color stream not valid" << std::endl;
    		break;
    	}
    	
    	if (STATUS_OK != color_stream.readFrame( &color_frame ))
    	{
    		std::cerr << "Can't read color frame'" << std::endl;
    		break;
    	}
    	
    	if (STATUS_OK != depth_stream.readFrame( &depth_frame ))
    	{
    		std::cerr << "Can't read depth frame'" << std::endl;
    		break;
    	}
    	
        
    	// Grab frames, result is frame and frame_depth
    	const cv::Mat color_frame_temp( color_frame.getHeight(), color_frame.getWidth(), CV_8UC3, (void*)color_frame.getData() );
        cv::Mat frame;
        cv::cvtColor(color_frame_temp, frame, CV_RGB2BGR);
        
    	const cv::Mat depth_frame_temp( depth_frame.getHeight(), depth_frame.getWidth(), CV_16UC1, (void*)depth_frame.getData() );
    	cv::Mat frame_depth;
    	depth_frame_temp.convertTo(frame_depth, CV_8U, 255. / max_depth);
        
        
        // Get current time and time difference
        double last_time = current_time; // [s]
        current_time = getTime(); // [s]
        double delta_time = current_time - last_time; // [s]
        

        // Detect spheres
    	cv::Mat frame_info = frame.clone();
    	std::vector<Measurement> measurements = sphere_detector.findSpheres(frame_info, frame_depth, current_time);
        
        
        // Update filter and get tracked spheres
        // sphere_filters.update(measurements, current_time);
        std::vector<Sphere> tracked_spheres = sphere_filters.getTrackedSpheres(current_time);
        
        
        // Draw spheres
        // sphere_detector.showSpheres(tracked_spheres, frame);
        

        // Calculate pose
		Pose pose = Pose();
		if (calculate_pose)
		{
			pose = pose_estimator.getPoseFromModel(copter, tracked_spheres);
	        pose.time = current_time;
	        if (tracked_spheres.size() >= 3)
	        {
	            Pose pose_result = pose - calibration_pose;
	            pose_history.push_back(pose_result);
	            std::cout << "X: " << pose.position(X) << " Y: " << pose.position(Y) << " Z: " << pose.position(Z) << " Roll: " << pose.orientation(ROLL) << " Pitch: " << pose.orientation(PITCH) << " Yaw: " << pose.orientation(YAW) << std::endl;
	        }
		}
        
        
        // React on keyboard input
    	char key = cv::waitKey(10);
        switch (key)
        {
            case 'p':
                sphere_detector.saveBackground(frame, frame_depth);
                break;
                
            case 's':
                std::cout << "Frame saved." << std::endl;
                cv::imwrite(save_image_dir, frame_info);
                break;
                
            case 'c':
                calibration_pose = pose;
                std::cout << "Pose and time calibrated." << std::endl;
                break;
                
            case 't':
                saveTrackingData(pose_history);
                break;
            
            // Change debug color for further analysis
            case 'y': sphere_detector.debug_color = Yellow; break;
            case 'g': sphere_detector.debug_color = Green; break;
            case 'b': sphere_detector.debug_color = Blue; break;
            case 'r': sphere_detector.debug_color = Red; break;
                
            case 'q':
                return 0;
                break;
        }
	}
    
    // Destroy objects, otherwise errors will occure next time...
	depth_stream.destroy();
	color_stream.destroy();
	device.close();
	OpenNI::shutdown();
    
    return 0;
}
