#include <iostream>
#include <fstream>
#include <chrono>
#include <sys/stat.h>
#include <math.h>

#include "ini.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>


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
float camera_roll; // [rad], 0 rad -> horizontal, pi/2 -> vertical down
float focal_length; // [a.u.]

std::string model_path;
std::string tracking_data_path;

std::string background_color_path;
std::string background_depth_path;
std::string video_color_path;
std::string video_depth_path;


bool calculate_pose;


// Save pose and time data in txt file
void saveTrackingData(std::vector<Pose> pose_history)
{
    std::ofstream myfile;
    myfile.open(tracking_data_path);
    
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

    sphere_radius = std::stof(config["sphere_radius"]);
    camera_roll = std::stof(config["camera_roll"]);
    focal_length = std::stof(config["focal_length"]);

    model_path = config["model_path"];
	tracking_data_path = config["tracking_data_path"];
	
	std::string directory = config["directory"];
	background_color_path = directory + config["background_color_name"];
	background_depth_path = directory + config["background_depth_name"];
	video_color_path = directory + config["video_color_name"];
	video_depth_path = directory + config["video_depth_name"];
    
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
	
	// Open video
    cv::VideoCapture capture_color = cv::VideoCapture(video_color_path);
    if (!capture_color.isOpened())
    {
        std::cout << "Could not read color video." << std::endl;
        return -1;
    }

    cv::VideoCapture capture_depth = cv::VideoCapture(video_depth_path);
    if (!capture_depth.isOpened())
    {
        std::cout << "Could not read depth video." << std::endl;
        return -1;
    }
	
	// Video size
    size = cv::Size(capture_color.get(3), capture_color.get(4));
	if (size.width != capture_depth.get(3) || size.height != capture_depth.get(4)) {
		std::cout << "Color and depth video have different sizes." << std::endl;
		return -1;
	}
	
	int length = capture_color.get(7);
	if (length != capture_depth.get(7)) {
		std::cout << "Color and depth video have different lengths. " << length << " vs. " << capture_depth.get(7) << std::endl;
		// return -1;
	}
	
	
    // Print help
	std::cout << "r / b / g / y - red / blue / green / yellow color analysis \n"
        << "q - quit" << std::endl;
	

    // Init sphere detector
	float sigma_accuracy = 4.;
	
    SphereDetector sphere_detector = SphereDetector(size, sphere_radius, camera_roll, focal_length, background_color_path, background_depth_path);
    SphereFilters sphere_filters = SphereFilters(sigma_accuracy);
    PoseEstimator pose_estimator = PoseEstimator();
	
    // Init calibration pose and time
    Pose calibration_pose;
    calibration_pose.position = Eigen::Vector3f::Zero();
    calibration_pose.orientation = Eigen::Vector3f::Zero();
    calibration_pose.time = 0;
    
    
    // Prepare while loop
	cv::Mat frame_color;
	cv::Mat frame_depth;
	int current_frame_index;
    std::vector<Pose> pose_history;
    std::vector<Sphere> current_spheres;
    bool recording = true;
	while (recording)
	{
		// Get new frame
    	if (!capture_color.grab() || !capture_depth.grab())
        {
            break;
        }
		capture_color.retrieve(frame_color);
		capture_depth.retrieve(frame_depth);
        current_frame_index = capture_color.get(1);

        // Detect spheres
    	cv::Mat frame_info = frame_color.clone();
    	std::vector<Measurement> measurements = sphere_detector.findSpheres(frame_info, frame_depth, current_frame_index);
        
        
        // Update filter and get tracked spheres
        sphere_filters.update(measurements, current_frame_index);
        std::vector<Sphere> tracked_spheres = sphere_filters.getTrackedSpheres(current_frame_index);
        
        
        // Draw spheres
        sphere_detector.drawSpheres(tracked_spheres, frame_color);
        

        // Calculate pose
		Pose pose = Pose();
		if (calculate_pose)
		{
			pose = pose_estimator.getPoseFromModel(copter, tracked_spheres);
	        pose.time = current_frame_index;
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
            // Change debug color for further analysis
            case 'y': sphere_detector.setDebugColor(Yellow); break;
            case 'g': sphere_detector.setDebugColor(Green); break;
            case 'b': sphere_detector.setDebugColor(Blue); break;
            case 'r': sphere_detector.setDebugColor(Red); break;
                
            case 'q':
                recording = false;
                break;
        }
	}
    
    return 0;
}
