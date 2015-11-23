#include "OpenNI.h"


#include "Helper.h"
#include "Sphere.h"
#include "Pose.h"
#include "SphereDetector.h"
#include "SphereFilters.h"
#include "PoseEstimator.h"



// Save pose and time data in txt file
void saveTrackingData(std::vector<Pose> pose_history)
{
    std::ofstream myfile;
    myfile.open (TRACKING_DATA_DIR);
    
    myfile << "t[s],x[m],y[m],z[m],roll[rad],pitch[rad],yaw[rad]" << std::endl;
    
    for (auto pose : pose_history)
    {
        myfile << ((float)pose.time) / 1000
            << "," << pose.position(0)
            << "," << pose.position(1) << ","
            << pose.position(2) << ","
            << pose.orientation(0) << ","
            << pose.orientation(1) << ","
            << pose.orientation(2) << std::endl;
    }
    
    myfile.close();
    
    std::cout << "File saved. It will be overwritten if it is not renamed." << std::endl;
}



using namespace openni;

int main(int argc, char *argv[])
{
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
    	mode.setResolution(WIDTH, HEIGHT);
    	mode.setFps(30);
    	mode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);

    	if ( depth_stream.setVideoMode(mode) != STATUS_OK )
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
    		mode.setResolution(WIDTH, HEIGHT);
    		mode.setFps(30);
    		mode.setPixelFormat(PIXEL_FORMAT_RGB888);
            
    		if (color_stream.setVideoMode(mode) != STATUS_OK)
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
        std::cerr << "Can't read color frame'" << std::endl;
        return -1;
    }
    if (STATUS_OK != depth_stream.readFrame(&depth_frame))
    {
        std::cerr << "Can't read depth frame'" << std::endl;
        return -1;
    }
    const cv::Mat color_frame_temp( color_frame.getHeight(), color_frame.getWidth(), CV_8UC3, (void*)color_frame.getData() );
    cv::Mat frame_color_temp;
    cv::cvtColor(color_frame_temp, frame_color_temp, CV_RGB2BGR);
    
    const cv::Mat depth_frame_temp( depth_frame.getHeight(), depth_frame.getWidth(), CV_16UC1, (void*)depth_frame.getData() );
    cv::Mat frame_depth_temp;
    depth_frame_temp.convertTo(frame_depth_temp, CV_8U, 255.0 / max_depth);
    
    
    // Init sphere detector
    SphereDetector sphere_detector = SphereDetector(frame_color_temp, frame_depth_temp);
    SphereFilters sphere_filters = SphereFilters();
    PoseEstimator pose_estimator = PoseEstimator();
    
    
    // Model for pose estimation, FlyPi Quadcopter
    float diameter = 0.75; // [m]
    float heigth = 0.08; // [m]
    std::vector<Sphere> copter;
    copter.push_back( Sphere(diameter / 2, 0, heigth, Green) ); // Front
    copter.push_back( Sphere(0, -diameter / 2, heigth, Red) ); // Left
    copter.push_back( Sphere(0, diameter / 2, heigth, Yellow) ); // Right
    copter.push_back( Sphere(-diameter / 2, 0, heigth, Blue) ); // Back
    
    
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
    	
    	if (color_stream.readFrame( &color_frame ) != STATUS_OK)
    	{
    		std::cerr << "Can't read color frame'" << std::endl;
    		break;
    	}
    	
    	if (depth_stream.readFrame( &depth_frame ) != STATUS_OK)
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
    	depth_frame_temp.convertTo(frame_depth, CV_8U, 255.0 / max_depth);
        
        
        // Get current time and time difference
        int last_time = current_time; // [ms]
        current_time = getTime(); // [ms]
        float delta_time = (float)(current_time - last_time) / 1e3; // [s]
        

        // Detect spheres
    	cv::Mat frame_info = frame.clone();
    	std::vector<Measurement> measurements = sphere_detector.findSpheres(frame_info, frame_depth, current_time);
        
        
        // Update filter and get tracked spheres
        sphere_filters.update(measurements, current_time);
        std::vector<Sphere> tracked_spheres = sphere_filters.getTrackedSpheres(current_time);
        
        
        // Draw spheres
        sphere_detector.showSpheres(tracked_spheres, frame);
        
        
        
        // Calculate pose
        /* Eigen::Vector3f pose_position, pose_orientation;
        pose_position << 0, 0, 0;
        pose_orientation << 0, 0, 0;
        Pose pose = {pose_position, pose_orientation, current_time}; */
        
        Pose pose = pose_estimator.getPoseFromModel(copter, tracked_spheres);
        pose.time = current_time;
        if (tracked_spheres.size() >= 3)
        {
            Pose pose_result = pose - calibration_pose;
            pose_history.push_back(pose_result);
            std::cout << "X: " << pose.position(0) << " Y: " << pose.position(1) << " Z: " << pose.position(2) << " Roll: " << pose.orientation(0) << " Pitch: " << pose.orientation(1) << " Yaw: " << pose.orientation(2) << std::endl;
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
                cv::imwrite(SAVE_IMAGE_DIR, frame_info);
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
