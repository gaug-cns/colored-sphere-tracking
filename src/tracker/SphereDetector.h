#ifndef _SPHERE_DETECTOR_H_
#define _SPHERE_DETECTOR_H_

#include "Sphere.h"
#include "Measurement.h"



class SphereDetector
{    
public:
    SphereDetector(cv::Size size, cv::Mat frame, cv::Mat depth_frame, 
        float ball_radius, float camera_x_angle, float camera_f, 
        std::string background_image_dir, std::string depth_background_image_dir) : 
        size(size), ball_radius(ball_radius), camera_x_angle(camera_x_angle), camera_f(camera_f), background_image_dir(background_image_dir), depth_background_image_dir(depth_background_image_dir)
    {
        background = cv::imread(background_image_dir, 1);
        depth_background = cv::imread(depth_background_image_dir, 1);
        if ( !background.data || !depth_background.data )
            saveBackground(frame, depth_frame);
        
        // debug_color = Red;
        
        // Colors for drawing
        draw_color_map[Red] = cv::Scalar(0, 0, 255);
        draw_color_map[Green] = cv::Scalar(0, 255, 0);
        draw_color_map[Yellow] = cv::Scalar(0, 255, 255);
        draw_color_map[Blue] = cv::Scalar(255, 0, 0);
        
        // Colors for detection
        // hue, hue_range, min saturation, min value, max saturation, max value
        color_map[Red] = {0, 10, 160, 70, 255, 255};
        color_map[Green] = {58, 16, 130, 80, 255, 255};
        color_map[Yellow] = {24, 12, 130, 150, 255, 255};
        color_map[Blue] = {110, 24, 100, 50, 255, 255};
    }
    
    void showSpheres(std::vector<Sphere> spheres, cv::Mat frame)
    {
        for (auto sphere : spheres)
        {
           
            std::vector<float> circle = getCircleDrawingFromPosition(sphere.position);
            cv::Point center(circle[0], circle[1]);
            int radius = circle[2];
            
            if (radius > 0)
                cv::circle(frame, center, radius, draw_color_map[sphere.color], 2, 8, 0);
        }
        cv::imshow("final spheres", frame);
    }
    
    std::vector<Measurement> findSpheres(cv::Mat frame, cv::Mat depth_frame, float time)
    {
        cv::Mat color_diff_threshold = getColorDifferenceBinary(frame);
        // cv::imshow("color_diff_threshold", color_diff_threshold);
        
        /* cv::Mat depth_diff_threshold = getDepthDifferenceBinary(depth_frame);
        cv::imshow("depth_diff_threshold", depth_diff_threshold); */
        
        cv::Mat diff_threshold;
        cv::bitwise_and(color_diff_threshold, color_diff_threshold, diff_threshold);
        
        // Hough paramters
        int min_distance = frame.rows / 50;
        int p1 = 40;
        int p2 = 10;
        int min_radius = 3;
        int max_radius = frame.rows / 5;
        
        float min_radius_deviation = 0.15; // 0.2;
        float max_radius_deviation = 1.5; // 1.5;
        
        std::vector<Measurement> result = {};
        for (auto it = color_map.begin(); it != color_map.end(); it++)
        {
            Color color = it->first;
            std::vector<int> color_range = it->second;
            cv::Mat frame_gray = inHSVRange(frame, color_range);
            // if (color == debug_color) { cv::imshow("color_filter", frame_gray); }
            
            // Take difference of frame
            cv::Mat diff_frame;
            cv::bitwise_and(diff_threshold, frame_gray, diff_frame);
            
            // Blur before hough circles
            cv::GaussianBlur(diff_frame, diff_frame, cv::Size(3,3), 1.5, 1.5);
            // if (color == debug_color) { cv::imshow("final_before_hough", diff_frame); }
            
            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(diff_frame, circles, CV_HOUGH_GRADIENT, 2, min_distance, p1, p2, min_radius, max_radius);
            
            std::vector<Measurement> measurements;
            for (auto circle : circles) {
                int x = cvRound(circle[0]);
                int y = cvRound(circle[1]);
                int r = cvRound(circle[2]);
                
                float d = 10. / 255. * (float)depth_frame.at<unsigned char>(y, x) + ball_radius;
                
                float r_detected = r;
                float r_expected = camera_f / d * ball_radius;
                
                // Check if radius of circle detection is coherant with the depth
                if (r_detected / r_expected > min_radius_deviation && r_detected / r_expected < max_radius_deviation)
                {
                    if (color == debug_color)
                        cv::circle(frame, cv::Point(x, y), r, cv::Scalar(0, 0, 0), 2, 8, 0);
                    
                    Eigen::Vector3f position = globalFromImageCoordinates(x, y, d);
                    
                    Measurement measurement = Measurement(position, color);
                    measurement.sigma = 0.05; // [m]
                    measurement.time = time; // [ms]
                    measurements.push_back(measurement);
                }
            }
            
            // Calculate probability
            std::vector<float> probabilities = {};
            for (int i = 0; i < measurements.size(); i++)
                probabilities.push_back( exp(- 0.6 * i) );
            
            float sum_probabilities = std::accumulate(probabilities.begin(), probabilities.end(), 0);
            for (int i = 0; i < measurements.size(); i++)
                measurements.at(i).probability = probabilities.at(i) / sum_probabilities;
            
            result.insert(result.end(), measurements.begin(), measurements.end());
            
            if (color == debug_color)
                cv::imshow("measurements", frame);
        }
        
        return result;
    }
    
    void saveBackground(cv::Mat frame, cv::Mat depth_frame)
    {
        cv::imwrite(background_image_dir, frame);
        cv::imwrite(depth_background_image_dir, depth_frame);
        
        background = frame;
        depth_background = depth_frame;
        
        std::cout << "Calibration image saved." << std::endl;
    }
    
    int debug_color;
    
    
private:
    cv::Mat background;
    cv::Mat depth_background;
    cv::Size size;
    
    float ball_radius;
    float camera_x_angle;
    float camera_f;
    
    std::string background_image_dir;
    std::string depth_background_image_dir;
    
    std::map<Color, cv::Scalar> draw_color_map;
    std::map<Color, std::vector<int>> color_map;
    
    
    cv::Mat inHSVRange(cv::Mat frame, std::vector<int> color_range)
    {
        int hue = color_range.at(0);
        int hue_range = color_range.at(1);
        int min_s = color_range.at(2);
        int min_v = color_range.at(3);
        int max_s = color_range.at(4);
        int max_v = color_range.at(5);
        
        cv::Mat frame_hsv;
        cv::cvtColor(frame, frame_hsv, CV_BGR2HSV);
        cv::Mat frame_gray;
        
        if (hue - hue_range < 0 && hue + hue_range > 0)
        {
            cv::Mat frame_gray_1, frame_gray_2;
            cv::inRange(frame_hsv, cv::Scalar( hue - hue_range + 180, min_s, min_v), cv::Scalar( 180, max_s, max_v), frame_gray_1);
            cv::inRange(frame_hsv, cv::Scalar( 0, min_s, min_v), cv::Scalar( hue + hue_range, max_s, max_v), frame_gray_2);
            cv::bitwise_or(frame_gray_1, frame_gray_2, frame_gray);
        }
        else if (hue - hue_range < 180 && hue + hue_range > 180)
        {
            cv::Mat frame_gray_1, frame_gray_2;
            cv::inRange(frame_hsv, cv::Scalar( hue - hue_range, min_s, min_v), cv::Scalar( 180, max_s, max_v), frame_gray_1);
            cv::inRange(frame_hsv, cv::Scalar( 0, min_s, min_v), cv::Scalar( hue + hue_range - 180, max_s, max_v), frame_gray_2);
            cv::bitwise_or(frame_gray_1, frame_gray_2, frame_gray);
        }
        else
        {
            cv::inRange(frame_hsv, cv::Scalar( hue - hue_range, min_s, min_v), cv::Scalar( hue + hue_range, max_s, max_v), frame_gray);
        }
        
        return frame_gray;
    }
    
    cv::Mat getColorDifferenceBinary(cv::Mat frame)
    {
        // Take difference
        cv::Mat diff;
        cv::absdiff(frame, background, diff);
        
        // Blur and gray difference
        cv::Mat diff_gray;
        cv::cvtColor(diff, diff_gray, CV_BGR2GRAY);
        cv::GaussianBlur(diff_gray, diff_gray, cv::Size(5,5), 1.5, 1.5);
        
        // Take threshold of differnce
        int threshold_difference = 25;
        cv::Mat diff_threshold;
        cv::threshold(diff_gray, diff_threshold, threshold_difference, 255, cv::THRESH_BINARY);
        
        return diff_threshold;
    }
    
    /* cv::Mat getDepthDifferenceBinary(cv::Mat depth_frame)
    {
        // Take difference
        cv::Mat diff_gray;
        cv::absdiff(depth_frame, depth_background, diff_gray);
        
        // Blur difference
        cv::GaussianBlur(diff_gray, diff_gray, cv::Size(1,1), 1.5, 1.5);
        
        // Take threshold of differnce
        int threshold_difference = 2;
        cv::Mat diff_threshold;
        cv::threshold(diff_gray, diff_threshold, threshold_difference, 255, cv::THRESH_BINARY);
        
        return diff_threshold;
    } */
    
    Eigen::Matrix3f getRotation(float roll)
    {
        Eigen::Matrix3f rotation;
        rotation << 1, 0, 0,
            0, cos(roll), sin(roll),
            0, - sin(roll), cos(roll);
            
        return rotation;
    }
    
    Eigen::Vector3f globalFromImageCoordinates(float cx, float cy, float d)
    {
        Eigen::Vector3f position;
        position(0) = 1. / camera_f * d * ( cx - size.width / 2 );
        position(1) = d;
        position(2) = -1. / camera_f * d * ( cy - size.height / 2 );
        
        Eigen::Vector3f transformed = getRotation(camera_x_angle) * position;
        return transformed;
    }
    
    std::vector<float> getCircleDrawingFromPosition(Eigen::Vector3f position)
    {
        Eigen::Vector3f transformed = getRotation(camera_x_angle) * position;
        
        float d = transformed(1);
        float image_x = camera_f / d * transformed(0) + size.width / 2;
        float image_y = - camera_f / d * transformed(2) + size.height / 2;
        float image_radius = camera_f * ball_radius / d;
        
        std::vector<float> circle;
        circle.push_back(image_x);
        circle.push_back(image_y);
        circle.push_back(image_radius);
        return circle;
    }
};

#endif