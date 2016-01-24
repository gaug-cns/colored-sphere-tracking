#ifndef _CIRCLE_DETECTOR_H_
#define _CIRCLE_DETECTOR_H_

#include <numeric>

#include "Sphere.h"
#include "Measurement.h"



class CircleDetector
{
public:
    CircleDetector(cv::Size size, float ball_radius, float camera_roll, float focal_length,
        std::string background_image_dir, std::string depth_background_image_dir) :
        size(size), ball_radius(ball_radius), camera_roll(camera_roll), focal_length(focal_length)
    {
        color_background = cv::imread(background_image_dir, 1);
        depth_background = cv::imread(depth_background_image_dir, 1);

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

        color_threshold_difference = 25;
        depth_threshold_difference = 2;

        center = cv::Point2f(size.width / 2, size.height / 2);

        // Hough paramters
        min_distance = size.width / 50;
        p1 = 40;
        p2 = 10;

        min_radius = 3;
        max_radius = size.width / 5;
    }



    std::vector<Measurement> findCircles(cv::Mat frame, cv::Mat depth_frame, double time)
    {
        cv::Mat color_diff_threshold = getColorDifferenceBinary(frame);
        // cv::Mat depth_diff_threshold = getDepthDifferenceBinary(depth_frame);

        cv::Mat diff_threshold;
        cv::bitwise_and(color_diff_threshold, color_diff_threshold, diff_threshold);

        float min_radius_deviation = 0.15; // 0.2;
        float max_radius_deviation = 1.5; // 1.5;

        std::vector<Measurement> result = {};
        for (auto it = color_map.begin(); it != color_map.end(); it++)
        {
            Color color = it->first;
            std::vector<int> color_range = it->second;

            cv::Mat frame_gray = inHSVRange(frame, color_range);

            // Take difference of frame
            cv::Mat diff_frame;
            cv::bitwise_and(diff_threshold, frame_gray, diff_frame);

            // Blur before hough circles
            cv::GaussianBlur(diff_frame, diff_frame, cv::Size(5, 5), 1.5, 1.5);
            if (color == debug_color)
            {
                cv::imshow("Final Before Hough", diff_frame);
            }

            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(diff_frame, circles, CV_HOUGH_GRADIENT, 2, min_distance, p1, p2, min_radius, max_radius);

            std::vector<Measurement> measurements;
            for (auto circle : circles)
            {
                int x = cvRound(circle[0]);
                int y = cvRound(circle[1]);
                int r = cvRound(circle[2]);

                float d = 10. / 255 * (float)depth_frame.at<unsigned char>(y, x) + ball_radius;
                float r_expected = focal_length / d * ball_radius;

                // Check if radius of circle detection is coherant with the depth
                if (r / r_expected > min_radius_deviation && r / r_expected < max_radius_deviation)
                {
                    if (color == debug_color)
                    {
                        cv::circle(frame, cv::Point(x, y), r, cv::Scalar(0, 0, 0));
                    }

                    Eigen::Vector3f position = globalFromImageCoordinates(cv::Point2f(x, y), d);

                    Measurement measurement = Measurement(position, color);
                    measurement.sigma = 0.05; // [m]
                    measurement.time = time; // [ms]
                    measurements.push_back(measurement);
                }
            }

            // Calculate probability
            std::vector<float> probabilities = {};
            for (int i = 0; i < measurements.size(); i++)
            {
                probabilities.push_back( exp(- 0.6 * i) );
            }

            float sum_probabilities = std::accumulate(probabilities.begin(), probabilities.end(), 0);
            for (int i = 0; i < measurements.size(); i++)
            {
                measurements.at(i).probability = probabilities.at(i) / sum_probabilities;
            }

            result.insert(result.end(), measurements.begin(), measurements.end());

            if (color == debug_color)
            {
                cv::imshow("Measurements", frame);
            }
        }

        return result;
    }


    void drawSpheres(std::vector<Sphere> spheres, cv::Mat frame)
    {
        for (auto sphere : spheres)
        {
            cv::Vec3f circle = getCircleDrawingFromPosition(sphere.position);
            if (circle[2] > 0)
            {
                std::cout << cv::Point(circle[0], circle[1]) << " " << circle[2] << std::endl;
                // cv::circle(frame, cv::Point(circle[0], circle[1]), circle[2], draw_color_map[sphere.color]);
            }
        }
        cv::imshow("Final Spheres", frame);
    }

    void setDebugColor(Color color)
    {
        debug_color = color;
    }


private:
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

        if (hue - hue_range < 0 && 0 < hue + hue_range)
        {
            cv::Mat frame_gray_1, frame_gray_2;
            cv::inRange(frame_hsv, cv::Scalar( hue - hue_range + 180, min_s, min_v), cv::Scalar(180, max_s, max_v), frame_gray_1);
            cv::inRange(frame_hsv, cv::Scalar( 0, min_s, min_v), cv::Scalar( hue + hue_range, max_s, max_v), frame_gray_2);
            cv::bitwise_or(frame_gray_1, frame_gray_2, frame_gray);
        }
        else if (hue - hue_range < 180 && 180 < hue + hue_range)
        {
            cv::Mat frame_gray_1, frame_gray_2;
            cv::inRange(frame_hsv, cv::Scalar(hue - hue_range, min_s, min_v), cv::Scalar(180, max_s, max_v), frame_gray_1);
            cv::inRange(frame_hsv, cv::Scalar(0, min_s, min_v), cv::Scalar( hue + hue_range - 180, max_s, max_v), frame_gray_2);
            cv::bitwise_or(frame_gray_1, frame_gray_2, frame_gray);
        }
        else
        {
            cv::inRange(frame_hsv, cv::Scalar(hue - hue_range, min_s, min_v), cv::Scalar(hue + hue_range, max_s, max_v), frame_gray);
        }

        return frame_gray;
    }

    cv::Mat getColorDifferenceBinary(cv::Mat color_frame)
    {
        // Take difference
        cv::Mat diff;
        cv::absdiff(color_frame, color_background, diff);

        // Blur and gray difference
        cv::Mat diff_gray;
        cv::cvtColor(diff, diff_gray, CV_BGR2GRAY);
        cv::GaussianBlur(diff_gray, diff_gray, cv::Size(5, 5), 1.5, 1.5);

        // Take threshold of differnce
        cv::Mat diff_threshold;
        cv::threshold(diff_gray, diff_threshold, color_threshold_difference, 255, cv::THRESH_BINARY);

        return diff_threshold;
    }

    cv::Mat getDepthDifferenceBinary(cv::Mat depth_frame)
    {
        // Take difference
        cv::Mat diff_gray;
        cv::absdiff(depth_frame, depth_background, diff_gray);

        // Blur difference
        cv::GaussianBlur(diff_gray, diff_gray, cv::Size(1,1), 1.5, 1.5);

        // Take threshold of differnce
        cv::Mat diff_threshold;
        cv::threshold(diff_gray, diff_threshold, depth_threshold_difference, 255, cv::THRESH_BINARY);

        return diff_threshold;
    }

    Eigen::Matrix3f getRotation(float roll)
    {
        Eigen::Matrix3f rotation;
        rotation << 1, 0, 0,
            0, cos(roll), sin(roll),
            0, - sin(roll), cos(roll);

        return rotation;
    }

    Eigen::Vector3f globalFromImageCoordinates(cv::Point2f point, float d)
    {
        Eigen::Vector3f position;
        position(0) = d / focal_length * (point.x - center.x);
        position(1) = d;
        position(2) = -d / focal_length * (point.y - center.y);

        Eigen::Vector3f transformed = getRotation(camera_roll) * position;
        return transformed;
    }

    cv::Vec3f getCircleDrawingFromPosition(Eigen::Vector3f position)
    {
        Eigen::Vector3f transformed = getRotation(camera_roll) * position;

        float d = transformed(1);
        float image_x = focal_length / d * transformed(0) + center.x;
        float image_y = - focal_length / d * transformed(2) + center.y;
        float image_radius = focal_length * ball_radius / d;

        return cv::Vec3f(image_x, image_y, image_radius);
    }



    cv::Mat color_background, depth_background;
    cv::Size size;
    cv::Point2f center;

    float ball_radius;
    float camera_roll;
    float focal_length;

    std::map<Color, cv::Scalar> draw_color_map;
    std::map<Color, std::vector<int>> color_map;

    int color_threshold_difference, depth_threshold_difference;

    int min_distance;
    int p1, p2;
    int min_radius, max_radius;

    Color debug_color;
};

#endif /* _CIRCLE_DETECTOR_H_ */
