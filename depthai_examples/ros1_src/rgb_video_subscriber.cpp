#include "ros/ros.h"
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/SpatialDetectionConverter.hpp>

// Inludes common necessary includes for development using depthai library

dai::rosBridge::ImageConverter inputConverter(true);
cv::Mat rgbImage;
depthai_ros_msgs::SpatialDetectionArray detectArray;

void rgbCallback(const sensor_msgs::ImagePtr& rgbImageMsg){
    try {
        rgbImage = cv_bridge::toCvCopy(rgbImageMsg, sensor_msgs::image_encodings::BGR8)->image;
        }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void detectCallback(const depthai_ros_msgs::SpatialDetectionArrayPtr& detectMsg){
    detectArray = *detectMsg;
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "rgb_subscriber_node");
    ros::NodeHandle nh;

    ros::Subscriber rgb_sub = nh.subscribe("/yolov4_publisher/color/image", 5, rgbCallback);
    ros::Subscriber detection_sub = nh.subscribe("/yolov4_publisher/color/yolov4_Spatial_detections", 5, detectCallback);
    ros::Publisher detected_pub = nh.advertise<sensor_msgs::Image>("/yolov4_publisher/color/detected_image", 1000);
    auto color = cv::Scalar(255, 255, 255);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        for(const auto& detection : detectArray.detections) {
            int x1 = detection.bbox.center.x - detection.bbox.size_x/2;
            int y1 = detection.bbox.center.y - detection.bbox.size_y/2;
            int x2 = detection.bbox.center.x + detection.bbox.size_x/2;
            int y2 = detection.bbox.center.y + detection.bbox.size_y/2;
            
            std::string labelStr = detection.tracking_id;
            cv::putText(rgbImage, labelStr, cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 5.0, 255);

            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.results[0].score * 100;
            cv::putText(rgbImage, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            std::stringstream depthX;
            depthX << "X: " << (double)detection.position.x * 1000 << " mm";
            cv::putText(rgbImage, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream depthY;
            depthY << "Y: " << (double)detection.position.y * 1000 << " mm";
            cv::putText(rgbImage, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream depthZ;
            depthZ << "Z: " << (double)detection.position.z * 1000 << " mm";
            cv::putText(rgbImage, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            cv::rectangle(rgbImage, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        sensor_msgs::ImagePtr detected_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbImage).toImageMsg();
        detected_pub.publish(detected_image);
        if(!rgbImage.empty()) {
            cv::imshow("rgb", rgbImage);
            cv::waitKey(1);
        }
        ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}


