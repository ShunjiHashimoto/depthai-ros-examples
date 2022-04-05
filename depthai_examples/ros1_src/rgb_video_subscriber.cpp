#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>

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

auto markerPublisher(depthai_ros_msgs::SpatialDetection spatialdetected){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/oak-d-base-frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = spatialdetected.position.x;
    marker.pose.position.y = spatialdetected.position.y;
    marker.pose.position.z = spatialdetected.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    return marker;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "rgb_subscriber_node");
    ros::NodeHandle nh;

    ros::Subscriber rgb_sub = nh.subscribe("/yolov4_publisher/color/image", 5, rgbCallback);
    ros::Subscriber detection_sub = nh.subscribe("/yolov4_publisher/color/yolov4_Spatial_detections", 5, detectCallback);
    ros::Publisher detected_pub = nh.advertise<sensor_msgs::Image>("/yolov4_publisher/color/detected_image", 1000);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker_for_okd", 1);
    auto color = cv::Scalar(255, 255, 255);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        for(auto detection : detectArray.detections) {
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

            if(detection.results[0].id == 0){
                ROS_INFO("detect Human Pos");
                marker_pub.publish(markerPublisher(detection));
            }
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


