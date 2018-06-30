#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/Bool.h>
#include <vision_tasks/gateFrontRangeConfig.h>
#include <vision_commons/morph.h>
#include <vision_commons/contour.h>
#include <vision_commons/threshold.h>
#include <vision_commons/filter.h>
#include <math.h>

//cv::Mat horizontalkern = (cv::Mat_<float>(3,3) << 1,1,1,0,0,0,1,1,1)/(float)(9);
//cv::Mat verticalkern = (cv::Mat_<float>(3,3) << 1,0,1,1,0,1,1,0,1)/(float)(9);
cv::Mat kern = (cv::Mat_<float>(3,3) << 1, 0, 0, 1, 1, 0, 1, 1, 1)/(float)(9);

double clahe_clip = 4.0;
int clahe_grid_size = 8;
int clahe_bilateral_iter = 8;
int balanced_bilateral_iter = 4;
double denoise_h = 10.0;
/*
 *int convolution_iter = 5;
 *double brightness_alpha = 5.0;
 *double convolved_clahe_clip = 2.0;
 *int convolved_clahe_grid_size = 3;
 */
int canny_threshold_low = 0;
int canny_threshold_high = 500;
int hough_threshold = 0;
int hough_minline = 0;
int hough_maxgap = 0;
int opening_mat_point = 1;
int opening_iter = 1;
int closing_mat_point = 1;
int closing_iter = 1;

image_transport::Publisher blue_filtered_pub;
image_transport::Publisher thresholded_pub;
//image_transport::Publisher convolved_pub;
image_transport::Publisher marked_pub;
image_transport::Publisher lines_pub;
ros::Publisher coordinates_pub;

std::string camera_frame = "auv-iitk";

float angleWrtY(const cv::Point &v1, const cv::Point &v2)
{
  cv::Point2f v3;
  v3.x = v1.x - v2.x;
  v3.y = v1.y - v2.y;
  float angle = atan2(v3.x, v3.y);
  if(angle < 0.0) angle = -angle;
  return angle*180/CV_PI;
}

cv::Point2i rotatePoint(const cv::Point2i &v1, const cv::Point2i &v2, float angle)
{
  if(v1.x > v2.x)
  {
    cv::Point2i v3 = v1 - v2;
    cv::Point2i finalVertex;
    finalVertex.x = v3.x * cos(angle) - v3.y * sin(angle);
    finalVertex.y = v3.x * sin(angle) + v3.y * cos(angle);
    finalVertex = finalVertex + v2;
    return finalVertex;
  }
  else
  {
    cv::Point2i v3 = v2 - v1;
    cv::Point2i finalVertex;
    finalVertex.x = v3.x * cos(angle) - v3.y * sin(angle);
    finalVertex.y = v3.x * sin(angle) + v3.y * cos(angle);
    finalVertex = finalVertex + v1;
    return finalVertex;
  }
}


void callback(vision_tasks::gateFrontRangeConfig &config, double level){
  clahe_clip = config.clahe_clip;
  clahe_grid_size = config.clahe_grid_size;
  clahe_bilateral_iter = config.clahe_bilateral_iter;
  balanced_bilateral_iter = config.balanced_bilateral_iter;
  denoise_h = config.denoise_h;
  /*
   *convolution_iter = config.convolution_iter;
   *brightness_alpha = config.brightness_alpha;
   *convolved_clahe_clip = config.convolved_clahe_clip;
   *convolved_clahe_grid_size = config.convolved_clahe_grid_size;
   */
  canny_threshold_low = config.canny_threshold_low;
  canny_threshold_high = config.canny_threshold_high;
  hough_threshold = config.hough_threshold;
  hough_minline = config.hough_minline;
  hough_maxgap = config.hough_maxgap;
  opening_mat_point = config.opening_mat_point;
  opening_iter = config.opening_iter;
  closing_mat_point = config.closing_mat_point;
  closing_iter = config.closing_iter;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
  cv_bridge::CvImagePtr cv_img_ptr;
  try{
    cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_img_ptr->image;
    cv::Mat image_marked = cv_img_ptr->image;
    if(!image.empty()){
      cv::Mat blue_filtered = vision_commons::Filter::blue_filter(image, clahe_clip, clahe_grid_size, clahe_bilateral_iter, balanced_bilateral_iter, denoise_h);
      blue_filtered_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", blue_filtered).toImageMsg());
      if(canny_threshold_high > canny_threshold_low) {
        /*
         *std::vector<cv::Mat> horizontal_planes(3);
         *std::vector<cv::Mat> vertical_planes(3);
         *cv::split(blue_filtered, horizontal_planes);
         *cv::split(blue_filtered, vertical_planes);
         *cv::Mat horizontal, vertical;
         *for(int j = 0; j<3; j++){
         *  for(int i = 0; i<3; i++)
         *    cv::filter2D(horizontal_planes[j], horizontal_planes[j], horizontal_planes[j].depth(), horizontalkern);
         *  for(int i = 0; i<3; i++)
         *    cv::filter2D(vertical_planes[j], vertical_planes[j], vertical_planes[j].depth(), verticalkern);
         *}
         *merge(horizontal_planes, horizontal);
         *merge(vertical_planes, vertical);
         */
        /*
         *std::vector<cv::Mat> planes(3);
         *cv::split(blue_filtered, planes);
         *for(int j = 0 ; j < 3 ; j++) {
         *  for(int i = 0 ; i < convolution_iter ; i++) cv::filter2D(planes[j], planes[j], planes[j].depth(), kern);
         *}
         *cv::Mat convolved;
         *merge(planes, convolved);
         *cv::addWeighted(horizontal, 0.5, vertical, 0.5, 0.0,  convolved);
         *cv::Mat copy;
         *convolved.copyTo(copy);
         *copy.convertTo(convolved, -1, brightness_alpha, 0.0);
         *convolved = vision_commons::Filter::clahe(convolved, convolved_clahe_clip, convolved_clahe_grid_size);
         *convolved_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", convolved).toImageMsg());
         */
        std::vector<cv::Vec4i> lines;
        cv::Mat image_lines;
        cv::Canny(blue_filtered, image_lines, canny_threshold_low, canny_threshold_high);
        image_lines = vision_commons::Morph::close(image_lines, 2*closing_mat_point+1, closing_mat_point, closing_mat_point, closing_iter);
        image_lines = vision_commons::Morph::open(image_lines, 2*opening_mat_point+1, opening_mat_point, opening_mat_point, opening_iter);
        lines_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_lines).toImageMsg());
        cv::HoughLinesP(image_lines, lines, 1, CV_PI/180, hough_threshold, hough_minline, hough_maxgap);
        cv::Vec4i lineOfInterest;
        float length2OfInterest = 0;
        for(int i = 0;i<lines.size(); i++)
        {
          cv::Point p1(lines[i][0], lines[i][1]);
          cv::Point p2(lines[i][2], lines[i][3]);
          float angle = angleWrtY(p1, p2);
          float length2 =  (p2.y - p1.y)*(p2.y - p1.y) + (p2.x - p1.x)*(p2.x - p1.x);
          if((angle < 15.0 || angle > 165.0) && length2 > length2OfInterest) {
            cv::line(image_marked, cv::Point2i(lines[i][0], lines[i][1]), cv::Point2i(lines[i][2], lines[i][3]), cv::Scalar(255, 255, 0), 3, 8, 0);
            length2OfInterest = length2;
            lineOfInterest = lines[i];
          }
        }
        geometry_msgs::PointStamped gate_point_message;
        gate_point_message.header.stamp = ros::Time();
        gate_point_message.header.frame_id = camera_frame.c_str();
        gate_point_message.point.x = 0.0;
        gate_point_message.point.y = (lineOfInterest[0] + lineOfInterest[2])/2 - image.size().width/2 + sqrt(length2OfInterest)/2;
        gate_point_message.point.z = image.size().height/2 - (lineOfInterest[1] + lineOfInterest[3])/2;
        ROS_INFO("Gate Center (y, z) = (%.2f, %.2f)", gate_point_message.point.y, gate_point_message.point.z);
        coordinates_pub.publish(gate_point_message);
        cv::circle(image_marked, cv::Point(gate_point_message.point.y + image.size().width/2, image.size().height/2 - gate_point_message.point.z), 1, cv::Scalar(0,155,155), 8, 0);
        cv::circle(image_marked, cv::Point(image.size().width/2, image.size().height/2), 1, cv::Scalar(0,155, 205), 8, 0);

        cv_bridge::CvImage marked_ptr;
        marked_ptr.header = msg->header;
        marked_ptr.encoding = sensor_msgs::image_encodings::BGR8;
        marked_ptr.image = image_marked;
        marked_pub.publish(marked_ptr.toImageMsg());
      }
    }
  }
  catch(cv_bridge::Exception &e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  catch(cv::Exception &e){
    ROS_ERROR("cv exception: %s", e.what());
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "gate_task_front");
  ros::NodeHandle nh;
  dynamic_reconfigure::Server<vision_tasks::gateFrontRangeConfig> server;
  dynamic_reconfigure::Server<vision_tasks::gateFrontRangeConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  image_transport::ImageTransport it(nh);
  blue_filtered_pub = it.advertise("/gate_task/front/blue_filtered", 1);
  thresholded_pub = it.advertise("/gate_task/front/thresholded", 1);
  //convolved_pub = it.advertise("/gate_task/front/convolved", 1);
  lines_pub = it.advertise("/gate_task/front/lines",1);
  marked_pub = it.advertise("/gate_task/front/marked", 1);
  coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/gate_task/front/gate_coordinates", 1000);
  image_transport::Subscriber front_image_sub = it.subscribe("/front_camera/image_raw", 1, imageCallback);
  ros::spin();
  return 0;
}
