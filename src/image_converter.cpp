#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <sys/stat.h>

//static const std::string OPENCV_WINDOW = "Image window";
static int frame_cnt = 0;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish images feed
    image_sub_ = it_.subscribe("/camera/image_mono", 20,
      &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    auto timestamp = msg->header.stamp.toSec();
      auto timestamp_sec = msg->header.stamp.sec;
      auto timestamp_nsec = msg->header.stamp.nsec;
/*
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
*/
    cv::Mat src = cv_ptr->image;
    cv::Mat dst;
    if (src.channels() > 1)
        cv::cvtColor(src, dst, CV_BGR2GRAY);
    else
        dst = src;

    // Get package name this node belongs to
    std::string package_path = ros::package::getPath("image_conversion");
    if (package_path.empty())
        std::cerr << "The package is not found!" << std::endl;

    if (access((package_path + "/images").c_str(), F_OK) != 0 )
        mkdir((package_path + "/images").c_str(), S_IRWXU);

    // Save the images
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << frame_cnt;
    std::string iamge_index = ss.str();
    cv::imwrite(package_path + "/images/" + iamge_index + ".jpg", dst);
    ++frame_cnt;

    std::ofstream times_writer(package_path + "/images/" + "times.txt", std::ofstream::app);
    if (times_writer)
    {
        //times_writer << iamge_index << ' ' << timestamp_sec << '.' <<
        //                std::setw(9) << std::setfill('0') << timestamp_nsec << std::endl;
        times_writer << iamge_index << ' ' << timestamp << std::endl;
        if (!times_writer)
        {
            ROS_INFO("FILE WRITING ERROR!");
            return ;
        }
    }
    else
    {
        ROS_INFO("FILE WRITING ERROR!");
        return ;
    }

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

