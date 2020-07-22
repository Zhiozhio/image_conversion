//
// Created by zhijun on 2020/7/21.
//

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <sys/stat.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");

    if (argc != 3)
    {
        ROS_INFO("usage: rosrun this_node rosbag_file.bag /image_topic_name");
    }

    rosbag::Bag bag;
    bag.open(argv[1], rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(argv[2]));

    unsigned int frame_cnt = 0;

    // Get package name this node belongs to
    std::string package_path = ros::package::getPath("image_conversion");
    if (package_path.empty())
    {
        std::cerr << "The package is not found!" << std::endl;
        return -1;
    }

    if (access((package_path + "/images").c_str(), F_OK) != 0 )
        mkdir((package_path + "/images").c_str(), S_IRWXU);

    std::ofstream times_writer(package_path + "/images/" + "times.txt", std::ofstream::app);

    for (rosbag::MessageInstance const m : view)
    {
        sensor_msgs::Image::ConstPtr img_msg = m.instantiate<sensor_msgs::Image>();

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return -1;
        }

        auto timestamp_sec = img_msg->header.stamp.sec;
        auto timestamp_nsec = img_msg->header.stamp.nsec;

        cv::Mat src = cv_ptr->image;
        cv::Mat dst;
        if (src.channels() > 1)
            cv::cvtColor(src, dst, CV_BGR2GRAY);
        else
            dst = src;


        // Save the images
        std::stringstream ss;
        ss << std::setw(5) << std::setfill('0') << frame_cnt;
        std::string iamge_index = ss.str();
        cv::imwrite(package_path + "/images/" + iamge_index + ".jpg", dst);

        if (times_writer)
        {
            times_writer << iamge_index << ' ' << timestamp_sec << '.' <<
                            std::setw(9) << std::setfill('0') << timestamp_nsec << std::endl;
            //times_writer << iamge_index << ' ' << timestamp << std::endl;
            if (!times_writer)
            {
                ROS_INFO("FILE WRITING ERROR!");
                return -2;
            }
        }
        else
        {
            ROS_INFO("FILE WRITING ERROR!");
            return -2;
        }

        ++frame_cnt;
    }

    bag.close();
    return 0;
}