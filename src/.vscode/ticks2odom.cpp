#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>

// Parameters
double wheeltrack = 0.300;   // distance between wheels
double wheelradius = 0.075;  // radius of the wheel in meters
int TPR = 2400 * 3;          // ticks per turn
double left_ticks = 0;
double right_ticks = 0;
double last_left_ticks = 0;
double last_right_ticks = 0;
double heading = 0;
bool reset_odom = false;

std::vector<double> imu_quaternion;

// x = 0
double x = 0.24; // consider robot front not base_link
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double heading_offset = 0.0; // offset para zerar o mpu

void resetCallback(const std_msgs::Bool::ConstPtr& msg)
{
    reset_odom = msg->data;
}

void leftTicksCallback(const std_msgs::Float32::ConstPtr& msg)
{
    left_ticks = msg->data;
}

void rightTicksCallback(const std_msgs::Float32::ConstPtr& msg)
{
    right_ticks = msg->data;
}

void headingCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_quaternion = {msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w};
    heading = tf::getYaw(tf::Quaternion(imu_quaternion[0], imu_quaternion[1], imu_quaternion[2], imu_quaternion[3]));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber left_ticks_sub = nh.subscribe("power/status/distance/ticks/left", 1, leftTicksCallback);
    ros::Subscriber right_ticks_sub = nh.subscribe("power/status/distance/ticks/right", 1, rightTicksCallback);
    ros::Subscriber heading_sub = nh.subscribe("sensor/orientation/imu", 1, headingCallback);
    ros::Subscriber reset_odom_sub = nh.subscribe("/odom/reset", 1, resetCallback);
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformBroadcaster base_link_broadcaster;
    ros::Time current_time;
    ros::Time last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(50);

    while (ros::ok())
    {
        current_time = ros::Time::now();
        // ROS_INFO_STREAM("Left ticks: " << left_ticks << " Right ticks: " << right_ticks);

        double delta_L = left_ticks - last_left_ticks;
        double delta_R = right_ticks - last_right_ticks;
        double dl = 2 * M_PI * wheelradius * delta_L / TPR;
        double dr = 2 * M_PI * wheelradius * delta_R / TPR;
        double dc = (dl + dr) / 2;
        double dt = (current_time - last_time).toSec();
        double dth = (dr - dl) / wheeltrack;

        if (dr == dl)
        {
            double dx = dr * cos(th);
            double dy = dr * sin(th);
            x += dx;
            y += dy;
        }
        else
        {
            double radius = dc / dth;
            double iccX = x - radius * sin(th);
            double iccY = y + radius * cos(th);
            double dx = cos(dth) * (x - iccX) - sin(dth) * (y - iccY) + iccX - x;
            double dy = sin(dth) * (x - iccX) + cos(dth) * (y - iccY) + iccY - y;
            x += dx;
            y += dy;
        }

        th = heading - heading_offset;

        if (reset_odom)
        {
            x = 0.24;
            y = 0;
            heading_offset = heading;
        }

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        // compute the odometry relative to the footprint frame
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom
