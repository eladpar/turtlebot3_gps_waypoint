#ifndef GPSFollower_HPP
#define GPSFollower_HPP

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <geographic_msgs/GeoPoint.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

class GPSFollower {
public:
    GPSFollower();
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& data);
    void targetCallback(const geographic_msgs::GeoPoint::ConstPtr& data);
    void getRotation(const nav_msgs::Odometry::ConstPtr& msg);
    geometry_msgs::Twist generateMoveCommand(double angle);
    void spin();

private:
    ros::NodeHandle nh;
    ros::Subscriber fixSub;
    ros::Subscriber targetSub;
    ros::Subscriber odomSub;
    ros::Publisher velPub;
    double targetLatitude;
    double targetLongitude;
    double tolerance;
    double roll, pitch, yaw;
    double target;
    double kp;
    bool once;
    bool start;
    double prevErrorAngle;
    double integralAngle;
    double dt;
    double kpAngle;
    double kiAngle;
    double kdAngle;
    double maxAngularVel;
};

#endif  // FILE_HPP

