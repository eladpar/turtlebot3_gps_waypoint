#include "GPSFollower.hpp"
#include "utils.hpp"

geometry_msgs::Twist GPSFollower::generateMoveCommand(double angle) {
    // Calculate current angle difference
    double angle_rad = static_cast<double>(M_PI / 180.0) * angle;
    double angleDiff = angle_rad - yaw;
    angleDiff = normalizeAngleDiff(angleDiff); // Normalize angle between -180 and 180 degrees

    // PID control for angle difference
    double errorAngle = angleDiff;
    integralAngle += errorAngle * dt;
    double derivativeAngle = (errorAngle - prevErrorAngle) / dt;

    // PID control law for angle
    double correctionAngle = (kpAngle * errorAngle) + (kiAngle * integralAngle) + (kdAngle * derivativeAngle);
    // For debug
    // ROS_INFO_STREAM("correctionAngle: " << correctionAngle << " | angle: " << angle << " | errorAngle " << errorAngle << " | angleDiff: " << angleDiff << " | yaw: " << yaw);

    // Update previous error for angle control
    prevErrorAngle = errorAngle;

    // Apply a saturation limit to the correction for angular velocity
    correctionAngle = std::max(std::min(correctionAngle, maxAngularVel), -maxAngularVel);
    // Create a Twist message to move the robot
    geometry_msgs::Twist moveCmd;
    moveCmd.linear.x = 0.5;  // Set linear velocity to 0.5 m/s
    moveCmd.angular.z = correctionAngle;  // Use PID output for angular velocity
    return moveCmd;
}
// Constructor
GPSFollower::GPSFollower() : nh("~") {
    // Initialize node handle, private node handle for getting parameters

    // Subscribers
    fixSub = nh.subscribe("/robot_location", 1, &GPSFollower::gpsCallback, this);
    targetSub = nh.subscribe("/target_location", 1, &GPSFollower::targetCallback, this);
    odomSub = nh.subscribe("/odom", 1, &GPSFollower::getRotation, this);

    // Publisher
    velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Initialize variables
    nh.param<double>("target_latitude", targetLatitude, 32.072617848159176);
    nh.param<double>("target_longitude", targetLongitude, 34.787039205658836);
    nh.param<double>("tolerance", tolerance, 1.0);

    // Set initial values for variables
    roll = pitch = yaw = 0.0;
    target = 90;
    kp = 0.5;
    once = false;
    start = false;
    prevErrorAngle = 0.0;
    integralAngle = 0.0;
    dt = 0.1;
    kpAngle = 0.2;
    kiAngle = 0.0;
    kdAngle = 0.05;
    maxAngularVel = 0.2;
}

void GPSFollower::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& data) {
    double currentLatitude = data->latitude;
    double currentLongitude = data->longitude;

    if (start) {
        // Calculate the angle between the current position and the target coordinates

        double bearing_ = calcBearing(currentLongitude, currentLatitude, targetLongitude, targetLatitude);
        double angleToTarget = 360 - bearing_;
        double distance = calcDistance(currentLongitude, currentLatitude, targetLongitude, targetLatitude);
  
        // If the robot hasn't reached the target coordinates yet
        if (distance > tolerance) {
            // Calculate the angle between the current position and the target coordinates

            // Generate move command using PID control
            geometry_msgs::Twist moveCmd = generateMoveCommand(angleToTarget);
            
            // For debug
            // ROS_INFO_STREAM("Distance: " << distance << " | angle to target: " << angleToTarget << " | yaw " << yaw << " | difference in angle is: " << (angleToTarget - yaw));
            
            velPub.publish(moveCmd);
        } else {
            // Stop the robot when it reaches the target coordinates
            geometry_msgs::Twist stopCmd;
            velPub.publish(stopCmd);
            // Return to initial Values
            kp = 0.5;
            once = false;
            start = false;
            prevErrorAngle = 0.0;
            integralAngle = 0.0;
            dt = 0.1;
            maxAngularVel = 0.2;
            ROS_INFO("Target coordinates reached!");
        }
    }
}

void GPSFollower::targetCallback(const geographic_msgs::GeoPoint::ConstPtr& data) {
    targetLatitude = data->latitude;
    targetLongitude = data->longitude;
    start = true; // Set start flag to true upon receiving target coordinates
    ROS_INFO_STREAM("New target coordinates received - Latitude: " << targetLatitude << ", Longitude: " << targetLongitude);
}

void GPSFollower::getRotation(const nav_msgs::Odometry::ConstPtr& msg) {
    // Assuming the orientation information is contained within the message
    // Extract orientation data from the Odometry message
    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

}


void GPSFollower::spin() {
    ros::Rate loopRate(10);  // Adjust loop rate as needed

    while (ros::ok()) {
        // Implement main loop logic
        ros::spinOnce();
        loopRate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_follower_node");
    GPSFollower gpsFollower;
    gpsFollower.spin();
    return 0;
}
