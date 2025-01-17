#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>


#ifndef PI
#define        PI         3.14159265358979323846
#endif
#define        FOURTHPI    PI/4
#define        deg2rad     PI / 180
#define        rad2deg     180.0 / PI
#define        equatRad    6378137.0     /* EquatorialRadius [meters]*/
#define        eccentSq    0.00669438   /* EccentricitySquared */

bool start = false;
double initial_point_latitude;
double initial_point_longitude;
nav_msgs::Path gps_path;
double yaw = 0;


void ll2planar(const double lat,
               const double lon,
               const double originLat,
               const double originLon,
               double &north,
               double &east) {
    /* This function implements Flamsteed or natural projection */
    /* The function permits to convert Lat/Long to linear coordinate */

    double latRad = lat * deg2rad;
    double lonRad = lon * deg2rad;

    double latOrigRad = originLat * deg2rad;
    double lonOrigRad = originLon * deg2rad;
    /* compute polar radius */
    double polarRad = sqrt((1 - eccentSq) * equatRad * equatRad);
    /* planar */
    double planarN = equatRad * equatRad /
            sqrt(equatRad * equatRad * cos(latOrigRad) * cos(latOrigRad) +
                 polarRad * polarRad * sin(latOrigRad) * sin(latOrigRad));

    east = planarN * cos(latRad) * (lonRad - lonOrigRad);
    north = planarN * (latRad - latOrigRad);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if(yaw == 0){
        tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
        );

    double roll, pitch, quat_yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, quat_yaw);

    yaw = - quat_yaw - 0.24;
    }
}

void gps2local_back(const sensor_msgs::NavSatFix::ConstPtr& input_msg, ros::Publisher& odom_pub, ros::Publisher& path_pub) {

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";

    if(!start) {
        // Init
        start = true;
        initial_point_latitude = input_msg->latitude;
        initial_point_longitude = input_msg->longitude;
    }
    else {
        // GPS2local
        double north;
        double east;
        ll2planar(input_msg->latitude, input_msg->longitude, initial_point_latitude, initial_point_longitude, north, east);
          
        double rotated_east = cos(yaw) * east - sin(yaw) * north;
        double rotated_north = sin(yaw) * east + cos(yaw) * north;

        odom_msg.pose.pose.position.x = rotated_east;
        odom_msg.pose.pose.position.y = rotated_north;
        odom_msg.pose.pose.position.z = 0.0;

        geometry_msgs::PoseStamped gps_pose_stamped;
        gps_pose_stamped.header = odom_msg.header;
        gps_pose_stamped.pose = odom_msg.pose.pose;
        gps_path.header = odom_msg.header;
        gps_path.poses.push_back(gps_pose_stamped);

        path_pub.publish(gps_path);
        odom_pub.publish(odom_msg);

        // Pubblicazione trasformazione world2GPS
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(rotated_east, rotated_north, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, atan2(north, east) + yaw);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ground_truth"));
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "GPS2local");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odometry/gps_local", 10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/gps/path", 10);

    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(
    "/gps/fix", 10, 
    [&odom_pub, &path_pub](const sensor_msgs::NavSatFix::ConstPtr& msg) {
        gps2local_back(msg, odom_pub, path_pub);
    });
    ros::Subscriber imu_sub = nh.subscribe("/imu/data_raw", 10, imuCallback);

    ros::spin();

    return 0;
}