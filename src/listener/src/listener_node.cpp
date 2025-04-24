#include <ros/ros.h>
#include <iostream>
#include <rp_stuff/dmap.h>
#include <rp_stuff/draw_helpers.h>
#include <rp_stuff/dmap_localizer.h>

#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <Eigen/Geometry>

// Global variables
ros::Publisher pose_pub;

GridMapping grid_mapping;
DMapLocalizer localizer;
std::vector<Vector2f> obstacles;
bool map_received = false;
bool initial_pose_received = false;

// Parameters
float resolution;
float influence_range = 10.0;
int occupancy_threshold = 90;

void mapCallback(const nav_msgs::OccupancyGrid& msg) {
    if (map_received) {
        ROS_WARN("Mappa già ricevuta, ignorando la nuova.");
        return;
    }
    ROS_INFO("Mappa ricevuta! Header: frame_id=%s", msg.header.frame_id.c_str());
    resolution = msg.info.resolution;
    Vector2f origin(msg.info.origin.position.x, msg.info.origin.position.y);

    obstacles.clear();
    grid_mapping.reset(origin, resolution);

    for (int y = 0; y < msg.info.height; ++y) {
        for (int x = 0; x < msg.info.width; ++x) {
            int index = y * msg.info.width + x;
            int8_t value = msg.data[index];
            if (value >= occupancy_threshold) {
                Vector2f coord = grid_mapping.grid2world(Vector2f(x,y));
                obstacles.push_back(coord);
            }
        }
    }

    localizer.setMap(obstacles, resolution, influence_range);
    localizer.X.setIdentity();
    map_received = true;
    ROS_INFO("Mappa processata con %lu ostacoli", obstacles.size());
}

void publishOdometry(const ros::Time& stamp) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "robot";

    odom.pose.pose.position.x = localizer.X.translation().x();
    odom.pose.pose.position.y = localizer.X.translation().y();

    float yaw = atan2(localizer.X.linear()(1,0), localizer.X.linear()(0,0));
    odom.pose.pose.orientation.z = sin(yaw * 0.5);
    odom.pose.pose.orientation.w = cos(yaw * 0.5);

    pose_pub.publish(odom);
}

void computeScanEndpoints(std::vector<Vector2f>& dest, const sensor_msgs::LaserScan& scan) {
    for (size_t i=0; i<scan.ranges.size(); ++i) {
        float alpha=scan.angle_min+i*scan.angle_increment;
        float r=scan.ranges[i];
        if (r< scan.range_min || r> scan.range_max)
            continue;
        dest.push_back(Vector2f(r*cos(alpha), r*sin(alpha)));
    }
}

void scanCallback(const sensor_msgs::LaserScan& scan) {
    if (!map_received || !initial_pose_received) {
        ROS_WARN_THROTTLE(1.0, "In attesa della mappa e della posa iniziale...");
        return;
    }

    std::vector<Vector2f> scan_points;
    computeScanEndpoints(scan_points, scan);
    localizer.localize(scan_points, 10);
    publishOdometry(scan.header.stamp);

    std::cout << "Localizer: "<<localizer.X.translation().transpose() << std::endl;

}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose) {
    if (initial_pose_received) {
        ROS_WARN("Posa iniziale già ricevuta, ignorando il nuovo messaggio.");
        return;
    }
    ROS_INFO("Posa iniziale ricevuta! Header: frame_id=%s", pose.header.frame_id.c_str());

    localizer.X.translation().x() = pose.pose.pose.position.x;
    localizer.X.translation().y() = pose.pose.pose.position.y;

    Eigen::Quaternionf q(
        pose.pose.pose.orientation.w,
        pose.pose.pose.orientation.x,
        pose.pose.pose.orientation.y,
        pose.pose.pose.orientation.z
    );
    localizer.X.linear() = Eigen::Rotation2Df(q.toRotationMatrix().eulerAngles(0,1,2)[2]).toRotationMatrix();

    initial_pose_received = true;

    std::cout << "Localizer: " << localizer.X.matrix() << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener_node");
    ros::NodeHandle nh;

    pose_pub = nh.advertise<nav_msgs::Odometry>("/odom_corrected", 10);
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 1, initialPoseCallback);
    ros::Subscriber scan_sub = nh.subscribe("/base_scan", 1, scanCallback);

    ROS_INFO("Nodo listener avviato");
    ros::spin();

    return 0;
}
