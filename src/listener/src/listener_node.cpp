#include <ros/ros.h>
#include <iostream>
#include <rp_stuff/dmap.h>
#include <rp_stuff/draw_helpers.h>
#include <rp_stuff/dmap_localizer.h>

#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// try to include tf2 and Geometry
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>

// Global variables
ros::Publisher pose_pub;
tf2_ros::TransformBroadcaster* tf_broadcaster;

GridMapping grid_mapping;
DMapLocalizer localizer;
std::vector<Vector2f> obstacles;
bool map_received = false;
bool initial_pose_received = false;

// Parameters
float resolution;
float influence_range = 10.0;
int occupancy_threshold = 70; // Check again

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    resolution = msg->info.resolution;
    Vector2f origin(msg->info.origin.position.x, msg->info.origin.position.y);
    
    // Pulisci ostacoli precedenti
    obstacles.clear();
    
    // Estrai celle occupate
    for (int y = 0; y < msg->info.height; ++y) {
        for (int x = 0; x < msg->info.width; ++x) {
            int index = y * msg->info.width + x;
            //Check again (dopo)
            int8_t value = msg->data[index];
            if (value >= occupancy_threshold) {
                //Check again dopo
                Vector2f coord = Vector2f(x, y).cast<float>();
                obstacles.push_back(coord);
            }
        }
    }
    
    // Inizializza il localizzatore con questi ostacoli
    localizer.setMap(obstacles, resolution, influence_range);
    grid_mapping.reset(origin, resolution);
    
    map_received = true;
    ROS_INFO("Mappa processata con %lu ostacoli", obstacles.size());
}

// Check again dopo
void publishOdometry(const ros::Time& stamp) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "robot";
    
    // Posizione
    odom.pose.pose.position.x = localizer.X.translation().x();
    odom.pose.pose.position.y = localizer.X.translation().y();
    
    // Orientamento
    float yaw = atan2(localizer.X.linear()(1,0), localizer.X.linear()(0,0));
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    odom.pose.pose.orientation = tf2::toMsg(q);
    
    pose_pub.publish(odom);
}
//Check again dopo
void publishTransform(const ros::Time& stamp) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "robot";
    
    // Posizione
    transform.transform.translation.x = localizer.X.translation().x();
    transform.transform.translation.y = localizer.X.translation().y();
    
    // Orientamento
    float yaw = atan2(localizer.X.linear()(1,0), localizer.X.linear()(0,0));
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    transform.transform.rotation = tf2::toMsg(q);
    
    tf_broadcaster->sendTransform(transform);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    if (!map_received || !initial_pose_received) {
        ROS_WARN_THROTTLE(1.0, "In attesa della mappa e della posa iniziale...");
        return;
    }

    // Converti scan in punti nel frame del robot
    std::vector<Vector2f> scan_points;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float angle = scan->angle_min + i * scan->angle_increment;
        float range = scan->ranges[i];
        
        if (range < scan->range_min || range > scan->range_max) {
            continue;
        }
        
        scan_points.emplace_back(range * cos(angle), range * sin(angle));
    }

    // Esegui localizzazione
    bool success = localizer.localize(scan_points, 10);
    
    if (success) {
        // Pubblica odometry corretto
        publishOdometry(scan->header.stamp);
        
        // Pubblica trasformazione TF
        publishTransform(scan->header.stamp);
        
        ROS_DEBUG_THROTTLE(1.0, "Localizzazione riuscita. Posa: (%.2f, %.2f)",
                          localizer.X.translation().x(), localizer.X.translation().y());
    } else {
        ROS_WARN_THROTTLE(1.0, "Localizzazione fallita - non abbastanza inlieri");
    }
}

//Check again
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {
    // Imposta la posa iniziale
    localizer.X.translation().x() = pose->pose.pose.position.x;
    localizer.X.translation().y() = pose->pose.pose.position.y;
    
    // Converti quaternione in matrice di rotazione 2D
    tf2::Quaternion q(
        pose->pose.pose.orientation.x,
        pose->pose.pose.orientation.y,
        pose->pose.pose.orientation.z,
        pose->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    localizer.X.linear() = Eigen::Rotation2Df(yaw).toRotationMatrix();
    
    initial_pose_received = true;
    ROS_INFO("Posa iniziale ricevuta: (%.2f, %.2f, %.2f)", 
             pose->pose.pose.position.x, 
             pose->pose.pose.position.y,
             yaw);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener_node");
    ros::NodeHandle nh;
    
    // Inizializza publisher e broadcaster TF
    pose_pub = nh.advertise<nav_msgs::Odometry>("/odom_corrected", 10);
    tf_broadcaster = new tf2_ros::TransformBroadcaster();
    
    // Sottoscrittori
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    
    //Check again
    ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 10, initialPoseCallback);
    ros::Subscriber scan_sub = nh.subscribe("/base_scan", 10, scanCallback);
    
    ROS_INFO("Nodo localizzatore avviato");
    while (ros::ok()) {
        ros::spinOnce();
    }
    
    delete tf_broadcaster;
    return 0;
}