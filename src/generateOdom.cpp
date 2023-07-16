#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "utility.h"

#include <cstring>
#include <vector>
#include <sstream>



class Odom_Generator
{

public:

    Odom_Generator()
    {
        ros::NodeHandle nh_local("~");
        nh_local.getParam("show_altitude", show_altitude);
        nh_local.getParam("parent_frame_id", odomParentFrameId);
        ROS_WARN("parent_frame_id:%s",odomParentFrameId.c_str());
        nh_local.getParam("child_frame_id", odomChildFrameId);
        nh_local.getParam("coordinate_type", coordinate_type);

        odom_pub = nh_.advertise<nav_msgs::Odometry>("/integrated_nav/Odom", 10);
        path_pub = nh_.advertise<nav_msgs::Path>("/integrated_nav/Path",10, true);
        ROS_WARN("If no Odometry message, maybe you NavsatFix message is empty");

        navsatfix_sub.subscribe(nh_, "/integrated_nav/NavsatFix", 1);
        imu_sub.subscribe(nh_, "/integrated_nav/Imu", 1);
        navsatfix_imu_sync.reset(new NAVSATFIX_IMU_Sync (NAVSATFIX_IMU_Policy (10), navsatfix_sub, imu_sub));
        navsatfix_imu_sync->registerCallback(boost::bind(&Odom_Generator::parseNavsatFixmsgIMUmsgCallback, this, _1, _2));

    }

    void parseNavsatFixmsgIMUmsgCallback(const sensor_msgs::NavSatFix::ConstPtr& msgNavsatFix_in, const sensor_msgs::Imu::ConstPtr& msgIMU_in)
    {
        // std::cout << "gps status: " << msg_in->status.status << std::endl;
        if (msgNavsatFix_in->status.status <= sensor_msgs::NavSatStatus::STATUS_NO_FIX){
            if (gnss_fixed){
                ROS_WARN("GNSS signal no fixed.");
            } else {
                ROS_ERROR("GNSS signal no fixed, please wait for fixed!");
                return;
            }
        }
        gnss_fixed = true;

        // filter nan pLLA.
        if (std::isnan(msgNavsatFix_in->latitude + msgNavsatFix_in->longitude + msgNavsatFix_in->altitude)) {
            ROS_ERROR("Position pLLA is NAN.");
            return;
        }

//        Eigen::Vector3d pLLA(msgNavsatFix_in->latitude, msgNavsatFix_in->longitude, msgNavsatFix_in->altitude);
        geographic_msgs::GeoPoint pLLA;
        pLLA.longitude = msgNavsatFix_in->longitude;
        pLLA.latitude =  msgNavsatFix_in->latitude;
        pLLA.altitude = msgNavsatFix_in->altitude;
        geodesy::UTMPoint pUTM;
        geodesy::fromMsg(pLLA, pUTM);

        if (!initOdometry) {
            ROS_INFO("GNSS Odometry original LLA: %f, %f, %f", msgNavsatFix_in->latitude, msgNavsatFix_in->longitude, msgNavsatFix_in->altitude);
            ROS_INFO("GNSS Odometry original UTM: %f, %f, %f; zone: %d, band: %d", pUTM.easting, pUTM.northing, pUTM.altitude, pUTM.zone, pUTM.band);
            originalUTM = pLLA;
            initOdometry = true;
//            return;
        }

        auto odom_trans = new geometry_msgs::TransformStamped();
        auto msg_out = new nav_msgs::Odometry();

        odom_trans->header.frame_id = odomParentFrameId.c_str();
        msg_out->header.frame_id = odomParentFrameId.c_str();
        odom_trans->header.stamp = msgNavsatFix_in->header.stamp;
        msg_out->header.stamp = msgNavsatFix_in->header.stamp;
        odom_trans->child_frame_id = odomChildFrameId.c_str();
        msg_out->child_frame_id = odomChildFrameId.c_str();

        double x = pUTM.easting - originalUTM.easting;
        double y = pUTM.northing - originalUTM.northing;
        double z = 0;
        if (show_altitude) {
            z = pUTM.altitude - originalUTM.altitude;
        }

        odom_trans->transform.translation.x = x;
        odom_trans->transform.translation.y = y;
        odom_trans->transform.translation.y = z;
        odom_trans->transform.rotation = msgIMU_in->orientation;

        msg_out->pose.pose.position.x = x;
        msg_out->pose.pose.position.y = y;
        msg_out->pose.pose.position.z = z;
        msg_out->pose.pose.orientation = msgIMU_in->orientation;

        // x, y, z, pitch, roll, yaw
        msg_out->pose.covariance[0] = msgNavsatFix_in->position_covariance[0];
        msg_out->pose.covariance[7] = msgNavsatFix_in->position_covariance[4];
        msg_out->pose.covariance[14] = msgNavsatFix_in->position_covariance[8];
        msg_out->pose.covariance[21] = msgIMU_in->orientation_covariance[0];
        msg_out->pose.covariance[28] = msgIMU_in->orientation_covariance[4];
        msg_out->pose.covariance[35] = msgIMU_in->orientation_covariance[8];

        // refer to LIO_SAM_6axis: https://github.com/JokerJohn/LIO_SAM_6AXIS/blob/main/LIO-SAM-6AXIS/src/simpleGpsOdom.cpp
        msg_out->pose.covariance[1] = x;
        msg_out->pose.covariance[2] = y;
        msg_out->pose.covariance[3] = z;
        msg_out->pose.covariance[4] = msgNavsatFix_in->status.status;

        msg_out->twist.twist.linear.x = msgIMU_in->linear_acceleration.x;
        msg_out->twist.twist.linear.y = msgIMU_in->linear_acceleration.y;
        msg_out->twist.twist.linear.z = msgIMU_in->linear_acceleration.z;

        sendOdomTF(odom_trans);
        pubOdom(msg_out);
        generatePathFromOdometry(msg_out);

//        ROS_INFO("Odometry tf parent frame id: %s",odom_trans->header.frame_id.c_str());
//        ROS_INFO("Odometry tf child frame id: %s",odom_trans->child_frame_id.c_str());
//
//        ROS_INFO("Odometry  parent frame id: %s",msg_out->header.frame_id.c_str());
//        ROS_INFO("Odometry  child frame id: %s",msg_out->child_frame_id.c_str());

        delete(odom_trans);
        delete(msg_out);

    }

    void generatePathFromOdometry(const nav_msgs::Odometry * msg_in){
        if(path_init_flag){
            odom_path.header = msg_in->header;
            path_init_flag = false;
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg_in->header;
        pose_stamped.pose = msg_in->pose.pose;

        odom_path.poses.push_back(pose_stamped);

//        ROS_INFO("Path frame id: %s",pose_stamped.header.frame_id.c_str());

        path_pub.publish(odom_path);

    }

    void pubOdom(const nav_msgs::Odometry * msg)
    {
        odom_pub.publish(*msg);
    }

    void sendOdomTF(const geometry_msgs::TransformStamped * msg)
    {
        odom_broadcaster.sendTransform(*msg);
    }



private:

    ros::NodeHandle nh_;
    bool show_altitude = false;
    std::string odomParentFrameId = "map";
    std::string odomChildFrameId = "base_link";
    bool gnss_fixed = false;

    tf2_ros::TransformBroadcaster odom_broadcaster;
    ros::Publisher odom_pub;
    ros::Publisher path_pub;

    nav_msgs::Path odom_path;

    // UTM的起点
    geodesy::UTMPoint originalUTM;

    bool path_init_flag = true;
    std::string coordinate_type = "LLA";
    bool initOdometry = false;

    message_filters::Subscriber<sensor_msgs::NavSatFix> navsatfix_sub;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu> NAVSATFIX_IMU_Policy;
    typedef message_filters::Synchronizer<NAVSATFIX_IMU_Policy> NAVSATFIX_IMU_Sync;
    boost::shared_ptr<NAVSATFIX_IMU_Sync> navsatfix_imu_sync;


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_generator_node");

    Odom_Generator generator;
    ros::spin();
    return 0;
}