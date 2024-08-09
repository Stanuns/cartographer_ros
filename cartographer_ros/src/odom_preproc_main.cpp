//Odometry(Pose) preproocess
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/version.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h> 
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "message_filters/subscriber.h"   
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace cartographer_ros {

class OdomPreproc : public rclcpp::Node{
public:
    explicit OdomPreproc();
    ~OdomPreproc(){};

    OdomPreproc(const OdomPreproc&) = delete;
    OdomPreproc& operator=(const OdomPreproc&) = delete;

private:
    rclcpp::Node::SharedPtr node_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;

    typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry,
        sensor_msgs::msg::Imu
    > SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomCombine_publisher_;

    void OdomCombinePub(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, const sensor_msgs::msg::Imu::ConstSharedPtr& imu);
    nav_msgs::msg::Odometry odom_proc_;
    void init();
    // _Float64 position_x, position_y, orientation_yaw; 
    _Float64 yaw; 

    rclcpp::Time last_timestamp_;
    rclcpp::Time now_timestamp_;
    bool FirstTime;
};

OdomPreproc::OdomPreproc():
    Node("odom_preproc_node"),
    sync_(SyncPolicy(10), odom_sub, imu_sub)
{
    init();

    odomCombine_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    odom_sub.subscribe(this, "/wheel_odom");
    imu_sub.subscribe(this, "/imu");

    sync_.registerCallback(&OdomPreproc::OdomCombinePub, this); 
}

void OdomPreproc::init(){
    FirstTime = true;
}

void OdomPreproc::OdomCombinePub(const nav_msgs::msg::Odometry::ConstSharedPtr& odom, const sensor_msgs::msg::Imu::ConstSharedPtr& imu){

    odom_proc_.header.stamp = odom->header.stamp;
    odom_proc_.header.frame_id = odom->header.frame_id;
    //need  child_frame_id, otherwise: [cartographer logger]: W0808 20:20:16.000000 867224 tf_bridge.cpp:53] Invalid argument "" passed to lookupTransform argument source_frame - in tf2 frame_ids cannot be empty
    odom_proc_.child_frame_id = odom->child_frame_id;

    // odom_proc_.pose.pose.position = odom->pose.pose.position;
    // odom_proc_.pose.pose.orientation = imu->orientation;
    // 

    //linear velocity from Odometry;
    odom_proc_.twist.twist.linear = odom->twist.twist.linear;
    double vx_temp = odom_proc_.twist.twist.linear.x;
    double vy_temp = odom_proc_.twist.twist.linear.y;
    //angular velocity from Imu 
    odom_proc_.twist.twist.angular = imu->angular_velocity;
    double vyaw_temp = odom_proc_.twist.twist.angular.z;

    odom_proc_.twist.covariance = odom->twist.covariance;
    if(FirstTime){
        // geometry_msgs::msg::Point PointInit(0.0, 0.0, 0.0);
        // odom_proc_.pose.pose.position.x = 23.4;
        odom_proc_.pose.pose.position = geometry_msgs::msg::Point();
        odom_proc_.pose.pose.orientation = geometry_msgs::msg::Quaternion();
        yaw = 0;

        last_timestamp_ = odom->header.stamp;
        FirstTime= false;
    }else{
        //get the position and orientation from [linear velocity from Odometry] and [angular velocity from Imu]
        now_timestamp_ = odom->header.stamp;
        double dt = (now_timestamp_ - last_timestamp_).seconds();
        double delta_x = ( vx_temp*cos(yaw) - vy_temp*sin(yaw) )*dt;
        double delta_y = ( vx_temp*sin(yaw) + vy_temp*cos(yaw) )*dt;
        double delta_yaw = vyaw_temp * dt;

        odom_proc_.pose.pose.position.x += delta_x;
        odom_proc_.pose.pose.position.y += delta_y;
        yaw += delta_yaw;
        //convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        odom_proc_.pose.pose.orientation.x = q.getX();
        odom_proc_.pose.pose.orientation.y = q.getY();
        odom_proc_.pose.pose.orientation.z = q.getZ();
        odom_proc_.pose.pose.orientation.w = q.getW();
        
        odom_proc_.pose.covariance = odom->pose.covariance;
        odom_proc_.twist.covariance = odom->twist.covariance;

        odomCombine_publisher_->publish(odom_proc_);
        last_timestamp_ = odom->header.stamp;
    }
}


}// namespace cartographer_ros


int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cartographer_ros::OdomPreproc>());
  rclcpp::shutdown();
  return 0;
}