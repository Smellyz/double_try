#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Point.h"
#include "costmap_interface.h"
#include "global_plan.h"
#include "local_plan.h"
#include <thread>
#include <chrono>
#include "actionlib/server/simple_action_server.h"
#include "robot_msgs/PlannerAction.h"
#include "geometry_msgs/PoseStamped.h"
#include "robot_msgs/TwistAccel.h"
#include <mutex>

namespace robot_decision_simulate
{
    typedef actionlib::SimpleActionServer<robot_msgs::PlannerAction> PlannerServer;
    typedef std::shared_ptr<robot_costmap::CostmapInterface> CostmapPtr;

    class PID
    {
        public:
        void SetParam(float p, float i, float d) {p_ = p; i_ = i; d_ = d;}
        float Cal(int error)
        {
            last_error = current_error;
            current_error = error;
            return p_ * error + i_ * error + d_ * (current_error - last_error);
        }
        private:
        float p_;
        float i_;
        float d_;
        int last_error;
        int current_error;
    };

    enum NodeState{
        IDLE,
        RUNNING,
        PAUSE,
        SUCCESS,
        FAILURE
    };

    class Robot
    {
        public:
        Robot(int id);
        ~Robot();
        void CarPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void GimbalPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void CarImageCallback(const sensor_msgs::Image::ConstPtr &msg);
        void PublishTf();
        void GoalCallback(const robot_msgs::PlannerGoal::ConstPtr &msg);
        void PlanThread();
        void VelConvert();
        private:
        int id_;
        std::string type_;
        geometry_msgs::Twist vel_;
        robot_msgs::TwistAccel cmd_vel_acc_;
        bool new_cmd_acc_;
        std::chrono::high_resolution_clock::time_point time_begin_;
        geometry_msgs::Twist gimbal_ctl_;
        PID pid_;
        ros::NodeHandle ros_nh_;
        ros::Publisher car_twist_pub_;
        ros::Publisher gimbal_twist_pub_;
        ros::Subscriber car_pose_sub_;
        ros::Subscriber car_img_sub_;
        ros::Subscriber gimbal_pose_sub_;
        geometry_msgs::Point position_;
        float car_yaw_;
        float gimbal_yaw_;
        CostmapPtr costmap_ptr_;
        std::shared_ptr<tf::TransformListener> tf_ptr_;
        std::string local_frame_;
        PlannerServer as_;
        NodeState node_state_;
        std::thread plan_thread_;
        std::unique_ptr<robot_planning::GlobalPlan> global_plan;
        std::unique_ptr<robot_planning::LocalPlan> local_plan;
        std::vector<geometry_msgs::PoseStamped> current_path;
        float goal_distance_tolerance_;
        float goal_angle_tolerance_;
        bool local_plan_running_;
        std::thread vel_converter_thread_;
        tf::TransformBroadcaster tf_broadcaster_;
        std::mutex cmd_mutex_;
        robot_msgs::TwistAccel tmp_cmd_vel_acc_;
    };
}

#endif