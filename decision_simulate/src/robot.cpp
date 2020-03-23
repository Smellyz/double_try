#include "robot.h"

namespace robot_decision_simulate
{
    Robot::Robot(int id) : 
        id_(id),
        as_(ros_nh_,"planner_node_action_"+std::to_string(id),boost::bind(&Robot::GoalCallback,this,_1),false),
        node_state_(NodeState::IDLE),
        local_plan_running_(false)
    {
        if(id_ < 4) type_ = "blue";
        else type_ = "red";

        std::string robot_id = std::to_string(id_);
        car_twist_pub_ = ros_nh_.advertise<geometry_msgs::Twist>("robot_"+robot_id+"/cmd_vel", 1);
        car_pose_sub_ = ros_nh_.subscribe("robot_"+robot_id+"/base_pose_ground_truth", 10, &Robot::CarPoseCallback, this);

        std::string gimbal_id = std::to_string(id_ + 1);
        car_img_sub_ = ros_nh_.subscribe("robot_"+gimbal_id+"/image", 10, &Robot::CarImageCallback, this);
        gimbal_twist_pub_ = ros_nh_.advertise<geometry_msgs::Twist>("robot_"+gimbal_id+"/cmd_vel", 1);
        gimbal_pose_sub_ = ros_nh_.subscribe("robot_"+gimbal_id+"/base_pose_ground_truth", 10, &Robot::GimbalPoseCallback, this);
        pid_.SetParam(0.1, 0, 0);

        // 初始化map与base_link的tf
        local_frame_ = "base_link_" + robot_id;
        
        as_.start();
        goal_distance_tolerance_ = 0.15;
        goal_angle_tolerance_ = 0.15;
        tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
        // 初始化全局路径规划
        std::string map_path = ros::package::getPath("robot_costmap") + "/config/costmap_for_simulate_global_" + robot_id + ".prototxt";
        costmap_ptr_ = std::make_shared<robot_costmap::CostmapInterface>("simulate_costmap_global_"+robot_id, *tf_ptr_, map_path.c_str());
        global_plan = std::make_unique<robot_planning::GlobalPlan>(costmap_ptr_);
        // 初始化局部路径规划
        map_path = ros::package::getPath("robot_costmap") + "/config/costmap_for_simulate_local_" + robot_id + ".prototxt";
        costmap_ptr_ = std::make_shared<robot_costmap::CostmapInterface>("simulate_costmap_local_"+robot_id, *tf_ptr_, map_path.c_str());
        local_plan = std::make_unique<robot_planning::LocalPlan>(costmap_ptr_, "robot_"+robot_id+"/base_pose_ground_truth");
        plan_thread_ = std::thread(&Robot::PlanThread, this);

        vel_converter_thread_ = std::thread(&Robot::VelConvert, this);
    }

    Robot::~Robot() {
        local_plan_running_ = false;
        if(plan_thread_.joinable())
        {
            plan_thread_.join();
        }
        if(vel_converter_thread_.joinable())
        {
            vel_converter_thread_.join();
        }
    }

    void Robot::CarPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        position_.x = msg->pose.pose.position.x;
        position_.y = msg->pose.pose.position.y;
        car_yaw_ = yaw;
        PublishTf();
    }

    void Robot::GimbalPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        gimbal_yaw_ = yaw;
    }

    void Robot::CarImageCallback(const sensor_msgs::Image::ConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr -> image;
        // std::string str;
        // std::stringstream ss;
        // std::string image_id = std::to_string(id_);
        // ss << "image" << image_id;
        // ss >> str;
        // cv::imshow(str, img);
        // cv::waitKey(1);
        std::vector<cv::Mat> channels;
        cv::split(img, channels);
        cv::threshold(channels.at(0), channels.at(0), 210, 255, cv::THRESH_BINARY);
        cv::threshold(channels.at(1), channels.at(1), 210, 255, cv::THRESH_BINARY);
        cv::threshold(channels.at(2), channels.at(2), 210, 255, cv::THRESH_BINARY);
        if(type_ == "red") {
            channels.at(0) -= channels.at(2);
            channels.at(1) -= channels.at(2);
        } else if(type_ == "blue") {
            channels.at(1) -= channels.at(0);
            channels.at(2) -= channels.at(0);
        }
        
        std::vector<std::vector<cv::Point> > contours_gimbal;
        std::vector<std::vector<cv::Point> > contours_chassis;
        std::vector<int> gimbal_position;
        cv::findContours(channels.at(1), contours_gimbal, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        if(contours_gimbal.size() == 0) return;
        for(int i = 0; i < contours_gimbal.size(); i++)
        {
            cv::Rect boundRect = cv::boundingRect(contours_gimbal[i]);
            gimbal_position.push_back((boundRect.tl().x + boundRect.br().x) / 2);
        }
        std::vector<int> chassis_position;
        if(type_ == "red")
            cv::findContours(channels.at(0), contours_chassis, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        else if(type_ == "blue")
            cv::findContours(channels.at(2), contours_chassis, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        if(contours_chassis.size() == 0) return;
        for(int i = 0; i < contours_chassis.size(); i++)
        {
            cv::Rect boundRect = cv::boundingRect(contours_chassis[i]);
            chassis_position.push_back((boundRect.tl().x + boundRect.br().x) / 2);
        }
        int target_position;
        for(int i = 0; i < gimbal_position.size(); i++)
        {
            for(int j = 0; j < chassis_position.size(); j++)
            {
                if(std::abs(gimbal_position[i] - chassis_position[j]) < 5)
                {
                    target_position = gimbal_position[i];
                    break;
                }
            }
        }
        gimbal_ctl_.angular.z = pid_.Cal(28 - target_position);
        if(gimbal_yaw_ >= 0.7 && target_position < 28) gimbal_ctl_.angular.z = 0;
        if(gimbal_yaw_ <= -0.7 && target_position > 28) gimbal_ctl_.angular.z = 0;
        gimbal_twist_pub_.publish(gimbal_ctl_);
    }

    void Robot::PublishTf()
    {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(position_.x, position_.y, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, car_yaw_);
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", local_frame_));
    }

    void Robot::GoalCallback(const robot_msgs::PlannerGoal::ConstPtr &msg)
    {
        robot_msgs::PlannerFeedback feedback;
        robot_msgs::PlannerResult result;

        global_plan->SetGoal(msg->goal);
        if(node_state_ != NodeState::RUNNING) {
            node_state_ = NodeState::RUNNING;
        }

        while(node_state_ == NodeState::RUNNING)
        {
            if (as_.isPreemptRequested()) {
                if (as_.isNewGoalAvailable()) {
                    as_.setPreempted();
                    local_plan_running_ = false;
                    ROS_INFO("Override!");
                    break;
                }else{
                    as_.setPreempted();
                    node_state_ = NodeState::IDLE;
                    ROS_INFO("Cancel!");
                    break;
                }
            }
            feedback.error_code = NodeState::RUNNING;
            as_.publishFeedback(feedback);
        }

        if(node_state_ == NodeState::SUCCESS)
        {
            result.error_code = NodeState::SUCCESS;
            as_.setSucceeded(result);
        }

        else if(node_state_ == NodeState::FAILURE)
        {
            result.error_code = NodeState::FAILURE;
            as_.setSucceeded(result);
        }
    }

    void Robot::PlanThread()
    {
        ros::Rate rate(30);
        while(ros::ok())
        {
            if(node_state_ == NodeState::RUNNING)
            {
                geometry_msgs::PoseStamped current_start;
                geometry_msgs::PoseStamped current_goal;

                if (!costmap_ptr_->GetRobotPose(current_start))
                {
                    ROS_ERROR_STREAM("Cannot Get Start Position!");
                    node_state_ = NodeState::FAILURE;
                }
                current_goal = global_plan->GetGoal();
                if (current_goal.header.frame_id != costmap_ptr_->GetGlobalFrameID())
                {
                    current_goal = costmap_ptr_->Pose2GlobalFrame(current_goal);
                    global_plan->SetGoal(current_goal);
                }
                // std::cout << "x: " << current_start.pose.position.x << ",y: " << current_start.pose.position.y << std::endl;
                if(local_plan_running_ == false)
                {
                    if(!global_plan->Plan(current_start, current_goal, current_path))
                    {
                        node_state_ = NodeState::FAILURE;
                    }
                    else
                    {
                        local_plan_running_ = true;
                    }
                }
                if(local_plan_running_ == true)
                {
                    if(!global_plan->IsPathAvailable(current_path) && !local_plan->IsGoalReached())
                    {
                        local_plan_running_ = false;
                        continue;
                    }
                    nav_msgs::Path path_;
                    path_.poses = current_path;
                    int local_plan_state = local_plan->Plan(path_, cmd_vel_acc_);
                    if(local_plan_state == 0)
                    {
                        node_state_ = NodeState::FAILURE;
                    }
                    else if(local_plan_state == 2)
                    {
                        node_state_ = NodeState::SUCCESS;
                        local_plan_running_ = false;
                        cmd_vel_acc_.twist.linear.x = 0;
                        cmd_vel_acc_.twist.linear.y = 0;
                        cmd_vel_acc_.twist.angular.z = 0;
                        cmd_vel_acc_.accel.linear.x = 0;
                        cmd_vel_acc_.accel.linear.y = 0;
                        cmd_vel_acc_.accel.angular.z = 0;
                        new_cmd_acc_ = true;
                        tmp_cmd_vel_acc_ = cmd_vel_acc_;
                        std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
                    }
                    else if(local_plan_state == 1)
                    {
                        // RUNNING
                        new_cmd_acc_ = true;
                        tmp_cmd_vel_acc_ = cmd_vel_acc_;
                        std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
                        std::cout << "linear.x: " << cmd_vel_acc_.twist.linear.x << 
                          ", linear.y: " << cmd_vel_acc_.twist.linear.y << 
                          ", accel.x: " << cmd_vel_acc_.accel.linear.x << 
                          ", accel.y: " << cmd_vel_acc_.accel.linear.y << std::endl;
                    }
                }
            }
            else if(node_state_ == NodeState::FAILURE)
            {
                cmd_vel_acc_.twist.linear.x = 0;
                cmd_vel_acc_.twist.linear.y = 0;
                cmd_vel_acc_.twist.angular.z = 0;
                cmd_vel_acc_.accel.linear.x = 0;
                cmd_vel_acc_.accel.linear.y = 0;
                cmd_vel_acc_.accel.angular.z = 0;
                new_cmd_acc_ = true;
                tmp_cmd_vel_acc_ = cmd_vel_acc_;
                std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
            }
            rate.sleep();
        }
    }

    void Robot::VelConvert()
    {
        while(ros::ok())
        {
            if (new_cmd_acc_) {
                std::lock_guard<std::mutex> cmd_guard(cmd_mutex_);
                vel_ = tmp_cmd_vel_acc_.twist;
                car_twist_pub_.publish(vel_);
                new_cmd_acc_ = false;
                time_begin_ = std::chrono::high_resolution_clock::now();
                continue;
            }
            auto actual_time = std::chrono::duration<double, std::ratio<1, 1>>(std::chrono::high_resolution_clock::now() - time_begin_).count();
            time_begin_ = std::chrono::high_resolution_clock::now();

            vel_.linear.x = vel_.linear.x + actual_time * tmp_cmd_vel_acc_.accel.linear.x;
            vel_.linear.y = vel_.linear.y + actual_time * tmp_cmd_vel_acc_.accel.linear.y;
            vel_.angular.z = vel_.angular.z + actual_time * tmp_cmd_vel_acc_.accel.angular.z;

            car_twist_pub_.publish(vel_);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}