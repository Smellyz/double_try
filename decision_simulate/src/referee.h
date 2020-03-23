#ifndef REFEREE_H
#define REFEREE_H

#include <ros/ros.h>
#include "robot_msgs/GameBuff.h"
#include "robot_msgs/GameResult.h"
#include "robot_msgs/GameStatus.h"
#include "robot_msgs/RobotDamage.h"
#include "robot_msgs/RobotStatus.h"
#include "robot_msgs/CornerDetection.h"
#include "robot_msgs/ShootInfo.h"
#include "robot_msgs/GameSurvivor.h"
#include <thread>
#include <opencv2/opencv.hpp>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

namespace robot_decision_simulate
{
    typedef struct
    {
        float x;
        float y;
        float yaw;
    } PoseData;
    typedef struct
    {
        int self_id;
        int enemy_id;
        int enemy_armor_id;
        int shoot_;
    } DetectInfo;

    typedef struct
    {
        uint8_t F1_zone_status;
        uint8_t F1_zone_buff_debuff_status;
        uint8_t F2_zone_status;
        uint8_t F2_zone_buff_debuff_status;
        uint8_t F3_zone_status;
        uint8_t F3_zone_buff_debuff_status;
        uint8_t F4_zone_status;
        uint8_t F4_zone_buff_debuff_status;
        uint8_t F5_zone_status;
        uint8_t F5_zone_buff_debuff_status;
        uint8_t F6_zone_status;
        uint8_t F6_zone_buff_debuff_status;
    } cmd_game_buff;

    class Referee; //???为什么要这么写
    class RefereeRobot
    {
        public:
        RefereeRobot(Referee *referee, int id);
        void ShootInfoCallback(const robot_msgs::ShootInfo::ConstPtr &msg);
        void Update(float time);
        //new
        void Update_bloodnum();
        void Update_bulletnum();
        void Update_debuff();
        //
        void CarPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void GimbalPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);
        PoseData GetCarPose();
        int GetRobotId();
        bool calDistance(float self_x, float self_y, float enemy_x, float enemy_y);  
        bool calDirection(float self_x, float self_y, float enemy_x, float enemy_y, float gimbal_yaw);      
        DetectInfo DetectEnemy(std::vector<RefereeRobot*> robot_vector);
        int GetRobotPartner(int currentrobot_id);
        bool ObstructOrNot(float self_x, float self_y, float enemy_x, float enemy_y);
        int WhichArmor(float self_x, float self_y, float enemy_x, float enemy_y, float enemy_yaw);
        bool ShootBullet();
        void ChangeBloodAndBulletnum();
        int GetRemainBlood();
        void SetRemainBlood(int remain_blood);
        int GetRemainBullet();

        private:
        int id_;
        Referee *referee_;
        ros::NodeHandle nh_;
        ros::Publisher robot_damage_pub_;
        ros::Publisher robot_status_pub_;
        ros::Subscriber shoot_info_sub_;
        int buff_type_;
        float rate;
        // 1为红方回血区 2为红方弹药补给区 3为蓝方回血区 4为蓝方弹药补给区 5为禁止射击区 6为禁止移动
        int buff_remain_time_;
        bool buff_blood_flag ;
        bool buff_bullet_flag ;
        int minute_only;
        //new
        int shoot_num_last;
        int bullet_remain_num_;
        int blood_remain_num_;
        int quant_heat;
        robot_msgs::RobotStatus robot_status_;
        int armor_id; //装甲id
        bool hurt_flag; //是否受到伤害
        float ShootBulletNum_count = 0;
        //
        bool gimbal_output_;
        bool chassis_output_;
        robot_msgs::ShootInfo shoot_info_;
        ros::Subscriber car_pose_sub_;
        ros::Subscriber gimbal_pose_sub_;
        PoseData car_pose_;
        PoseData gimbal_pose_;
        int attack_info[4][4];

        float vertical_lines[16][3] ={
            {1.0, 3.85, 4.1},   //1
            {1.5, 2.425, 2.675},   //2
            {2.5, 2.425, 2.675},
            {1.5, 0, 1},  //3
            {1.75, 0, 1},
            {3.6, 1, 1.25}, //4
            {4.6, 1, 1.25},
            {3.5, 3.85, 4.1},  //5
            {4.5, 3.85, 4.1},
            {5.6, 2.425, 2.675},  //6
            {6.6, 2.425, 2.675},
            {7.1, 1, 1.25},  //7
            {6.35, 4.1, 5.1},  //8
            {6.6, 4.1, 5.1},
            {3.9, 2.4, 2.7},// 9
            {4.2, 2.4, 2.7}
        };
        float horizontal_lines[16][3] ={
            {3.85, 0, 1},   //1
            {4.1, 0, 1},
            {2.425, 1.5, 2.5},   //2
            {2.675, 1.5, 2.5},
            {1, 1.5, 1.75},  //3
            {1, 3.6, 4.6}, //4
            {1.25, 3.6, 4.6},
            {3.85, 3.5, 4.5},  //5
            {4.1, 3.5, 4.5},
            {2.425, 5.6, 6.6},  //6
            {2.675, 5.6, 6.6},
            {1, 7.1, 8.1},  //7
            {1.25, 7.1, 8.1},  
            {4.1, 6.35, 6.6}, //8
            {2.4, 3.9, 4.2},  //9
            {2.7, 3.9, 4.2}
        };
    };
    
    // referee类是用于模拟裁判系统，给机器人的裁判系统接收类robotreferee类发指令
    class Referee
    {
        public:
        Referee();
        ~Referee();
        void RefereeJudge();
        void ShowUI();
        static void on_MouseHandle(int event, int x, int y, int flag, void *param);
        void onMouse(int event, int x, int y);

        // 下面是新增的××××××××××××××××××××××××××××××
        void buff_data_init();
        void buffPub();
        void buffActive();
        void buffJudge(int robot_id, int buff_property);
        RefereeRobot *Id_Return_Robot(int id);
        void BloodChange(DetectInfo robot_detect_info);
        void GetAllRobotsInfo();
        void gamesurvivorPub();
        void gameResultPub();
        void gameStatusPub(float time,int game_status_data_);
        robot_msgs::CornerDetection cornerdetection_update();
          //00110000->48比赛未开始 49准备阶段,50自检阶段,51 5秒倒计时 52对战中,53比赛结算中
        bool buff_debuff_flag[6];
        int buff_debuff_id[6];
        int referee_rate;
    
        private:
        ros::NodeHandle ros_nh_;
        ros::Publisher game_buff_pub_;
        ros::Publisher game_result_pub_;
        ros::Publisher game_status_pub_;
        ros::Publisher game_survivor_pub_;
        ros::Publisher corner_detection_pub_0;
        ros::Publisher corner_detection_pub_1;
        ros::Publisher corner_detection_pub_2;
        ros::Publisher corner_detection_pub_3;
        robot_msgs::CornerDetection corner_detection_;
        robot_msgs::GameStatus game_status_;
        robot_msgs::GameResult game_result_;
        robot_msgs::GameSurvivor game_survivor_;
        robot_msgs::GameBuff game_buff_;
        bool game_over_;
        int game_result_data;
        std::vector<RefereeRobot*> robots_;
        std::thread referee_thread_;
        //cmd_game_buff game_buff_; //cmd_game_buff结构体
        bool game_start_;
        ros::Time begin_time;
        float game_remain_time_;
        bool buff_change_flag;
        int one_minute_only;
        int two_minute_only;
        int buff_rand[6];
        

        RefereeRobot *robot_red_one;
        RefereeRobot *robot_red_two;
        RefereeRobot *robot_bule_one;
        RefereeRobot *robot_bule_two;


        // buff是否激活
        int F1_active = 0;
        int F2_active = 0;
        int F3_active = 0;
        int F4_active = 0;
        int F5_active = 0;
        int F6_active = 0;

        // f1~f6的区域
        float buff_areas[6][2] = {
            {0.23, 2.55},//1
            {1.63, 1.41}, //2
            {3.77, 3.795},//3
            {7.31, 1.45},   //4
            {5.91, 2.59},  //5
            {3.77, 0.205}//6
        };
        int robots_info[4][2]{
            {0, 0},
            {0, 0},
            {0, 0},
            {0, 0}
        };
        
        float buff_width = 0.54;
        float buff_height = 0.48;

    };
}


#endif