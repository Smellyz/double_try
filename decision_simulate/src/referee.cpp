#include "referee.h"

// game_status 1hz
// game_buff 1hz
// robot_status 10hz
// bullet_remain 1hz
// 初始血量2000
// 比赛时间倒计时
// buff控制 开始时，比赛一分钟，比赛两分钟，双方加成区对称，回血区200点，弹丸100颗，惩罚区惩罚时间为10秒
// 击打识别，伤害统计 20hz，前20，左右40，后60
// 热量计算 10hz，每发射一颗速度V的弹丸，热量增加V，热量上限240，每秒冷却120，血量低于400时，每秒冷却240
// 设热量为Q，如果360 > Q > 240，每周期扣除血量(Q - 240) * 4，扣完血后计算冷却，若Q > 360，立即扣除血量(Q - 360) * 4，然后令Q = 360
// 弹量统计，比赛刚开始1号机器人有50发弹丸可发射

// 机器人存储buff状态（何种buff，惩罚剩余时间），裁判系统判断机器人是否激活buff，并更新buff激活状态
// 弹量统计由机器人类实现，热量计算由机器人类实现
// 弹丸击打由裁判系统类实现，先判断机器人连线斜率与云台的的方向是否一样，再根据机器人方位与云台方位的夹角判断伤害

namespace robot_decision_simulate
{
    RefereeRobot::RefereeRobot(Referee *referee, int id)
    {
        id_ = id;
        referee_ = referee;
        rate = (float)referee_->referee_rate;
        gimbal_output_ = true;
        chassis_output_ = true;
       // buff_type_ = 0;
        buff_remain_time_= 0;
        buff_blood_flag =true;
        buff_bullet_flag =true;
        minute_only =0;
        shoot_num_last = 0;
        //new
        blood_remain_num_ = 2000;//初始化每台机器人2000点血
        quant_heat = 0;          //初始化枪口热量为0
        if(id_ == 0||id_ == 4)
          bullet_remain_num_ = 50;
        else
          bullet_remain_num_ = 0;
        //
        std::string robot_id = std::to_string(id);
        // 针对不同的机器人，发布不同的状态信息的话题
        robot_damage_pub_ = nh_.advertise<robot_msgs::RobotDamage>("robot_damage_"+robot_id, 1);
        // robot_damage_是发布受到的
        robot_status_pub_ = nh_.advertise<robot_msgs::RobotStatus>("robot_status_"+robot_id, 1);
        shoot_info_sub_ = nh_.subscribe("shoot_info_"+robot_id, 1, &RefereeRobot::ShootInfoCallback, this);
        car_pose_sub_ = nh_.subscribe("robot_"+robot_id+"/base_pose_ground_truth", 1, &RefereeRobot::CarPoseCallback, this);
        gimbal_pose_sub_ = nh_.subscribe("robot_"+std::to_string(id_+1)+"/base_pose_ground_truth", 1, &RefereeRobot::GimbalPoseCallback, this);
    }

    void RefereeRobot::ShootInfoCallback(const robot_msgs::ShootInfo::ConstPtr &msg)
    {
        shoot_info_ = *msg;
    }

//new
    void RefereeRobot::Update(float time)
    {
        //std::cout << "rate is  "<< rate << std::endl;
        // for(int a = 0; a < 4; a++){
        //     for(int b = 0; b < 4; b++){
        //         attack_info[a][b] = referee_->back_attack_info[a][b];
        //     }
        //}
        // 我觉的是更新机器人的状态（血量，热值，受损情况）
        if(blood_remain_num_ > 0){
            robot_status_.id =id_;
            Update_bloodnum();
            Update_bulletnum();
            Update_debuff();
            //std::cout << "refereerobot"<<id_<< std::endl;   
            if ( 60< time && minute_only == 0){
                    buff_blood_flag = true;
                    buff_bullet_flag = true;
                    minute_only = 1;
                }
            if ( 120 <time && minute_only == 1){
                    buff_blood_flag = true;
                    buff_bullet_flag = true;
                    minute_only = 2;
                }
        }
        else{
            std::cout << "robot "<<id_<<" is dead"<< std::endl;
            robot_status_.remain_hp =0;
        } 
    robot_status_pub_.publish(robot_status_);
    }
    //new
    void RefereeRobot::Update_bloodnum()
    { 
        //buff加血
        //  for(int i=0; i<6;i++)
        // std::cout<<(referee_)->buff_debuff_flag[i]<<std::endl;
        if(buff_blood_flag){
            if(id_ == 0||id_ == 2){
                if((referee_)->buff_debuff_flag[0]){
                    blood_remain_num_ =blood_remain_num_ + 200;
                    if(blood_remain_num_ >2000){blood_remain_num_ = 2000;}
                    buff_blood_flag =false;
                    std::cout<<"RED ---robot_0 and robot_2 add blood"<<std::endl;
                }
            }
            else if(id_ == 4||id_ == 6){
                if((referee_)->buff_debuff_flag[2]){
                    blood_remain_num_ =blood_remain_num_ + 200;
                    if(blood_remain_num_ >2000){blood_remain_num_ = 2000;}
                    buff_blood_flag =false;
                    std::cout<<"BLUE ---robot_4 and robot_6 add blood"<<std::endl;
                }
            } 
        }
 
        //子弹攻击扣血
        // for(int i = 0; i < 4; i++){
        //     if(attack_info[i][0] == 1){
        //         std::cout<<"i was hurt!!!!!!!!!qianmian "<<std::endl;
        //         if(attack_info[i][2] == id_){
        //             std::cout<<"i was hurt!!!!!!!!!houmian "<<std::endl;
        //             int attacked_armor = attack_info[i][3];
        //             if(attacked_armor == 1){
        //                 blood_remain_num_ = blood_remain_num_ - 20;
        //             }else if(attacked_armor == 2 || attacked_armor == 3){
        //                 blood_remain_num_ = blood_remain_num_ - 40;
        //             }else if(attacked_armor == 4){
        //                 blood_remain_num_ = blood_remain_num_ - 60;
        //             }
        //         }
        //     }
        // }      
        robot_status_.remain_hp =blood_remain_num_;
    }

// 
    void RefereeRobot::Update_bulletnum()
    {  
        //buff加子弹
     if(buff_bullet_flag){
        if(id_ == 0||id_ == 2) {
          if((referee_)->buff_debuff_flag[1])
          {
               bullet_remain_num_ +=50;
               buff_bullet_flag =false;
               std::cout<<"RED ---robot_0 and robot_2 add bullets"<<std::endl;
          }
        }
        else {
          if((referee_)->buff_debuff_flag[3])
          {
              bullet_remain_num_ +=50;
              buff_bullet_flag =false;
              std::cout<<"BLUE ---robot_4 and robot_6 add bullets"<<std::endl;
          }
        } 
       }
       robot_status_.bullet_remain_num =bullet_remain_num_;
    }

    void RefereeRobot::Update_debuff()
    {
        if(((referee_)->buff_debuff_flag[4])&&(((referee_)->buff_debuff_id[4] == id_))){ //禁止射击
            buff_remain_time_ = 50;
            (referee_)->buff_debuff_flag[4] = false;
            gimbal_output_ = false;  //禁止射击
            robot_status_.gimbal_output = gimbal_output_;
            std::cout << " *************oh no, i can not shoot!!!!!!!!!!!!!!!!!!" << std::endl;
         } 
        if(((referee_)->buff_debuff_flag[5])&&(((referee_)->buff_debuff_id[5] == id_))){//禁止移动
            buff_remain_time_ = 50;
            (referee_)->buff_debuff_flag[5] = false;
            chassis_output_ = false;
            robot_status_.chassis_output = chassis_output_;
            std::cout << " *************oh no, i can not move!!!!!!!!!!!!!!!!!!" << std::endl;
         }   
        if(buff_remain_time_ >= 0){
             buff_remain_time_--;  //裁判系统发布频率5HZ,10秒就是50
             gimbal_output_ = false;  //禁止射击
             robot_status_.gimbal_output = gimbal_output_;
             chassis_output_ = false;
             robot_status_.chassis_output = chassis_output_;
         }else{
             gimbal_output_ = true; 
             robot_status_.gimbal_output = gimbal_output_;
             chassis_output_ = true;
             robot_status_.chassis_output = chassis_output_;
        }
    }

    void RefereeRobot::CarPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        car_pose_.x = msg->pose.pose.position.x;
        car_pose_.y = msg->pose.pose.position.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        car_pose_.yaw = yaw;
    }

    void RefereeRobot::GimbalPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        gimbal_pose_.x = msg->pose.pose.position.x;
        gimbal_pose_.y = msg->pose.pose.position.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        gimbal_pose_.yaw = yaw;
    }

    int RefereeRobot::GetRobotId(){
        return id_;
    }

    PoseData RefereeRobot::GetCarPose(){
        return car_pose_;
    }

    int RefereeRobot::GetRemainBlood(){
        return blood_remain_num_;
    }
    int RefereeRobot::GetRemainBullet(){
        return bullet_remain_num_;
    }

    void RefereeRobot::SetRemainBlood(int remain_blood){
        blood_remain_num_ = remain_blood;
    }

    void RefereeRobot::ChangeBloodAndBulletnum(){
        //根据子弹初速度扣血
       // 在超出射速的情况下，每检测到一颗子弹扣一次血
        int shootrate = shoot_info_.friction_wheel_speed;
        if( (25 < shootrate) && (shootrate<30)){
            blood_remain_num_ = blood_remain_num_ - 200;
            }
        else if((30 <=shootrate) && (shootrate<35)){
            blood_remain_num_ = blood_remain_num_ - 1000;
        }
        else if(35<= shootrate){
            blood_remain_num_ = blood_remain_num_ - 2000;
        }
       //热量扣血
        quant_heat =quant_heat + shoot_info_.friction_wheel_speed;
        if((350 > quant_heat)&&(quant_heat > 240))
         blood_remain_num_ -=(quant_heat - 240) * 4 * (10 / rate) ; // 枪口热量按10Hz频率结算
        if(quant_heat >= 360){
          blood_remain_num_ -=(quant_heat-240)*40;
          quant_heat= 360;
        }
       // 冷却
        if(blood_remain_num_ > 400){
            //quant_heat -=12;
            quant_heat = quant_heat - (1 / rate) * 120;
            robot_status_.heat_cooling_rate = 120;
        }else {
            //quant_heat -=24;
            quant_heat = quant_heat - (1 / rate) * 240;
            robot_status_.heat_cooling_rate = 240;
        }
        robot_status_.heat_cooling_limit = 240;

        //子弹数
       bullet_remain_num_ = bullet_remain_num_ - 1;
       if(bullet_remain_num_==0)
         robot_status_.shooter_output = false;
    }

    bool RefereeRobot::calDistance(float self_x, float self_y, float enemy_x, float enemy_y){
        // 计算两个机器人之间的距离,判断距离是否太远而超出打击范围
        float valid_shoot_distance = 3.0;
        float distance = sqrt(pow((self_x-enemy_x), 2) + pow((self_y-enemy_y), 2));
        // std::cout<< " the distance is: "<< distance <<std::endl;
        if(distance <= valid_shoot_distance){
            return true;
        }else{
            return false; // 不满足返回一个负数
        }
    }

    bool RefereeRobot::calDirection(float self_x, float self_y, float enemy_x, float enemy_y, float gimbal_yaw){
        // 计算机器人连线的方向，以云台相机的视角出发，是否可以发现敌方机器人
        float valid_detect_range = 0.7854; // 假设云台摄像头范围90度
        float k = (enemy_y - self_y) / (enemy_x - self_x);
        float direction = atan(k);
        if(enemy_y <= self_y && enemy_x <= self_x){
            direction = -3.14159 + direction;
        }else if(enemy_y > self_y && enemy_x < self_x){
            direction = 3.14159 + direction;
        }
        float yaw_max = gimbal_yaw + valid_detect_range;
        float yaw_min = gimbal_yaw - valid_detect_range;
        if(yaw_max > 3.14159){
            yaw_max = -(3.14159 * 2 - yaw_max);
            if((direction >= yaw_min && direction <= 3.14159) ||(direction >= -3.14159 && direction <= yaw_max)){
                return true;
            }else{
                return false;}
        }else if(yaw_min < -3.14159){
            yaw_min = 3.14159 * 2 + yaw_min;
            if((direction >yaw_min && direction < 3.14159) ||(direction > -3.14159 && direction < yaw_max)){
                return true;
            }else{
                return false;}
        }else{
            if(direction > yaw_min && direction < yaw_max){
                return true;
            }else{
                return false;}
        }
    }

    bool RefereeRobot::ObstructOrNot(float self_x, float self_y, float enemy_x, float enemy_y){
        // 判断场上障碍物和小车是否阻挡了小车视线
        for(int i = 0; i < 16; i++){    // 判断垂直线
            if(self_x != enemy_x){
                if((vertical_lines[i][0] >= self_x && vertical_lines[i][0] <=enemy_x)||
                (vertical_lines[i][0] <= self_x && vertical_lines[i][0] >=enemy_x)){
                    float point_y = (vertical_lines[i][0] - 
                    enemy_x)*(self_y - enemy_y)/(self_x- enemy_x) + enemy_y;
                    if(point_y >= vertical_lines[i][1] && point_y <= vertical_lines[i][2]){
                        //std::cout<< id_ << " --i haved been obstructed  " <<std::endl;
                        return true;
                    }
                }
            }
        }
        for(int j = 0; j < 16; j++){
            if(self_y != enemy_y){    // 判断水平线
                if((horizontal_lines[j][0] >= self_y && horizontal_lines[j][0] <=enemy_y)||
                (horizontal_lines[j][0] <= self_y && horizontal_lines[j][0] >=enemy_y)){
                    float point_x = (horizontal_lines[j][0] - 
                    enemy_y)/(self_y - enemy_y)*(self_x- enemy_x) + enemy_x;
                    if(point_x >= horizontal_lines[j][1] && point_x <= horizontal_lines[j][2]){
                        //std::cout<< " i haved been obstructed  " <<std::endl;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    int RefereeRobot::WhichArmor(float self_x, float self_y, float enemy_x, float enemy_y, float enemy_yaw){
        float direction = 0;
        float robot_angle = 0.64577;
        if((enemy_x - self_x) != 0){
            float k = (enemy_y - self_y) / (enemy_x - self_x);
            direction = atan(k);
            if(enemy_x < self_x){
                direction = 3.14159 + direction;
            }else if(enemy_y < self_y && enemy_x > self_x){
                direction = 3.14159 * 2 + direction;
            }else{direction = direction;}
        }else{
            if(enemy_y > self_x){direction = 3.14159 / 2;}
            else{direction = 3.14159 * 3 / 2;}
        }
        if(enemy_yaw < 0){enemy_yaw = 3.14159 * 2 + enemy_yaw;}
        float diff_angle = enemy_yaw - direction;
        if(diff_angle < 0){diff_angle = 3.14159 * 2 + diff_angle;}
        if((diff_angle >= 0 && diff_angle <= robot_angle) || (diff_angle >= (3.14159*2-robot_angle) && diff_angle < 3.14159*2)){
            return 2; // 后侧装甲
        }else if(diff_angle > robot_angle && diff_angle <= (3.14159 - robot_angle)){
            return 3; // 左侧装甲
        }else if(diff_angle > (3.14159 - robot_angle) && diff_angle <= (3.14159 + robot_angle)){
            return 1; // 前侧装甲
        }else{
            return 4; // 右侧装甲
        }   
    }

    int RefereeRobot::GetRobotPartner(int currentrobot_id){
        if(currentrobot_id == 0){return 2;}
        if(currentrobot_id == 2){return 0;}
        if(currentrobot_id == 4){return 6;}
        if(currentrobot_id == 6){return 4;}
    }

    bool RefereeRobot::ShootBullet(){
        if(bullet_remain_num_ <=0 || gimbal_output_ == false){
            //std::cout << "  jinlai false*** ShootBulletNum_count is " << ShootBulletNum_count <<std::endl;
            return false;
        }
        else{
            float freq = float(shoot_info_.shoot_freq); //假设每秒shoot_freq颗
            if(ShootBulletNum_count >= (1/freq)){
                //std::cout << "  i/freq is " << (1/freq) <<std::endl;
                //std::cout << "  dayu ShootBulletNum_count is " << ShootBulletNum_count <<std::endl;
                ShootBulletNum_count = 0;
                return true;   
            }else{
                //std::cout << " bumanzu ShootBulletNum_count is " << ShootBulletNum_count <<std::endl;
                ShootBulletNum_count = ShootBulletNum_count + (1/rate);
                return false;
            }    
        }      
    }

    DetectInfo RefereeRobot::DetectEnemy(std::vector<RefereeRobot*> robot_vector){
        // 判断是否发现其他机器人，并返回（谁发现谁，是否射击等信息）
        DetectInfo detect_info;
        float self_gimbal_yaw = gimbal_pose_.yaw;
        float self_robot_yaw = car_pose_.yaw;
       // robot_yaw 和gimbal_yaw 是一致的,同时变化
        int partner_id = GetRobotPartner(id_); 
        for(auto robot = robot_vector.begin(); robot != robot_vector.end(); ++robot){
            if((*robot)->GetRobotId() != id_ && (*robot)->GetRobotId() != partner_id){
                //计算和其他机器人
                float enemy_x =  (*robot)->GetCarPose().x;
                float enemy_y =  (*robot)->GetCarPose().y;
                float enemy_yaw = (*robot)->GetCarPose().yaw;
                int enemy_id = (*robot)->GetRobotId();
                bool distance_valid = calDistance(car_pose_.x, car_pose_.y, enemy_x, enemy_y);
                if(distance_valid){
                    // std::cout<< "distance is satisfy "<<std::endl;
                    bool direction_valid = calDirection(car_pose_.x, car_pose_.y, enemy_x, enemy_y, self_gimbal_yaw);
                    if(direction_valid){
                        // 判断墙(己方机器人?还没加进去)
                        bool obstructed = ObstructOrNot(car_pose_.x, car_pose_.y, enemy_x, enemy_y);
                        if(!obstructed){
                            armor_id = WhichArmor(car_pose_.x, car_pose_.y, enemy_x, enemy_y, enemy_yaw);
                            detect_info.self_id = id_;
                            detect_info.enemy_id = enemy_id;
                            detect_info.enemy_armor_id = armor_id;
                            //std::cout << " detect enemy's armor id is: " << armor_id << std::endl;
                            bool shoot_ = ShootBullet();
                            if(shoot_){  
                                detect_info.shoot_ = 1;
                                //std::cout << " robot_" <<id_<<" shoot!!!!"<< std::endl;
                                //一旦射出子弹，就去处理可能由超速或者热量导致的扣血，以及子弹的变化
                                ChangeBloodAndBulletnum();
                            }else{
                                detect_info.shoot_ = 0;
                            }
                            return detect_info; // 其实只会按robot_vector顺序返回发现的第一个敌人
                        }
                    }
                }
            }
        }
        detect_info.self_id = -1; 
        detect_info.enemy_armor_id = -1;
        detect_info.enemy_armor_id = -1;
        detect_info.shoot_ = 0;
        return detect_info;     
    }



// *********************************以下是总的裁判系统******************************************

    Referee::Referee()
    {
        // 下面先为发布不同的裁判系统信息做好准备
        game_buff_pub_ = ros_nh_.advertise<robot_msgs::GameBuff>("game_buff", 1);
        game_result_pub_ = ros_nh_.advertise<robot_msgs::GameResult>("game_result", 1);
        game_status_pub_ = ros_nh_.advertise<robot_msgs::GameStatus>("game_status", 1);
        game_survivor_pub_= ros_nh_.advertise<robot_msgs::GameSurvivor>("game_survivor", 1);
        corner_detection_pub_0= ros_nh_.advertise<robot_msgs::CornerDetection>("corner_detection_0", 1);
        corner_detection_pub_1= ros_nh_.advertise<robot_msgs::CornerDetection>("corner_detection_1", 1);
        corner_detection_pub_2= ros_nh_.advertise<robot_msgs::CornerDetection>("corner_detection_2", 1);
        corner_detection_pub_3= ros_nh_.advertise<robot_msgs::CornerDetection>("corner_detection_3", 1);
        // 将四个机器人接收裁判信息的系统放进vector中
        referee_rate = 5;
        robot_red_one = new RefereeRobot(this, 0);
        robots_.push_back(robot_red_one);
        robot_red_two = new RefereeRobot(this, 2);
        robots_.push_back(robot_red_two);
        robot_bule_one = new RefereeRobot(this, 4);
        robots_.push_back(robot_bule_one);
        robot_bule_two = new RefereeRobot(this, 6);
        robots_.push_back(robot_bule_two);
        buff_change_flag = true;
        one_minute_only = 0;
        two_minute_only = 0;
        referee_thread_ = std::thread(&Referee::RefereeJudge, this);   // 开启线程
        
    }

    Referee::~Referee()
    {
        if(referee_thread_.joinable())
        {
            referee_thread_.join();
        }
    }

    void Referee::RefereeJudge()
    {
        game_start_ = false;
        game_remain_time_ = 180; // 比赛时间
        buff_change_flag = true;
        game_over_ =false;
        buff_data_init();
        gameStatusPub(0,48);
        ros::Rate rate(referee_rate);
        while(ros::ok())
        {   gameStatusPub(0,49);
            ShowUI();
            float active_time;
            if(game_start_ == true && game_over_ ==false){
                ros::Time current = ros::Time::now();
                ros::Duration active_time_ = current - begin_time;
                active_time = (active_time_).toSec();
                game_remain_time_ = 180 - active_time;
                // std::cout << "game_remain_time: " << game_remain_time_ << std::endl;
                // 裁判系统发布信息
                buffPub();
                buffActive();
                int k = 0;
                for(auto robot = robots_.begin(); robot != robots_.end(); ++robot) {
                    // 获得每个机器人发现了谁，对谁发射子弹（这里是每次一颗，因为频率足够高）
                    //std::cout << " i have entered the xiancheng " << std::endl;
                    DetectInfo robot_detect_info = (*robot)->DetectEnemy(robots_);
                    BloodChange(robot_detect_info);//扣血操作
                    (*robot)->Update(active_time);
                }
                GetAllRobotsInfo();
                corner_detection_pub_0.publish(cornerdetection_update());
                corner_detection_pub_1.publish(cornerdetection_update());
                corner_detection_pub_2.publish(cornerdetection_update());
                corner_detection_pub_3.publish(cornerdetection_update());
                gameStatusPub(game_remain_time_,52);
            }
            gamesurvivorPub();   

            if( active_time>180){
                game_over_ =true;
                game_result_data =0;
            }
            if((robot_red_one)->GetRemainBlood() <=0 && (robot_red_two)->GetRemainBlood()<=0){
                game_over_ =true;
                game_result_data =1;
            }
            if((robot_bule_one)->GetRemainBlood() <=0 && (robot_bule_two)->GetRemainBlood()<=0){
                game_over_ =true;
                game_result_data =2;
            }
            if(game_over_){
                gameStatusPub(0,53);
                gameResultPub();
                std::cout << "game over over "<<std::endl;
            }
            // ros::spinOnce();
            rate.sleep();
        }
    }

    void Referee::BloodChange(DetectInfo robot_detect_info){
        if(robot_detect_info.shoot_ == 1){ //扣血操作
            int hurt_id = robot_detect_info.enemy_id;
            int armor_ = robot_detect_info.enemy_armor_id;
            for(auto robot = robots_.begin(); robot != robots_.end(); ++robot){
                if((*robot)->GetRobotId() == hurt_id){
                    int remain_blood;
                    if(armor_ == 1){remain_blood = (*robot)->GetRemainBlood() - 20;}
                    if(armor_ == 4 || armor_ == 3){remain_blood = (*robot)->GetRemainBlood() - 40;}
                    if(armor_ == 2){remain_blood = (*robot)->GetRemainBlood() - 60;}
                    (*robot)->SetRemainBlood(remain_blood);
                }
            }
        }
    }

    void Referee::GetAllRobotsInfo(){
        for(auto robot = robots_.begin(); robot != robots_.end(); ++robot){
            if((*robot)->GetRobotId() == 0){
                robots_info[0][0] = (*robot)->GetRemainBlood();
                robots_info[0][1] = (*robot)->GetRemainBullet();
            }
            if((*robot)->GetRobotId() == 2){
                robots_info[1][0] = (*robot)->GetRemainBlood();
                robots_info[1][1] = (*robot)->GetRemainBullet();
            }
            if((*robot)->GetRobotId() == 4){
                robots_info[2][0] = (*robot)->GetRemainBlood();
                robots_info[2][1] = (*robot)->GetRemainBullet();
            }
            if((*robot)->GetRobotId() == 6){
                robots_info[3][0] = (*robot)->GetRemainBlood();
                robots_info[3][1] = (*robot)->GetRemainBullet();
            }
        }
    }

    void Referee::ShowUI()
    {
        cv::Mat ui(500, 800, CV_8UC3, cv::Scalar(255,255,255));
        cv::putText(ui, "HITSZ ICRA SIMULATION", cv::Point(200, 50), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0));
        int minute = floor(game_remain_time_ / 60);
        int sec = floor(game_remain_time_ - minute * 60);
        std::string time_str = "Time Remain " + std::to_string(minute) + " : " + std::to_string(sec);
        cv::putText(ui, time_str, cv::Point(200, 100), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));

        std::string robot_red_0 = "robot_0 : Blood_remain--" + std::to_string(robots_info[0][0]) + 
                                    "   bullets_remain--" + std::to_string(robots_info[0][1]);
        cv::putText(ui, robot_red_0, cv::Point(50, 200), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));

        std::string robot_red_2 = "robot_2 : Blood_remain--" + std::to_string(robots_info[1][0]) + 
                                    "   bullets_remain--" + std::to_string(robots_info[1][1]);
        cv::putText(ui, robot_red_2, cv::Point(50, 230), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));

        std::string robot_bule_4 = "robot_4 : Blood_remain--" + std::to_string(robots_info[2][0]) + 
                                    "   bullets_remain--" + std::to_string(robots_info[2][1]);
        cv::putText(ui, robot_bule_4, cv::Point(50, 260), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));

        std::string robot_bule_6 = "robot_6 : Blood_remain--" + std::to_string(robots_info[3][0]) + 
                                    "   bullets_remain--" + std::to_string(robots_info[3][1]);
        cv::putText(ui, robot_bule_6, cv::Point(50, 290), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));
            
        cv::putText(ui, "Begin", cv::Point(10, 490), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0));
        cv::putText(ui, "End", cv::Point(720, 490), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0));
        cv::imshow("Referee", ui);
        cv::setMouseCallback("Referee", on_MouseHandle, this);
        cv::waitKey(1);
    }

    void Referee::on_MouseHandle(int event, int x, int y, int flag, void *param)
    {
        Referee* temp = reinterpret_cast<Referee*>(param);
	    temp->onMouse(event, x, y);
    }

    void Referee::onMouse(int event, int x, int y)
    {
        if(event == cv::EVENT_LBUTTONDOWN)
        {
            if(y >= 464 && y <= 499) {
                if(x >= 11 && x <= 99 && game_start_ == false) {
                    // 按下开始按钮
                    game_start_ = true;
                    // 获取按下开始的时间
                    begin_time = ros::Time::now();
                } else if(x >= 721 && x <= 781) {
                    // 按下停止按钮
                    game_start_ = false;
                }
            }
        }
    }

  void Referee::buffPub()
    {
       if(buff_change_flag)
       {
        srand(time(NULL));
        int rand_m=0,rand_n=0,rand_k=0,rand_l=0;
        while(rand_m == rand_n)
        {
            rand_m=rand()%3;
            rand_n=rand()%3;
            if(rand_m==rand_n)
            continue;
        }
        for(int i=0;i<3;i++)
        {
         rand_k=i;
        if(rand_k !=rand_m &&rand_k !=rand_n)
                break;
        }
        rand_l=rand()%2;
        buff_rand[rand_m+3*rand_l] =1;
        buff_rand[rand_m+3*(!rand_l)] =3;
        rand_l=rand()%2;
        buff_rand[rand_n+3*rand_l] =2;
        buff_rand[rand_n+3*(!rand_l)] =4;
        rand_l=rand()%2;
        buff_rand[rand_k+3*rand_l] =5;
        buff_rand[rand_k+3*(!rand_l)] =6;

        game_buff_.F1_zone_buff_debuff_status= buff_rand[0];
        game_buff_.F2_zone_buff_debuff_status= buff_rand[1];
        game_buff_.F3_zone_buff_debuff_status= buff_rand[2];
        game_buff_.F4_zone_buff_debuff_status= buff_rand[3];
        game_buff_.F5_zone_buff_debuff_status= buff_rand[4];
        game_buff_.F6_zone_buff_debuff_status= buff_rand[5];
        
        buff_change_flag = false;
        for(int i=0;i<6;i++)
         std::cout << "buff "<<buff_rand[i]<<std::endl;
       }
    
      if (game_remain_time_ < 120 && one_minute_only == 0){
            buff_change_flag = true;
            one_minute_only = 1;
        }
        if (game_remain_time_ < 60 && two_minute_only == 0){
            buff_change_flag = true;
            two_minute_only = 1;
        }
        game_buff_.F1_zone_status = F1_active;
        game_buff_.F2_zone_status = F2_active;
        game_buff_.F3_zone_status = F3_active;
        game_buff_.F4_zone_status = F4_active;
        game_buff_.F5_zone_status = F5_active;
        game_buff_.F6_zone_status = F6_active;
        game_buff_pub_.publish(game_buff_);
    }

    // 判断各个buff区是否被激活
    void Referee::buffActive()
    {
        //获取每个机器人位置
        for(auto robot = robots_.begin(); robot != robots_.end(); ++robot) {
            PoseData current_pose = (*robot)->GetCarPose();
            if(current_pose.x > buff_areas[0][0] && current_pose.x < (buff_areas[0][0] + buff_width) && \
            current_pose.y > buff_areas[0][1] && current_pose.y < (buff_areas[0][1] + buff_height)){
                // 经过f1
                if(F1_active == 0){
                     F1_active = 1;
                    int robot_id = (*robot)->GetRobotId();
                    int buff_property = buff_rand[0];
                    buffJudge(robot_id, buff_property);
                }
            }
            else if(current_pose.x > buff_areas[1][0] && current_pose.x < (buff_areas[1][0] + buff_width) && \
            current_pose.y > buff_areas[1][1] && current_pose.y < (buff_areas[1][1] + buff_height)){
                // 经过f2
                if(F2_active == 0){
                    F2_active = 1;
                    int robot_id = (*robot)->GetRobotId();
                    int buff_property = buff_rand[1];
                    buffJudge(robot_id, buff_property);
                }
            }
            else if(current_pose.x > buff_areas[2][0] && current_pose.x < (buff_areas[2][0] + buff_width) && \
            current_pose.y > buff_areas[2][1] && current_pose.y < (buff_areas[2][1] + buff_height)){
                // 经过f3
                if(F3_active == 0){
                    F3_active = 1;
                    int robot_id = (*robot)->GetRobotId();
                    int buff_property = buff_rand[2];
                    buffJudge(robot_id, buff_property);
                }
            }
            else if(current_pose.x > buff_areas[3][0] && current_pose.x < (buff_areas[3][0] + buff_width) && \
            current_pose.y > buff_areas[3][1] && current_pose.y < (buff_areas[3][1] + buff_height)){
                // 经过f4
                if(F4_active == 0){
                    F4_active = 1;
                    int robot_id = (*robot)->GetRobotId();
                    int buff_property = buff_rand[3];
                    buffJudge(robot_id, buff_property);
                }
            }
            else if(current_pose.x > buff_areas[4][0] && current_pose.x < (buff_areas[4][0] + buff_width) && \
            current_pose.y > buff_areas[4][1] && current_pose.y < (buff_areas[4][1] + buff_height)){
                // 经过f5
                if(F5_active == 0){
                    F5_active = 1;
                    int robot_id = (*robot)->GetRobotId();
                    int buff_property = buff_rand[4];
                    buffJudge(robot_id, buff_property);
                }
            }
            else if(current_pose.x > buff_areas[5][0] && current_pose.x < (buff_areas[5][0] + buff_width) && \
            current_pose.y > buff_areas[5][1] && current_pose.y < (buff_areas[5][1] + buff_height)){
                // 经过f6
                if(F6_active == 0){
                    F6_active = 1;
                    int robot_id = (*robot)->GetRobotId();
                    int buff_property = buff_rand[5];
                    buffJudge(robot_id, buff_property);
                }
            }
        }
    }

    RefereeRobot *Referee::Id_Return_Robot(int id){
        if(id ==0){return robot_red_one;}
        if(id ==2){return robot_red_two;}
        if(id ==4){return robot_bule_one;}
        if(id ==6){return robot_bule_two;}

    }

    void Referee::buffJudge(int robot_id, int buff_property){
        // 判断不同机器人激活不同buff，不同的效果
        // std::cout << "i crash!!! the robot_id is: " << robot_id << "buff is:" << buff_property <<std::endl;
        if(buff_property == 1){
            buff_debuff_flag[0] = true;
            buff_debuff_id[0] = robot_id;
            // RefereeRobot *current_robot_1 = Id_Return_Robot(0);
            // RefereeRobot *current_robot_2 = Id_Return_Robot(2);
            //红方两个机器人加血200
            std::cout << "here buff is 1, red_robot adds blood 200" << std::endl;
        }
        if(buff_property == 2){
            buff_debuff_flag[1] = true;
            buff_debuff_id[1] = robot_id;
            // RefereeRobot *current_robot_1 = Id_Return_Robot(0);
            // RefereeRobot *current_robot_2 = Id_Return_Robot(2);
            //红方两个机器人加子弹
            std::cout << "here buff is 2, red_robot adds bullet" << std::endl;
        }
        if(buff_property == 3){
            buff_debuff_flag[2] = true;
            buff_debuff_id[2] = robot_id;
            // RefereeRobot *current_robot_3 = Id_Return_Robot(4);
            // RefereeRobot *current_robot_4 = Id_Return_Robot(6);
            //蓝方两个机器人加血200
            std::cout << "here buff is 3, bule_robot adds blood 200" << std::endl;
        }
        if(buff_property == 4){
            buff_debuff_flag[3] = true;
            buff_debuff_id[3] = robot_id;
            // RefereeRobot *current_robot_3 = Id_Return_Robot(4);
            // RefereeRobot *current_robot_4 = Id_Return_Robot(6);
            //蓝方两个机器人加子弹
            std::cout << "here buff is 4, bule_robot adds bullet" << std::endl;
        }
        if(buff_property == 5){
            buff_debuff_flag[4] = true;
            buff_debuff_id[4] = robot_id;
            std::cout << "debuff5" << std::endl;
            // RefereeRobot *current_robot = Id_Return_Robot(robot_id);
            //std::cout << " this robot position X is: " << (*current_robot).GetCarPose().x << std::endl;
            //(*current_robot).SetBuff(5);
        }
        if(buff_property == 6){
            buff_debuff_flag[5] = true;
            buff_debuff_id[5] = robot_id;
            // RefereeRobot *current_robot = Id_Return_Robot(robot_id);
            //std::cout << " this robot position X is: " << (*current_robot).GetCarPose().x << std::endl;
            //(*current_robot).SetBuff(6);
            std::cout << "debuff6" << std::endl;
        }
    }

     void Referee::buff_data_init(){
        F1_active = 0;
        F2_active = 0;
        F3_active = 0;
        F4_active = 0;
        F5_active = 0;
        F6_active = 0;
        for(int i =0;i<6;i++){
            buff_debuff_flag[i] = false;
            buff_debuff_id[i] = -1;
        }
     }

robot_msgs::CornerDetection Referee::cornerdetection_update()
{
    for(int i=0;i<4;i++){
        srand(time(NULL));
        int rand_findnum =rand()%3;
        int find_robotone,find_robottwo;
        if(rand_findnum ==0) {
        find_robotone =0;
        find_robottwo =0;
        }
        else{
            if(rand_findnum ==1){
                find_robotone =1;
                find_robottwo =0;
            }
            else{
                find_robotone =1;
                find_robottwo =1;
            }
        }
        corner_detection_.camera_id =i;
        corner_detection_.num_find =rand_findnum;

        PoseData corner_find_pose_ =(robot_bule_one)->GetCarPose();
        corner_detection_.robot_id_1 =4*find_robotone;
        corner_detection_.position_x_1 =corner_find_pose_.x*find_robotone;
        corner_detection_.position_y_1 =corner_find_pose_.y*find_robotone;
        corner_detection_. angular_theta_1=corner_find_pose_.yaw *find_robotone;

        corner_find_pose_ =(robot_bule_two)->GetCarPose();
        corner_detection_.robot_id_2 =6*find_robottwo;
        corner_detection_.position_x_2 =corner_find_pose_.x *find_robottwo;
        corner_detection_.position_y_2 =corner_find_pose_.y *find_robottwo;
        corner_detection_.angular_theta_2 =corner_find_pose_.yaw *find_robottwo;
      return corner_detection_;
   }

}

void Referee::gamesurvivorPub(){
   if((robot_red_one)->GetRemainBlood() <=0 )
      game_survivor_.red3 = false;
   else
      game_survivor_.red3 = true;
   if((robot_red_two)->GetRemainBlood() <=0 )
      game_survivor_.red4  = false;
   else
      game_survivor_.red4 = true;
   if((robot_bule_one)->GetRemainBlood() <=0 )
      game_survivor_.blue3 = false;
   else
      game_survivor_.blue3 = true;
   if((robot_bule_two)->GetRemainBlood() <=0)
      game_survivor_.blue4 = false;
   else
      game_survivor_.blue4 = true;
   game_survivor_pub_.publish(game_survivor_);

}

void Referee::gameResultPub(){
    //  game_result_.DRAW =0;
    //  game_result_.RED_WIN=1;
    //  game_result_.BLUE_WIN=2;
     game_result_.result =game_result_data;
     game_result_pub_.publish(game_result_);

 } 

void Referee::gameStatusPub(float time ,int game_status_data_){
    game_status_.game_status =game_status_data_;
    game_status_.remaining_time =time;
    game_status_pub_.publish(game_status_);
}


}
