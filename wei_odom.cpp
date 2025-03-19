#include <math.h>
#include <stdio.h> 
#include <stdlib.h>
#include <iostream>
#include <string>
#include <chrono>
#include <time.h>
#include <algorithm> 
#include <ros/ros.h>   
// #include "b-spline.h"
////#include <std_msgs/String.h> 
#include <std_msgs/Float64.h> 
#include <std_msgs/Int32.h> 
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

#define save_distance      "/home/user/wei_ws/src/controller/src/position_random/2easy-distance.txt"  //左/右/前測距離
#define save_local_target  "/home/user/wei_ws/src/controller/src/position_random/local_target-111.txt"//區域目標
#define save_local_target_change  "position_random/local_target-change1.txt"//區域目標
#define save_old "/home/user/wei_ws/src/controller/src/position_random/old_.txt"
#define save_new "/home/user/wei_ws/src/controller/src/position_random/new_.txt"
#define load_data_clu "/home/user/wei_ws/src/controller/src/fuzzy_controller/along_wall_controller/10.txt"
#define load_data_FC "/home/user/wei_ws/src/controller/src/fuzzy_controller/search_target_controller/FUZZY_PID/temp_PID.txt" //important controller.
#define position_target    "/home/user/wei_ws/src/controller/src/position_random/position_targetxx.txt"

#define max_step 40000
#define _out_varl 6   /*  輸出變數 (左輪&右輪) */
#define _in_varl  3   /*  輸入變數    */  
#define _rule_number 10   /* 規則數 */ 
#define  _max_rule  30    /* 開記憶體空間 */
#define _rule_delta 10 
#define left_wheel_speed 50.0  /* 左輪輪速 */
#define right_wheel_speed  50.0   /* 右輪輪速  */
#define radius 0.03

#define MIN(a,b) (a < b ? a : b)
// #define _input_scale (1/sensor_range)  /* 給感測器讀值正規化的參數  */
#define randomize() srand((unsigned) time(NULL)) /*  以時間為基底產生隨機值的初始化，不要動這行  */
#define random(x) (rand() % x)   /*  隨機產生亂數，不要動這行  */

double front_safe_distance = 0.4; 
double side_safe_distance = 0.4; 
double weight_speed;
double in[_in_varl+1] ; //感測器正規化後的輸入
double out_y[_out_varl+1] ; //馬達輸出

const int _mem_length = 2*_in_varl+ _out_varl ; // 一個rule的長度

int _in_clu ; //rule_number的暫存變數	
int status;
int counts=0;
int a, ik, ss = 111;
int HH;
double final_z ;

double fuzzy_real[_mem_length*_max_rule+1] ; //  實際輸出規則的變數
double K[_out_varl+1] ; 
double  ave_motor_speed = 0.0;  //平均輪速
double  ave_motor_displacement = 0.0;  //平均位移 ///202011
double min_m[_in_varl+1],max_m[_in_varl+1];//最小中心點 & 最大中心點	

double local_target_x[max_step + 1],
        local_target_y[max_step + 1],
        local_target_z[max_step + 1]; // 紀錄全局路線的方向
double amcl_x, amcl_y, amcl_z;
double local_target_x_record[max_step + 1], local_target_y_record[max_step + 1];
double local_error_x, local_error_y;
double local_error_z, local_error_z_deg;
double trans_new_amcl;//重要
double trans_degree;
double error_orientation_z, error_final_z, Error_orientation_z[max_step+1];
double goal_x, goal_y, goal_z;
double dis_x, dis_y, dis_z; // 現在位置與目標的距離

double error, intergral ,derivative, lastError, error_1, derivative_1, lastError_1, intergral_1;     
double v1, v2;
double final_x,final_y,final_zz;
double angular,linear;
double angular1,linear1;
double laser_temp[361],margin_laser[361],left_min, right_min, straight_min;
float laser_temp_scan[897];
double Tp = 0, turn, angle_deviate;
double rms, local_rms;
double roll,pitch,yaw;
double roll_g,pitch_g,yaw_g;
double roll_s,pitch_s,yaw_s;
double sum_error;
int js = 0;
const float dis = 0.3791; // distance between two wheel

using namespace std;
geometry_msgs::Twist msg;
///////////////// 高斯函數 ////////////////////{

inline double phi(double x,double m,double v)
{ return(  exp( - (x-m)*(x-m)/(v*v) )) ; }

class C_Rule
{
  friend class Population ;
  public :
    double  in_mu_1, in_mu_2;
    double  con[_out_varl+1] ;
    void    In_mem_fir(double *,int) ;
    friend  void Fir_str(double *, int) ;
};

C_Rule  _Rule[_max_rule+1];
void C_Rule::In_mem_fir(double *in,int _rule)  
{
//這裡的 _rule，等於設定變數的_rule_number的值，而且這裡的_rule是一個動態變數，所以會隨著 Fir_str 的 k 值做改變。 
  int i ,l;
  in_mu_1 =  1. ;  // "1." 代表1的float值
  in_mu_2 =  1. ;
  for(i=1;i<= _in_varl;i++)
  {      
    in_mu_1 =  in_mu_1 * phi(in[i] , fuzzy_real[(_rule-1)*_mem_length+i*2-1] , fuzzy_real[(_rule-1)*_mem_length+i*2] ) ;
  // in_mu_2 =  in_mu_2 * phi(in[l] , fuzzy_real[(_rule-1)*_mem_length+l*2-1] , fuzzy_real[(_rule-1)*_mem_length+l*2] ) ;
  }  
}
class amcl_pose_sub_pub_1 
{
  private:
    ros::NodeHandle n;  
    ros::Publisher pub; 
    ros::Publisher pub_goal; 
    // ros::Publisher chatter_pub1;
    ros::Publisher chatter_pub2;
    // ros::Publisher chatter_pub3;
    ros::Subscriber sub_final_goal;
    ros::Subscriber sub_path;
    ros::Subscriber sub_amcl;
    ros::Subscriber sub_laser;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_data1;
    ros::Subscriber sub_data2;
    ros::Subscriber sub_data3;
    ros::Subscriber sub_data4;
  public:
    amcl_pose_sub_pub_1()
    {
      pub=n.advertise<geometry_msgs::Twist>("/cmd_vel_1",3000);  // cmd_vel_1
      pub_goal=n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
      // wei need change, change the theta_d from local target to final goal
      // chatter_pub1=n.advertise<std_msgs::Int32>("chatter1",3000);
      chatter_pub2=n.advertise<std_msgs::Float64>("chatter2",3000);
      // chatter_pub3=n.advertise<std_msgs::Float64>("chatter3",3000);
      //sub_laser=n.subscribe("/p3dx/laser/scan",3000,&amcl_pose_sub_pub_1::lasercallback,this);
      sub_laser=n.subscribe("/scan",3000,&amcl_pose_sub_pub_1::lasercallback,this);
      sub_final_goal = n.subscribe("/move_base_simple/goal", 3000, &amcl_pose_sub_pub_1::Final_Goal,this);
      // sub_path = n.subscribe("/move_base/GlobalPlanner/plan", 3000, &amcl_pose_sub_pub_1::Path_Callback,this);
      sub_path = n.subscribe("/move_base/NavfnROS/plan", 3000, &amcl_pose_sub_pub_1::Path_Callback,this); // 訂閱全局路線
      sub_amcl = n.subscribe("/move_base/feedback", 3000, &amcl_pose_sub_pub_1::amcl_Callback,this);
      sub_odom = n.subscribe("/odom",3000, &amcl_pose_sub_pub_1::final_Callback,this);
      sub_data1 = n.subscribe("/info_1",3000, &amcl_pose_sub_pub_1::chatter1Callback1, this);
      sub_data2 = n.subscribe("/info_2",3000, &amcl_pose_sub_pub_1::chatter1Callback2, this);
      sub_data3 = n.subscribe("/info_3",3000, &amcl_pose_sub_pub_1::chatter1Callback3, this);
      sub_data4 = n.subscribe("/info_4",3000, &amcl_pose_sub_pub_1::chatter1Callback4, this);

      void Final_Goal(const::geometry_msgs::PoseStamped::ConstPtr& F_goal);
      void lasercallback(const::sensor_msgs::LaserScan::ConstPtr& scan);
      void Path_Callback(const ::nav_msgs::Path::ConstPtr & msg);
      void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl);
      void Search_Target();
      void speed_limit(void) ;   // 速度限制器
      void Final_stop();
      void Fir_str(double *, int) ; //Fuzzy系統:讀取後件部與計算激發量
      void fuzzy(int);    // Fuzzy系統:解模糊
      void fuzzy_in(int) ;  // Fuzzy系統:把感測器的讀值做正規化
      void test_open();
      void save_robot_wheelspeed(int);  //左右輪輪速的存檔
      void save_robot_displacement(int);  //離牆距離的存檔  ///202011
      // wei need change, the following seems not the same
      void final_Callback(const nav_msgs::Odometry::ConstPtr& msg);
      void Information(int jj) ;///印出資訊;
      double minimum(int ,int ,int&);
    }
    void chatter1Callback1(const std_msgs::Float64::ConstPtr& info_1){
      ss =  info_1->data;
    }
    void chatter1Callback2(const std_msgs::Int32::ConstPtr& info_2){
      HH =  info_2->data;
    }
    void chatter1Callback3(const std_msgs::Int32::ConstPtr& info_3){
      linear1 =  info_3->data;
    }
    void chatter1Callback4(const std_msgs::Int32::ConstPtr& info_4){
      angular1 =  info_4->data;
    }
    void Final_Goal(const geometry_msgs::PoseStamped::ConstPtr & F_goal) //全域目標的X Y Z 的數值
    {
      goal_x = F_goal->pose.position.x;
      goal_y = F_goal->pose.position.y;
      //orientation_z1=F_goal->pose.orientation.z;
      tf2::Quaternion s;
      tf2::convert(F_goal->pose.orientation,s);
      tf2::Matrix3x3(s).getRPY(roll_s,pitch_s,yaw_s);
      goal_z = yaw_s;
      if( (goal_z <= -M_PI/2) && goal_z >= -M_PI)
        goal_z= -(M_PI/2 + (M_PI + goal_z)); //第四象限為90~180
      else
        goal_z = -(goal_z - M_PI/2);  
    }
    // wei need change
    void final_Callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      final_x = msg->pose.pose.position.x;
      final_y = msg->pose.pose.position.y;
      final_zz = msg->pose.pose.orientation.z;
    }
    void lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan)//抓取雷射數值,右半邊為90~160,前方為160~200,左方為200~270
    {
      int k ;
      for(int i=1;i<=scan->ranges.size();i++)
      {
        laser_temp_scan[i] = scan->ranges[i]; 
      }
      for (int i = 0; i <= scan->ranges.size(); ++i)
      {
        double angle_rad = (scan->angle_min + i * scan->angle_increment) + M_PI;
        int angle_deg = angle_rad*180/M_PI;
        if (!std::isinf(laser_temp_scan[i]))
        {
          if (laser_temp_scan[i]!=0)
          {
            laser_temp[angle_deg] = laser_temp_scan[i];  // 將原始雷達過濾並刪減 保留0～360度
          }
        }         
      }
    }
    void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl)
    {
      int ll = 0;
      // wei need change
      // if(ik < 20 && counts > 50){
      //   amcl_position_x = final_x;
      //   amcl_position_y = final_y;
      //   cout<<"11111111111111111111111111111111111111111111"<<endl;
      // }

      // {
      //   amcl_position_x = amcl->feedback.base_position.pose.position.x;
      //   amcl_position_y = amcl->feedback.base_position.pose.position.y;

      //   cout<<"222222222222222222222222222222222222222222222"<<endl;
      // }
      amcl_x = amcl->feedback.base_position.pose.position.x;
      amcl_y = amcl->feedback.base_position.pose.position.y;
      tf2::Quaternion q;
      tf2::convert(amcl->feedback.base_position.pose.orientation,q);
      tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
      amcl_z = yaw;
      // wei need change, 座標系轉換
      if( amcl_z <= -M_PI/2 && amcl_z >= -M_PI)
        amcl_z= -( M_PI + M_PI/2 + amcl_z); //第四象限為90~180
      else
        amcl_z = -(amcl_z - M_PI/2); 
    }
    void Path_Callback(const ::nav_msgs::Path::ConstPtr & msg) //規劃路徑後產生的所有點,這裡取第10個目標作為區域目標
    { 
      double xx1, yy1;
      int i=0;
      // wei need change
      std::vector<geometry_msgs::PoseStamped> data = msg->poses;
      for(std::vector<geometry_msgs::PoseStamped>::const_iterator it= data.begin(); it!= data.end(); ++it)
      {  
        local_target_x[i]=msg->poses[i].pose.position.x;
        local_target_y[i]=msg->poses[i].pose.position.y;
        tf2::Quaternion g;
        tf2::convert(msg->poses[i].pose.orientation,g);
        tf2::Matrix3x3(g).getRPY(roll_g,pitch_g,yaw_g);
        local_target_z[i] = yaw_g;
        // wei need change, 不太一樣
        local_rms = sqrt(pow(amcl_x-local_target_x[i],2)+pow(amcl_y-local_target_y[i],2));
        if ( local_rms <= 1 - 0.5*fabs(error_orientation_z) / (M_PI / 2))
        {
          ik=i;
        }
        i++;
      } 
        if(xx1!=local_target_x[ik])
        {
        FILE *pfoutold; 
        pfoutold=fopen(position_target,"a");
        if(pfoutold==NULL)
        {
          printf("Fail to open file");
          exit(1);
        }  
        for(int ii=10;ii<20;ii++)
          fprintf(pfoutold,"%lf \t %lf\t",local_target_x[ii],local_target_y[ii]);
          fprintf(pfoutold,"\n");
        fclose(pfoutold);
        }
        xx1 = local_target_x[ik];
        yy1 = local_target_y[ik];

        if(i<=20)
        {  
          ik=i;
          local_target_x[i] = goal_x;
          local_target_y[i] = goal_y; 
          local_target_z[i] = goal_z;
        }
      
    } 
    void coordinate_transform (int jj) //R0S世界徑度角有問題,此程式將其轉換 
    { 
      local_error_x = local_target_x[ik] - amcl_x;
      local_error_y = local_target_y[ik] - amcl_y;
      dis_x = goal_x - amcl_x;
      dis_y = goal_y - amcl_y;
      error_final_z = goal_z-amcl_z;
      rms=sqrt(pow(dis_x,2)+pow(dis_y,2));
      
    }
    void fuzzy_in(int jj)
    { 

        if (local_error_y == 0) local_error_z = 0;
        else local_error_z =atan(abs(local_error_x/local_error_y));
        local_error_z_deg = local_error_z * 180 / M_PI ; 
        trans_new_amcl = amcl_z;
        // wei need change
        //用X、Y分量差位置判別角度差    四個象限對應到的角度與向量軸有差異->>>atan()無法轉換時無法判別分子/母正負 手動調整角度,以符合實際.
        if ( local_error_x>0 && local_error_y >0)   local_error_z_deg =  local_error_z_deg;
        else if( local_error_x<0 && local_error_y >0)  local_error_z_deg = -local_error_z_deg;
        else if( local_error_x<0 && local_error_y <0)  local_error_z_deg = -180 + local_error_z_deg;
        else if( local_error_x>0 && local_error_y <0)  local_error_z_deg =  180 - local_error_z_deg; 	
        
      // 車頭方向 - 位置的角度差 = 車頭需要旋轉的角度 ,判別左旋轉或右旋轉較小的的角度
        if ((local_error_z_deg > 90) && (trans_new_amcl*180/M_PI) <-90) 
        {
          trans_degree = -360+abs(local_error_z_deg)+abs(trans_new_amcl*180/M_PI) ;
        } 
        else if ((local_error_z_deg <-90) && (trans_new_amcl*180/3.14158) >90) 
        {
          trans_degree = 360-abs(local_error_z_deg)-abs(trans_new_amcl*180/M_PI);
        }
        else 
        {
          trans_degree = (local_error_z_deg - trans_new_amcl*180/M_PI);
        }
        error_orientation_z = trans_degree * M_PI / 180;   // error_orientation_z 應該是跟局部路線的誤差角度
        if(abs(error_orientation_z) > M_PI) 
        {
          cout<<"this is bigger than 3.14159 "<<endl;
          cout<<"befor transfer angular is ="<<error_orientation_z<<endl;
          // wei need change, not the same
          if (error_orientation_z>0 && left_min>0.5) error_orientation_z = -2 * M_PI + error_orientation_z;
          else if (error_orientation_z<0 && right_min>0.5) error_orientation_z = 2 * M_PI + error_orientation_z;
          cout<<"after transfer angular is == "<<error_orientation_z<<endl;
        }        
        sum_error = sqrt(pow(local_target_x[ik] - amcl_x, 2)+pow(local_target_y[ik] - amcl_y, 2));
        Error_orientation_z[jj] = error_orientation_z;
        in[1] = abs(error_orientation_z) / M_PI;
        in[2] = abs(Error_orientation_z[jj]-Error_orientation_z[jj-1]) / M_PI; 
        // wei need change
        in[3] = sum_error/1.5; 
    }
    void decideRotation( int &kkk)
    {
      int k;
      int rotation_left = 100, rotation_right = 100;
      left_min = minimum(205, 269,k);
      right_min = minimum(90, 155,k);
      straight_min = minimum(155, 205,k);
      margin_laser[35] = 0.49207; // wei change Tracer20250109
      margin_laser[40] = 0.44073; 
      margin_laser[45] = 0.40190; 
      margin_laser[50] = 0.37247; 
      margin_laser[310] = 0.37247; 
      margin_laser[315] = 0.40190; 
      margin_laser[320] = 0.44073; 
      margin_laser[325] = 0.49207; 
      
      margin_laser[90] = 0.29010;
      margin_laser[95] = 0.29170;
      margin_laser[100] = 0.29562;
      margin_laser[105] = 0.30188;
      margin_laser[110] = 0.31096;
      margin_laser[115] = 0.31017;
      margin_laser[120] = 0.26386;
      margin_laser[125] = 0.23164;
      margin_laser[130] = 0.20743;
      margin_laser[135] = 0.18912;
      margin_laser[140] = 0.17525;
      margin_laser[145] = 0.16425;
      margin_laser[150] = 0.15581;
      margin_laser[155] = 0.14915;
      margin_laser[160] = 0.14418;
      margin_laser[165] = 0.14049;
      margin_laser[170] = 0.13806;
      margin_laser[175] = 0.13670;
      margin_laser[180] = 0.13641;
      margin_laser[185] = 0.13715;
      margin_laser[190] = 0.13898;
      margin_laser[195] = 0.14192;
      margin_laser[200] = 0.14617;
      margin_laser[205] = 0.15179;
      margin_laser[210] = 0.15924;
      margin_laser[215] = 0.16863;
      margin_laser[220] = 0.18088;
      margin_laser[225] = 0.19669;
      margin_laser[230] = 0.21694;
      margin_laser[235] = 0.24445;
      margin_laser[240] = 0.28163;
      margin_laser[245] = 0.31727;
      margin_laser[250] = 0.30667;
      margin_laser[255] = 0.29880;
      margin_laser[260] = 0.29361;
      margin_laser[265] = 0.29069;
      margin_laser[269] = 0.29005;
      margin_laser[270] = 0.29005;
      double turn_safe_dis = 0.1;
      // wei need change, kkk should be removed
      // for(int lsr=90;lsr<=150;lsr=lsr+5)
      // {
      //   if(laser_temp[lsr] < margin_laser[lsr] + turn_safe_dis && laser_temp[lsr] >= 0.15){
      //     kkk=1;
      //     printf("HITTTTTTTTTTTTTTTTTTTTT  Right\n");
      //     printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
      //     printf("右側車子最小距離：%lf\n",right_min); 
      //   }
      // }
      // for(int lsr=150;lsr<=210;lsr=lsr+5)
      // {
      //   if(laser_temp[lsr]<margin_laser[lsr] + turn_safe_dis && laser_temp[lsr] >= 0.15){
      //     kkk=1;
      //     printf("HITTTTTTTTTTTTTTTTTTTTT\n");
      //     printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
      //     printf("前方車子最小距離：%lf\n",straight_min); 
      //   }
      // }
      // for(int lsr=210;lsr<=270;lsr=lsr+5)
      // {
      //   if( laser_temp[lsr]< margin_laser[lsr] + turn_safe_dis && laser_temp[lsr] >= 0.15){
      //     kkk=1;
      //     printf("HITTTTTTTTTTTTTTTTTTTTT  Left\n");
      //     printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
      //     printf("左側車子最小距離：%lf\n",left_min);
      //   }
      // }
    }
    void Search_Target(int jj) 
    {  
      int kkk = 0;
      decideRotation(kkk);
      if(HH!=1)
      {
        intergral = 0;
        intergral_1 = 0;
      }
      // if ( kkk!=1 ) // kkk 是用來檢查是否快撞到的flag
      // { 
        error = error_orientation_z;
        error_1 = in[3];
        derivative = in[2]; 
        if( jj%10==0 )
        {
          intergral+= error/1.; 
          intergral_1 += error_1/1.;
        }
        // wei need change, not the same
        if(intergral_1>30) intergral_1=0;
        if(intergral>15) intergral=0;

        derivative_1 = error_1 - lastError_1;
        turn = K[1]*error + K[2]*intergral + K[3]*derivative ;
        Tp =  abs( K[4]*error_1+ K[5]*intergral_1+K[6]*derivative_1);

        double ang = 0.1745; // 0.1745 = 10 degree, 0.0873 = 5 degree
        if (abs(error)<ang) {   //5.7degree 內直線行走
          intergral=0;
          intergral_1=0;
          printf("==== 直走 ====\n");
          // status=1;
        } 
        else if ( (error>ang) ){ //一般情況下正為右旋轉，但跨過180度時則相反  || (error<-0.05 &&  angle_turn == 1)
          printf("==== 右轉 ====\n");
          lastError=error;
          lastError_1=error_1;	
          // status = 3; 
        }
        else if ( (error<-ang)  ){ //一般情況下負為左旋轉，但跨過180度時則相反 //|| (error>0.05 &&  angle_turn == 1)
          printf("==== 左轉 ====\n");
          lastError=error; 
          lastError_1=error_1;
          // status=2;
        } 
        out_y[1] = Tp + turn;          
        out_y[2] = Tp - turn; 
        // cout<<" out_y[1]= "<< out_y[1]<<endl;
        // cout<<" out_y[2]= "<< out_y[2]<<endl;
        speed_limit();
        v1 = out_y[1] * radius * 1.0;
        v2 = out_y[2] * radius * 1.0;

        cout<<"v1= "<<v1<<endl;
        cout<<"v2= "<<v2<<endl;
        msg.linear.x = ((v1+v2)/2);
        msg.angular.z =((v2-v1)/dis);
        // wei need change, not the same
        if(msg.linear.x >1.3) msg.linear.x = 1.3;
        if (msg.angular.z > 7.5) msg.angular.z = 7.5;
        else if (msg.angular.z < -7.5)  msg.angular.z= -7.5;
        pub.publish(msg);
      // }    
      // else  // 快撞到的情境
      // { 
      //   Tp=0;  
      //   intergral=0;
      //   // wei need change, the following are not the same
      //   if(straight_min <= front_safe_distance )
      //   {
      //     if (status==2 )
      //       {
      //         msg.angular.z=0.5;
      //         msg.linear.x=0;
      //         pub.publish(msg); 
      //         printf("\n距離過近 向左旋轉 ##-.-\n\n");
      //       }
      //       else if(status ==3)
      //       {
      //         msg.angular.z = -0.5;
      //         msg.linear.x=0;
      //         pub.publish(msg); 
      //         printf("\n距離過近 向右旋轉 ##-.-\n\n");
      //       }        
      //   }
      //   else if (left_min < side_safe_distance && right_min < side_safe_distance)
      //   {
      //     if (status==2 )
      //     {
      //       msg.angular.z = 0.2;
      //       msg.linear.x = 0.3;
      //       pub.publish(msg); 
      //       printf("\n距離過近 向左前方旋轉 ##-.-\n\n");
      //     }
      //     else 
      //     {
      //       msg.angular.z = -0.2;
      //       msg.linear.x = 0.3;
      //       pub.publish(msg); 
      //       printf("\n距離過近 向右前方旋轉 ##-.-\n\n");
      //     }        
      //   }
      //   else if (left_min <= side_safe_distance)
      //   {
      //     if(straight_min > front_safe_distance)
      //     { 
      //       msg.angular.z=-0.5;
      //       msg.linear.x= 0.3;
      //       pub.publish(msg);
      //       printf("\n距離過近 向右前方旋轉 ##-.-\n\n");
      //     }
      //   }
      //   else if (right_min <= side_safe_distance)
      //   {
      //     if(straight_min > front_safe_distance)
      //     { 
      //       msg.angular.z= 0.5;
      //       msg.linear.x= 0.3;
      //       // msg.linear.x= 0.;
      //       pub.publish(msg);
      //       printf("\n距離過近 向左前方旋轉 ##-.-\n\n");
      //     }
      //   }
        
      // }
      
      linear=msg.linear.x ;  
      angular=msg.angular.z;   
      a=10;
    }
  void speed_limit()
  {
	/// 使 Robot 維持在最高速及最低速
	  if ( out_y[1] >= (  left_wheel_speed) )
		  out_y[1] =  left_wheel_speed ;
	  if ( out_y[2] >= (right_wheel_speed) )
		  out_y[2] = right_wheel_speed ;
	  if ( out_y[1] <= ( - left_wheel_speed) )
		  out_y[1] =  - left_wheel_speed ;
	  if ( out_y[2] <= ( - right_wheel_speed) )
	  	out_y[2] =  - right_wheel_speed ;	 
  }
  void Final_stop(int jj)
  {        
    linear = msg.linear.x ;
    angular = msg.angular.z;
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    pub.publish(msg);
  }
  void publisher_info(int jj ) //印出訊息並傳遞數值給其他程式從pub1~pub10;
  {        
    std_msgs::Float64 info2,info3;
    std_msgs::Int32  info1;
    angle_deviate = trans_degree;
    // info1.data = status;  //車子與目標的角度差
    info2.data = error_orientation_z ;             //經過轉換後的機器人Z軸角度
    // chatter_pub1.publish(info1); 
    chatter_pub2.publish(info2);
  }
  void local_target(int jj)//紀錄
  { 
    local_target_x_record[jj] = local_target_x[ik];
    local_target_y_record[jj] = local_target_y[ik];
    if (local_target_x_record[jj] != local_target_x_record[jj-1] || local_target_y_record[jj] != local_target_y_record[jj-1] )
    {  
      FILE *pfout1; //all path ;
     // printf("\n\nread file\n");
      pfout1=fopen(save_local_target,"a");
			if(pfout1==NULL)
      {
			  printf("Fail to open file");
			  exit(1);
		  }  
      fprintf(pfout1,"%lf \t %lf \n", local_target_x[ik], local_target_y[ik]);
      fclose(pfout1);
      FILE *pfout100; //all path ;
     // printf("\n\nread file\n");
    }
  }
  void stop()
  {
	  if(counts == max_step)
	  {
	    msg.linear.x = 0.;
	    msg.angular.z = 0.;
	    pub.publish(msg);
    }
  }
  void Fir_str(double *in, int _rule)     
  {
    //這裡的ber，等於main函式內的ber，又ber的值為popsize的值。
    //這裡的 _rule，等於設定變數的_rule_number的值。
    //這裡的in，代表輸入的變數的數量
    int j, k;
    for(k=1; k<=_rule; k++)  //_rule值=_rule_number=10
    {
      for (j=1; j<=_out_varl; j++)
      {
        _Rule[k].con[j] = fuzzy_real[k*_mem_length-_out_varl+j] ; // 將左右輪的後件部值做"轉存"
      } 	             
      _Rule[k].In_mem_fir(in,k) ; // 計算激發量
    }
  }
  void fuzzy(int _rule ) 
  {
    int i, j;
    double den[_out_varl+1],   //分母項
          num[_out_varl+1] ;   //分子項
    for (j=1; j<=_out_varl; j++)
    {
      den[j] = 0. ;  //初始化歸零
      num[j] = 0. ;  //初始化歸零
      
      for (i=1; i<=_rule; i++)
      {
        num[j] = num[j] + _Rule[i].in_mu_1 * _Rule[i].con[j];  //num為sigma的寫法
        den[j] = den[j] + _Rule[i].in_mu_1 ;   //den為sigma的寫法
      }
      if ( fabs(den[j]) < 1E-8 )
        K[j] = 0.;     //如果den很小時
      else
        K[j] = num[j]/den[j] ;  //權重式平均法
    }
    //   for (int kk=1; kk<=_out_varl; kk++) printf("den[%d]=%lf \t num=%lf\n",kk,den[kk],num[kk]);
  }
  void test_open()
  {
    for(int ssss=1; ssss<=_in_varl; ssss++)
    {
        min_m[ssss]=1.0;  //為了找最小值，所以初始的最小值令為1，即為"最大"的最小值。
        max_m[ssss]=0.0;  //為了找最大值，所以初始的最大值令為0，即為"最小"的最大值。		         
    }
    FILE *fnoise1,*fnoise0;  
    //printf("read file\n");		
    if((fnoise0=fopen(load_data_clu,"r"))==NULL)
    {
      printf("Nothing\n");
      exit(1);
    }
    fscanf(fnoise0,"%d", &_in_clu);
    fclose(fnoise0);
    if((fnoise1=fopen(load_data_FC,"r"))==NULL)
    {
      printf("Nothing\n");
      exit(1);
    }
    for(int i=1;i<=_mem_length*_in_clu;i++)
    {
      fscanf(fnoise1,"%lf \n", &fuzzy_real[i]);
    }  
    fclose(fnoise1);  
    for(int jj=1; jj<=_rule_delta; jj++)
    {
      for(int jjj=1; jjj<=_in_varl; jjj++)
      {   
        if(fuzzy_real[(jj-1)*_mem_length+jjj*2-1] < min_m[jjj])
        {   
          min_m[jjj]=fuzzy_real[(jj-1)*_mem_length+jjj*2-1];
        }
        if(fuzzy_real[(jj-1)*_mem_length+jjj*2-1] > max_m[jjj])
        {
          max_m[jjj]=fuzzy_real[(jj-1)*_mem_length+jjj*2-1];
        }
      }
    }	
  }
  void Information(int jj)
  {  
    for (int ilk=1 ;ilk<=3;ilk++) cout<<"in["<<ilk<<"]= "<<in[ilk]<<" \t";
    cout<<"\n\n"; 
    for (int i=0;i<3;i++)
    {
      printf("K[%d]=%lf \t",i+1,K[i+1]);
    }
    cout<<"\n";
    for (int i=3;i<6;i++)
    {
      printf("K[%d]=%lf \t",i+1,K[i+1]);
    }
    // wei need change, HH seems no use
    if(HH=1) printf("\n linear=%lf \t angular=%lf\n\n", linear, angular);
    else  printf("\n linear=%lf \t angular=%lf\n\n", linear1, angular1);
  }
  double minimum(int i, int j ,int &k)
  {  
    double laser_min = 100;
    for ( k=i ;k<=j;k++)
    {        
      if (laser_min>laser_temp[k])
      { 
			  laser_min=laser_temp[k];       
			}    
    }
    return laser_min;    
  }

};
 
int main(int argc,char ** argv)
{ 
  auto start = chrono::high_resolution_clock::now();
  ros::init(argc,argv,"amcl_pose_sub_pub_1");
  amcl_pose_sub_pub_1 ctrl;
	while(counts<max_step && ros::ok())
	{
    // wei need change
    // if(goal_x != 0)
    // {
		  counts++;
    // }
    printf("counts: %d\n", counts);
    ctrl.test_open(); //to get max and min
    ctrl.coordinate_transform(counts);
    ctrl.fuzzy_in(counts) ;  //輸入是雷射數值，只是調整fuzzy的input比例(做正規化)			
    ctrl.Fir_str(in , _rule_number) ; //讀取後件部計算每條rule機發量
    ctrl.fuzzy(_rule_number); //解模糊產生out_y
    ctrl.publisher_info(counts);
		if(counts < 5){
		  ctrl.stop(); //機器人前5個counts不動作
		}
    if(rms>0.1 && (a!=0 || a!=1)){
      ctrl.Search_Target(counts);   
    }
    else if(rms<0.1 && counts>10) 
    {
     cout<<"final_z= "<<final_z<<endl;
     ctrl.Final_stop(counts);
    }
    ctrl.speed_limit() ; // 速度限制器
    ctrl.Information(counts); //印資料	
    ctrl.local_target(counts);
		ros::spinOnce();
		ros::Duration(0.01).sleep(); //訊息傳送間隔0.01秒
	}
  return 0;
}




