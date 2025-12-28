/*********************************************************************************
 * @file       RvizPlotNode
 * @brief      Using rviz2 to plot the info of planning and control
 * @author     XL Jiatan(Design and Build)
 * @version    1.0
 * @date       20251022
 **********************************************************************************/
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "planning_msgs/msg/trajectory.hpp"
#include "perception_msgs/msg/track_list.hpp"
#include "vehicle_msgs/msg/chassis.hpp"
#include "lomp_msgs/msg/localization.hpp"
#include "control_msgs/msg/control_cmd.hpp"
#include "control_msgs/msg/control_debug.hpp"
#include <memory>
#include <mutex>
#include "tf2/LinearMath/Quaternion.h" // <--- 添加 TF2 四元数头文件
#include "tf2/LinearMath/Matrix3x3.h"   // <--- 添加 TF2 矩阵头文件
#include "tf2/LinearMath/Transform.h"

#include "visualization/plot.h"

using namespace cartesian_planner;


// 读取CSV文件的函数 - 使用两个单独的vector
bool readPathFromCSV(const std::string& filename, std::vector<double>& x_coords, std::vector<double>& y_coords) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }
    
    std::string line;
    // 跳过可能的表头
    // std::getline(file, line);
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> row;
        
        // 按逗号分割每一行
        while (std::getline(ss, token, ',')) {
            try {
                row.push_back(std::stod(token));
            } catch (const std::exception& e) {
                row.clear();
                break;
            }
        }
        
        y_coords.push_back(row[2]);  // 第三列
        // 确保行中有足够的数据
        if (row.size() >= 3) {
            y_coords.push_back(row[2]);  // 第三列 
            y_coords.push_back(row[2]);  // 第三列
            y_coords.push_back(row[2]);  // 第三列
        }
    }
    
    file.close();
    return true;
}

 

VehicleParam vehicle_Param; 

class RvizPlotNode : public rclcpp::Node   
{
public:
  RvizPlotNode()
      : Node("RvizPlotNode")
  {
    RCLCPP_INFO(this->get_logger(), "RVIZ可视化节点 'RvizPlotNode' 正在初始化...");
    auto qos_default = rclcpp::QoS(1).best_effort();
    auto qos_cmdout = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
     
    control_cmd_subscriber_ = this-> create_subscription<control_msgs::msg::ControlCmd>(
        "/control/command",qos_default,
        std::bind(&RvizPlotNode::control_cmd_callback, this, std::placeholders::_1));
    control_debug_subscriber_ = this-> create_subscription<control_msgs::msg::ControlDebug>(
        "/control/debug",qos_default,
        std::bind(&RvizPlotNode::control_debug_callback, this, std::placeholders::_1));    

    trajectory_subscriber_ = this->create_subscription<planning_msgs::msg::Trajectory>(
        "/planning/trajectory", qos_default,
        std::bind(&RvizPlotNode::trajectory_callback, this, std::placeholders::_1));

    chassis_subscriber_ = this->create_subscription<vehicle_msgs::msg::Chassis>(
        "/vehicle/chassis", qos_default,
        std::bind(&RvizPlotNode::chassis_callback, this, std::placeholders::_1));

    localization_subscriber_ = this->create_subscription<lomp_msgs::msg::Localization>(
        "/jinlvlomp/localization/estimate_pose", qos_default,
        std::bind(&RvizPlotNode::localization_callback, this, std::placeholders::_1));

    perception_objects_subscriber_ = this->create_subscription<perception_msgs::msg::TrackList>(
        "/perception/tracked_objects", qos_default,
        std::bind(&RvizPlotNode::perception_objects_callback, this, std::placeholders::_1));    
   
    rviz_plot_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&RvizPlotNode::rviz_plot_timer_callback, this));  
    load_reference_line_data();      
    // 初始化可视化
    //visualization::Init(this->shared_from_this(), "map", "rviz_markers");    

    RCLCPP_INFO(this->get_logger(), "RVIZ可视化节点 'RvizPlotNode' 初始化完成！"); 
  }

private:

  void control_cmd_callback(const control_msgs::msg::ControlCmd::SharedPtr msg){
    latest_control_cmd_ = msg;
  }

  void control_debug_callback(const control_msgs::msg::ControlDebug::SharedPtr msg){
    latest_control_debug_ = msg;
  }

  void trajectory_callback(planning_msgs::msg::Trajectory::SharedPtr msg){
    latest_trajectory_ = msg;
  }
  
  void chassis_callback(vehicle_msgs::msg::Chassis::SharedPtr msg){
    latest_chassis_ = msg;
  }

  void localization_callback(lomp_msgs::msg::Localization::SharedPtr msg){
    latest_localization_ = msg;
  }

  void load_reference_line_data()
  {
    this->declare_parameter<std::string>("traj_file_path", "default_str");
    std::string reference_line_path = this->get_parameter("traj_file_path").as_string();
    RCLCPP_INFO(this->get_logger(), "从参数 'reference_line_path' 获取到路径: '%s'", reference_line_path.c_str());
    reference_line_x_coords_.clear();
    reference_line_y_coords_.clear();
    readPathFromCSV(reference_line_path, reference_line_x_coords_, reference_line_y_coords_);
  }

  void plot_reference_line()
  {
    if (reference_line_x_coords_.size() == 0 || reference_line_y_coords_.size() == 0) {
      return;
    }
    visualization::Color path_color = visualization::Color::White;
    double path_line_width = 0.05;
    visualization::Plot(reference_line_x_coords_, reference_line_y_coords_, path_line_width, path_color, 1, "Path");
    //visualization::Trigger();
  }


  void perception_objects_callback(perception_msgs::msg::TrackList::SharedPtr msg) {
    lastest_perception_objects_ = msg;
    if (latest_localization_ && lastest_perception_objects_->header.frame_id != "map") {
          // 1. 构造自车UTM的变换（tf2::Transform = 旋转 + 平移）
        tf2::Quaternion q_car_utm;
        q_car_utm.setX(latest_localization_->utm_pose.pose.orientation.x);
        q_car_utm.setY(latest_localization_->utm_pose.pose.orientation.y);
        q_car_utm.setZ(latest_localization_->utm_pose.pose.orientation.z);
        q_car_utm.setW(latest_localization_->utm_pose.pose.orientation.w);
        q_car_utm.normalize();
        tf2::Vector3 t_car_utm(latest_localization_->utm_pose.pose.position.x, latest_localization_->utm_pose.pose.position.y, 
                               latest_localization_->utm_pose.pose.position.z);
        tf2::Transform tf_car_utm(q_car_utm, t_car_utm);

        // 2. 构造障碍物车体的变换
        for (auto& object:lastest_perception_objects_->tracks) {
          tf2::Quaternion q_obs_car;
          q_obs_car.setX(object.orientation.x);
          q_obs_car.setY(object.orientation.y);
          q_obs_car.setZ(object.orientation.z);
          q_obs_car.setW(object.orientation.w);
          q_obs_car.normalize();
          tf2::Vector3 t_obs_car(object.position.x, object.position.y, object.position.z);
          tf2::Transform tf_obs_car(q_obs_car, t_obs_car);

          // 3. 计算障碍物UTM的变换：tf_car_utm * tf_obs_car（变换叠加）
          tf2::Transform tf_obs_utm = tf_car_utm * tf_obs_car;
          object.position.x = tf_obs_utm.getOrigin().x();
          object.position.y = tf_obs_utm.getOrigin().y();
          object.position.z = tf_obs_utm.getOrigin().z();

          tf2::Quaternion q_obs_utm = tf_obs_utm.getRotation();
          object.orientation.x = q_obs_utm.x();
          object.orientation.y = q_obs_utm.y();
          object.orientation.z = q_obs_utm.z();
          object.orientation.w = q_obs_utm.w();
        }
    }
  }
  
  void plot_perception_objects() 
  {
    visualization::ClearNamespace("object1");
    if (lastest_perception_objects_ == nullptr) {
      return;
    }
    int id = 0;
    visualization::Color object1_color = visualization::Color::Red;
    visualization::Color object2_color = visualization::Color::Red;

    for (const auto& object:lastest_perception_objects_->tracks) {
      double x = object.position.x;
      double y = object.position.y;
      tf2::Quaternion q_(
        object.orientation.x,
        object.orientation.y,
        object.orientation.z,
        object.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_).getRPY(roll, pitch, yaw);
      std::cout << "x = " << x << ", y = " << y << ", yaw = " << yaw << ", length = " << object.dimensions.x << ",width = " << object.dimensions.y
                << std::endl;
      double cos_yaw = std::cos(yaw);
      double sin_yaw = std::sin(yaw);
      double half_length = object.dimensions.x / 2.0;
      double half_width = object.dimensions.y / 2.0;
      double dx1 = cos_yaw * half_length;
      double dy1 = sin_yaw * half_length;
      double dx2 = sin_yaw * half_width;
      double dy2 = -cos_yaw * half_width;
      std::vector<double> x_data;
      std::vector<double> y_data;
      x_data.push_back(x + dx1 + dx2);
      y_data.push_back(y + dy1 + dy2);
      x_data.push_back(x + dx1 - dx2);
      y_data.push_back(y + dy1 - dy2);
      x_data.push_back(x - dx1 - dx2);
      y_data.push_back(y - dy1 - dy2);
      x_data.push_back(x - dx1 + dx2);
      y_data.push_back(y - dy1 + dy2);
      x_data.push_back(x + dx1 + dx2);
      y_data.push_back(y + dy1 + dy2);
      double line_width = 0.2;
      std::string name("object:");
      name += std::to_string(object.id);
      
      visualization::Plot(x_data, y_data, line_width, object1_color, object.id, "object1");

      // if(object.id == 1)
      // {
      //   visualization::Plot(x_data, y_data, line_width, object1_color, 10, "object1");
      // }else{
      //   visualization::Plot(x_data, y_data, line_width, object2_color, 12, "object2");
      // }
      
    }
  }


  void plot_vehicle()
  {
    double veh_x = latest_localization_->utm_pose.pose.position.x;
    double veh_y = latest_localization_->utm_pose.pose.position.y;
    tf2::Quaternion q_(
        latest_localization_->utm_pose.pose.orientation.x,
        latest_localization_->utm_pose.pose.orientation.y,
        latest_localization_->utm_pose.pose.orientation.z,
        latest_localization_->utm_pose.pose.orientation.w);
    double veh_roll, veh_pitch, veh_yaw;
    tf2::Matrix3x3(q_).getRPY(veh_roll, veh_pitch, veh_yaw);
        
    //绘制车辆
    double veh_steer = latest_chassis_->steering_angle;
    visualization::PlotVehicle(1, {veh_x, veh_y, veh_yaw},veh_steer,vehicle_Param);
  }

  void plot_planning_Trajectory()
  {
     //绘制局部轨迹
    std::vector<double> traj_x;
    std::vector<double> traj_y;
    std::vector<double> traj_v;
    for (const auto &point : latest_trajectory_->trajectory_points)
    {
      traj_x.push_back(point.x);
      traj_y.push_back(point.y);
      traj_v.push_back(point.v);
    }
    double max_v = 100;
    double line_width = 0.1;  
    visualization::Color base_color = visualization::Color::Red;
    visualization::PlotTrajectory(traj_x, traj_y, traj_v, max_v, line_width, base_color, 1, "Trajectory");
    // visualization::PlotPoints (traj_x,traj_y,0.15,base_color,1,"Point");          
  }


  void plot_purepursuit_info()
  {
    //绘制投影点
    std::vector<double> proj_x;
    std::vector<double> proj_y;
    proj_x.insert(proj_x.begin(),latest_control_debug_->projection_point_x);
    proj_y.insert(proj_y.begin(),latest_control_debug_->projection_point_y);  
 
    double proj_line_width = 0.2;
    visualization::Color proj_color = visualization::Color::Blue;
    visualization::PlotPoints(proj_x,proj_y,proj_line_width,proj_color,3,"Proj_Point"); 
    
    //绘制预瞄点
    std::vector<double> lp_x;
    std::vector<double> lp_y;
    lp_x.insert(lp_x.begin(),latest_control_debug_->lookahead_point_x);
    lp_y.insert(lp_y.begin(),latest_control_debug_->lookahead_point_y); 
    // lp_x.clear();   
    // lp_x.reserve(2);   
    // lp_x.push_back(latest_control_debug_->lookahead_point_x);
    // lp_x.push_back(latest_control_debug_->lookahead_point_x+0.001);

    // lp_y.clear();
    // lp_y.reserve(2);
    // lp_y.push_back(latest_control_debug_->lookahead_point_y);
    // lp_y.push_back(latest_control_debug_->lookahead_point_y+0.001);

    
    double lp_line_width = 0.2;
    visualization::Color lp_color = visualization::Color::Blue;
    visualization::PlotPoints(lp_x,lp_y,lp_line_width,lp_color,4,"lp_Point"); 
  }

  void plot_mpc_info()
  {
    //绘制mpc预测轨迹
    double mpc_pre_line_width = 0.2;
    visualization::Color mpc_pre_color = visualization::Color::Green;
    if(!latest_control_debug_->mpc_pre_x.empty())
    {
      visualization::Plot(latest_control_debug_->mpc_pre_x,latest_control_debug_->mpc_pre_y,mpc_pre_line_width,mpc_pre_color,5,"mpc_Point"); 
    } 
  }

  void rviz_plot_timer_callback()
  {
    plot_reference_line();
    
    if(!latest_localization_||!latest_chassis_||!latest_trajectory_)
    {
      return;
    }

    plot_vehicle();
    plot_planning_Trajectory();
    plot_purepursuit_info();
    plot_mpc_info();
    plot_perception_objects();
    
  }
 
  rclcpp::Subscription<control_msgs::msg::ControlCmd>::SharedPtr control_cmd_subscriber_;
  rclcpp::Subscription<control_msgs::msg::ControlDebug>::SharedPtr control_debug_subscriber_;
  rclcpp::Subscription<planning_msgs::msg::Trajectory>::SharedPtr trajectory_subscriber_;
  rclcpp::Subscription<lomp_msgs::msg::Localization>::SharedPtr localization_subscriber_;
  rclcpp::Subscription<vehicle_msgs::msg::Chassis>::SharedPtr chassis_subscriber_;
  rclcpp::Subscription<perception_msgs::msg::TrackList>::SharedPtr perception_objects_subscriber_;
  rclcpp::TimerBase::SharedPtr rviz_plot_timer_;
  
  planning_msgs::msg::Trajectory::SharedPtr latest_trajectory_;
  vehicle_msgs::msg::Chassis::SharedPtr latest_chassis_;
  lomp_msgs::msg::Localization::SharedPtr latest_localization_;
  control_msgs::msg::ControlCmd::SharedPtr latest_control_cmd_;
  control_msgs::msg::ControlDebug::SharedPtr latest_control_debug_;
  perception_msgs::msg::TrackList::SharedPtr lastest_perception_objects_;

  std::vector<double> reference_line_x_coords_;
  std::vector<double> reference_line_y_coords_;

};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RvizPlotNode>();
  visualization::Init(node, "map", "rviz_markers");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}