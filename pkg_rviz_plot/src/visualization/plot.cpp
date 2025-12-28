/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#include "visualization/plot.h"

namespace cartesian_planner {
namespace visualization {

namespace {
  std::string frame_ = "map";
  std::mutex mutex_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  visualization_msgs::msg::MarkerArray arr_;
}

void Init(rclcpp::Node::SharedPtr node, const std::string& frame, const std::string& topic) {
  frame_ = frame;
  publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(topic, rclcpp::QoS(10).transient_local());
}


// 新增：清除指定命名空间的所有Marker
void ClearNamespace(const std::string& ns) {
  visualization_msgs::msg::Marker clear_msg;
  clear_msg.header.frame_id = frame_;
  clear_msg.header.stamp = rclcpp::Time();
  clear_msg.ns = ns;
  clear_msg.action = visualization_msgs::msg::Marker::DELETEALL;
  
  std::lock_guard<std::mutex> lock(mutex_);
  arr_.markers.push_back(clear_msg);
}

void Plot(const Vector &xs, const Vector &ys, double width, Color color, int id, const std::string &ns) {
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp =  rclcpp::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::msg::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    pt.z = 0.1 * id;
    msg.points.push_back(pt);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  arr_.markers.push_back(msg);
}

 
void Plot(const Vector &xs, const Vector &ys, double width,
          const std::vector<Color> &color, int id, const std::string &ns) {
  assert(xs.size() == color.size());

  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = rclcpp::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::msg::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
    msg.colors.push_back(color[i].toColorRGBA());
  }

  std::lock_guard<std::mutex> lock(mutex_);
  arr_.markers.push_back(msg);
}



void PlotPolygon(const Vector &xs, const Vector &ys, double width, Color color, int id,
                 const std::string &ns) {
  auto xxs = xs;
  auto yys = ys;
  xxs.push_back(xxs[0]);
  yys.push_back(yys[0]);
  Plot(xxs, yys, width, color, id, ns);
}

void PlotPolygon(const Polygon2d &polygon, double width, Color color, int id,
                 const std::string &ns) {
  std::vector<double> xs, ys;
  for (auto &pt: polygon.points()) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  PlotPolygon(xs, ys, width, color, id, ns);
}

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity, double width,
                    const Color &color, int id, const std::string &ns) {
  std::vector<Color> colors(xs.size());
  float h, tmp;
  color.toHSV(h, tmp, tmp);

  for (size_t i = 0; i < xs.size(); i++) {
    double percent = (vs[i] / max_velocity);
    // colors[i] = Color::fromHSV(h, percent, 1.0);
    colors[i] = Color::fromHSV(h, tmp, tmp);
  }

  Plot(xs, ys, width, colors, id, ns);
}

 

void PlotPoints(const Vector &xs, const Vector &ys, double width, const Color &color, int id,
                const std::string &ns) {
  assert(xs.size() == ys.size());

  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = rclcpp::Time();
  msg.ns = ns.empty() ? "Points" : ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.type = visualization_msgs::msg::Marker::POINTS;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::msg::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  arr_.markers.push_back(msg);
}


std::array<math::Box2d, 4> GenerateTireBoxes(const math::Pose pose, double phi,VehicleParam vehicle){
    auto front_pose = pose.extend(vehicle.wheel_base);
    auto track_width = vehicle.width - 0.195;
    double rear_track_width_2 = track_width / 2, front_track_width_2 = track_width / 2;
    double box_length = 0.6345;
    double sin_t = sin(pose.theta());
    double cos_t = cos(pose.theta());
    return {
      math::Box2d({pose.x() - rear_track_width_2 * sin_t, pose.y() + rear_track_width_2 * cos_t}, pose.theta(),
                  box_length, 0.195),
      math::Box2d({pose.x() + rear_track_width_2 * sin_t, pose.y() - rear_track_width_2 * cos_t}, pose.theta(),
                  box_length, 0.195),
      math::Box2d({front_pose.x() - front_track_width_2 * sin_t, front_pose.y() + front_track_width_2 * cos_t},
                  front_pose.theta() + phi, box_length, 0.195),
      math::Box2d({front_pose.x() + front_track_width_2 * sin_t, front_pose.y() - front_track_width_2 * cos_t},
                  front_pose.theta() + phi, box_length, 0.195),
    };
  }

void PlotVehicle(int id, const math::Pose &pt, double phi,VehicleParam vehicle) {
    auto tires = GenerateTireBoxes({pt.x(), pt.y(), pt.theta()}, phi,vehicle);

    int tire_id = 1;
    for (auto &tire: tires) {
      visualization::PlotPolygon(math::Polygon2d(tire), 0.1, visualization::Color::White, id * (tire_id++),
                                 "Tires");
    }
    visualization::PlotPolygon(math::Polygon2d(vehicle.GenerateBox({pt.x(), pt.y(), pt.theta()})), 0.1,
                               visualization::Color::Yellow, id, "Footprint");
    visualization::Trigger();
  }


void Trigger()
{
  std::lock_guard<std::mutex> lock(mutex_);
  publisher_->publish(arr_);
  arr_.markers.clear();
}

 
void Clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  arr_.markers.clear();

  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_;
  msg.ns = "Markers";

  msg.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(msg);
  publisher_->publish(arr);
}

}
}