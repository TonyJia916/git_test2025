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

#pragma once

#include "vehicle_param.h"

#include "math/vec2d.h"
#include "math/polygon2d.h"

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include <geometry_msgs/msg/point.hpp>

#include "color.h"

namespace cartesian_planner {
namespace visualization {

using math::Vec2d;
using math::Polygon2d;

using Vector = std::vector<double>;

void Init(rclcpp::Node::SharedPtr node, const std::string& frame, const std::string& topic);

void ClearNamespace(const std::string& ns);

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color(1, 1, 1),
          int id = -1, const std::string &ns = "");

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, const std::vector<Color> &color = {},
          int id = -1, const std::string &ns = "");

void PlotPolygon(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotPolygon(const Polygon2d &polygon, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity = 10.0,
                    double width = 0.1, const Color &color = Color::Blue,
                    int id = -1, const std::string &ns = "");

void PlotPoints(const Vector &xs, const Vector &ys, double width = 0.1, const Color &color = Color::White,
                int id = -1, const std::string &ns = "");

std::array<math::Box2d, 4> GenerateTireBoxes(const math::Pose pose, double phi,VehicleParam vehicle);

void PlotVehicle(int id, const math::Pose &pt, double phi,VehicleParam vehicle);

void Trigger();

void Clear();
}

}