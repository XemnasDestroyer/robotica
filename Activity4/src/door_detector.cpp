//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>


Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, const NominalRoom &room)
{
    // compute peaks in lidar data
    Peaks peaks;
    for (const auto &p : points | iter::sliding_window(2))
    {
        const auto &p1 = p[0];
        const auto &p2 = p[1];
        auto difference = abs((p2.distance2d - p1.distance2d));
        auto closest = p1.distance2d < p2.distance2d ? p1 : p2;
        if (difference > 600.f) peaks.push_back(std::make_tuple(Eigen::Vector2f(closest.x,closest.y), closest.phi));
    }
    if (peaks.empty()) return {};

    // non-maximum suppression of peaks: remove peaks closer than 500mm
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
        if (const bool too_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2) { return (p - std::get<0>(p2)).norm() < 500.f; }); not too_close)
            nms_peaks.emplace_back(p, a);
    peaks = nms_peaks;

    if (nms_peaks.empty()) return {};

    // compute doors in peaks data
    Doors doors;
    for (const auto &c : iter::combinations(nms_peaks, 2))
    {
        const auto &p1 = c[0];
        const auto &p2 = c[1];
        const auto dist = std::sqrt(std::pow(std::get<0>(p2).x()-std::get<0>(p1).x(), 2) + std::pow(std::get<0>(p2).y()-std::get<0>(p1).y(), 2));
        if (800 < dist && dist < 1200)
        {
            auto wall = room.point_to_wall(std::get<0>(p1));
            float distance = room.distance_from_wall(wall, std::get<0>(p1));
            doors.push_back(Door(std::get<0>(p1), std::get<1>(p1),
                                   std::get<0>(p2),std::get<1>(p2),
                                   wall, distance));
        }
    }

    return doors;
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points)
{
    const auto doors = detect(points, );
    if(doors.empty()) return points;
    qInfo() << "Hay " << doors.size() << " puertas.";

    RoboCompLidar3D::TPoints filtered;
    for(const auto &p : points)
    {
        bool remove_point = false;

        for(const auto &d : doors)
        {
            const float dist_to_door = d.center().norm();
            const bool angle_wraps = d.p2_angle < d.p1_angle;

            bool point_in_angular_range;
            if (angle_wraps)
            {
                point_in_angular_range = (p.phi > d.p1_angle) || (p.phi < d.p2_angle);
            }
            else
            {
                float offset = qDegreesToRadians(3);
                point_in_angular_range = (p.phi > d.p1_angle - offset) && (p.phi < d.p2_angle + offset);
            }

            // Si el punto está dentro del rango angular de la puerta y más lejos que ella → eliminar
            if(point_in_angular_range && p.distance2d >= dist_to_door)
            {
                remove_point = true;
                break; // no hace falta comprobar más puertas
            }
        }

        if(!remove_point)
            filtered.emplace_back(p);
    }

    return filtered;
}
