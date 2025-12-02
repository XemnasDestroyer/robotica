//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>


Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    // compute peaks in lidar data
    Peaks peaks;
    for (const auto &p : points | iter::sliding_window(2))
    {
        const auto &p1 = p[0];
        const auto &p2 = p[1];
        auto difference = abs((p2.distance2d - p1.distance2d));
        auto closest = p1.distance2d < p2.distance2d ? p1 : p2;
        if (difference > 1000.f) peaks.push_back(std::make_tuple(Eigen::Vector2f(closest.x,closest.y), closest.phi));
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
        auto dist = std::sqrt(std::pow(std::get<0>(p2).x()-std::get<0>(p1).x(), 2) + std::pow(std::get<0>(p2).y()-std::get<0>(p1).y(), 2));
        if (800 < dist && dist < 1200)
            doors.push_back(Door(std::get<0>(p1), std::get<1>(p1), std::get<0>(p2), std::get<1>(p2)));
    }

    return doors;
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return points;

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    int counter = 0;
    for(const auto &p : points)
    {
        counter = 0;
        for(const auto &d : doors)
        {
            const float dist_to_door = d.center().norm();
            // Check if the angular range wraps around the -π/+π boundary
            const bool angle_wraps = d.p2_angle < d.p1_angle;
            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) or (p.phi < d.p2_angle);
            }
            else
            {
                float offset = qDegreesToRadians(3);
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > d.p1_angle - offset) and (p.phi < d.p2_angle + offset);
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if(point_in_angular_range and p.distance2d >= dist_to_door)
                counter++;
        }

        if (counter != doors.size())
            filtered.emplace_back(p);
    }
    return filtered;
}