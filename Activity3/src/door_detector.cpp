//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>


#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <QGraphicsItem>
#include <Eigen/Dense>
#include <cmath>

Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    Doors detected_doors;

    if (points.size() < 3) return detected_doors; // necesitamos al menos 3 puntos para la ventana deslizante

    // Parámetros: ventana angular máxima para considerar que hay un marco de puerta
    const float max_angle_span = M_PI / 4.0f; // 45 grados aprox.
    const float min_door_width = 500.f; // ancho mínimo de puerta en mm
    const float max_door_width = 1500.f; // ancho máximo de puerta en mm

    // Recorremos los puntos usando una ventana deslizante de 3 elementos
    for (auto win : iter::sliding_window(points, 3))
    {
        const auto &p1 = win[0];
        const auto &p2 = win[1];
        const auto &p3 = win[2];

        // Calculamos ancho candidato de puerta
        Eigen::Vector2f vec1(p1.x, p1.y);
        Eigen::Vector2f vec3(p3.x, p3.y);
        float door_width = (vec3 - vec1).norm();

        if (door_width < min_door_width || door_width > max_door_width)
            continue; // descartamos si no está dentro de rango

        // Calculamos diferencias angulares
        float angle1 = p1.phi;
        float angle3 = p3.phi;
        float angle_span = std::abs(angle3 - angle1);
        if (angle_span > max_angle_span) continue; // descartamos si angulo demasiado grande

        // Creamos el objeto Door y lo añadimos a la lista
        detected_doors.emplace_back(
            Eigen::Vector2f(p1.x, p1.y), angle1,
            Eigen::Vector2f(p3.x, p3.y), angle3
        );

        // Opcional: dibujar la puerta en la escena
        if (scene)
        {
            QPen pen(Qt::yellow, 4);
            scene->addLine(p1.x, p1.y, p3.x, p3.y, pen);
        }
    }

    return detected_doors;
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return points;

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    for(const auto &d : doors)
    {
        const float dist_to_door = d.center().norm();
        // Check if the angular range wraps around the -π/+π boundary
        const bool angle_wraps = d.p2_angle < d.p1_angle;
        for(const auto &p : points)
        {
            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) or (p.phi < d.p2_angle);
            }
            else
            {
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) and (p.phi < d.p2_angle);
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if(point_in_angular_range and p.distance2d >= dist_to_door)
                continue;

            //qInfo() << __FUNCTION__ << "Point angle: " << p.phi << " Door angles: " << d.p1_angle << ", " << d.p2_angle << " Point distance: " << p.distance2d << " Door distance: " << dist_to_door;
            filtered.emplace_back(p);
        }
    }
    return filtered;
}
