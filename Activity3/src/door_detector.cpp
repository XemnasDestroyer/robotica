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
    Doors doors;
    if (points.size() < 2)
        return doors;

    // 1. Detección de picos usando sliding window
    std::vector<RoboCompLidar3D::TPoint> peaks;

    for (auto &&pair : iter::sliding_window(points, 2)) {
        const auto &p1 = pair[0];
        const auto &p2 = pair[1];

        float diff = std::fabs(p1.distance2d - p2.distance2d);
        if (diff > 1000.f)
            peaks.push_back((p1.distance2d < p2.distance2d) ? p1 : p2);
    }

    // 2. Dibujar picos para verificación
    if (scene) {
        QBrush yellowBrush(Qt::yellow);
        QPen yellowPen(Qt::yellow);

        for (const auto &p : peaks) {
            auto item = scene->addRect(-60, -60, 120, 120, yellowPen, yellowBrush);
            item->setPos(p.x, p.y);
        }
    }

    // 3. Filtro de supresión de no-máximos
    Peaks nms_peaks;
    for (const auto &p : peaks) {
        bool too_close = false;
        for (const auto &existing_peak : nms_peaks) {
            const auto &p2_point = std::get<0>(existing_peak);
            // Calcular distancia correctamente
            float dx = p.x - p2_point.x();
            float dy = p.y - p2_point.y();
            float distance = std::sqrt(dx*dx + dy*dy);
            if (distance < 500.f) {
                too_close = true;
                break;
            }
        }
        if (!too_close) {
            // Crear Eigen::Vector2f desde TPoint
            Eigen::Vector2f point_vec(p.x, p.y);
            nms_peaks.emplace_back(point_vec, std::atan2(p.y, p.x));
        }
    }

    // Actualizar peaks con el resultado del NMS - necesitamos convertir de vuelta
    peaks.clear();
    for (const auto &[point_vec, angle] : nms_peaks) {
        RoboCompLidar3D::TPoint new_point;
        new_point.x = point_vec.x();
        new_point.y = point_vec.y();
        new_point.z = 0; // Asumir plano 2D
        new_point.distance2d = point_vec.norm();
        peaks.push_back(new_point);
    }

    // 4. Detección de puertas usando combinaciones de picos
    for (auto &&pair : iter::combinations(peaks, 2)) {
        const auto &p1 = pair[0];
        const auto &p2 = pair[1];

        // Calcular distancia correctamente
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        float distance = std::sqrt(dx*dx + dy*dy);

        // Verificar si la distancia está en el rango de una puerta (800-1200 mm)
        if (distance >= 800.f && distance <= 1200.f) {
            // Crear Vector2f para ambos puntos
            Eigen::Vector2f point1(p1.x, p1.y);
            Eigen::Vector2f point2(p2.x, p2.y);

            // Calcular ángulos para ambos puntos
            float angle1 = std::atan2(p1.y, p1.x);
            float angle2 = std::atan2(p2.y, p2.x);

            // Añadir a la lista de puertas detectadas con los 4 parámetros requeridos
            doors.emplace_back(point1, angle1, point2, angle2);

            // Dibujar la puerta detectada si hay escena
            if (scene) {
                QBrush greenBrush(Qt::green);
                QPen greenPen(Qt::green);
                greenPen.setWidth(3);

                // Dibujar línea entre los dos picos que forman la puerta
                scene->addLine(p1.x, p1.y, p2.x, p2.y, greenPen);

                // Dibujar punto en el centro de la puerta
                float mid_x = (p1.x + p2.x) / 2.0f;
                float mid_y = (p1.y + p2.y) / 2.0f;
                auto center_item = scene->addEllipse(-40, -40, 80, 80, greenPen, greenBrush);
                center_item->setPos(mid_x, mid_y);
            }
        }
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
