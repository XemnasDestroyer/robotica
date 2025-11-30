//
// Created by pbustos on 11/11/25.
//

#ifndef DOORDETECTOR_H
#define DOORDETECTOR_H

#include "common_types.h"
#include <Lidar3D.h>
#include <QGraphicsScene>
#include <expected>

class DoorDetector
{
public:
    DoorDetector() = default;
    ~DoorDetector() = default;

<<<<<<< HEAD
    Doors detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene = nullptr);
    RoboCompLidar3D::TPoints filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
    [[nodiscard]] Doors doors() const { return doors_cache; };
=======
        Doors detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene = nullptr);
        std::tuple<RoboCompLidar3D::TPoints, Doors> filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
        [[nodiscard]] Doors doors() const { return doors_cache; };
        [[nodiscard]] std::expected<Door, std::string> get_current_door() const;
>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438

private:
    Doors doors_cache;
};

#endif //DOORDETECTOR_H