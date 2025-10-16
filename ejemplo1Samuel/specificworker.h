/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// Uncomment to enable automatic period reduction when the component is idle
// #define HIBERNATION_ENABLED

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <optional>

/**
 * \brief SpecificWorker implements the main logic of the component.
 *
 * This class handles 3D LIDAR processing, robot visualization,
 * emergency handling, and component initialization.
 */
class SpecificWorker : public GenericWorker
{
    Q_OBJECT

public:

    enum class RobotState {
        IDLE,
        FORWARD,
        TURN,
        WALL,
        SPIRAL
    };

    /**
     * \brief Constructor
     * \param configLoader Loads component configuration parameters.
     * \param tprx Tuple of required proxies.
     * \param startup_check If true, perform startup checks on initialization.
     */
    SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

    /**
     * \brief Destructor
     */
    ~SpecificWorker();

public slots:
    /**
     * \brief One-time initialization of the worker.
     */
    void initialize();

    /**
     * \brief Main compute loop. Reads LIDAR, updates visualization, and processes robot logic.
     */
    void compute();

    /**
     * \brief Updates the robot position in the Qt viewer.
     */
    void update_robot_position();

    /**
     * \brief Emergency handler loop.
     */
    void emergency();

    /**
     * \brief Restore the component from emergency state.
     */
    void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return 0 on success.
     */
    int startup_check();

    /**
     * \brief Slot to receive mouse click coordinates from the viewer.
     * \param p World coordinates of the clicked point.
     */
    void new_target_slot(QPointF p);

private:
    // ------------------------
    // Configuration & state
    // ------------------------
    bool startup_check_flag;   ///< Indicates whether startup checks are enabled

    // ------------------------
    // Constants
    // ------------------------
    const float MAX_ADV;        ///< Maximum linear speed (mm/s)
    const float MAX_ROT;        ///< Maximum rotational speed (rad/s)
    const float SAFE_DISTANCE;  ///> Minium safe distance (mm)
    const float WALL_DISTANCE;  ///< Desired distance to wall (mm)
    const float SPIRAL_ADV_INIT;///< Linear speed for spiral motion
    const float SPIRAL_ROT_INIT;///< Rotational speed for spiral motion
    float a_spiral;
    float b_spiral;
    RobotState state;          ///< Current robot state
    float spiral_adv;
    float spiral_rot;
    float spiral_theta;

    // ------------------------
    // Graphics
    // ------------------------
    QRectF dimensions;                         ///< Scene dimensions
    AbstractGraphicViewer* viewer;             ///< Viewer instance
    const int ROBOT_LENGTH = 400;             ///< Robot size in mm
    QGraphicsPolygonItem* robot_polygon;      ///< Polygon representing the robot

    // ------------------------
    // LIDAR processing
    // ------------------------
    std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(
        const RoboCompLidar3D::TPoints& points
    );

    void draw_lidar(const auto& points, QGraphicsScene* scene);

    // ----------------------------------------
    // Movement & control
    // ----------------------------------------
    std::tuple<RobotState, float, float> FORWARD_method(float min_distance);
    std::tuple<RobotState, float, float> TURN_method(float min_distance, float angle_min);
    std::tuple<RobotState, float, float> WALL_method(const RoboCompLidar3D::TPoints &points);
    std::tuple<RobotState, float, float> SPIRAL_method(float min_distance);

    // ----------------------------------------
    // Brakes
    // ----------------------------------------
    float brake_adv(float dist, float dist_threshold = 1000.0);
    float brake_rot(float rot);

signals:
    // Custom signals can be added here
    // void customSignal();
};

#endif // SPECIFICWORKER_H
