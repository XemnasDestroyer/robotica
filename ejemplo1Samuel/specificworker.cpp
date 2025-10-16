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

#include "specificworker.h"
#include <cppitertools/itertools.hpp>

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor & Destructor
///////////////////////////////////////////////////////////////////////////////////////////////////////////
SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check)
    : GenericWorker(configLoader, tprx),
    MAX_ADV(1000.0f),
    MAX_ROT(1.0f),
    SAFE_DISTANCE(500.0f),
    WALL_DISTANCE(400.0f),
    SPIRAL_ADV_INIT(50.0f),
    SPIRAL_ROT_INIT(1.0f),
    a_spiral(50.0f),
    b_spiral(20.0f),
    state(RobotState::FORWARD),
    spiral_adv(SPIRAL_ADV_INIT),
    spiral_rot(SPIRAL_ROT_INIT),
    spiral_theta(0.0f)
{
    this->startup_check_flag = startup_check;

    if (this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
#ifdef HIBERNATION_ENABLED
        hibernationChecker.start(500); // Periodically check for hibernation
#endif

        // Example statemachine:
        /***
    //Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
    states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period,
                                                        std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
                                                        std::bind(&SpecificWorker::customEnter, this), // On-enter function
                                                        std::bind(&SpecificWorker::customExit, this)); // On-exit function

    //Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
    states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
    states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

    //Add your custom state
    statemachine.addState(states["CustomState"].get());
    ***/

        // Example state machine (placeholder)
        statemachine.setChildMode(QState::ExclusiveStates);
        statemachine.start();

        auto error = statemachine.errorString();
        if (!error.isEmpty())
        {
            qWarning() << error;
            throw error;
        }
    }
}

SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::initialize()
{
    std::cout << "Initialize worker" << std::endl;

    // Setup Qt viewer for robot and lidar visualization
    this->dimensions = QRectF(-6000, -3000, 12000, 6000);
    viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
    this->resize(900, 450);
    viewer->show();

    const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
    robot_polygon = std::get<0>(rob);
    qInfo() << "Robot position: " << robot_polygon->pos();

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

    // Placeholder: Load parameters, open devices, etc.

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name")
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Compute Loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::compute()
{
    try
    {
        // 1. Read 3D lidar data (filtered in 2D)
        const auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);
        qInfo() << "Full LIDAR points:" << data.points.size();

        if (data.points.empty())
        {
            qWarning() << "No points received";
            omnirobot_proxy->setSpeedBase(200,0,0);
            return;
        }

        // 2. Filter lidar to keep closest points per angle
        const auto filter_data = filter_min_distance_cppitertools(data.points);
        qInfo() << "Filtered points:" << filter_data.value().size();

        // 3. Check distance to closest obstacles and its angle
        auto points = filter_data.value();

        float min_distance = 0.0f;
        float angle_min = 0.0f;

        if(!points.empty()) {
            auto min_it = std::min_element(points.begin(), points.end(),
                                           [](const auto &a, const auto &b){ return a.r < b.r; });
            min_distance = min_it->r;
            angle_min = min_it->phi;

            // --- FIX: Simple collision detection until lidar is fixed ---
            bool obstacle_ahead = false;
            for(const auto &p : points)
            {
                if(p.phi < M_PI/2 && p.phi > -M_PI/2 && p.r < 450)
                {
                    obstacle_ahead = true;
                    break;
                }
            }
            if(obstacle_ahead)
            {
                min_distance = 0.0f; // Force robot to stop / turn
            }
        }

        // 4. Decide whether to continue or to stop and turn
        float vlin = 0, vrot = 0;
        RobotState next_state = state;
        switch(state)
        {
        case RobotState::FORWARD:
            std::tie(next_state, vlin, vrot) = FORWARD_method(min_distance);
            break;

        case RobotState::TURN:
            std::tie(next_state, vlin, vrot) = TURN_method(min_distance, angle_min);
            break;

        case RobotState::WALL:
            std::tie(next_state, vlin, vrot) = WALL_method(points);
            break;

        case RobotState::SPIRAL:
            std::tie(next_state, vlin, vrot) = SPIRAL_method(min_distance);
            break;

        case RobotState::IDLE:
        default:
            vlin = 0; vrot = 0; next_state = RobotState::IDLE;
            break;
        }

        state = next_state;
        try {
            omnirobot_proxy->setSpeedBase(vlin, 0, vrot);
        } catch(const Ice::Exception &e) {
            std::cout << e.what() << std::endl;
        }

        // 5. Draw lidar points in Qt scene
        if (filter_data.has_value())
            draw_lidar(filter_data.value(), &viewer->scene);


        // 6. Update robot position in viewer
        update_robot_position();

    }
    catch (const Ice::Exception& e)
    {
        std::cout << e.what() << std::endl;
        return;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lidar Visualization
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene* scene)
{
    static std::vector<QGraphicsItem*> draw_points;

    // Clear previously drawn points
    for (const auto &p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();

    const QColor color("LightGreen");
    const QPen pen(color, 10);
    //const QBrush brush(color, Qt::SolidPattern);

    // Draw each filtered point
    for (const auto &p : points)
    {
        const auto dp = scene->addRect(-25, -25, 50, 50, pen);
        dp->setPos(p.x, p.y);
        draw_points.push_back(dp); // add to the list of points to be deleted next time
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// FORWARD Method
///////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<SpecificWorker::RobotState, float, float> SpecificWorker::FORWARD_method(float min_distance)
{
    if(min_distance < SAFE_DISTANCE)
        return {RobotState::TURN, 0.0f, 0.0f};
    else
    {
        float vlin = MAX_ADV * brake_adv(min_distance, SAFE_DISTANCE);
        float vrot = 0.0f;
        return {RobotState::FORWARD, vlin, vrot};
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// TURN Method
///////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<SpecificWorker::RobotState, float, float> SpecificWorker::TURN_method(float min_distance, float angle_min)
{
    if(min_distance > SAFE_DISTANCE)
    {
        RobotState next = (min_distance < 1200.0f) ? RobotState::WALL : RobotState::SPIRAL;
        return {next, 0.0f, 0.0f};
    }
    else
    {
        float rot_speed = ((angle_min > 0) ? MAX_ROT : -MAX_ROT) * brake_rot(angle_min);
        return {RobotState::TURN, 0.0f, rot_speed};
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// WALL Method
///////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<SpecificWorker::RobotState, float, float> SpecificWorker::WALL_method(const RoboCompLidar3D::TPoints &points)
{
    float min_front = std::numeric_limits<float>::max();
    float min_side  = std::numeric_limits<float>::max();
    float side_angle = 0;

    for(const auto &p : points)
    {
        if(std::abs(p.phi) < M_PI/6) min_front = std::min(min_front, p.r);
        if(p.phi > M_PI/6 && p.phi < M_PI/2 && p.r < min_side)
        {
            min_side = p.r;
            side_angle = p.phi;
        }
    }

    if(min_front < SAFE_DISTANCE)
        return {RobotState::TURN, 0.0f, 0.0f};

    float error = min_side - WALL_DISTANCE;
    float ylin = std::clamp(-std::abs(error)/ROBOT_LENGTH + 1.0f, 0.2f, 1.0f);
    float yrot = std::clamp(1.0f/(std::abs(error)/ROBOT_LENGTH + 0.001f), 0.1f, 1.0f);

    float adv = MAX_ADV * ylin * brake_adv(min_side, WALL_DISTANCE);
    float rot_speed = MAX_ROT * ((error > 0) ? -yrot : yrot) * brake_rot(side_angle);

    return {RobotState::WALL, adv, rot_speed};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// SPIRAL Method
///////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<SpecificWorker::RobotState, float, float> SpecificWorker::SPIRAL_method(float min_distance)
{
    if(min_distance < SAFE_DISTANCE)
    {
        spiral_theta = 0.0f;
        return {RobotState::TURN, 0.0f, 0.0f};
    }

    // Update spiral
    spiral_theta += 0.05f; // increment angle
    spiral_adv = std::min(spiral_adv + 5.0f, MAX_ADV);
    spiral_rot = std::max(spiral_rot - 0.01f, 0.1f);

    // Check if spiral finished (simple example, can define max radius)
    float r = a_spiral + b_spiral * spiral_theta;
    if(r > 3000.0f) // max spiral radius
    {
        spiral_theta = 0.0f;
        spiral_adv = SPIRAL_ADV_INIT;
        spiral_rot = SPIRAL_ROT_INIT;
        return {RobotState::FORWARD, 0.0f, 0.0f};
    }

    float vlin = spiral_adv * brake_adv(min_distance, SAFE_DISTANCE);
    float vrot = spiral_rot * brake_rot(spiral_theta);

    return {RobotState::SPIRAL, vlin, vrot};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Brakes
///////////////////////////////////////////////////////////////////////////////////////////////////////////
float SpecificWorker::brake_adv(float dist, float threshold)
{
    return std::clamp(dist / threshold, 0.0f, 1.0f);
}

float SpecificWorker::brake_rot(float rot)
{
    // rot en [-1,1]
    return rot >= 0 ? std::clamp(1.0f - rot, 0.0f, 1.0f) : std::clamp(1.0f + rot, 0.0f, 1.0f);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Update Robot Pose in Viewer
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::update_robot_position()
{
    try
    {
        RoboCompGenericBase::TBaseState bState;
        omnirobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(bState.alpha * 180.0 / M_PI);
        robot_polygon->setPos(bState.x, bState.z);
        std::cout << bState.alpha << " " << bState.x << " " << bState.z << std::endl;
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e.what() << std::endl;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lidar Filtering: Keep closest point per angle
///////////////////////////////////////////////////////////////////////////////////////////////////////////
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points)
{
    if (points.empty())
        return std::nullopt;

    RoboCompLidar3D::TPoints result;
    result.reserve(points.size());

    constexpr float precision = 100.0f; // Round phi to 2 decimals

    // Loop over the groups produced by iter::groupby
    for (auto&& [angle, group] : iter::groupby(points, [](const auto& p) {
             return std::floor(p.phi * precision) / precision;
         }))
    {
        // Find closest point (min 'r') for current angle
        auto min_it = std::min_element(
            std::begin(group),
            std::end(group),
            [](const auto& a, const auto& b) { return a.r < b.r; }
            );

        result.emplace_back(RoboCompLidar3D::TPoint{
            .x   = min_it->x,
            .y   = min_it->y,
            .phi = min_it->phi,
            .r   = min_it->r
        });
    }

    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Emergency & Restore
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    // Implement emergency behavior

    //if (SUCCESSFUL) // The componet is safe for continue
    //  emmit goToRestore()
}

void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    // Restore from emergency state

    // Restore emergency component
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Startup Check
///////////////////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Mouse Target Slot
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::new_target_slot(QPointF p)
{
    qInfo() << "World coordinates" << p;
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams
