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

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
    this->startup_check_flag = startup_check;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
#ifdef HIBERNATION_ENABLED
        hibernationChecker.start(500);
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

        statemachine.setChildMode(QState::ExclusiveStates);
        statemachine.start();

        auto error = statemachine.errorString();
        if (error.length() > 0){
            qWarning() << error;
            throw error;
        }
    }
}

SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

    this->dimensions = QRectF(-6000, -3000, 12000, 6000);
    viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
    this->resize(900,450);
    viewer->show();
    const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
    robot_polygon = std::get<0>(rob);

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
}

void SpecificWorker::compute()
{
    float min_distance = 10000.0;
    float min_angle = 0.0;
    float min_x = 0.0;
    float min_y = 0.0;

    try
    {
        const auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);
        qInfo() << "full" << data.points.size();
        if (data.points.empty()) { qWarning() << "No points received"; return; }

        const auto filter_data = filter_min_distance_cppitertools(data.points);
        qInfo() << filter_data.value().size();

        if (filter_data.has_value())
            draw_lidar(filter_data.value(), &viewer->scene);


        float pi = 3.1416;

        const auto& puntos = filter_data.value();  // Obtener el vector de puntos
        for (const auto& p : puntos) {
            //if (p.phi < 0.5 and p.phi > -0.5 and p.r < min_distance) {
            if (p.phi < pi/2 and p.phi > -pi/2 and p.r<min_distance){
                min_distance = p.r;
                min_angle = p.phi;
                min_x = p.x;
                min_y = p.y;
            }
        }

        update_robot_position();
    }
    catch (const Ice::Exception& e) {std::cout << e.what() << std::endl;}

    qInfo() << "Distancia minima =" << min_distance;

    std::tuple<State, float, float> result = Forward(min_distance, min_angle, min_x, min_y);	//State -> enum class
    State state = std::get<State>(result);
    float velocityX = std::get<1>(result);
    float velocityR = std::get<2>(result);
    switch(state)
    {
    case State::IDLE:
        break;
    case State::FORWARD:
        this->omnirobot_proxy->setSpeedBase(0.0, velocityX, velocityR);
        break;
    case State::TURN:
        this->omnirobot_proxy->setSpeedBase(0.0, velocityX, velocityR);
        break;
    case State::FOLLOW_WALL:
        break;
    case State::SPIRAL:
        this->omnirobot_proxy->setSpeedBase(0.0, velocityX, velocityR);
        break;
    }
}

std::tuple<State, float, float> SpecificWorker::Forward(float& min_distance, float& min_angle, float& min_x, float& min_y) {
    std::tuple<State, float, float> result;
    float rot = atan2(min_x, min_y);
    if(min_distance < 100){
        qInfo() << "Angulo =" << min_angle;
        if(min_angle < 0.0) {
            result = {State::TURN, 0.0, rot};
            qInfo() << "Izquierda" << "rot = " << rot;
        }
        else{
            result = {State::TURN, 0.0, -rot};
            qInfo() << "Derecha" << "rot = " << -rot;
        }
    }
    else{
        if(min_distance < 2000) {
            spiral_vel = 300.0;
            result = {State::FORWARD, 400.0, 0.0};
        }
        else
            result = {State::SPIRAL, spiral_vel+=2, 0.8};
        //result = {State::SPIRAL, spiral_vel+=5, rot};
    }
    return result;
}

/*float break_adv(float dist){
    return  std::clamp(dist * (1/DIST_THRESHOLD), 1, 0);
}

float break_rot(float rot){
    return rot>=0 ? std::clamp(rot+1, 1, 0) : std::clamp(1-rot, 0, 1);
}*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene* scene)
{
    static std::vector<QGraphicsItem*> draw_points;
    for (const auto &p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();


    const QColor color("LightGreen");
    const QPen pen(color, 10);
    //const QBrush brush(color, Qt::SolidPattern);
    for (const auto &p : points)
    {
        const auto dp = scene->addRect(-25, -25, 50, 50, pen);
        dp->setPos(p.x, p.y);
        draw_points.push_back(dp);   // add to the list of points to be deleted next time
    }
}

void SpecificWorker::update_robot_position()
{
    try
    {
        RoboCompGenericBase::TBaseState bState;
        omnirobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(bState.alpha*180/M_PI);
        robot_polygon->setPos(bState.x, bState.z);
        std::cout << bState.alpha << " " << bState.x << " " << bState.z << std::endl;
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
}

std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points){
    //non empty condition
    if (points.empty())
        return {};

    RoboCompLidar3D::TPoints result; result.reserve(points.size());

    // 3. Loop over the groups produced by iter::groupby
    for (auto&& [angle, group] : iter::groupby(points, [](const auto& p)
                                               {
                                                   float multiplier = std::pow(10.0f, 2); return std::floor(p.phi * multiplier) / multiplier;
                                               }))
    {
        // 'group' is an iterable object containing all Points for the current angle.
        auto min_it = std::min_element(std::begin(group), std::end(group),
                                       [](const auto& a, const auto& b) { return a.r < b.r;});
        result.emplace_back(RoboCompLidar3D::TPoint{.x=min_it->x, .y=min_it->y, .phi=min_it->phi, .r=min_it->r});
    }

    return result;
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}



//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
    return 0;
}

void SpecificWorker::new_target_slot(QPointF p) {
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

