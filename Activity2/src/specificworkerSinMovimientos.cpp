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
#include "hungarian.h"
#include "munkres.hpp"
#include "ransac_line_detector.h"
#include "room_detector.h"

// Inicialización de la sala nominal (igual que tu original)
NominalRoom room(10000.f, 5000.f, Corners{
        {QPointF{-5000.f, -2500.f}, 0.f, 0.f},
        {QPointF{5000.f, -2500.f}, 0.f, 0.f},
        {QPointF{5000.f, 2500.f}, 0.f, 0.f},
        {QPointF{-5000.f, 2500.f}, 0.f, 0.f}
});

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

    // Carga el robot para que se dibuje
    this->dimensions = QRectF(-6000, -3000, 12000, 6000);
    viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
    viewer->show();
    const auto rob = viewer->add_robot(400, 400, 0, 190, QColor("Blue"));
    robot_polygon = std::get<0>(rob);

    //connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

    viewer_room = new AbstractGraphicViewer(this->frame_room, dimensions);
    auto [rr, re] = viewer_room->add_robot(movimientos.params.ROBOT_LENGTH, movimientos.params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
    robot_room_draw = rr;

    // draw room in viewer_room
    QRectF dimensions2 = QRectF(-5000, -2500, 10000, 5000);
    viewer_room->scene.addRect(dimensions2, QPen(Qt::black, 30));
    viewer_room->show();

    // initialise robot pose
    robot_pose.setIdentity();
    robot_pose.translate(Eigen::Vector2d(0.0,0.0));
}

void SpecificWorker::compute()
{
    try {
        const auto data = lidar3d_proxy->getLidarDataWithThreshold2d(movimientos.params.LIDAR_NAME_HIGH, 12000, 1);
        if (data.points.empty()) {
            qWarning() << "No points received";
            return;
        }

        const auto filter_data = filter_min_distance_cppitertools(data.points);
        if (!filter_data.has_value()) return;

        auto &points = filter_data.value();
        auto filterPoints = filter_isolated_points(points, 200);

        draw_lidar(data.points, &viewer->scene);

        Corners measured_corners = room_detector.compute_corners(data.points, nullptr);

        Corners nominal_corners = room.transform_corners_to(robot_pose.inverse());

        Match match = hungarian.match(measured_corners, nominal_corners, 1000);

        Eigen::MatrixXd W(match.size() * 2, 3);
        Eigen::VectorXd b(match.size() * 2);
        for (auto &&[i, m]: match | iter::enumerate) {
            auto &[meas_c, nom_c, _] = m;
            auto &[p_meas, __, ___] = meas_c;
            auto &[p_nom, ____, _____] = nom_c;

            b(2 * i) = p_nom.x() - p_meas.x();

            b(2 * i + 1) = p_nom.y() - p_meas.y();
            W.block<1, 3>(2 * i, 0) << 1.0, 0.0, -p_meas.y();
            W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0, p_meas.x();
        }

        const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;
        if (r.array().isNaN().any())
            return;

        robot_pose.translate(Eigen::Vector2d(r(0), r(1)));
        robot_pose.rotate(r[2]);

        robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
        const double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
        robot_room_draw->setRotation(qRadiansToDegrees(angle));

        std::tuple<State, float, float, float> result;
        switch (current_state) {
            case State::SPIRAL:
                result = movimientos.Spiral(filterPoints);
                break;
            case State::FORWARD:
                result = movimientos.Forward(filterPoints);
                break;
            case State::FOLLOW_WALL:
                result = movimientos.Follow_Wall(filterPoints);
                break;
            case State::TURN:
                result = movimientos.Turn(filterPoints);
                break;
        }
        // Desempaquetar resultado
        auto [st, velocityX, velocityZ, rotation] = result;
        current_state = st;  // Actualizamos el estado global

        try {
            omnirobot_proxy->setSpeedBase(velocityX, velocityZ, rotation * movimientos.params.rot_direction);
        } catch (const Ice::Exception &e) { std::cout << e << std::endl; }

    } catch (const Ice::Exception &e) {
        std::cout << e.what() << std::endl;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &filtered_points, QGraphicsScene *scene) {
    if (filtered_points.empty()) return;

    static std::vector<QGraphicsItem *> items;

    for (auto i: items) {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    QBrush greenBrush(Qt::green);
    QPen greenPen(Qt::green);

    for (const auto &p: filtered_points) {
        auto item = scene->addRect(-50, -50, 100, 100, greenPen, greenBrush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }

    auto offset_begin = closest_lidar_index_to_given_angle(filtered_points, -movimientos.params.FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(filtered_points, movimientos.params.FRONT_SECTION);

    if (!offset_begin || !offset_end) return;

    int ob = std::clamp(offset_begin.value(), 0, static_cast<int>(filtered_points.size()) - 1);
    int oe = std::clamp(offset_end.value(), 0, static_cast<int>(filtered_points.size()) - 1);
    if (ob > oe) std::swap(ob, oe);

    auto min_point = std::min_element(
            filtered_points.begin() + ob,
            filtered_points.begin() + oe + 1,
            [](const auto &a, const auto &b) { return a.r < b.r; }
    );
    if (min_point == filtered_points.end()) return;

    QColor dcolor = (min_point->r < 400) ? Qt::red : Qt::magenta;
    auto ditem = scene->addRect(-100, -100, 200, 200, dcolor, QBrush(dcolor));
    ditem->setPos(min_point->x, min_point->y);
    items.push_back(ditem);

    // lateral points
    auto wall_right = closest_lidar_index_to_given_angle(filtered_points, movimientos.params.SIDE_SECTION);
    auto wall_left = closest_lidar_index_to_given_angle(filtered_points, -movimientos.params.SIDE_SECTION);
    if (!wall_right || !wall_left) return;

    auto right_point = filtered_points[wall_right.value()];
    auto left_point = filtered_points[wall_left.value()];
    auto min_obj = (right_point.r < left_point.r) ? right_point : left_point;

    auto item_obj = scene->addRect(-100, -100, 200, 200, QColorConstants::Svg::orange,
                                   QBrush(QColorConstants::Svg::orange));
    item_obj->setPos(min_obj.x, min_obj.y);
    items.push_back(item_obj);

    auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f), QPointF(min_obj.x, min_obj.y)),
                                    QPen(QColorConstants::Svg::orange, 10));
    items.push_back(item_line);

    // frontal lines
    auto res_right = closest_lidar_index_to_given_angle(filtered_points, movimientos.params.FRONT_SECTION);
    auto res_left = closest_lidar_index_to_given_angle(filtered_points, -movimientos.params.FRONT_SECTION);
    if (!res_right || !res_left) return;

    float right_length = filtered_points[res_right.value()].r;
    float left_length = filtered_points[res_left.value()].r;
    float angle1 = filtered_points[res_left.value()].phi;
    float angle2 = filtered_points[res_right.value()].phi;

    QLineF line_left{QPointF(0.f, 0.f),
                     robot_polygon->mapToScene(left_length * sin(angle1), left_length * cos(angle1))};
    QLineF line_right{QPointF(0.f, 0.f),
                      robot_polygon->mapToScene(right_length * sin(angle2), right_length * cos(angle2))};

    auto line1 = scene->addLine(line_left, QPen(Qt::blue, 10));
    auto line2 = scene->addLine(line_right, QPen(Qt::red, 10));
    items.push_back(line1);
    items.push_back(line2);
}

std::expected<int, std::string>
SpecificWorker::closest_lidar_index_to_given_angle(const RoboCompLidar3D::TPoints &points, float angle) {
    if (points.empty())
        return std::unexpected("Empty points container");

    auto res = std::min_element(points.begin(), points.end(), [angle](const auto &a, const auto &b) {
        return std::abs(a.phi - angle) < std::abs(b.phi - angle);
    });

    if (res != points.end())
        return std::distance(points.begin(), res);
    else
        return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
}

void SpecificWorker::update_robot_position() {
    try {
        RoboCompGenericBase::TBaseState bState;
        omnirobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(bState.alpha * 180.0 / M_PI);
        robot_polygon->setPos(bState.x, bState.z);
        std::cout << bState.alpha << " " << bState.x << " " << bState.z << std::endl;
    }
    catch (const Ice::Exception &e) {
        std::cout << e.what() << std::endl;
    }
}

std::optional<RoboCompLidar3D::TPoints>
SpecificWorker::filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints &points) {
    if (points.empty())
        return std::nullopt;

    RoboCompLidar3D::TPoints result;
    result.reserve(points.size());

    constexpr float precision = 100.0f; // Round phi to 2 decimals

    // Loop over the groups produced by iter::groupby
    for (auto &&[angle, group]: iter::groupby(points, [](const auto &p) {
        return std::floor(p.phi * precision) / precision;
    })) {
        // Find closest point (min 'r') for current angle
        auto min_it = std::min_element(
                std::begin(group),
                std::end(group),
                [](const auto &a, const auto &b) { return a.r < b.r; }
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

RoboCompLidar3D::TPoints
SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d) // set to 200mm
{
    if (points.empty()) return {};

    const float d_squared = d * d;  // Avoid sqrt by comparing squared distances
    std::vector<bool> hasNeighbor(points.size(), false);

    // Create index vector for parallel iteration
    std::vector<size_t> indices(points.size());
    std::iota(indices.begin(), indices.end(), size_t{0});

    // Parallelize outer loop - each thread checks one point
    std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i) {
        const auto &p1 = points[i];
        // Sequential inner loop (avoid nested parallelism)
        for (auto &&[j, p2]: iter::enumerate(points)) {
            if (i == j) continue;
            const float dx = p1.x - p2.x;
            const float dy = p1.y - p2.y;
            if (dx * dx + dy * dy <= d_squared) {
                hasNeighbor[i] = true;
                break;
            }
        }
    });

    // Collect results
    std::vector<RoboCompLidar3D::TPoint> result;
    result.reserve(points.size());
    for (auto &&[i, p]: iter::enumerate(points))
        if (hasNeighbor[i])
            result.push_back(points[i]);
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

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(std::string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(std::string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(std::string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

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
