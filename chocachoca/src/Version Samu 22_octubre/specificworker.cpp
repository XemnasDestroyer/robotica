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
	last_turn_time = std::chrono::steady_clock::now();
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
	const auto rob = viewer->add_robot(params.ROBOT_LENGTH, params.ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);

	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
}


void SpecificWorker::compute()
{
	try {
		const auto data = lidar3d_proxy->getLidarData(params.LIDAR_NAME_LOW, 0, 2*M_PI, 1);
		if (data.points.empty()) { qWarning() << "No points received"; return; }

		const auto filter_data = filter_min_distance_cppitertools(data.points);
		if(!filter_data.has_value()) return;

		auto &points = filter_data.value();

		draw_lidar(points, &viewer->scene);

		computeDistances(points);

		std::tuple<State, float, float> result = {State::IDLE, 0.f, 0.f};

		// Llamamos a la función según el estado actual
		switch(current_state) {
			case State::IDLE:
				if(dist.min_360 > 250.0f) {
					result = Spiral();
					qInfo() << "Spiral";
				} else {
					result = Forward();
					qInfo() << "Forward";
				}
				break;
			case State::FORWARD:
				result = Forward();
				qInfo() << "Forward";
				break;
			case State::TURN:
				result = Turn();
				qInfo() << "Turn";
				break;
			case State::FOLLOW_WALL:
				result = Follow_Wall();
				qInfo() << "Follow_wall";
				break;
			case State::SPIRAL:
				result = Spiral();
				qInfo() << "Spiral";
				break;
		}

		// Desempaquetar resultado
		auto [st, adv, rot] = result;
		current_state = st;  // Actualizamos el estado global

		// Enviar velocidades al robot
		try { omnirobot_proxy->setSpeedBase(0, adv, rot); }
		catch(const Ice::Exception &e){ std::cout << e << std::endl; }

		update_robot_position();

	} catch (const Ice::Exception& e) {
		std::cout << e.what() << std::endl;
	}
}

void SpecificWorker::computeDistances(const RoboCompLidar3D::TPoints &points) {

    float min_distance_360 = std::numeric_limits<float>::max();

    // FRONTAL: sector de 60° (-30° a +30°)
    auto front_begin = closest_lidar_index_to_given_angle(points, -params.FRONT_SECTION);
    auto front_end = closest_lidar_index_to_given_angle(points, params.FRONT_SECTION);
    if(front_begin && front_end) {
        auto min_front = std::min_element(points.begin() + front_begin.value(),
                                         points.begin() + front_end.value() + 1,
                                         [](auto &a, auto &b) { return a.r < b.r; });
        if(min_front != points.begin() + front_end.value() + 1 && min_front->r > 10) {
            dist.front = min_front->r;
        }
    }

    // IZQUIERDA: sector de 90° (30° a 120°)
    auto left_begin = closest_lidar_index_to_given_angle(points, params.FRONT_SECTION);  // 30°
    auto left_end = closest_lidar_index_to_given_angle(points, params.SIDE_SECTION);     // 120°
    if(left_begin && left_end) {
        int start = std::min(left_begin.value(), left_end.value());
        int end = std::max(left_begin.value(), left_end.value());
        auto min_left = std::min_element(points.begin() + start,
                                        points.begin() + end + 1,
                                        [](auto &a, auto &b) { return a.r < b.r; });
        if(min_left != points.begin() + end + 1 && min_left->r > 10) {
            dist.left = min_left->r;
        }
    }

    // DERECHA: sector de 90° (-30° a -120°)
    auto right_begin = closest_lidar_index_to_given_angle(points, -params.FRONT_SECTION);  // -30°
    auto right_end = closest_lidar_index_to_given_angle(points, -params.SIDE_SECTION);     // -120°
    if(right_begin && right_end) {
        int start = std::min(right_begin.value(), right_end.value());
        int end = std::max(right_begin.value(), right_end.value());
        auto min_right = std::min_element(points.begin() + start,
                                         points.begin() + end + 1,
                                         [](auto &a, auto &b) { return a.r < b.r; });
        if(min_right != points.begin() + end + 1 && min_right->r > 10) {
            dist.right = min_right->r;
        }
    }

    // MIN_360
    for(const auto &point : points) {
        if(point.r < min_distance_360 && point.r > 10) {
            min_distance_360 = point.r;
        }
    }

    dist.min_360 = min_distance_360;
}

// Forward
std::tuple<State,float,float> SpecificWorker::Forward() {
	if(dist.front < params.ADVANCE_THRESHOLD * 0.7f) {
		return {State::TURN, 0.f, (dist.left < dist.right ? +0.5f : -0.5f)};
	}

	if(dist.front > params.ADVANCE_THRESHOLD * 2.5f &&
	   dist.left > params.ADVANCE_THRESHOLD * 2.0f &&
	   dist.right > params.ADVANCE_THRESHOLD * 2.0f) {
		return {State::SPIRAL, 0.f, 0.f};
	}

    return {State::FORWARD, params.MAX_ADV_SPEED, 0.f};
}

// Turn
std::tuple<State,float,float> SpecificWorker::Turn() {
	static float rot_sign = 1.0f; // Mantener dirección de giro

	// Si hay espacio libre al frente → selección aleatoria entre FORWARD y FOLLOW_WALL
	if(dist.front > params.ADVANCE_THRESHOLD) {
		// Selección binaria aleatoria (50% cada uno)
		static std::random_device rd;
		static std::mt19937 gen(rd());
		static std::uniform_int_distribution<> dis(0, 1);

		if(dis(gen) == 0) {
			return {State::SPIRAL, 0.f, 0.f};
		} else {
			return {State::FOLLOW_WALL, params.MAX_ADV_SPEED, 0.f};
		}
	}

	return {State::TURN, 0.f, rot_sign * params.MAX_ROT_SPEED};
}

std::tuple<State,float,float> SpecificWorker::Follow_Wall()
{
	// 1. Colisión inminente
	if(dist.front < params.ADVANCE_THRESHOLD) {
		return {State::TURN, 0.f, (dist.left < dist.right ? +0.5f : -0.5f)};
	}

	bool follow_right = (dist.right < dist.left);
	float current_dist = follow_right ? dist.right : dist.left;

	const float DELTA = params.WALL_MIN_DISTANCE * 0.15f; // 15% de margen

	float rot_speed = 0.0f;

	// 2. Control simplificado del pseudocódigo
	if(current_dist < params.WALL_MIN_DISTANCE - DELTA) {
		rot_speed = -0.3f; // Alejarse
	}
	else if(current_dist > params.WALL_MIN_DISTANCE + DELTA) {
		rot_speed = +0.3f; // Acercarse
	}

	// Invertir si seguimos pared derecha
	if(follow_right) rot_speed = -rot_speed;

	// 3. Velocidad adaptativa (opcional)
	float adv_speed = params.MAX_ADV_SPEED * 1.2f;
	if(std::abs(rot_speed) > 0.1f) {
		adv_speed *= 0.7f; // Reducir velocidad al corregir
	}

	return {State::FOLLOW_WALL, adv_speed, rot_speed};
}

std::tuple<State,float,float> SpecificWorker::Spiral()
{
	static float spiral_theta = 0.1f;
	static const float DIRECTION = 1.0f;

	// ESPIRAL QUE SE VE COMO ESPIRAL
	const float a = 40.0f;
	const float b = 25.0f;           // Crecimiento visible
	const float v = 500.0f;          // Velocidad para ver la expansión

	spiral_theta += 0.12f;
	float r = a + b * spiral_theta;

	float omega = (v / r) * DIRECTION;
	omega = std::clamp(std::abs(omega), 0.1f, 0.8f) * DIRECTION;

	// CONDICIONES DE SALIDA MEJORADAS
	bool should_exit = false;

	// 1. Verificar obstáculo frontal (más sensible)
	if(dist.front < 400.0f) { // 400mm en lugar de 100mm
		std::cout << "SPIRAL → Obstrucción frontal: " << dist.front << "mm" << std::endl;
		should_exit = true;
	}

	// 2. Verificar obstáculos laterales también
	if(dist.left < 125.0f || dist.right < 125.0f) {
		std::cout << "SPIRAL → Obstrucción lateral detectada" << std::endl;
		should_exit = true;
	}

	// 3. Radio máximo más conservador
	if(r > 3000.0f) { // 2000mm en lugar de 3000mm
		std::cout << "SPIRAL → Radio máximo alcanzado: " << r << "mm" << std::endl;
		should_exit = true;
	}

	if(should_exit) {
		return {State::FORWARD, 450.0f, 0.f};
	}

	return {State::SPIRAL, v, omega};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints& filtered_points, QGraphicsScene *scene)
{
    if(filtered_points.empty()) return;

    static std::vector<QGraphicsItem*> items;

    for(auto i: items) { scene->removeItem(i); delete i; }
    items.clear();

    QBrush greenBrush(Qt::green);
    QPen greenPen(Qt::green);

    for(const auto &p : filtered_points) {
        auto item = scene->addRect(-50, -50, 100, 100, greenPen, greenBrush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }

    auto offset_begin = closest_lidar_index_to_given_angle(filtered_points, -params.FRONT_SECTION);
    auto offset_end   = closest_lidar_index_to_given_angle(filtered_points,  params.FRONT_SECTION);

    if(!offset_begin || !offset_end) return;

    int ob = std::clamp(offset_begin.value(), 0, static_cast<int>(filtered_points.size()) - 1);
    int oe = std::clamp(offset_end.value(),   0, static_cast<int>(filtered_points.size()) - 1);
    if(ob > oe) std::swap(ob, oe);

    auto min_point = std::min_element(
        filtered_points.begin() + ob,
        filtered_points.begin() + oe + 1,
        [](const auto &a, const auto &b){ return a.r < b.r; }
    );
    if(min_point == filtered_points.end()) return;

    QColor dcolor = (min_point->r < 400) ? Qt::red : Qt::magenta;
    auto ditem = scene->addRect(-100, -100, 200, 200, dcolor, QBrush(dcolor));
    ditem->setPos(min_point->x, min_point->y);
    items.push_back(ditem);

    // lateral points
    auto wall_right = closest_lidar_index_to_given_angle(filtered_points, params.SIDE_SECTION);
    auto wall_left  = closest_lidar_index_to_given_angle(filtered_points, -params.SIDE_SECTION);
    if(!wall_right || !wall_left) return;

    auto right_point = filtered_points[wall_right.value()];
    auto left_point  = filtered_points[wall_left.value()];
    auto min_obj     = (right_point.r < left_point.r) ? right_point : left_point;

    auto item_obj = scene->addRect(-100, -100, 200, 200, QColorConstants::Svg::orange, QBrush(QColorConstants::Svg::orange));
    item_obj->setPos(min_obj.x, min_obj.y);
    items.push_back(item_obj);

    auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f), QPointF(min_obj.x, min_obj.y)), QPen(QColorConstants::Svg::orange, 10));
    items.push_back(item_line);

    // frontal lines
    auto res_right = closest_lidar_index_to_given_angle(filtered_points, params.FRONT_SECTION);
    auto res_left  = closest_lidar_index_to_given_angle(filtered_points, -params.FRONT_SECTION);
    if(!res_right || !res_left) return;

    float right_length = filtered_points[res_right.value()].r;
    float left_length  = filtered_points[res_left.value()].r;
    float angle1       = filtered_points[res_left.value()].phi;
    float angle2       = filtered_points[res_right.value()].phi;

    QLineF line_left{QPointF(0.f,0.f), robot_polygon->mapToScene(left_length * sin(angle1), left_length * cos(angle1))};
    QLineF line_right{QPointF(0.f,0.f), robot_polygon->mapToScene(right_length * sin(angle2), right_length * cos(angle2))};

    auto line1 = scene->addLine(line_left, QPen(Qt::blue, 10));
    auto line2 = scene->addLine(line_right, QPen(Qt::red, 10));
    items.push_back(line1);
    items.push_back(line2);
}


std::expected<int, std::string> SpecificWorker::closest_lidar_index_to_given_angle(const RoboCompLidar3D::TPoints& points, float angle)
{
	if(points.empty())
		return std::unexpected("Empty points container");

	auto res = std::min_element(points.begin(), points.end(), [angle](const auto &a, const auto &b) {
		return std::abs(a.phi - angle) < std::abs(b.phi - angle);
	});

	if(res != points.end())
		return std::distance(points.begin(), res);
	else
		return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
}

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



