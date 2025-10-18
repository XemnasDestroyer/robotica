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
		switch(current_state)
		{
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
				result = Forward(); // o tu función correspondiente
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

// Precomputación de distancias


void SpecificWorker::computeDistances(const RoboCompLidar3D::TPoints &points) {
    float front = std::numeric_limits<float>::max();
    float left  = std::numeric_limits<float>::max();
    float right = std::numeric_limits<float>::max();

    auto offset_front_left  = closest_lidar_index_to_given_angle(points, -params.FRONT_SECTION);
    auto offset_front_right = closest_lidar_index_to_given_angle(points,  params.FRONT_SECTION);
    if(offset_front_left)  front = std::min(front, points[offset_front_left.value()].r);
    if(offset_front_right) front = std::min(front, points[offset_front_right.value()].r);

    auto offset_left  = closest_lidar_index_to_given_angle(points, -params.SIDE_SECTION);
    auto offset_right = closest_lidar_index_to_given_angle(points,  params.SIDE_SECTION);
    if(offset_left)  left  = points[offset_left.value()].r;
    if(offset_right) right = points[offset_right.value()].r;

    dist.front = front;
	dist.right = right;
	dist.left = left;
}

// Forward
std::tuple<State,float,float> SpecificWorker::Forward() {
    if(dist.front > params.ADVANCE_THRESHOLD)
        return {State::FORWARD, params.MAX_ADV_SPEED, 0.f};
    else
        return {State::TURN, 0.f, 0.f};
}

// Turn
std::tuple<State,float,float> SpecificWorker::Turn() {
    static int rot_sign = 1; // Mantener dirección de giro
    float min_side_dist = std::min(dist.left, dist.right);

    // Pared lateral cerca → follow wall
    if(min_side_dist < params.WALL_MIN_DISTANCE)
        return {State::FOLLOW_WALL, 0.5f*params.MAX_ADV_SPEED, 0.f};

    // Despejado frontal → forward
    if(dist.front > params.ADVANCE_THRESHOLD)
        return {State::FORWARD, params.MAX_ADV_SPEED, 0.f};

    // Mantener giro
    return {State::TURN, 0.f, rot_sign * params.MAX_ROT_SPEED};
}

// Follow wall con histéresis y corrección
std::tuple<State,float,float> SpecificWorker::Follow_Wall() {

	// Elegir lado más cercano dinámicamente
	bool follow_right = (dist.right < dist.left);
	float current_dist = follow_right ? dist.right : dist.left;

	// Error lateral respecto a la distancia deseada
	float target_dist = params.WALL_MIN_DISTANCE;
	float error = target_dist - current_dist;

	// Filtrado exponencial del error para suavizar giros
	static float filtered_error = 0.f;
	const float alpha = 0.3f;
	filtered_error = alpha * error + (1.f - alpha) * filtered_error;

	// Control proporcional de rotación
	float rot_speed = std::clamp(filtered_error / target_dist * params.MAX_ROT_SPEED,
								-params.MAX_ROT_SPEED, params.MAX_ROT_SPEED);
	if(follow_right) rot_speed = -rot_speed; // invertir si seguimos pared derecha

	// Velocidad de avance ajustada según error lateral
	float adv_speed = std::clamp(params.MAX_ADV_SPEED * (1.f - std::abs(filtered_error)/target_dist),
								 0.0f, params.MAX_ADV_SPEED);

	// Manejo de obstáculo frontal
	if(dist.front < params.ADVANCE_THRESHOLD)
	{
		return {State::TURN, 0.f, rot_speed > 0 ? 0.5f : -0.5f};
	}

	// Manejo de pérdida de pared o esquinas
	if(current_dist > params.WALL_EXIT_THRESHOLD)
	{
		return {State::TURN, 0.f, rot_speed > 0 ? 0.5f : -0.5f};
	}

	return {State::FOLLOW_WALL, adv_speed, rot_speed};
}

/*
std::tuple<State,float,float> SpecificWorker::Forward(const RoboCompLidar3D::TPoints& points)
{
	if(points.empty()) return {State::IDLE, 0.f, 0.f};

	// Sector frontal ±LIDAR_FRONT_SECTION
	auto offset_front_left  = closest_lidar_index_to_given_angle(points, -params.FRONT_SECTION);
	auto offset_front_right = closest_lidar_index_to_given_angle(points,  params.FRONT_SECTION);

	float front_dist = std::numeric_limits<float>::max();
	if(offset_front_left)  front_dist = std::min(front_dist, points[offset_front_left.value()].r);
	if(offset_front_right) front_dist = std::min(front_dist, points[offset_front_right.value()].r);

	// Si despejado → Forward
	if(front_dist > params.ADVANCE_THRESHOLD)
		return {State::FORWARD, params.MAX_ADV_SPEED, 0.f};

	// Si obstáculo frontal → pasar a Turn
	return {State::TURN, 0.f, 0.f};
}
*/
/*
std::tuple<State, float, float> SpecificWorker::Forward(const RoboCompLidar3D::TPoints& points)
{
	if(points.empty()) {
		qWarning() << "Empty container at Forward()";
		return {State::IDLE, 0.f, 0.f};
	}

	auto offset_begin = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
	auto offset_end   = closest_lidar_index_to_given_angle(points,  params.LIDAR_FRONT_SECTION);

	if(!offset_begin || !offset_end) {
		qWarning() << "Invalid lidar offsets in Forward(): "
				   << QString::fromStdString(offset_begin.error()) << " "
				   << QString::fromStdString(offset_end.error());
		return {State::IDLE, 0.f, 0.f};
	}

	auto min_point = std::min_element(
		points.begin() + offset_begin.value(),
		points.begin() + offset_end.value(),
	[](const auto &a, const auto &b){ return a.distance2d < b.distance2d; });

	if(min_point == points.end()) {
		qWarning() << "No minimum point found in Forward()";
		return {State::IDLE, 0.f, 0.f};
	}

	if(min_point->r < params.ADVANCE_THRESHOLD)
		return {State::TURN, 0.f, 0.f};

	return {State::FORWARD, params.MAX_ADV_SPEED, 0.f};
}
*/
/*
std::tuple<State, float, float> SpecificWorker::Turn(const RoboCompLidar3D::TPoints &points)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<int> dist(0, 1);
    static bool first_time = true;
    static int sign = 1;

    if(points.empty())
        return {State::IDLE, 0.f, 0.f};

    // Front detection
    auto offset_begin = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
    auto offset_end   = closest_lidar_index_to_given_angle(points,  params.LIDAR_FRONT_SECTION);
    if(!offset_begin || !offset_end) return {State::TURN, 0.f, 0.f};

    auto min_point_it = std::min_element(
        points.begin() + offset_begin.value(),
        points.begin() + offset_end.value(),
        [](const auto &a, const auto &b){ return a.distance2d < b.distance2d; }
    );

    // Si hay margen suficiente para avanzar, pasamos a FORWARD/FOLLOW_WALL
    if(min_point_it != points.end() && min_point_it->distance2d > params.ADVANCE_THRESHOLD)
    {
        first_time = true;
        last_turn_time = std::chrono::steady_clock::now();
        return {State::FOLLOW_WALL, params.MAX_ADV_SPEED, 0.f};
    }

    // Evitar cambios rápidos de signo
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - last_turn_time).count();

    if(first_time)
    {
        auto half = closest_lidar_index_to_given_angle(points, 0.f);
        if(!half) return {State::TURN, 0.f, 0.f};

        float left_sum  = std::accumulate(points.begin(), points.begin() + half.value(), 0.f,
                                          [](float acc, const auto &p){ return acc + 1.f/p.distance2d; });
        float right_sum = std::accumulate(points.begin() + half.value(), points.end(), 0.f,
                                          [](float acc, const auto &p){ return acc + 1.f/p.distance2d; });

        if(std::fabs(left_sum - right_sum) < 1.f)
            sign = dist(gen) == 0 ? -1 : 1;
        else
            sign = left_sum > right_sum ? 1 : -1;

        first_time = false;
        last_turn_time = now;
    }

    float adv = (elapsed > params.TURN_COOLDOWN) ? 0.f : params.MAX_ADV_SPEED * 0.2f; // avanzar ligeramente si hay espacio
    return {State::TURN, adv, sign * params.MAX_ROT_SPEED};
}
*/
/*
std::tuple<State,float,float> SpecificWorker::Turn(const RoboCompLidar3D::TPoints& points)
{
	if(points.empty()) return {State::IDLE, 0.f, 0.f};

	// Sector frontal ±LIDAR_FRONT_SECTION
	auto offset_front_left  = closest_lidar_index_to_given_angle(points, -params.FRONT_SECTION);
	auto offset_front_right = closest_lidar_index_to_given_angle(points,  params.FRONT_SECTION);

	float front_dist = std::numeric_limits<float>::max();
	if(offset_front_left)  front_dist = std::min(front_dist, points[offset_front_left.value()].r);
	if(offset_front_right) front_dist = std::min(front_dist, points[offset_front_right.value()].r);

	// Revisar laterales ±SIDE_SECTION
	auto offset_left  = closest_lidar_index_to_given_angle(points, -params.SIDE_SECTION);
	auto offset_right = closest_lidar_index_to_given_angle(points,  params.SIDE_SECTION);

	float dist_left  = (offset_left)  ? points[offset_left.value()].r  : std::numeric_limits<float>::max();
	float dist_right = (offset_right) ? points[offset_right.value()].r : std::numeric_limits<float>::max();
	float min_side_dist = std::min(dist_left, dist_right);

	// Si pared lateral cerca → Follow_wall
	if(min_side_dist < params.WALL_MIN_DISTANCE)
		return {State::FOLLOW_WALL, 0.5f*params.MAX_ADV_SPEED, 0.f};

	// Si despejado frontal → Forward
	if(front_dist > params.ADVANCE_THRESHOLD)
		return {State::FORWARD, params.MAX_ADV_SPEED, 0.f};

	// Determinar dirección de giro según pared más cercana
	int rot_sign = (dist_left < dist_right) ? 1 : -1;

	// Mantener giro constante
	return {State::TURN, 0.f, rot_sign * params.MAX_ROT_SPEED};
}

std::tuple<State,float,float> SpecificWorker::Follow_Wall(const RoboCompLidar3D::TPoints& points)
{
	if(points.empty()) return {State::IDLE, 0.f, 0.f};

    // Distancias laterales ±SIDE_SECTION
    auto offset_left  = closest_lidar_index_to_given_angle(points, -params.SIDE_SECTION);
    auto offset_right = closest_lidar_index_to_given_angle(points,  params.SIDE_SECTION);

    float dist_left  = (offset_left)  ? points[offset_left.value()].r  : std::numeric_limits<float>::max();
    float dist_right = (offset_right) ? points[offset_right.value()].r : std::numeric_limits<float>::max();

    bool follow_right = (dist_right < dist_left);
    float side_dist = follow_right ? dist_right : dist_left;

    // Sector frontal ±FRONT_SECTION
    auto offset_front_left  = closest_lidar_index_to_given_angle(points, -params.FRONT_SECTION);
    auto offset_front_right = closest_lidar_index_to_given_angle(points,  params.FRONT_SECTION);

    float front_dist = std::numeric_limits<float>::max();
    if(offset_front_left)  front_dist = std::min(front_dist, points[offset_front_left.value()].r);
    if(offset_front_right) front_dist = std::min(front_dist, points[offset_front_right.value()].r);

    // Si obstáculo frontal → pasar a Turn
    if(front_dist < params.ADVANCE_THRESHOLD)
        return {State::TURN, 0.f, 0.f};

    // Error lateral y corrección de rotación
    float error = side_dist - params.WALL_MIN_DISTANCE;

    // Si error grande al iniciar → alinear primero sin avanzar demasiado
    float rot_speed = std::clamp(error / params.WALL_MIN_DISTANCE * params.MAX_ROT_SPEED,
                                -params.MAX_ROT_SPEED, params.MAX_ROT_SPEED);
    if(follow_right) rot_speed = -rot_speed;

    float adv_speed = params.MAX_ADV_SPEED;

    // Reducir avance si error lateral grande
    const float error_threshold = 0.3f * params.WALL_MIN_DISTANCE;
    if(std::abs(error) > error_threshold)
        adv_speed *= 0.5f;  // reduce velocidad mientras corrige

    adv_speed = std::clamp(adv_speed, 0.1f, 0.6f*params.MAX_ADV_SPEED);

    return {State::FOLLOW_WALL, adv_speed, rot_speed};
}
*/

/*
std::tuple<State, float, float> SpecificWorker::Follow_Wall(const RoboCompLidar3D::TPoints &points)
{
    static bool first_time = true;
    static enum class HANDNESS { LEFT, RIGHT } handness;

    auto offset_begin = closest_lidar_index_to_given_angle(points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(points, params.LIDAR_FRONT_SECTION);

    if(!offset_begin || !offset_end) return {State::FOLLOW_WALL, 0.f, 0.f};

    auto min_point = std::min_element(points.begin() + offset_begin.value(),
                                      points.begin() + offset_end.value(),
                                      [](auto &a, auto &b){ return a.distance2d < b.distance2d; });

    // Pre-detección: si estamos muy cerca, pasamos a Turn
    if(min_point != points.end() && min_point->distance2d < params.STOP_THRESHOLD)
    {
        first_time = true;
        last_turn_time = std::chrono::steady_clock::now();
        return {State::TURN, 0.f, 0.f};
    }

    // Lateral points
    auto res_right = closest_lidar_index_to_given_angle(points, params.LIDAR_RIGHT_SIDE_SECTION);
    auto res_left  = closest_lidar_index_to_given_angle(points, params.LIDAR_LEFT_SIDE_SECTION);
    if(!res_right || !res_left) return {State::FOLLOW_WALL, 0.f, 0.f};

    auto right_point = points[res_right.value()];
    auto left_point  = points[res_left.value()];

    if(first_time)
    {
        handness = (right_point.distance2d < left_point.distance2d) ? HANDNESS::RIGHT : HANDNESS::LEFT;
        first_time = false;
    }

    RoboCompLidar3D::TPoint min_obj = (handness == HANDNESS::RIGHT) ? right_point : left_point;

    float error = min_obj.distance2d - params.WALL_MIN_DISTANCE;
    float adv_brake = std::clamp(-1.f / (params.ROBOT_LENGTH / 2.f) * std::fabs(error) + 1.f, 0.f, 1.f);
    float rot_brake = std::clamp(1.f / (params.ROBOT_LENGTH / 3.f) * std::fabs(error), 0.f, 1.f);

    // Cooldown para evitar cambios rápidos de dirección
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - last_turn_time).count();
    if(elapsed < params.TURN_COOLDOWN)
        adv_brake *= 0.5f;  // reducir avance si acabamos de girar

    if(handness == HANDNESS::RIGHT)
        return {State::FOLLOW_WALL, params.MAX_ADV_SPEED * adv_brake, (error > 0 ? 1.f : -1.f) * params.MAX_ROT_SPEED * rot_brake};
    else
        return {State::FOLLOW_WALL, params.MAX_ADV_SPEED * adv_brake, (error > 0 ? -1.f : 1.f) * params.MAX_ROT_SPEED * rot_brake};
}
*/
/*float break_adv(float dist){
	return  std::clamp(dist * (1/DIST_THRESHOLD), 1, 0);
}

float break_rot(float rot){
	return rot>=0 ? std::clamp(rot+1, 1, 0) : std::clamp(1-rot, 0, 1);
}*/


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



