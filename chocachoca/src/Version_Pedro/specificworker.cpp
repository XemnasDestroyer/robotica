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

	// Inicializa la ventana principal
	setupUi(this);

	// Carga el robot para que se dibuje
	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	viewer->show();
	const auto rob = viewer->add_robot(params.ROBOT_LENGTH, params.ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);

	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

	// Carga el viewer (donde se dibuja el robot) dentro del frame en mainUI
	QVBoxLayout *layout = new QVBoxLayout(this->frame);
	layout->addWidget(frame);
	frame->setLayout(layout);
	show();

	//Inicia de forma aleatoria la dirección del primer giro de spiral del robot
	std::srand(std::time(nullptr));
	int rand = std::rand() % 2;
	if(rand == 1) {
		params.rot_direction = -1;
		qInfo() << "Valor iniciado -1";
	}else
		qInfo() << "Valor iniciado a 1";
}


void SpecificWorker::compute()
{
	try {
		const auto data = lidar3d_proxy->getLidarData(params.LIDAR_NAME_LOW, 0, 2*M_PI, 1);
		if (data.points.empty()) { qWarning() << "No points received"; return; }

		const auto filter_data = filter_min_distance_cppitertools(data.points);
		if(!filter_data.has_value()) return;

		auto &points = filter_data.value();
		auto filterPoints = filter_isolated_points(points, 200);

		draw_lidar(points, &viewer->scene);

		std::tuple<State, float, float, float> result;
		QString text;
		switch(current_state)
		{
            case State::SPIRAL:
                result = Spiral(filterPoints);
				text = "Espiral";
				actualAction->setText(text);

				//Resetea el número de forwards y giros hechos
				params.forwardCount = 0;
				params.turnCount = 0;
                break;
			case State::FORWARD:
				result = Forward(filterPoints);
				text = "Avanzar";
				actualAction->setText(text);

				//Resetea la velocidad de giro, cambia (aleatoriamente) la dirección de giro, incrementa el valor que controla las veces que se ha ejecutado forward y resetea la variable de giros hechos
				params.THETA = 1.0f;
				params.rot_direction = params.rot_direction*1;
				params.turnCount = 0;
				params.backCount = 0;
				break;
			case State::FOLLOW_WALL:
				result = Follow_Wall(filterPoints);
				text = "Seguir muro";
				actualAction->setText(text);

				//Resetea la variable de giro a 1, para que podamos controlar eficientemente el recorrido en el muro
				params.rot_direction = 1;
				break;
			case State::BACK:
				result = Back();
				text = "Atrás";
				actualAction->setText(text);
				break;
			case State::TURN:
				result = Turn();
				text = "Girar";
				actualAction->setText(text);
				break;
		}
		// Desempaquetar resultado
		auto [st, advx, advz, rot] = result;
		current_state = st;  // Actualizamos el estado global

		// Enviar velocidades al robot
		try {
			omnirobot_proxy->setSpeedBase(advx, advz, rot*params.rot_direction);
		}catch(const Ice::Exception &e){ std::cout << e << std::endl; }

		update_robot_position();

	} catch (const Ice::Exception& e) {
		std::cout << e.what() << std::endl;
	}
}

std::tuple<State,float,float,float> SpecificWorker::Spiral(const RoboCompLidar3D::TPoints &points){
    float rotation = params.THETA;

    // Disminuir rotación para abrir la espiral
    params.THETA = std::max(float(params.THETA) - 0.003f, 0.0f);

    /* Si no hay ningún punto alrededor suficientemente cercano, sigue en espiral. Si no, pasa a forward
     * Hay 2 condiciones para filtrar posibles errores de lectura del Lidar
     */
	RoboCompLidar3D::TPoint closestPoint = closest_lidar_point(points, false, 0).value();
	minDistNum->display(double(closestPoint.r));
	minFrontDistNum->display(0);
	if (closestPoint.r < 770.0){
		return {State::FORWARD, 0.0f, params.MAX_ADV_SPEED, rotation};
	}
	return {State::SPIRAL, 0.0f, params.MAX_ADV_SPEED, rotation};
}

std::tuple<State,float,float,float> SpecificWorker::Forward(const RoboCompLidar3D::TPoints &points) {
	// Comprueba que no haya ningún punto cercano de frente
	RoboCompLidar3D::TPoint closestFrontPoint = closest_lidar_point(points, true, 70).value();
	minDistNum->display(0);
	minFrontDistNum->display(double(closestFrontPoint.r));
	if(closestFrontPoint.r < 700.0) {
		return {State::FOLLOW_WALL, 0.0f, 0.0f, params.THETA};
	}

    /* Si follow wall se ha ejecutado menos veces que el valor en params.forwardTrigger, incremente ese valor
     * y sigue ejecutando forward
     */
    if(params.forwardCount < params.forwardTrigger) {
		//Incrementa la variable que controla la cantidad de forwards hechos
		params.forwardCount++;
		return {State::FORWARD, 0.0f, params.MAX_ADV_SPEED, 0.f};
	}
    /* Si no se cumple nada de lo anterior, es que no hay muro cerca y se ha ejecutado muchas veces forward, por lo
     * que retrocedemos a spiral
     */
	else
		return {State::SPIRAL, 0.0f, params.MAX_ADV_SPEED, 0.f};
}

std::tuple<State,float,float,float> SpecificWorker::Follow_Wall(const RoboCompLidar3D::TPoints &points) {
	float rotation = params.THETA;
	float velocityX = params.MAX_ADV_SPEED;
	float velocityZ = 0.0f;

    // Busca el punto más cercano al robot. Si está, pone frontObstacle a true
	RoboCompLidar3D::TPoint closestFrontPoint = closest_lidar_point(points, true, 30).value();
	minDistNum->display(0);
	minFrontDistNum->display(double(closestFrontPoint.r));
	if(closestFrontPoint.r < 700){
		return {State::TURN, 0.0f, 0.0f, rotation};
	}

    // Aleatoriamente sale del Follow_Wall para ir al turn 1 de cada 100 veces
	int num = std::rand() % 100; // Número entre 0 y 99
	if(num == 0) {
        // Aleatoriamente cambia la dirección de giro una de cada
		num = std::rand() % 2;
		if (num == 1)
			params.rot_direction = params.rot_direction * -1;
		return {State::TURN, 0.0f, velocityX, rotation};
	}

	RoboCompLidar3D::TPoint closestPoint = closest_lidar_point(points, false, 0).value();
	minDistNum->display(double(closestPoint.r));

    // Si el punto más cercano está cerca se aleja de él. Si no, lo contrario
    if (closestPoint.r <= 500)
		velocityZ = params.MAX_ADV_SPEED/2;
	else
		velocityZ = -params.MAX_ADV_SPEED/2;

    // Decide la dirección de giro dependiendo de dónde esté el punto más cercano. Está dividido en sectores
	qInfo() << "Angulo minimo: " << closestPoint.phi;

	float desiredDistance = 500.0;
	float distanceError = std::abs((closestPoint.r/desiredDistance)-1);
	float velocityZCorrected = velocityZ*distanceError;
	 velocityX = velocityX/1.5;

	float rotationError = (std::abs(closestPoint.phi)/M_PI/2);
	float rotationCorrected = rotation*std::abs(rotationError);
	if(-M_PI < closestPoint.phi && closestPoint.phi <= -M_PI/2){
		return {State::FOLLOW_WALL, velocityZCorrected, velocityX, -rotationCorrected};
	}
	else if(-M_PI/2 < closestPoint.phi && closestPoint.phi <= 0){
		return {State::FOLLOW_WALL, velocityZCorrected, velocityX, rotationCorrected};
	}
	else if(0 < closestPoint.phi && closestPoint.phi <= M_PI/2){
		return {State::FOLLOW_WALL, -velocityZCorrected, velocityX, -rotationCorrected};
	}
	else{
		return {State::FOLLOW_WALL, velocityZCorrected, velocityX, rotationCorrected};
	}
}

std::tuple<State,float,float,float> SpecificWorker::Back() {
    // Si back se ha repetido menos de params.backTrigger veces, sigue haciendo back
	if(params.backCount < params.backTrigger) {
		//Incrementa la variable que controla la cantidad de back hechos
		params.backCount++;
		return {State::BACK, 0.0f, -params.MAX_ADV_SPEED/3, 0.f};
	}
	else
		return {State::TURN, 0.0f, -params.MAX_ADV_SPEED/3, 0.f};
}

std::tuple<State,float,float,float> SpecificWorker::Turn() {
	float rotation = params.THETA;
    // Si back se ha repetido menos de params.turnTrigger veces, sigue haciendo turn
	if(params.turnCount < params.turnTrigger) {
		//Incrementa la variable que controla la cantidad de giros hechos
		params.turnCount++;
		return {State::TURN, 0.0f, 0.0f, rotation};
	}
	else
		return {State::FORWARD, 0.0f, 0.0f, rotation};
}

std::expected<RoboCompLidar3D::TPoint, std::string> SpecificWorker::closest_lidar_point(const RoboCompLidar3D::TPoints &points, bool front, int range){
	if (points.empty())
		return std::unexpected("Empty points container");

	RoboCompLidar3D::TPoints::const_iterator res;
	if(front){
		int frontPointIndex = points.size()/2;
		int offset = range;
		res = std::min_element(points.begin() + (frontPointIndex-offset),
									points.begin() + (frontPointIndex+offset),
									[](const auto &a, const auto &b){
			return a.r < b.r;
		});
	}else {
		res = std::min_element(points.begin(),
									points.end(),
									[](const auto &a, const auto &b) {
			return a.r < b.r;
		});
	}
	return *res;
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

RoboCompLidar3D::TPoints SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d) // set to 200mm
{
	if (points.empty()) return {};

	const float d_squared = d * d;  // Avoid sqrt by comparing squared distances
	std::vector<bool> hasNeighbor(points.size(), false);

	// Create index vector for parallel iteration
	std::vector<size_t> indices(points.size());
	std::iota(indices.begin(), indices.end(), size_t{0});

	// Parallelize outer loop - each thread checks one point
	std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
	{
		const auto& p1 = points[i];
		// Sequential inner loop (avoid nested parallelism)
		for (auto &&[j,p2] : iter::enumerate(points))
		{
			if (i == j) continue;
			const float dx = p1.x - p2.x;
			const float dy = p1.y - p2.y;
			if (dx * dx + dy * dy <= d_squared)
			{
				hasNeighbor[i] = true;
				break;
			}
		}
	});

	// Collect results
	std::vector<RoboCompLidar3D::TPoint> result;
	result.reserve(points.size());
	for (auto &&[i, p] : iter::enumerate(points))
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



