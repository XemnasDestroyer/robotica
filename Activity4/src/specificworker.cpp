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
#include <QGraphicsScene>
#include <QBrush>
#include <QPen>
#include <cmath>
#include <vector>
#include <algorithm>
#include <ranges>
#include <Eigen/Dense>
#include <iostream>
#include <cppitertools/groupby.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/itertools.hpp>
#include <cppitertools/enumerate.hpp>

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

void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{

}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize()
{
    std::cout << "Initialize worker" << std::endl;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        // Viewer para robot
        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_draw = r;

        // Viewer para la habitación
        viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
        auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_room_draw = rr;

        // Dibuja la habitación nominal
        habitacion = viewer_room->scene.addRect(nominal_rooms[idHabitacion].rect(), QPen(Qt::black, 30));
        show();

        // Inicializa pose del robot
        robot_pose.setIdentity();
        robot_pose.translate(Eigen::Vector2d(0.0,0.0));

        // Inicializa plotter del error de matching
        TimeSeriesPlotter::Config plotConfig;
        plotConfig.title = "Maximum Match Error Over Time";
        plotConfig.yAxisLabel = "Error (mm)";
        plotConfig.timeWindowSeconds = 15.0;
        plotConfig.autoScaleY = false;
        plotConfig.yMin = 0;
        plotConfig.yMax = 1000;
        time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
        match_error_graph = time_series_plotter->addGraph("", Qt::blue);

        // Stop inicial del robot
        move_robot(0, 0, 0);
    }
}

void SpecificWorker::compute()
{
    auto data = read_data();

    draw_lidar(data);

    // Calcular esquinas y líneas de la habitación
    const auto &[corners, _] = room_detector.compute_corners(data, &viewer->scene);

    // Emparejar esquinas detectadas con esquinas nominales transformadas al marco del robot
    const auto match = hungarian.match(
        corners,
        nominal_rooms[idHabitacion].transform_corners_to(robot_pose.inverse())
    );

    // Calcular error máximo de emparejamiento
    float max_match_error = 99999.f;
    if (!match.empty())
    {
        // Encontrar el error máximo en el emparejamiento
        const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
            { return std::get<2>(a) < std::get<2>(b); });
        max_match_error = static_cast<float>(std::get<2>(*max_error_iter));

        // Añadir punto de datos al plotter de errores
        time_series_plotter->addDataPoint(match_error_graph, max_match_error);
    }

    // Actualizar pose del robot solo si está localizado
    if (localised)
        update_robot_pose(match);

    // Procesar máquina de estados - pasar centro convertido
    auto [st, adv, rot] = process_state();
    state = st;

    move_robot(adv, rot, max_match_error);
    robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());

    // Calcular y establecer ángulo de rotación del robot
    const double angle = qRadiansToDegrees(std::atan2(robot_pose.rotation()(1,0), robot_pose.rotation()(0,0)));
    robot_room_draw->setRotation(angle);

    // Actualizar interfaz gráfica
    time_series_plotter->update();
    lcdNumber_adv->display(adv);        // Mostrar velocidad de avance
    lcdNumber_rot->display(rot);        // Mostrar velocidad de rotación
    lcdNumber_x->display(robot_pose.translation().x());  // Mostrar posición X
    lcdNumber_y->display(robot_pose.translation().y());  // Mostrar posición Y
    lcdNumber_angle->display(angle);    // Mostrar ángulo

    // Actualizar tiempo de última iteración
    last_time = std::chrono::high_resolution_clock::now();
}

// Lectura de datos y filtrado
RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
    const auto data = lidar3d_proxy->getLidarDataWithThreshold2d(params.LIDAR_NAME_HIGH, 12000, 2);

    RoboCompLidar3D::TPoints salida; salida.reserve(data.points.size());
    // Agrupar por phi y obtener el mínimo de r por grupo en una línea, usando push_back para almacenar en el std::vector
    for (auto&& [angle, group] : iter::groupby(data.points, [](const auto& p)
    {
        float factor = std::pow(10.0f, 2);  // Potencia de 10 para mover el punto decimal
        return std::floor(p.phi * factor) / factor;  // Redondear y devolver con la cantidad deseada de decimales
    })) {
        auto min_r = std::min_element(std::begin(group), std::end(group),
            [](const auto& p1, const auto& p2) { return p1.r < p2.r; });
        salida.emplace_back(*min_r);
    }

    doors = door_detector.detect(salida, &viewer->scene, nominal_rooms);

    // Filtrar puntos fuera de la habitación usando el detector de puertas
    salida = door_detector.filter_points(salida, &viewer->scene);

    calculate_center(salida);

    return salida;
}

std::optional<rc::PointcloudCenterEstimator::Point2D> SpecificWorker::calculate_center(const RoboCompLidar3D::TPoints &data)
{
    // Crear el estimador
    rc::PointcloudCenterEstimator estimator;

    // Estimar el centro de la habitación a partir de las paredes
    center = estimator.estimate(data);

    return center;
}

// Dibujo de LiDAR
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &data)
{
    if (data.empty()) return;

    static std::vector<QGraphicsItem *> items;

    // Limpiar puntos antiguos
    for (auto i : items)
    {
        viewer->scene.removeItem(i);
        delete i;
    }
    items.clear();

    QBrush greenBrush(Qt::green);
    QPen greenPen(Qt::green);

    // Dibujar los puntos del LIDAR
    for (const auto &p : data)
    {
        auto item = viewer->scene.addRect(-50, -50, 100, 100, greenPen, greenBrush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }

    // Dibujar el centro
    if(center.has_value())
    {
        QBrush blueBrush(Qt::cyan);
        QPen bluePen(Qt::cyan);
        auto c = viewer->scene.addEllipse(-150, -150, 300, 300, bluePen, blueBrush);
        c->setPos(center.value().x(), center.value().y());
        items.push_back(c);
    }

    // Dibujar las puertas
    for (auto &d : doors)
    {
        QBrush blueBrush(Qt::cyan);
        QPen bluePen(Qt::cyan);
        auto c1 = viewer->scene.addRect(-50, -50, 100, 100, bluePen, blueBrush);
        auto c2 = viewer->scene.addRect(-50, -50, 100, 100, bluePen, blueBrush);
        c1->setPos(d.p1.x(), d.p1.y());
        c2->setPos(d.p2.x(), d.p2.y());

        items.push_back(c1);
        items.push_back(c2);

        bluePen.setWidth(20);
        items.push_back(viewer->scene.addLine(c1->x(), c1->y(), c2->x(), c2->y(), bluePen));
    }
}

// Máquina de estados y lógica de control
SpecificWorker::RetVal SpecificWorker::process_state()
{
    switch(state)
    {
        case STATE::GOTO_ROOM_CENTER:
        {
            return goto_room_center();
        }
        case STATE::TURN:
        {
            return turn();
        }
        case STATE::ORIENT_TO_DOOR:
        {
            return orient_to_door();
        }
        case STATE::GOTO_DOOR:
        {
            return goto_door();
        }
        case STATE::ORIENT_TO_DOOR_CENTER:
            {
                return orient_to_door_center();
            }
        case STATE::CROSS_DOOR:
        {
            return cross_door();
        }
    }
}

SpecificWorker::RetVal SpecificWorker::goto_room_center()
{
    if(center.has_value()) {
        auto dist = center.value().norm();

        // Si el robot está cerca del centro
        if (dist < 250.f)
            return {STATE::TURN,0.f, 0.f};

        Eigen::Vector2f center_float = center.value().cast<float>();

        auto [adv, rot] = robot_controller(center_float);

        return {STATE::GOTO_ROOM_CENTER, adv, rot};
    }else{
        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    }
}

SpecificWorker::RetVal SpecificWorker::turn()
{
    const auto &[success, turn] = image_processor.check_colour_patch_in_image(camera360rgb_proxy, color, label_img);
    if (success)
    {
        localised = true;
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};
    }

    return {STATE::TURN, 0.f, 1.0f};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door()
{
    static int counter = 0;
    if (counter < 50) {
        counter++;
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.4f};
    }if(doors.empty()) {
        counter = 0;
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.4f};
    }

    auto target = doors[0].center_before(robot_pose.translation());
    auto [_, w] = robot_controller(target);
    if (abs(w) < 0.03f) {
        counter = 0;
        return {STATE::GOTO_DOOR, 0.f, 0.f};
    }
    return {STATE::ORIENT_TO_DOOR, 0.f, w};
}

SpecificWorker::RetVal SpecificWorker::goto_door()
{
    if (doors.empty()) {
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.2f};
    }

    auto robot_position = robot_pose.translation();
    auto target = doors[0].center_before(robot_position);
    if (target.norm() < 150.f)
        return {STATE::ORIENT_TO_DOOR_CENTER, 0.f, 0.f};

    const auto &[adv, rot] = robot_controller(target);
    return {STATE::GOTO_DOOR, adv*0.4, rot};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door_center()
{
    auto target = doors[0].center();
    auto [_, w] = robot_controller(target);
    if (abs(w) < 0.03f) {
        return {STATE::CROSS_DOOR, 0.f, 0.f};
    }
    return {STATE::ORIENT_TO_DOOR_CENTER, 0.f, w};
}

SpecificWorker::RetVal SpecificWorker::cross_door()
{
    static int contador = 0;

    contador++;
    if (contador >= 60)
    {
        contador = 0;
        idHabitacion = (idHabitacion + 1) % 2;
        if (idHabitacion == 0)
        {
            color = "red";
        }
        else
        {
            color = "green";
        }
        viewer_room->scene.removeItem(habitacion);
        habitacion = viewer_room->scene.addRect(nominal_rooms[idHabitacion].rect(), QPen(Qt::black, 30));
        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    }

    return {STATE::CROSS_DOOR, 500.f, 0.f};
}

std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target)
{
    // rotation
    auto angle = atan2(target.x(), target.y());
    double k = 0.5f;
    double rot_vel = angle * k;

    // break rotation
    const double R = std::log(0.2) * 2 / -M_PI_4 * M_PI_4;
    auto break_rot = std::exp(-angle * angle * R);

    // advance
    double adv_vel = params.MAX_ADV_SPEED * break_rot;
    return {adv_vel, rot_vel};
    /*auto dist = target.norm();

    // Si el robot está cerca del punto objetivo
    if (dist < 150.f)
        return {0.f, 0.f};

    auto theta = std::atan2(target.x(), target.y());
    float rot = 0.25f * theta;
    float angle_break = exp((-theta * theta)/(M_PI/6.f));
    float adv = 1000.f * angle_break;

    return {adv, rot};*/
}

bool SpecificWorker::update_robot_pose(const Match &match)
{
    if (match.empty())
        return false;

    // Construir matrices para el cálculo de pose
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

    // Calcular transformación óptima
    const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;

    if (r.array().isNaN().any()) {
        qWarning() << "NaN values in pose calculation";
        return false;
    }

    // Actualizar pose del robot
    robot_pose.translate(Eigen::Vector2d(r(0), r(1)));
    robot_pose.rotate(r(2));

    return true;
}

void SpecificWorker::move_robot(float adv, float rot, float max_match_error)
{
    // Enviar velocidades al robot
    try {
        omnirobot_proxy->setSpeedBase(0.0f, adv, rot);
    } catch (const Ice::Exception &e) { std::cout << e << std::endl; }
}

void SpecificWorker::print_match(const Match &match, const float error) const {

}

// Auxiliares
std::expected<int, std::string> SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle) {
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

RoboCompLidar3D::TPoints SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d) {
    if (points.empty()) return {};

    const float d_squared = d * d;  // Avoid sqrt by comparing squared distances
    std::vector<bool> hasNeighbor(points.size(), false);

    // Create index std::vector for parallel iteration
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////

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

