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
#include <algorithm>
#include <ranges>
#include <Eigen/Dense>
#include <iostream>

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
// Inicialización del controlador
this->startup_check_flag = startup_check;
	if(this->startup_check_flag) {
		this->startup_check();
	} else {
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

void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data){}

SpecificWorker::~SpecificWorker() {
	std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize() {
    std::cout << "Initialize worker" << std::endl;
    if(this->startup_check_flag){
        this->startup_check();
    } else {
        nominal_rooms.push_back(NominalRoom{5500.f, 4000.f});
        nominal_rooms.push_back(NominalRoom{8000.f, 4000.f});
        std::cout << "DEBUG: nominal_rooms size: " << nominal_rooms.size() << std::endl;

        // Viewer para robot
        if (this->frame == nullptr) {
            std::cerr << "FATAL ERROR: this->frame es NULL. No se puede crear el viewer principal." << std::endl;
            return;
        }
        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_draw = r;

        // Viewer para la habitación
        if (this->frame_room == nullptr) {
            std::cerr << "FATAL ERROR: this->frame_room es NULL. No se puede crear el viewer de la habitación." << std::endl;
            return;
        }

        viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
        auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_room_draw = rr;

        // Dibuja la habitación nominal
        viewer_room->scene.addRect(nominal_rooms[0].rect(), QPen(Qt::black, 30));
        show();

        // Inicializa pose del robot
        robot_pose.setIdentity();
        robot_pose.translate(Eigen::Vector2d(0.0,0.0));

        if (this->frame_plot_error == nullptr) {
            std::cerr << "FATAL ERROR: frame_plot_error es NULL. No se puede crear el plotter." << std::endl;
            return;
        }

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
<<<<<<< HEAD
    auto data = read_data();

    draw_lidar(data);

    // Calcular esquinas y líneas de la habitación
    const auto &[corners, _] = room_detector.compute_corners(data, &viewer->scene);
=======

    RoboCompLidar3D::TPoints data = read_data();

    // Filtrar puntos fuera de la habitación usando el detector de puertas
    const auto &[filter, doorsDetect] = door_detector.filter_points(data, &viewer->scene);
    doors = doorsDetect;

    // Calcular esquinas y líneas de la habitación
    const auto &[corners, lines] = room_detector.compute_corners(data, &viewer->scene);

    // Estimar el centro de la habitación a partir de las paredes
    const auto center_opt = room_detector.estimate_center_from_walls(lines);
    if (!center_opt.has_value()) return;
    draw_lidar(data, center_opt.value(), &viewer->scene);

    // Convertir centro de Vector2d (double) a Vector2f (float) para compatibilidad
    std::optional<Eigen::Vector2f> center_opt_float;
    if (center_opt.has_value()) {
        center_opt_float = center_opt.value().cast<float>();
    }
>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438

    // Emparejar esquinas detectadas con esquinas nominales transformadas al marco del robot
    const auto match = hungarian.match(
        corners,
        nominal_rooms[0].transform_corners_to(robot_pose.inverse())
    );

    // Calcular error máximo de emparejamiento
    float max_match_error = 99999.f;
    if (!match.empty()) {
        const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
            { return std::get<2>(a) < std::get<2>(b); });
        max_match_error = static_cast<float>(std::get<2>(*max_error_iter));
<<<<<<< HEAD

        // Añadir punto de datos al plotter de errores
        time_series_plotter->addDataPoint(match_error_graph, max_match_error);
=======
        time_series_plotter->addDataPoint(0,max_match_error);
        //print_match(match, max_match_error);
>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438
    }

    // Actualizar pose del robot solo si está localizado
    if (localised)
        update_robot_pose(match);
<<<<<<< HEAD
    //update_robot_pose(match);

    // Procesar máquina de estados - pasar centro convertido
    auto [st, adv, rot] = process_state();
=======

    // Procesar máquina de estados - pasar centro convertido
    RetVal ret_val = process_state(filter, match, center_opt_float, corners, viewer);
    auto [st, adv, rot] = ret_val;
>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438
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

<<<<<<< HEAD
// Lectura de datos y filtrado
RoboCompLidar3D::TPoints SpecificWorker::read_data() {
    const auto data = lidar3d_proxy->getLidarDataWithThreshold2d(params.LIDAR_NAME_HIGH, 12000, 1);

    auto data_without_isolated_points = filter_isolated_points(data.points, 200);

    doors = door_detector.detect(data_without_isolated_points, &viewer->scene);

    // Filtrar puntos fuera de la habitación usando el detector de puertas
    data_without_isolated_points = door_detector.filter_points(data_without_isolated_points, &viewer->scene);

    center = calculate_center(data_without_isolated_points);

    return data_without_isolated_points;
}

std::optional<rc::PointcloudCenterEstimator::Point2D> SpecificWorker::calculate_center(const RoboCompLidar3D::TPoints &data)
=======
std::tuple<float, float> SpecificWorker::robot_controller(std::optional<Eigen::Vector2f> &target)
>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438
{
    // Crear configuración opcional
    rc::PointcloudCenterEstimator::Config cfg;

<<<<<<< HEAD
    // Crear el estimador
    rc::PointcloudCenterEstimator estimator(cfg);

    // Estimar el centro de la habitación a partir de las paredes
    center = estimator.estimate(data);
=======
    // 1. Calcular distancia al objetivo
    Eigen::Vector2f to_target = *target - robot_pos;
    float distance = to_target.norm();

    // Calcular error angular θe
    float target_angle = std::atan2(to_target.y(), to_target.x());
    float angle_error = target_angle - robot_angle;
>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438

    return center;
}

// Dibujo de LiDAR
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &data)
{
    if (data.empty()) return;

<<<<<<< HEAD
    static std::vector<QGraphicsItem *> items;
=======
    if (dt < 0.001f) dt = 0.001f; // Evitar división por cero en el primer frame
>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438

    // Limpiar puntos antiguos
    for (auto i : items)
    {
        viewer->scene.removeItem(i);
        delete i;
    }
    items.clear();

<<<<<<< HEAD
    QBrush greenBrush(Qt::green);
    QPen greenPen(Qt::green);

    // Dibujar los puntos del LIDAR
    for (const auto &p : data)
    {
        auto item = viewer->scene.addRect(-50, -50, 100, 100, greenPen, greenBrush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }

    if(center.has_value())
    {
        QBrush blueBrush(Qt::cyan);
        QPen bluePen(Qt::cyan);
        auto c = viewer->scene.addEllipse(-150, -150, 300, 300, bluePen, blueBrush);
        c->setPos(center.value().x(), center.value().y());
        items.push_back(c);
    }

    // Sección frontal
    auto offset_begin = closest_lidar_index_to_given_angle(data, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(data, params.LIDAR_FRONT_SECTION);
    if (!offset_begin || !offset_end) return;

    int ob = std::clamp(offset_begin.value(), 0, static_cast<int>(data.size()) - 1);
    int oe = std::clamp(offset_end.value(), 0, static_cast<int>(data.size()) - 1);
    if (ob > oe) std::swap(ob, oe);

    auto min_point = std::min_element(data.begin() + ob,
                                      data.begin() + oe + 1,
                                      [](const auto &a, const auto &b) { return a.r < b.r; });
    if (min_point == data.end()) return;

    QColor dcolor = (min_point->r < 400) ? Qt::red : Qt::magenta;
    auto ditem = viewer->scene.addRect(-100, -100, 200, 200, dcolor, QBrush(dcolor));
    ditem->setPos(min_point->x, min_point->y);
    items.push_back(ditem);

    // Puntos laterales
    auto wall_right = closest_lidar_index_to_given_angle(data, params.LIDAR_RIGHT_SIDE_SECTION);
    auto wall_left = closest_lidar_index_to_given_angle(data, -params.LIDAR_RIGHT_SIDE_SECTION);
    if (!wall_right || !wall_left) return;

    auto right_point = data[wall_right.value()];
    auto left_point = data[wall_left.value()];
    auto min_obj = (right_point.r < left_point.r) ? right_point : left_point;

    auto item_obj = viewer->scene.addRect(-100, -100, 200, 200,
                                   QColorConstants::Svg::orange,
                                   QBrush(QColorConstants::Svg::orange));
    item_obj->setPos(min_obj.x, min_obj.y);
    items.push_back(item_obj);

    auto item_line = viewer->scene.addLine(QLineF(QPointF(0.f, 0.f), QPointF(min_obj.x, min_obj.y)),
                                    QPen(QColorConstants::Svg::orange, 10));
    items.push_back(item_line);

    // Líneas frontales
    /*auto res_right = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_FRONT_SECTION);
    auto res_left = closest_lidar_index_to_given_angle(filtered_points, -params.LIDAR_FRONT_SECTION);
    if (!res_right || !res_left) return;

    float right_length = filtered_points[res_right.value()].r;
    float left_length = filtered_points[res_left.value()].r;
    float angle1 = filtered_points[res_left.value()].phi;
    float angle2 = filtered_points[res_right.value()].phi;

    QLineF line_left{QPointF(0.f, 0.f),
                     robot_draw->mapToScene(left_length * sin(angle1), left_length * cos(angle1))};
    QLineF line_right{QPointF(0.f, 0.f),
                      robot_draw->mapToScene(right_length * sin(angle2), right_length * cos(angle2))};

    auto line1 = scene->addLine(line_left, QPen(Qt::blue, 10));
    auto line2 = scene->addLine(line_right, QPen(Qt::red, 10));
    items.push_back(line1);
    items.push_back(line2);*/
}

// Máquina de estados y lógica de control
SpecificWorker::RetVal SpecificWorker::process_state() {
=======
    // Aplicar control PD
    float rot_speed = controller_params.Kp_angular * angle_error +
                    controller_params.Kd_angular * angle_error_derivative;
    rot_speed = std::clamp(rot_speed, -1.0f, 1.0f);

    // Frenado angular gaussiano: f(θe) = exp(-θe²/(2σ²))
    float angle_brake = std::exp(-(angle_error * angle_error) /
                                (2 * controller_params.sigma_theta * controller_params.sigma_theta));

    // Frenado de distancia sigmoide: fd(d) = 1 / (1 + exp(k(d - d_stop)))
    float distance_brake = 1.0f / (1.0f + std::exp(controller_params.k_steepness *
                                                  (distance - controller_params.d_stop)));

    float adv_speed = controller_params.v_max * angle_brake * distance_brake;
    adv_speed = std::clamp(adv_speed, 0.0f, controller_params.v_max);

    return std::make_tuple(adv_speed, rot_speed);
}

// Máquina de estados y lógica de control
SpecificWorker::RetVal SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data,
                                                     const Match &match,
                                                     std::optional<Eigen::Vector2f> &center_opt,
                                                     const Corners &corners,
                                                     AbstractGraphicViewer *viewer) {
>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438
    float adv_speed = 0.0f;
    float rot_speed = 0.0f;
    STATE next_state = state;

<<<<<<< HEAD
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
        case STATE::GOTO_DOOR:
        {
            return goto_door();
        }
        case STATE::ORIENT_TO_DOOR:
        {
            return orient_to_door();
        }
        case STATE::CROSS_DOOR:
        {
            return cross_door();
        }
        case STATE::LOCALISE:
        {
            //std::tie(next_state, adv_speed, rot_speed) = localise(match, data);
            break;
        }
        case STATE::UPDATE_POSE:
        {
            //std::tie(next_state, adv_speed, rot_speed) = update_pose(match);
            break;
        }
        case STATE::SEARCH_DOORS:
        {
            //std::tie(next_state, adv_speed, rot_speed) = search_doors(data);
            break;
        }
        case STATE::IDLE:  // Inactivo
        default:
        {
            adv_speed = 0.f;
            rot_speed = 0.f;
            next_state = STATE::GOTO_ROOM_CENTER;
            break;
        }
=======
    switch(state) {
    case STATE::GOTO_ROOM_CENTER:
        std::tie(next_state, adv_speed, rot_speed) = goto_room_center(center_opt);
        break;
    case STATE::TURN:
        std::tie(next_state, adv_speed, rot_speed) = turn(corners);
        break;
    case STATE::GOTO_DOOR:
        std::tie(state, adv_speed, rot_speed) = goto_door();
        break;
    case STATE::ORIENT_TO_DOOR:
        std::tie(state, adv_speed, rot_speed) = orient_to_door();
        break;
    case STATE::CROSS_DOOR:
        std::tie(state, adv_speed, rot_speed) = cross_door(data);
        break;
    case STATE::IDLE:  // Inactivo
    default:
        adv_speed = 0.f;
        rot_speed = 0.f;
        next_state = STATE::IDLE;
        break;
>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438
    }

    qDebug() << "State transition:" << to_string(state) << "->" << to_string(next_state);
    return {next_state, adv_speed, rot_speed};
}

<<<<<<< HEAD
SpecificWorker::RetVal SpecificWorker::goto_room_center()
{
    if(center.has_value()) {
        auto dist = center.value().norm();

        // Si el robot está cerca del centro
        if (dist < 100.f)
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
        return {STATE::GOTO_DOOR, 0.f, 0.f};
    }

    return {STATE::TURN, 0.f, 0.4f};
}

SpecificWorker::RetVal SpecificWorker::goto_door()
{
    if (doors.empty()) {
        qInfo() << "No hay puerta";
        return {};
    }
    auto rp = robot_pose.translation();
    auto target = doors[0].center_before(rp, 1000.f);
    if ( target.norm() < 500.f )
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};

    const auto &[adv, rot] = robot_controller(target);
    return {STATE::GOTO_DOOR, adv*0.4, rot};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door()
{
    static int contador = 0;
    if (doors.empty()) {
        qInfo() << "No hay puerta";
        return {};
    }

    auto rp = robot_pose.translation();
    auto u = doors[0].center();
    auto v = doors[0].p2 - doors[0].p1;

    float dot = u.dot(v);
    float norm_u = u.norm();
    float norm_v = v.norm();

    float cos_angle = dot / (norm_u * norm_v);

    cos_angle = std::clamp(cos_angle, -1.0f, 1.0f);

    auto angle = std::acos(cos_angle);

    if (qRadiansToDegrees(angle) > 70 and qRadiansToDegrees(angle) < 110)
    {
        auto [_, w] = robot_controller(u);
        qInfo() << "rotación:" << w;
        if (w < 0.04f)
            return {STATE::CROSS_DOOR, 0.f, 0.f};
        return {STATE::ORIENT_TO_DOOR, 0.f, w};
    }


    float signo = (angle > 0.f) ? 1.f : -1.f;
    return {STATE::ORIENT_TO_DOOR, 0.f, 0.3f * signo};
}

SpecificWorker::RetVal SpecificWorker::cross_door()
{
    static int contador = 0;

    contador++;
    if (contador == 50)
    {
        contador = 0;
        idHabitacion = (idHabitacion + 1) % 2;
        if (idHabitacion == 0)
        {
            color = Qt::red;
        }
        else
        {
            color = "green";
        }
        viewer_room->scene.addRect(nominal_rooms[idHabitacion].rect(), QPen(Qt::black, 30));
        show();
        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    }

    return {STATE::CROSS_DOOR, 500.f, 0.f};
}

std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target)
{
    auto dist = target.norm();

    // Si el robot está cerca del punto objetivo
    if (dist < 100.f)
        return {0.f, 0.f};

    auto theta = std::atan2(target.x(), target.y());
    float rot = 0.5f * theta;
    float angle_break = exp((-theta * theta)/(M_PI/6.f));
    float adv = 1000.f * angle_break;

    return {adv, rot};
}

// 🟩 ESTADO UPDATE_POSE - Simple según diagrama
SpecificWorker::RetVal SpecificWorker::update_pose(const Match &match) {
    float adv_speed = 0.0f;
    float rot_speed = 0.0f;
    STATE next_state = STATE::UPDATE_POSE;

    // POSE UPDATED → Siempre volver a LOCALISE (según diagrama)
    if (update_robot_pose(match)) {
        qInfo() << "Pose updated successfully → LOCALISE";
=======
SpecificWorker::RetVal SpecificWorker::goto_room_center(std::optional<Eigen::Vector2f> &center_opt) {
    auto toCenter = center_opt.value().norm();
    if (toCenter < 300.f)
        return {STATE::TURN,0.f, 0.f};

    auto [adv_speed, rot_speed] = robot_controller(center_opt);
    return {STATE::GOTO_ROOM_CENTER, adv_speed, rot_speed};
}

SpecificWorker::RetVal SpecificWorker::turn(const Corners &corners) {
    QColor color1 = QColor(Qt::red);
    QColor color2 = QColor(Qt::green);
    const auto &[success, turn] = image_processor.check_colour_patch_in_image(camera360rgb_proxy, color2 , label_img);
    if (success) {
>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438
        localised = true;
        return {STATE::GOTO_DOOR, 0.f, 0.f};
    }

    // do my thing
    return {STATE::TURN, 0.f, 0.3f};
}

SpecificWorker::RetVal SpecificWorker::update_pose(const Match &match) {}

bool SpecificWorker::update_robot_pose(const Match &match) {
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

    qDebug() << "Robot pose updated - x:" << robot_pose.translation().x()
             << "y:" << robot_pose.translation().y()
             << "angle:" << r(2);

    return true;
}

<<<<<<< HEAD
void SpecificWorker::move_robot(float adv, float rot, float max_match_error)
{
=======
SpecificWorker::RetVal SpecificWorker::goto_door() {

    if (doors.empty()) return {};
    Eigen::Vector2d robot_position(robot_pose.translation().x(), robot_pose.translation().y());
    auto target_vector = doors[0].center_before(robot_position, 1000.f);
    if (target_vector.norm() < 300.f)
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};

    std::optional<Eigen::Vector2f> target_optional = target_vector;
    const auto &[adv_speed, rot_speed] = robot_controller(target_optional);
    return{STATE::GOTO_DOOR, adv_speed, rot_speed};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door() {
    if (doors.empty()) return {};

    // Usamos p1 y p2 para definir los extremos de la puerta
    const Eigen::Vector2f& p1 = doors[0].p1.cast<float>();
    const Eigen::Vector2f& p2 = doors[0].p2.cast<float>();

    float current_robot_angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));

    // Vector a lo largo de la puerta (dirección de la puerta)
    Eigen::Vector2f door_vector = p2 - p1;

    // Vector Normal: Rotamos el vector 'door_vector' 90 grados.
    // [x, y] rotado 90° en sentido antihorario es [-y, x].
    // Esto apunta a uno de los dos lados de la puerta.
    Eigen::Vector2f normal_vector(-door_vector.y(), door_vector.x());

    // El ángulo deseado es la orientación de ese vector normal.
    float desired_angle = std::atan2(normal_vector.y(), normal_vector.x());

    // Error Angular
    float angle_error = desired_angle - current_robot_angle;

    // Normalizar el error al rango [-π, π] para el giro más corto.
    if (angle_error > M_PI) angle_error -= 2 * M_PI;
    if (angle_error < -M_PI) angle_error += 2 * M_PI;

    if (std::abs(angle_error) < 0.15f)
        return {STATE::CROSS_DOOR, 0.f, 0.f}; // Cambiar de estado y parar el movimiento.

    // PID (Solo proporcionaL)
    const float KP_ROTATION = 0.8f;
    float rot_vel = KP_ROTATION * angle_error;

    rot_vel = std::clamp(rot_vel, -1.0f, 1.0f);
    return {STATE::ORIENT_TO_DOOR, 0.f, rot_vel};
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points) {

    // Bandera para detectar la primera entrada
    static bool first_entry = true;

    // Almacenar el tiempo inicial
    static std::chrono::steady_clock::time_point start_time;

    if (first_entry) { // Se ejecuta SOLO la primera vez
        start_time = std::chrono::steady_clock::now();
        first_entry = false;

        const float ADV_VEL = 0.3f; // Velocidad de avance (ajustable)
        return {STATE::CROSS_DOOR, ADV_VEL, 0.f};
    }

    // Tiempo Transcurrido
    auto current_time = std::chrono::steady_clock::now();
    float elapsed_seconds = std::chrono::duration<float>(current_time - start_time).count();

    const float TIME_TO_CROSS = 2.0f; // Tiempo requerido (2 segundos)

    if (elapsed_seconds < TIME_TO_CROSS) {// Si aún no han pasado 2 segundos, seguir avanzando
        const float ADV_VEL = 0.3f;
        return {STATE::CROSS_DOOR, ADV_VEL, 0.f}; // Mantener el estado y avanzar
    }

    first_entry = true;
    return {STATE::IDLE, 0.f, 0.f};
}

void SpecificWorker::move_robot(float adv, float rot, float max_match_error) {
>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438
    // Enviar velocidades al robot
    try {
        omnirobot_proxy->setSpeedBase(0.0f, adv, rot);
    } catch (const Ice::Exception &e) { std::cout << e << std::endl; }
}

void SpecificWorker::print_match(const Match &match, const float error) const {}

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

<<<<<<< HEAD
=======
// Dibujo de LiDAR
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &filtered_points, std::optional<Eigen::Vector2d> center, QGraphicsScene *scene) {
    if (filtered_points.empty()) return;

    static std::vector<QGraphicsItem *> items;

    // Limpiar puntos antiguos
    for (auto i : items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    QBrush greenBrush(Qt::green);
    QPen greenPen(Qt::green);

    // Dibujar los puntos del LIDAR
    for (const auto &p : filtered_points)
    {
        auto item = scene->addRect(-50, -50, 100, 100, greenPen, greenBrush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }

    // Si hay centro definido, lo dibujamos
    if (center.has_value())
    {
        QBrush blueBrush(Qt::cyan);
        QPen bluePen(Qt::cyan);
        auto c = scene->addEllipse(-150, -150, 300, 300, bluePen, blueBrush);
        c->setPos(center->x(), center->y());
        items.push_back(c);
    }

    // Sección frontal
    auto offset_begin = closest_lidar_index_to_given_angle(filtered_points, -params.LIDAR_FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_FRONT_SECTION);
    if (!offset_begin || !offset_end) return;

    int ob = std::clamp(offset_begin.value(), 0, static_cast<int>(filtered_points.size()) - 1);
    int oe = std::clamp(offset_end.value(), 0, static_cast<int>(filtered_points.size()) - 1);
    if (ob > oe) std::swap(ob, oe);

    auto min_point = std::min_element(filtered_points.begin() + ob,
                                      filtered_points.begin() + oe + 1,
                                      [](const auto &a, const auto &b) { return a.r < b.r; });
    if (min_point == filtered_points.end()) return;

    QColor dcolor = (min_point->r < 400) ? Qt::red : Qt::magenta;
    auto ditem = scene->addRect(-100, -100, 200, 200, dcolor, QBrush(dcolor));
    ditem->setPos(min_point->x, min_point->y);
    items.push_back(ditem);

    // Puntos laterales
    auto wall_right = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_RIGHT_SIDE_SECTION);
    auto wall_left = closest_lidar_index_to_given_angle(filtered_points, -params.LIDAR_RIGHT_SIDE_SECTION);
    if (!wall_right || !wall_left) return;

    auto right_point = filtered_points[wall_right.value()];
    auto left_point = filtered_points[wall_left.value()];
    auto min_obj = (right_point.r < left_point.r) ? right_point : left_point;

    auto item_obj = scene->addRect(-100, -100, 200, 200,
                                   QColorConstants::Svg::orange,
                                   QBrush(QColorConstants::Svg::orange));
    item_obj->setPos(min_obj.x, min_obj.y);
    items.push_back(item_obj);

    auto item_line = scene->addLine(QLineF(QPointF(0.f, 0.f), QPointF(min_obj.x, min_obj.y)),
                                    QPen(QColorConstants::Svg::orange, 10));
    items.push_back(item_line);

    // Líneas frontales
    auto res_right = closest_lidar_index_to_given_angle(filtered_points, params.LIDAR_FRONT_SECTION);
    auto res_left = closest_lidar_index_to_given_angle(filtered_points, -params.LIDAR_FRONT_SECTION);
    if (!res_right || !res_left) return;

    float right_length = filtered_points[res_right.value()].r;
    float left_length = filtered_points[res_left.value()].r;
    float angle1 = filtered_points[res_left.value()].phi;
    float angle2 = filtered_points[res_right.value()].phi;

    /*QLineF line_left{QPointF(0.f, 0.f),
                     robot_polygon->mapToScene(left_length * sin(angle1), left_length * cos(angle1))};
    QLineF line_right{QPointF(0.f, 0.f),
                      robot_polygon->mapToScene(right_length * sin(angle2), right_length * cos(angle2))};*/

    /*auto line1 = scene->addLine(line_left, QPen(Qt::blue, 10));
    auto line2 = scene->addLine(line_right, QPen(Qt::red, 10));
    items.push_back(line1);
    items.push_back(line2);*/
}

// Lectura de datos y filtrado
RoboCompLidar3D::TPoints SpecificWorker::read_data() {
    RoboCompLidar3D::TData data;
    RoboCompLidar3D::TPoints filter_data;
    try {
        data = lidar3d_proxy->getLidarDataWithThreshold2d(params.LIDAR_NAME_HIGH, 12000, 1);
        filter_data = filter_same_phi(data.points);
        if (filter_data.empty()) return {};

    } catch (const Ice::Exception &e) {
        std::cout << e << " " << "Conexión con Laser" << std::endl; return{};
    }

    return filter_isolated_points(filter_data, 200);;
}

RoboCompLidar3D::TPoints SpecificWorker::filter_same_phi(const RoboCompLidar3D::TPoints &points) {

    if (points.empty()) return{};
    std::map<float, RoboCompLidar3D::TPoint> filtered_map;

    for (const auto& point : points)
    {
        float angle = point.phi; // El ángulo del punto actual
        float distance = point.r; // La distancia del punto actual

        // Comprobar si ya existe un punto para este ángulo
        auto it = filtered_map.find(angle);
        if (it == filtered_map.end()) {
            filtered_map[angle] = point;
        } else {
            // El ángulo ya existe: Comprobar si la distancia actual es menor, y es así
            // reemplazamos el punto
            if (distance < it->second.r)
                it->second = point;
        }
    }

    RoboCompLidar3D::TPoints result;
    for (const auto& pair : filtered_map)
        result.push_back(pair.second);
    return result;
}

>>>>>>> c51fa747c0c13b5c7f071906e525a1a8b24bf438
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
