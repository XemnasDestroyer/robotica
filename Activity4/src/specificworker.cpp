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

using namespace iter;

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) 
    : GenericWorker(configLoader, tprx)
{
    this->startup_check_flag = startup_check;
    if(this->startup_check_flag) {
        this->startup_check();
    } else {
        #ifdef HIBERNATION_ENABLED
            hibernationChecker.start(500);
        #endif

        statemachine.setChildMode(QState::ExclusiveStates);
        statemachine.start();

        auto error = statemachine.errorString();
        if (error.length() > 0){
            qWarning() << error;
            throw error;
        }
    }
}

SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data) {
    // TODO: implement joystick control if needed
}

void SpecificWorker::initialize() {
    std::cout << "Initialize worker" << std::endl;
    if(this->startup_check_flag) {
        this->startup_check();
    } else {
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
        robot_pose.translate(Eigen::Vector2f(0.0,0.0));

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
    // 1. Lectura de datos del LIDAR y filtrado de puertas
    RoboCompLidar3D::TPoints data = read_data();
    data = door_detector.filter_points(data);
    draw_local_doors(&viewer->scene);

    // 2. Cálculo de esquinas y centro de la habitación
    const auto &[corners, lines] = room_detector.compute_corners(data, &viewer->scene);
    const auto center_opt = center_estimator.estimate(data);
    draw_lidar(data, center_opt, &viewer->scene);

    // 3. Inicialización de error máximo y match
    float max_match_error = 99999.f;
    Match match;
    if (localised)
    {
        // match corners transformando esquinas nominales al frame del robot
        match = hungarian.match(corners, nominal_rooms[current_room].transform_corners_to(robot_pose.inverse()));

        // calcular máximo error del match
        if (!match.empty())
        {
            const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
                { return std::get<2>(a) < std::get<2>(b); });
            max_match_error = static_cast<float>(std::get<2>(*max_error_iter));
            time_series_plotter->addDataPoint(match_error_graph, max_match_error);
            //print_match(match, max_match_error); //debugging
        }
        update_robot_pose(current_room, corners, robot_pose, true);
    }

    // 4. Procesar la máquina de estados
    RetVal ret_val = process_state(data, corners, match, &viewer->scene, &viewer_room->scene);
    auto [st, adv, rot] = ret_val;
    state = st;

    // 5. Enviar comandos al robot
    move_robot(adv, rot, max_match_error);

    // 6. Dibujar robot en la habitación
    robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
    const double angle = qRadiansToDegrees(std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0)));
    robot_room_draw->setRotation(angle);

    // 7. Actualizar GUI
    time_series_plotter->update();
    lcdNumber_adv->display(adv);
    lcdNumber_rot->display(rot);
    lcdNumber_x->display(robot_pose.translation().x());
    lcdNumber_y->display(robot_pose.translation().y());
    lcdNumber_room->display(current_room);
    lcdNumber_angle->display(angle);
    last_time = std::chrono::high_resolution_clock::now();

	draw_graph(&viewer_room->scene);
}

// Lectura de datos y filtrado
RoboCompLidar3D::TPoints SpecificWorker::read_data() {
    const auto data = lidar3d_proxy->getLidarDataWithThreshold2d(params.LIDAR_NAME_HIGH, 12000, 2);

    RoboCompLidar3D::TPoints salida; salida.reserve(data.points.size());
    for (auto&& [angle, group] : iter::groupby(data.points, [](const auto& p) {
        float factor = std::pow(10.0f, 2);
        return std::floor(p.phi * factor) / factor;
    })) {
        auto min_r = std::min_element(std::begin(group), std::end(group),
            [](const auto& p1, const auto &p2){ return p1.r < p2.r; });
        salida.emplace_back(*min_r);
    }

    doors = door_detector.detect(salida, nominal_rooms[idHabitacion]);
    salida = door_detector.filter_points(salida, nominal_rooms[idHabitacion]);
    calculate_center(salida);

    return salida;
}

void SpecificWorker::calculate_center(const RoboCompLidar3D::TPoints &data) {
    rc::PointcloudCenterEstimator estimator;
    center = estimator.estimate(data);
}

// Dibujo de LiDAR
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &data) {
    if (data.empty()) return;

    static std::vector<QGraphicsItem *> items;
    static std::vector<QGraphicsItem *> items2;

    for (auto i : items) {
        viewer->scene.removeItem(i);
        delete i;
    }
    items.clear();

    for (auto i : items2) {
        viewer_room->scene.removeItem(i);
        delete i;
    }
    items2.clear();

    QBrush greenBrush(Qt::green);
    QPen greenPen(Qt::green);

    for (const auto &p : data) {
        auto item = viewer->scene.addRect(-50, -50, 100, 100, greenPen, greenBrush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }

    if(center.has_value()) {
        QBrush blueBrush(Qt::cyan);
        QPen bluePen(Qt::cyan);
        auto c = viewer->scene.addEllipse(-150, -150, 300, 300, bluePen, blueBrush);
        c->setPos(center.value().x(), center.value().y());
        items.push_back(c);
    }

    for (auto &d : doors) {
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
SpecificWorker::RetVal SpecificWorker::process_state() {
    switch(state) {
        case STATE::GOTO_ROOM_CENTER:
            return goto_room_center();
        case STATE::TURN:
            return turn();
        case STATE::ORIENT_TO_DOOR:
            return orient_to_door();
        case STATE::GOTO_DOOR:
            return goto_door();
        case STATE::ORIENT_TO_DOOR_CENTER:
            return orient_to_door_center();
        case STATE::CROSS_DOOR:
            return cross_door();
        default:
            qWarning() << __FUNCTION__ << "Unknown state, stopping robot.";
            return {state, 0.f, 0.f};
    }
}

SpecificWorker::RetVal SpecificWorker::localise(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene) {
    // Inicializa pose del robot
    robot_pose.setIdentity();
    robot_pose.translate(Eigen::Vector2f(0.0, 0.0));
    localised = false;

    // Estima el centro de la habitación con el centro de paredes
    if (const auto center_est = center_estimator.estimate(points); center_est.has_value()) {
        // Si no está lo suficientemente centrado, mover al centro
        if (center_est.value().norm() > params.RELOCAL_CENTER_EPS)
            return {STATE::GOTO_ROOM_CENTER, 0.0f, 0.0f};

        // Si está centrado, pasar a girar para relocalizar
        if (center_est.value().norm() < params.RELOCAL_CENTER_EPS)
            return {STATE::TURN, 0.0f, 0.0f};
    }

    qWarning() << __FUNCTION__ << "No se pudo estimar centro, continuar localizando.";
    return {STATE::LOCALISE, 0.0f, 0.0f};
}

SpecificWorker::RetVal SpecificWorker::goto_room_center() {
    // Verifica si el centro de la habitación ha sido estimado
    if (center.has_value()) {
        float dist = center.value().norm(); // distancia al centro

        // Si el robot está cerca del centro, pasar al siguiente estado
        if (dist < 250.f)
            return {STATE::TURN, 0.f, 0.f};

        Eigen::Vector2f center_float = center.value().cast<float>();
        auto [adv, rot] = robot_controller(center_float);

        // Si estamos entrando a una puerta, trackearlo
        door_crossing.track_entering_door(door_detector.doors());
        return {STATE::GOTO_ROOM_CENTER, adv, rot};
    } else {
        // Si no se pudo estimar el centro, detener robot
        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
    }
}


SpecificWorker::RetVal SpecificWorker::turn(const Corners &corners){
    const auto &[success, room_index, left_right] = image_processor.check_colour_patch_in_image(camera360rgb_proxy, label_img);

    if (success) {
        // Actualizamos la habitación actual
        current_room = room_index;
		graph.add_node(Graph::NodeType::ROOM, current_room);
        // Match de esquinas con la habitación nominal
        const auto m = hungarian.match(corners, nominal_rooms[current_room].corners());
        if (m.empty() || m.size() < 3) {
            qInfo() << __FUNCTION__ << "Match insuficiente";
            return {STATE::TURN, 0.0f, left_right * params.RELOCAL_ROT_SPEED};
        }

        const auto max_error_iter = std::ranges::max_element(m, [](const auto &a, const auto &b)
            { return std::get<2>(a) < std::get<2>(b); });
        if (const auto max_match_error = std::get<2>(*max_error_iter);
            max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR)
        {
            qInfo() << __FUNCTION__ << "Error de match demasiado alto: " << max_match_error;
            return {STATE::TURN, 0.0f, left_right * params.RELOCAL_ROT_SPEED};
        }

        // Actualizamos la pose del robot
        update_robot_pose(current_room, corners, robot_pose, false);

        // Guardar puertas detectadas en la habitación nominal
        auto doors = door_detector.doors();
        if (doors.empty()) { 
            qWarning() << __FUNCTION__ << "No hay puertas detectadas"; 
            return {STATE::TURN, 0.0f, left_right * params.RELOCAL_ROT_SPEED};
        }

        for (auto &d : doors) {
            d.p1_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose * d.p1);
            d.p2_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose * d.p2);
        }	
        nominal_rooms[current_room].doors = doors;
		for (int i = 0; i < nominal_rooms[current_room].doors.size(); ++i) {
    		graph.add_node(Graph::NodeType::DOOR, i);
    		graph.add_edge({Graph::NodeType::ROOM, current_room}, {Graph::NodeType::DOOR, i});
		}


        // Elegimos la puerta a seguir
        current_door = 0; // TODO: elección más sofisticada
        const auto dn = nominal_rooms[current_room].doors[current_door];
        const auto ds = door_detector.doors();
        const auto sd = std::ranges::min_element(ds, [dn, this](const auto &a, const auto &b)
        { return (a.center() - robot_pose.inverse() * dn.center_global()).norm() <
                 (b.center() - robot_pose.inverse() * dn.center_global()).norm(); });

        nominal_rooms[current_room].doors[current_door].p1 = sd->p1;
        nominal_rooms[current_room].doors[current_door].p2 = sd->p2;

        draw_nominal_room(current_room, &viewer_room->scene);
        draw_nominal_doors(current_room, current_door, &viewer_room->scene);

        localised = true;
        return {STATE::GOTO_DOOR, 0.0f, 0.0f};
    }

    // Si no se detecta la habitación, seguimos girando
    return {STATE::TURN, 0.0f, left_right * params.RELOCAL_ROT_SPEED};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door()
{
   const auto doors = door_detector.doors();
   if (localised){
       const auto dn = nominal_rooms[current_room].doors[current_door];
       const auto sd = std::ranges::min_element(doors, [dn, this](const auto &a, const auto &b)
           {  return (a.center() - robot_pose.inverse() * dn.center_global()).norm() <
                     (b.center() - robot_pose.inverse() * dn.center_global()).norm(); });
       //qInfo() << __FUNCTION__ << "Localised, selecting door closest to nominal door" << sd->center_angle() << params.RELOCAL_MAX_ORIENTED_ERROR << doors.size();
       if ( abs(sd->center_angle()) < params.RELOCAL_MAX_ORIENTED_ERROR)
           return {STATE::CROSS_DOOR, 0.1, 0.f};
       else
           return {STATE::ORIENT_TO_DOOR, 0.f, std::get<1>(robot_controller(sd->center()))};
   } else { // select the one closest to the robot's heading direction
	   qInfo() << __FUNCTION__ << "Not localised, selecting door closest to robot heading";
       const auto sd = std::ranges::min_element(doors, [](const auto &a, const auto &b)
              {  return std::fabs(a.center_angle()) < std::fabs(b.center_angle());} );
       if (abs(sd->center_angle()) < params.RELOCAL_MAX_ORIENTED_ERROR)
           return {STATE::CROSS_DOOR, 0.5f, 0.f};
       else
           return {STATE::ORIENT_TO_DOOR, 0.f, std::get<1>(robot_controller(sd->center()))};
   }
}

SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene){
    auto doors = door_detector.doors();
    if (doors.empty()) {
        qInfo() << __FUNCTION__ << "No se detectan puertas, esperar";
        return {STATE::GOTO_DOOR, 0.f, 0.f};
    }

    Door target_door;
    if (localised) {
        const auto dn = nominal_rooms[current_room].doors[current_door];
        const auto sd = std::ranges::min_element(doors, [dn, this](const auto &a, const auto &b)
        { return (a.center() - robot_pose.inverse() * dn.center_global()).norm() <
                 (b.center() - robot_pose.inverse() * dn.center_global()).norm(); });
        target_door = *sd;
    } else {
        const auto sd = std::ranges::min_element(doors, [](const auto &a, const auto &b)
        { return std::fabs(a.p1_angle) < std::fabs(b.p1_angle); });
        target_door = *sd;
    }

    constexpr float offset = 600.f;
    const auto target = target_door.center_before(robot_pose.translation(), offset);
    const auto dist_to_door = target.norm();

    // Dibujar objetivo
    static QGraphicsItem *door_target_draw = nullptr;
    if (door_target_draw) scene->removeItem(door_target_draw);
    door_target_draw = scene->addEllipse(-50, -50, 100, 100, QPen(Qt::magenta), QBrush(Qt::magenta));
    door_target_draw->setPos(target.x(), target.y());

    if (dist_to_door < params.DOOR_REACHED_DIST)
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};

    const auto &[adv, rot] = robot_controller(target);
    return {STATE::GOTO_DOOR, adv, rot};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door(const float &max_match_error) {
    static std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
    static bool first_time = true;
    if (first_time) {first_time = false; last_time = std::chrono::high_resolution_clock::now();}

    if (max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR) {
        if (first_time) {last_time = std::chrono::high_resolution_clock::now(); first_time = false;}

        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
        if (elapsed > 3000) {
            localised = false;
            return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
        }
    } else first_time = true;

    const auto doors = door_detector.doors();
    if (localised) {
        const auto nominal_door = nominal_rooms[current_room].doors[current_door];
        // qInfo() << __FUNCTION__  << "Current door before: (" << nominal_door.p1.x() << nominal_door.p1.y() << ") (" << nominal_door.p2.x() << nominal_door.p2.y() << ")";
        const auto selected_door = std::ranges::min_element(doors, [nominal_door, this](const auto &a, const auto &b) {
            return (a.center() - robot_pose.inverse() * nominal_door.center_global()).norm() < (b.center() - robot_pose.inverse() * nominal_door.center_global()).norm(); });
        // static Door target_door = *selected_door;
        // qInfo() << __FUNCTION__ << "Current door after: (" << target_door.p1.x() << target_door.p1.y() << ") (" << target_door.p2.x() << target_door.p2.y() << ")";
        draw_target(Eigen::Vector2d(selected_door->center().x(), selected_door->center().y()), &viewer->scene, true);
        qInfo() << __FUNCTION__ << abs(selected_door->center_angle()) << params.RELOCAL_MAX_ORIENTED_ERROR;
        if (abs(selected_door->center_angle()) < params.RELOCAL_MAX_ORIENTED_ERROR)
            return {STATE::CROSS_DOOR, 0.7f, 0.f};
        else {
            const auto target = Eigen::Vector2d(selected_door->center().x(), selected_door->center().y());
            return {STATE::ORIENT_TO_DOOR, 0.f, std::get<1>(do_work(target))};
        }
    }
    else { // select the one closest to the robot's heading direction
        const auto selected_door = std::ranges::min_element(doors, [](const auto &a, const auto &b)
            {return std::fabs(a.center_angle()) < std::fabs(b.center_angle()); });
        draw_target(Eigen::Vector2d(selected_door->center().x(), selected_door->center().y()), &viewer->scene, true);
        if (abs(selected_door->center_angle()) < params.RELOCAL_MAX_ORIENTED_ERROR)
            return {STATE::CROSS_DOOR, 0.7f, 0.f};
        else {
            const auto target = Eigen::Vector2d(selected_door->center().x(), selected_door->center().y());
            return {STATE::ORIENT_TO_DOOR, 0.f, std::get<1>(do_work(target))};
        }
    }
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points){
    static bool first_time = true;
    static std::chrono::time_point<std::chrono::system_clock> start;

    if (first_time){
        first_time = false;
        start = std::chrono::high_resolution_clock::now();
        return {STATE::CROSS_DOOR, 500.f, 0.f};
    } else {
        const auto elapsed = std::chrono::high_resolution_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > 3000) {
            first_time = true;
            const auto &leaving_door = nominal_rooms[current_room].doors[current_door];
            int next_room_idx = leaving_door.connects_to_room;

            if (next_room_idx >= 0 && nominal_rooms[next_room_idx].visited) {
                // Entramos a una habitación conocida: relocalizamos pose
                int next_door_idx = leaving_door.connects_to_door;
                current_room = next_room_idx;
                current_door = next_door_idx;

                const auto &entering_door = nominal_rooms[current_room].doors[current_door];
                Eigen::Vector2f door_center = entering_door.center_global();
                door_center.y() -= 500; // colocar 500 mm dentro de la habitación

                robot_pose.setIdentity();
                robot_pose.translate(door_center);
                robot_pose.rotate(0);

                localised = true;
                return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
            } else {
                // Habitación nueva: track de puerta
                door_crossing = DoorCrossing{current_room, current_door};
				graph.add_node(Graph::NodeType::ROOM, next_room_idx);
				graph.add_node(Graph::NodeType::DOOR, next_door_idx);

				graph.add_edge({Graph::NodeType::ROOM, current_room}, {Graph::NodeType::DOOR, current_door});
				graph.add_edge({Graph::NodeType::DOOR, current_door}, {Graph::NodeType::ROOM, next_room_idx});

                nominal_rooms[current_room].doors[current_door].visited = true;
                return {STATE::LOCALISE, 0.f, 0.f};
            }
        } else {
            return {STATE::CROSS_DOOR, 500.f, 0.f};
        }
    }
}

std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target)
{
    // Ángulo al objetivo
    float angle = std::atan2(target.y(), target.x());

    // Ganancia proporcional
    float k = 0.5f;
    float rot_vel = angle * k;

    // Limitación de rotación con función exponencial
    float R = std::log(0.2) * 2 / (-M_PI_4 * M_PI_4);
    float break_rot = std::exp(-angle * angle * R);

    // Velocidad de avance limitada según rotación
    float adv_vel = params.MAX_ADV_SPEED * break_rot;
    return {adv_vel, rot_vel};
}

std::optional<std::pair<Eigen::Affine2f, float>> SpecificWorker::update_robot_pose(int room_index,
                                                        const Corners &corners,
                                                        const Eigen::Affine2f &r_pose,
                                                        bool transform_corners)
{
    // match corners  transforming first nominal corners to robot's frame
    Match match;
    if (transform_corners)
        match = hungarian.match(corners, nominal_rooms[room_index].transform_corners_to(r_pose.inverse()));
    else
        match = hungarian.match(corners, nominal_rooms[room_index].corners());

    if (match.empty() or match.size() < 3)
        return {};

    const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
      { return std::get<2>(a) < std::get<2>(b); });
    const auto max_match_error = std::get<2>(*max_error_iter);

    // create matrices W and b for pose estimation
    auto r = solve_pose(corners, match);
    if (r.array().isNaN().any())
    {
        qWarning() << __FUNCTION__ << "NaN values in r ";
        return {};
    }

    auto r_pose_copy = r_pose;
    r_pose_copy.translate(Eigen::Vector2f(r(0), r(1)));
    r_pose_copy.rotate(r[2]);
    return {{r_pose_copy, max_match_error}};
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

void SpecificWorker::draw_graph(QGraphicsScene *scene)
{
    scene->clear();
    int x = 0;

    for (const auto &[node, neighbors] : graph.data())
    {
        QPointF pos(x * 300, 0);
        x++;

        QColor color = (node.first == Graph::NodeType::ROOM) ? Qt::blue : Qt::darkGreen;
        scene->addEllipse(-50, -50, 100, 100, QPen(color), QBrush(color))->setPos(pos);
        scene->addText(QString::number(node.second))->setPos(pos + QPointF(-10, -10));
    }
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

