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

SpecificWorker::SpecificWorker(const ConfigLoader &configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(
        configLoader, tprx) {
    this->startup_check_flag = startup_check;
    last_turn_time = std::chrono::steady_clock::now();
    if (this->startup_check_flag) {
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
        if (error.length() > 0) {
            qWarning() << error;
            throw error;
        }
    }
}

SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize() {
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
    if (rand == 1) {
        params.rot_direction = -1;
        qInfo() << "Valor iniciado -1";
    } else
        qInfo() << "Valor iniciado a 1";
}


void SpecificWorker::compute() {
    try {
        const auto data = lidar3d_proxy->getLidarData(params.LIDAR_NAME_LOW, 0, 2 * M_PI, 1);
        if (data.points.empty()) {
            qWarning() << "No points received";
            return;
        }

        const auto filter_data = filter_min_distance_cppitertools(data.points);
        if (!filter_data.has_value()) return;

        auto &points = filter_data.value();
        auto filterPoints = filter_isolated_points(points, 200);

        draw_lidar(points, &viewer->scene);

        std::tuple<State, float, float, float> result;
        switch (current_state) {
            case State::SPIRAL:
                result = Spiral(filterPoints);
                break;
            case State::FORWARD:
                result = Forward(filterPoints);
                break;
            case State::FOLLOW_WALL:
                result = Follow_Wall(filterPoints);
                break;
            case State::TURN:
                result = Turn(filterPoints);
                break;
        }
        // Desempaquetar resultado
        auto [st, velocityX, velocityZ, rotation] = result;
        current_state = st;  // Actualizamos el estado global

        // Enviar velocidades al robot
        try {
            omnirobot_proxy->setSpeedBase(velocityX, velocityZ, rotation * params.rot_direction);
        } catch (const Ice::Exception &e) { std::cout << e << std::endl; }

        update_robot_position();

    } catch (const Ice::Exception &e) {
        std::cout << e.what() << std::endl;
    }
}

std::tuple<State, float, float, float> SpecificWorker::Spiral(const RoboCompLidar3D::TPoints &points) {
    // Establece los valores de velocidad lateral, frontal y de rotación
    float velocityX = 0.0f;
    float velocityZ = params.MAX_ADV_SPEED;
    float rotation = params.MAX_ROTATION_SPEED;

    // Disminuir rotación para abrir la espiral
    params.MAX_ROTATION_SPEED = std::max(float(params.MAX_ROTATION_SPEED) - 0.006f, 0.0f);

    // Calcula el puntno más cercano a espiral
    RoboCompLidar3D::TPoint closestPoint = closest_lidar_point(points, false, 0).value();

    // Muestra los puntos cercanos en la UI
    setMinDistanceUI(closestPoint.r);
    setFrontMinDistanceUI(0);

    // Si hay algun punto cercano, pasa a forward
    if (closestPoint.r < 740.0)
        return {State::FORWARD, velocityX, velocityZ, rotation};

    // Muestra la accion espiral en la UI
    setActionNameUI("Espiral");

    return {State::SPIRAL, velocityX, velocityZ, rotation};
}

std::tuple<State, float, float, float> SpecificWorker::Forward(const RoboCompLidar3D::TPoints &points) {
    // Resetea la velocidad de giro
    params.MAX_ROTATION_SPEED = 1.6f;

    // Establece el valor de velocidad frontal
    float velocityX = 0.0f;
    float velocityZ = params.MAX_ADV_SPEED;
    float rotation = 0.0f;

    // Cambia (aleatoriamente) la dirección de giro
    //params.rot_direction = params.rot_direction * -1;

    // Calcula el punto más cercano de frente
    RoboCompLidar3D::TPoint closestFrontPoint = closest_lidar_point(points, true, 80).value();

    // Muestra los valores calculados en pantalla
    setMinDistanceUI(0);
    setFrontMinDistanceUI(closestFrontPoint.r);

    // Si el punto frontal más cercano está demasiado cerca, pasamos a seguir el obstáculo
    if (closestFrontPoint.r < 700.0) {
        velocityZ = 0.0f;
        rotation = params.MAX_ROTATION_SPEED;
        return {State::FOLLOW_WALL, velocityX, velocityZ, rotation};
    }

    // Si el follow wall se ejecuta por primera vez, calcula la hora a la que ha comenzado.
    if (!params.previousForward) {
        // Pone goingForward a true, para indicar que ya habiamos ejecutado goingForward antes
        params.previousForward = true;

        // Guarda el primer instante en el que se ejecuta
        params.action_start_time = std::chrono::steady_clock::now();

        // Aleatoriza la cantidad de tiempo que se va a repetir forward
        int rand = ((std::rand() % 2) + 2);
        params.action_duration = std::chrono::milliseconds(1000 * rand);

        //Muestra la accion en la UI
        setActionNameUI("Avanzar");

        return {State::FORWARD, velocityX, velocityZ, rotation};

    // Si es la segunda vez, comprueba que no se ejecute durante más tiempo del especificado en action_duration
    }else{
        // Calcula el tiempo que hace desde que comenzó forward
        auto elapse = std::chrono::steady_clock::now() - params.action_start_time;

        /* Si el tiempo que se ha ejecutado es menor que el tiempo que queremos que se ejecute, sigue
         * ejecutando forward
         */
        if(elapse < params.action_duration){
            //Muestra la accion en la UI
            setActionNameUI("Avanzar");

            //Aleatoriza el giro
            params.rot_direction*=-1;

            return {State::FORWARD, velocityX, velocityZ, rotation};
        }
        // Si llevamos más tiempo haciendo forward del que deberíamos, pasamos a espiral
        else{
            params.previousForward = false;
            return {State::SPIRAL, velocityX, velocityZ, rotation};
        }
    }
}

std::tuple<State, float, float, float> SpecificWorker::Follow_Wall(const RoboCompLidar3D::TPoints &points) {
    // Establece los valores de velocidad lateral, frontal y de rotación
    float velocityX = params.MAX_ADV_SPEED;
    float velocityZ = params.MAX_ADV_SPEED;
    float rotation = params.MAX_ROTATION_SPEED;
    std::tuple<State, float, float, float> result;

    // Calcula el punto más cercano al robot y el más cercano de frente
    RoboCompLidar3D::TPoint closestPoint = closest_lidar_point(points, false, 0).value();
    RoboCompLidar3D::TPoint closestFrontPoint = closest_lidar_point(points, true, 30).value();

    // Muestra los punto más cercano en la UI
    setMinDistanceUI(closestPoint.r);
    setFrontMinDistanceUI(closestFrontPoint.r);

    // Si el punto frontal está demasiado cerca, pasa a girar
    if (closestFrontPoint.r < 750) {
        // Resetea el timer de follow wall
        params.previousFollowWall = false;

        // Solo queremos un pequeño giro de turn, para ir ajustando
        params.previousTurn = true;
        // Para que valga 0
        params.action_start_time = std::chrono::steady_clock::now();
        params.action_duration = std::chrono::milliseconds(0);

        if(closestPoint.phi <= 0)
            params.rot_direction = 1;
        else
            params.rot_direction = -1;

        result = {State::TURN, 0.0f, 0.0f, rotation};
        return result;
    }

    // Si está cerca o (exclusivo ese o porque usamos XOR con ^) está a la izquierda
    if (params.followWallSafeDistance < closestPoint.r && closestPoint.phi <= 0)
        velocityX *=-1;

    // Si está lejos, se acerca
    else if (params.followWallSafeDistance > closestPoint.r && closestPoint.phi > 0)
        velocityX *=-1;

    // Calcula la diferencia entre la distancia en la que estamos y en la que queremos estar
    float distanceError = std::abs((closestPoint.r / params.followWallSafeDistance) - 1);
    // Multiplicamos la velocidad por el error. A más error, más velocidad;
    float velocityXCorrected = velocityX * distanceError;

    /* Calcula la diferencia entre el angulo en el que estamos y en el que queremos estar (en paralelo). El calculo
     * al final se un porcentaje
     */
    float rotationError = (std::abs(closestPoint.phi) / M_PI/2);
    // Multiplicamos la velocidad de rotation. A más error, más velocidad de rotación.
    float rotationCorrected = rotation * std::abs(rotationError);

    // Muestra la accion en la UI
    setActionNameUI("Seguir muro");

    // Resetea la variable de giro a 1, para que podamos controlar correctamente el recorrido en el muro
    params.rot_direction = 1;

    // Dependiendo de en qué sector del robot está el punto cercano, gira hacia una dirección o hacia otra
    if (-M_PI < closestPoint.phi && closestPoint.phi <= -M_PI / 2)
        params.rot_direction = -1;
    else if (0 < closestPoint.phi && closestPoint.phi <= M_PI / 2)
        params.rot_direction = -1;

    // Si es la primera vez que se ejecuta follow wall
    result = {State::FOLLOW_WALL, velocityXCorrected, velocityZ, rotationCorrected};
    if(!params.previousFollowWall){
        // Actualiza la variable para indicar que ya se ejecutó follow wall antes
        params.previousFollowWall = true;

        // Guarda el momento en el que comenzó follow wall
        params.action_start_time = std::chrono::steady_clock::now();

        // Aleatoriza la duración del follow wall y guarda la cantidad de tiempo que se va a repetir
        int rand = ((std::rand() % 6) + 1);
        params.action_duration = std::chrono::milliseconds(1000 * rand);

        // Muestra la accion en la UI
        setActionNameUI("Seguir muro");

        return result;

    // Si no es la primera vez seguida que se ejecuta follow wall
    }else{
        // Calcula el tiempo que pasó desde la primera ejecución de follow wall y esta
        auto elapsed = std::chrono::steady_clock::now() - params.action_start_time;

        // Si el tiempo es menor que la duración que queremos, vuelve a repetir follow wall
        if(elapsed <= params.action_duration) {
            // Muestra la accion en la UI
            setActionNameUI("Seguir muro");

            return result;
        }

        // Si el tiempo es mayor, pasamos a turn
        else{
            // Reestablece la variable previousfollowwall
            params.previousFollowWall = false;

            // Establece turn a una cantidad suficiente para un giro de 90 grados aproximadamente
            params.action_duration = std::chrono::milliseconds(1000);

            // Calcula el giro que debe hacer para alejarse del muro
            if (closestPoint.phi <= 0) {
                params.rot_direction = 1;
            } else{
                params.rot_direction = -1;
            }

            // Cambiamos el estado siguiente a SPIRAL
            std::get<0>(result) = State::TURN;
            return result;
        }
    }
}

std::tuple<State, float, float, float> SpecificWorker::Turn(const RoboCompLidar3D::TPoints &points) {
    // Establece los valores de velocidad lateral, frontal y de rotación
    float velocityX = 0.0;
    float velocityZ = 0.0;
    float rotation = params.MAX_ROTATION_SPEED;

    // Si es la primera vez que se ejecuta turn
    if(!params.previousTurn){
        // Actualiza la variable para indicar que ya se ejecutó turn antes
        params.previousTurn = true;

        // Guarda el momento en el que comenzó turn
        params.action_start_time = std::chrono::steady_clock::now();

        // Muestra la accion en la UI
        setActionNameUI("Girar");
        return {State::TURN, velocityX, velocityZ, rotation};

    // Si el punto no está cerca y tampoco es la primera vez seguidad que se ejecuta turn
    }else{
        // Calcula el tiempo que pasó desde la primera ejecución de turn y esta
        auto elapsed = std::chrono::steady_clock::now() - params.action_start_time;

        // Si el tiempo es menor que la duración que queremos, vuelve a repetir turn
        if(elapsed < params.action_duration) {
            // Muestra la accion en la UI
            setActionNameUI("Girar");
            return {State::TURN, velocityX, velocityZ, rotation};
        }

        // Si el tiempo es mayor, pasamos a forward
        else{
            setActionNameUI("Girar");
            params.previousTurn = false;
            return {State::FORWARD, velocityX, velocityZ, rotation};
        }
    }
}

std::expected<RoboCompLidar3D::TPoint, std::string>
SpecificWorker::closest_lidar_point(const RoboCompLidar3D::TPoints &points, bool front, int range) {
    if (points.empty())
        return std::unexpected("Empty points container");

    RoboCompLidar3D::TPoints::const_iterator res;
    if (front) {
        int frontPointIndex = points.size() / 2;
        int offset = range;
        res = std::min_element(points.begin() + (frontPointIndex - offset),
                               points.begin() + (frontPointIndex + offset),
                               [](const auto &a, const auto &b) {
                                   return a.r < b.r;
                               });
    } else {
        res = std::min_element(points.begin(),
                               points.end(),
                               [](const auto &a, const auto &b) {
                                   return a.r < b.r;
                               });
    }
    return *res;
}

void SpecificWorker::setMinDistanceUI(float distance){
    minDistNum->display(distance);
}

void SpecificWorker::setFrontMinDistanceUI(float distance){
    minFrontDistNum->display(distance);

}

void SpecificWorker::setActionNameUI(std::string text){
    QString qStringText = text.data();
    actualAction->setText(qStringText);
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

    auto offset_begin = closest_lidar_index_to_given_angle(filtered_points, -params.FRONT_SECTION);
    auto offset_end = closest_lidar_index_to_given_angle(filtered_points, params.FRONT_SECTION);

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
    auto wall_right = closest_lidar_index_to_given_angle(filtered_points, params.SIDE_SECTION);
    auto wall_left = closest_lidar_index_to_given_angle(filtered_points, -params.SIDE_SECTION);
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
    auto res_right = closest_lidar_index_to_given_angle(filtered_points, params.FRONT_SECTION);
    auto res_left = closest_lidar_index_to_given_angle(filtered_points, -params.FRONT_SECTION);
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


void SpecificWorker::emergency() {
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore() {
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check() {
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



