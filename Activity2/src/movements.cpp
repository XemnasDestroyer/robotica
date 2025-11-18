#include "movements.h"

Movements::Movements() {}

std::tuple<State, float, float, float> Movements::Spiral(const RoboCompLidar3D::TPoints &points) {
    // Establece los valores de velocidad lateral, frontal y de rotación
    float velocityX = 0.0f;
    float velocityZ = params.MAX_ADV_SPEED;
    float rotation = params.MAX_ROTATION_SPEED;

    // Disminuir rotación para abrir la espiral
    params.MAX_ROTATION_SPEED = std::max(float(params.MAX_ROTATION_SPEED) - 0.006f, 0.0f);

    // Calcula el puntno más cercano a espiral
    RoboCompLidar3D::TPoint closestPoint = closest_lidar_point(points, false, 0).value();

    // Si hay algun punto cercano, pasa a forward
    if (closestPoint.r < 740.0)
        return {State::FORWARD, velocityX, velocityZ, rotation};

    return {State::SPIRAL, velocityX, velocityZ, rotation};
}

std::tuple<State, float, float, float> Movements::Forward(const RoboCompLidar3D::TPoints &points) {
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

        return {State::FORWARD, velocityX, velocityZ, rotation};

        // Si es la segunda vez, comprueba que no se ejecute durante más tiempo del especificado en action_duration
    }else{
        // Calcula el tiempo que hace desde que comenzó forward
        auto elapse = std::chrono::steady_clock::now() - params.action_start_time;

        /* Si el tiempo que se ha ejecutado es menor que el tiempo que queremos que se ejecute, sigue
         * ejecutando forward
         */
        if(elapse < params.action_duration){

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

std::tuple<State, float, float, float> Movements::Follow_Wall(const RoboCompLidar3D::TPoints &points) {
    // Establece los valores de velocidad lateral, frontal y de rotación
    float velocityX = params.MAX_ADV_SPEED;
    float velocityZ = params.MAX_ADV_SPEED;
    float rotation = params.MAX_ROTATION_SPEED;
    std::tuple<State, float, float, float> result;

    // Calcula el punto más cercano al robot y el más cercano de frente
    RoboCompLidar3D::TPoint closestPoint = closest_lidar_point(points, false, 0).value();
    RoboCompLidar3D::TPoint closestFrontPoint = closest_lidar_point(points, true, 30).value();

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

        return result;

        // Si no es la primera vez seguida que se ejecuta follow wall
    }else{
        // Calcula el tiempo que pasó desde la primera ejecución de follow wall y esta
        auto elapsed = std::chrono::steady_clock::now() - params.action_start_time;

        // Si el tiempo es menor que la duración que queremos, vuelve a repetir follow wall
        if(elapsed <= params.action_duration) {

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

std::tuple<State, float, float, float> Movements::Turn(const RoboCompLidar3D::TPoints &points) {
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
        return {State::TURN, velocityX, velocityZ, rotation};

        // Si el punto no está cerca y tampoco es la primera vez seguidad que se ejecuta turn
    }else{
        // Calcula el tiempo que pasó desde la primera ejecución de turn y esta
        auto elapsed = std::chrono::steady_clock::now() - params.action_start_time;

        // Si el tiempo es menor que la duración que queremos, vuelve a repetir turn
        if(elapsed < params.action_duration) {
            // Muestra la accion en la UI
            return {State::TURN, velocityX, velocityZ, rotation};
        }

            // Si el tiempo es mayor, pasamos a forward
        else{
            params.previousTurn = false;
            return {State::FORWARD, velocityX, velocityZ, rotation};
        }
    }
}

std::expected<RoboCompLidar3D::TPoint, std::string>
Movements::closest_lidar_point(const RoboCompLidar3D::TPoints &points, bool front, int range) {
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
