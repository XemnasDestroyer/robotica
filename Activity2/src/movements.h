#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include "common_types.h"
#include "hungarian.h"
#include "room_detector.h"
// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "common_types.h"
#include <expected>
#include <cppitertools/itertools.hpp>
//Dibuja una ventana
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <algorithm>
#include <cppitertools/enumerate.hpp>

enum class State {
    FORWARD,
    TURN,
    FOLLOW_WALL,
    SPIRAL
};

struct Params
    {
        float ROBOT_LENGTH = 400;  // mm
		float MAX_ADV_SPEED = 1000; // mm/s
		float MAX_ROTATION_SPEED = 1.6f;
        float FRONT_SECTION = M_PI/6;  // 30°
        float SIDE_SECTION = M_PI/2; // 90°
		int rot_direction = 1;

		// Si previamente se ejecutó forward
		bool previousForward = false;

		// Si previamente se ejecutó followWall
		bool previousFollowWall = false;
		// Distancia a la que nos queremos mantener del muro
		float followWallSafeDistance = 500;

		// Veces que se repite turn
		bool previousTurn = false;

		// Variables para calcular el tiempo que se ejecutan las acciones
		std::chrono::steady_clock::time_point action_start_time;
		// Maximo tiempo que una accion se va a ejecutar
		std::chrono::milliseconds action_duration;

        std::string LIDAR_NAME_LOW = "bpearl";
        std::string LIDAR_NAME_HIGH = "helios";
};

class Movements
{
public:

    Params params;
    std::expected<RoboCompLidar3D::TPoint, std::string> closest_lidar_point(const RoboCompLidar3D::TPoints &points, bool front, int range);


    explicit Movements();
    /**
    * @brief Comportamiento de espiral del robot.
    * Disminuye gradualmente el ángulo de rotación para abrir la espiral.
    * Si detecta un obstáculo en el rango [700, 730], cambia a FORWARD.
    *
    * @param points Vector de puntos Lidar 3D.
    * @return std::tuple<State, float, float> Estado siguiente, velocidad de avance y rotación.
    */
    std::tuple<State, float, float, float> Spiral(const RoboCompLidar3D::TPoints &points);

    /**
    * @brief Comportamiento de avance recto del robot.
    * Verifica si hay obstáculos en el centro del campo de visión.
    *
    * @param points Vector de puntos Lidar 3D.
    * @return std::tuple<State, float, float> Estado siguiente, velocidad de avance y rotación.
    */
    std::tuple<State, float, float, float> Forward(const RoboCompLidar3D::TPoints &points);

    /**
    * @brief Comportamiento de seguimiento de pared.
    * Busca el punto más cercano y evalúa si hay obstáculos frontales.
    * Puede cambiar aleatoriamente a TURN o a otros estados según condiciones.
    *
    * @param points Vector de puntos Lidar 3D.
    * @return std::tuple<State, float, float> Estado siguiente, velocidad de avance y rotación.
    */
    std::tuple<State, float, float, float> Follow_Wall(const RoboCompLidar3D::TPoints &points);

    /**
    * @brief Comportamiento de giro en el lugar.
    * Gira durante un número limitado de ciclos antes de cambiar a espiral.
    *
    * @return std::tuple<State, float, float> Estado siguiente, velocidad cero y rotación.
    */
    std::tuple<State, float, float, float> Turn(const RoboCompLidar3D::TPoints &points);

private:
};

#endif // MOVEMENTS_H





