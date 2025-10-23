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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <expected>
#include <random>
#include <chrono>

//Dibuja una ventana
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

enum class State {
	FORWARD,
	TURN,
	FOLLOW_WALL,
    BACK,
	SPIRAL
};

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();


public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();
	void update_robot_position();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

	//graphics
	void new_target_slot(QPointF p);

private:

	struct Params
	{
		float ROBOT_LENGTH = 400;  // mm
		float MAX_ADV_SPEED = 700; // mm/s
		float FRONT_SECTION = M_PI/6;  // 30°
		float SIDE_SECTION = M_PI/2; // 90°
		float THETA = 1.0f;
		int rot_direction = 1;

		int forwardCount = 0;
		int forwardTrigger = 50;

		int turnCount = 0;
		int turnTrigger = 17;

        int backCount = 0;
        int backTrigger = 10;

		std::string LIDAR_NAME_LOW = "bpearl";
		std::string LIDAR_NAME_HIGH = "helios";
	};
	Params params;

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	// graphics
	QRectF dimensions;
	AbstractGraphicViewer *viewer;
	QGraphicsPolygonItem *robot_polygon;
	std::chrono::steady_clock::time_point last_turn_time;
	std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points);
	void draw_lidar(const RoboCompLidar3D::TPoints& filtered_points, QGraphicsScene *scene);

	// aux
	std::expected<int, std::string> closest_lidar_index_to_given_angle(const RoboCompLidar3D::TPoints& points, float angle);

	//movimiento
	State current_state = State::SPIRAL;

	/**
    * @brief Comportamiento de espiral del robot.
    * Disminuye gradualmente el ángulo de rotación para abrir la espiral.
    * Si detecta un obstáculo en el rango [700, 730], cambia a FORWARD.
    *
    * @param points Vector de puntos Lidar 3D.
    * @return std::tuple<State, float, float> Estado siguiente, velocidad de avance y rotación.
    */
	std::tuple<State,float,float> Spiral(const RoboCompLidar3D::TPoints &points);

    /**
    * @brief Comportamiento de avance recto del robot.
    * Verifica si hay obstáculos en el centro del campo de visión.
    *
    * @param points Vector de puntos Lidar 3D.
    * @return std::tuple<State, float, float> Estado siguiente, velocidad de avance y rotación.
    */
	std::tuple<State, float, float> Forward(const RoboCompLidar3D::TPoints &points);

    /**
    * @brief Comportamiento de seguimiento de pared.
    * Busca el punto más cercano y evalúa si hay obstáculos frontales.
    * Puede cambiar aleatoriamente a TURN o a otros estados según condiciones.
    *
    * @param points Vector de puntos Lidar 3D.
    * @return std::tuple<State, float, float> Estado siguiente, velocidad de avance y rotación.
    */
	std::tuple<State, float, float> Follow_Wall(const RoboCompLidar3D::TPoints &points);

    /**
    * @brief Comportamiento de marcha atrás del robot.
    * Retrocede durante un número limitado de ciclos antes de girar.
    *
    * @return std::tuple<State, float, float> Estado siguiente, velocidad negativa y rotación.
    */
    std::tuple<State, float, float> Back();

    /**
    * @brief Comportamiento de giro en el lugar.
    * Gira durante un número limitado de ciclos antes de cambiar a espiral.
    *
    * @return std::tuple<State, float, float> Estado siguiente, velocidad cero y rotación.
    */
	std::tuple<State, float, float> Turn();


signals:
	//void customSignal();
};

#endif

