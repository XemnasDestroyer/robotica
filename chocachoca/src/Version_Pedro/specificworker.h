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

//Para paralelizar filter_isolated_points()
#ifdef emit
#undef emit
#endif
#include <algorithm>
#include <execution>
#include<cppitertools/enumerate.hpp>

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
		float MAX_ADV_SPEED = 800; // mm/s
		float FRONT_SECTION = M_PI/6;  // 30°
		float SIDE_SECTION = M_PI/2; // 90°
		float THETA = 1.5f;
		int rot_direction = 1;

		// Veces que se repite forward
		int forwardCount = 0;
		// Maximas repeticiones que puede tener forward (este numero fue elegido por ensayo y error)
		int forwardTrigger = 50;

		// Veces que se repite turn
		int turnCount = 0;
		// Maximas repeticiones que puede tener turn (este numero fue elegido por ensayo y error)
		int turnTrigger = 5;

		// Veces que se repite back
        int backCount = 0;
		// Maximas repeticiones que puede tener back (este numero fue elegido por ensayo y error)
        int backTrigger = 7;

		std::string LIDAR_NAME_LOW = "bpearl";
		std::string LIDAR_NAME_HIGH = "helios";
	};
	Params params;
	// Movimiento con el que comienza
	State current_state = State::SPIRAL;

	/**
     * @brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	// graphics
	QRectF dimensions;
	AbstractGraphicViewer *viewer;
	QGraphicsPolygonItem *robot_polygon;
	std::chrono::steady_clock::time_point last_turn_time;

	// Estos métodos nos lo da pablo ya hechos en el doc
    void draw_lidar(const RoboCompLidar3D::TPoints& filtered_points, QGraphicsScene *scene);
    /**
	* @brief Calculates the index of the closest lidar point to the given angle.
	*
	* This method searches through the provided std::list of lidar points and finds the point
	* whose angle (phi value) is closest to the specified angle. If a matching point is found,
	* the index of the point in the std::list is returned. If no point is found that matches the condition,
 	* an error message is returned.
	*
	* @param points The collection of lidar points to search through.
	* @param angle The target angle to find the closest matching point.
	* @return std::expected<int, std::string> containing the index of the closest lidar point if found,
	* or an error message if no such point exists.
	*/
	std::expected<int, std::string> closest_lidar_index_to_given_angle(const RoboCompLidar3D::TPoints& points, float angle);

	std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points);

	// Filter isolated points: keep only points with at least one neighbour within distance d (mm)
    RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);

	// Estos métodos ya son nuestros
	/**
    * @brief Comportamiento de espiral del robot.
    * Disminuye gradualmente el ángulo de rotación para abrir la espiral.
    * Si detecta un obstáculo en el rango [700, 730], cambia a FORWARD.
    *
    * @param points Vector de puntos Lidar 3D.
    * @return std::tuple<State, float, float> Estado siguiente, velocidad de avance y rotación.
    */
	std::tuple<State,float,float,float> Spiral(const RoboCompLidar3D::TPoints &points);

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
    * @brief Comportamiento de marcha atrás del robot.
    * Retrocede durante un número limitado de ciclos antes de girar.
    *
    * @return std::tuple<State, float, float> Estado siguiente, velocidad negativa y rotación.
    */
    std::tuple<State, float, float, float> Back();

    /**
    * @brief Comportamiento de giro en el lugar.
    * Gira durante un número limitado de ciclos antes de cambiar a espiral.
    *
    * @return std::tuple<State, float, float> Estado siguiente, velocidad cero y rotación.
    */
	std::tuple<State, float, float, float> Turn();

	std::expected<RoboCompLidar3D::TPoint, std::string> closest_lidar_point(const RoboCompLidar3D::TPoints &points, bool front, int range);


signals:
	//void customSignal();
};

#endif

