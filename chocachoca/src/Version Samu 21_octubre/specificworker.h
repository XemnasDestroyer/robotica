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
	IDLE,
	FORWARD,
	TURN,
	FOLLOW_WALL,
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
	void computeDistances(const RoboCompLidar3D::TPoints &points);
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
		float WALL_ENTER_MIN = 0.9 * WALL_MIN_DISTANCE;
		float WALL_ENTER_MAX = 1.2 * ADVANCE_THRESHOLD;
		float FOLLOW_WALL_ENTER_MAX = 700;
		float FOLLOW_WALL_EXIT_MIN = 450;
		float MAX_ADV_SPEED = 800; // mm/s
		float MAX_ROT_SPEED = 2;   // rad/s
		float ADVANCE_THRESHOLD = 700; // mm
		float WALL_MIN_DISTANCE = 500; // mm
		float FRONT_SECTION = M_PI/6;      // 30° a cada lado → sector frontal de 60°
		float SIDE_SECTION = M_PI/3;       // 60° para sectores laterales
		float FRONT_ANGLE_RANGE = M_PI/6;  // 30° (para el cálculo que hicimos arriba
		float WALL_FOLLOW_THRESHOLD = 450;
		float WALL_EXIT_THRESHOLD = 600;
		float THETA = 0.f;
		float SPIRAL_RADIUS = 100.f;  // <- AÑADIR ESTO
		float SPIRAL_ADV = 400.0f;
		float SPIRAL_ROT = 0.3f;

		std::string LIDAR_NAME_LOW = "bpearl";
		std::string LIDAR_NAME_HIGH = "helios";
		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

	};

	struct LidarDistances {
		float front;
		float left;
		float right;
		float min_360;
	};
	LidarDistances dist;
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
	State current_state = State::FORWARD;
	std::tuple<State, float, float> Forward();
	std::tuple<State, float, float> Turn();
	std::tuple<State, float, float> Follow_Wall();
	std::tuple<State,float,float> Spiral();


signals:
	//void customSignal();
};

#endif

