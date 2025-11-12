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

//Para paralelizar filter_isolated_points()
#ifdef emit
#undef emit
#endif
#include <algorithm>
#include <execution>
#include <cppitertools/enumerate.hpp>

enum class StatelLocation {
	LOCALISE,
	GOTTO_ROOM_CENTER,
	TURN,
	UPDATE_POSE
};

enum class StateDoor {
	IDLE,
	ORIENT_TO_ROOM,
	SEARCH_DOORS,
	ORIENT_TO_DOOR,
	GOTO_DOOR
};

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */

struct NominalRoom
{
   float width;  // mm
   float length;
   Corners corners;

   explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners corners_ = {})
       : width(width_), length(length_), corners(std::move(corners_)) {}

   Corners transform_corners_to(const Eigen::Affine2d &transform) const
   {
       Corners transformed_corners;
       for(const auto &[p, _, __] : corners)
       {
           auto ep = Eigen::Vector2d{p.x(), p.y()};
           Eigen::Vector2d tp = transform * ep;
           transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
       }
       return transformed_corners;
   }
};
extern NominalRoom room;


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

	/**
	 * \brief ________________________
	 */
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

private:struct Params
    {
        float ROBOT_LENGTH = 400;  // mm
		float MAX_ADV_SPEED = 500; // mm/s
		float MAX_ROTATION_SPEED = 0.2f;
        float FRONT_SECTION = M_PI/6;  // 30°
        float SIDE_SECTION = M_PI/2; // 90°
		int rot_direction = 1;

        std::string LIDAR_NAME_LOW = "bpearl";
        std::string LIDAR_NAME_HIGH = "helios";
    };
    Params params;

	/**
     * @brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	// graphics
	QRectF dimensions;
	AbstractGraphicViewer *viewer;
	QGraphicsPolygonItem *robot_polygon;

	AbstractGraphicViewer *viewer_room;  // new frame to show the room
	Eigen::Affine2d robot_pose;          // rotation + translation
	std::vector<NominalRoom> nominal_rooms {
		NominalRoom(5500.f, 4000.f),
		NominalRoom(3000.f, 4000.f)
	};
	rc::Room_Detector room_detector;     // compute corners
	rc::Hungarian hungarian;             // match corners
	QGraphicsPolygonItem *robot_room_draw; // draw robot in the room

	enum class STATE {
		IDLE,
		LOCALISE,
		UPDATE_POSE,
		SEARCH_DOORS,
		ORIENT_TO_ROOM,
		GOTO_ROOM_CENTER,
		ORIENT_TO_DOOR,
		GOTO_DOOR,
		TURN
	};

	inline const char* to_string(const STATE s) const
	{
		switch (s)
		{
			case STATE::IDLE: return "IDLE";
			case STATE::LOCALISE: return "LOCALISE";
			case STATE::UPDATE_POSE: return "UPDATE_POSE";
			case STATE::SEARCH_DOORS: return "SEARCH_DOORS";
			case STATE::ORIENT_TO_ROOM: return "ORIENT_TO_ROOM";
			case STATE::GOTO_ROOM_CENTER: return "GOTO_ROOM_CENTER";
			case STATE::ORIENT_TO_DOOR: return "ORIENT_TO_DOOR";
			case STATE::GOTO_DOOR: return "GOTO_DOOR";
			case STATE::TURN: return "TURN";
			default: return "UNKNOWN";
		}
	}

	STATE state = STATE::LOCALISE;

	std::tuple<STATE, float, float> goto_door(const RoboCompLidar3D::TPoints &points);
	std::tuple<STATE, float, float> orient_to_door(const RoboCompLidar3D::TPoints &points);
	std::tuple<STATE, float, float> orient_to_room(const RoboCompLidar3D::TPoints &points);
	std::tuple<STATE, float, float> localise(const Match &match, const RoboCompLidar3D::TPoints &points);
	std::tuple<STATE, float, float> search_doors(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
	std::tuple<STATE, float, float> goto_room_center(const RoboCompLidar3D::TPoints &points);
	std::tuple<STATE, float, float> update_pose(const Corners &corners, const Match &match);
	std::tuple<STATE, float, float> turn(const RoboCompLidar3D::TPoints &points);

	// Lógica general de control de estados
	std::tuple<STATE, float, float> process_state(const RoboCompLidar3D::TPoints &data,
												  const Corners &corners,
												  const Match &match,
												  AbstractGraphicViewer *viewer);

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

	// std::expected<RoboCompLidar3D::TPoint, std::string> closest_lidar_point(const RoboCompLidar3D::TPoints &points, bool front, int range);

	void draw_lidar(const RoboCompLidar3D::TPoints& filtered_points, std::optional<Eigen::Vector2d> center, QGraphicsScene *scene);

    // Filter isolated points: keep only points with at least one neighbour within distance d (mm)
    RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);

signals:
	//void customSignal();
};

#endif
