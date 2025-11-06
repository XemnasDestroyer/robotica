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
#include "hungarian.h"
#include "munkres.hpp"
#include "ransac_line_detector.h"
#include "room_detector.h"

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

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

    // Carga el robot para que se dibuje
    this->dimensions = QRectF(-6000, -3000, 12000, 6000);
    viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
    viewer->show();
    const auto rob = viewer->add_robot(400, 400, 0, 190, QColor("Blue"));
    robot_polygon = std::get<0>(rob);

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

	viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
	auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
	robot_room_draw = rr;  
	
	// draw room in viewer_room
	viewer_room->scene.addRect(params.GRID_MAX_DIM, QPen(Qt::black, 30));
	viewer_room->show();
	
	// initialise robot pose
  	robot_pose.setIdentity();
 	robot_pose.translate(Eigen::Vector2d(0.0,0.0));

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

}


void SpecificWorker::compute()
{
    // 1) Obtener datos del LIDAR
    const auto data = lidar3d_proxy->getLidarData("pearl", 0, 2 * M_PI, 1);
    if (data.points.empty()) {
        qWarning() << "No points received";
        return;
    }

    // 2) Calcular las esquinas medidas con RANSAC desde el point cloud (mi)
    Corners measured_corners = room_detector.compute_corners(data.points, nullptr);
    if (measured_corners.empty()) {
        qInfo() << "No measured corners found";
        return;
    }

    // 3) Obtener las esquinas nominales del entorno (definidas en NominalRoom)
    //    Estas están en el marco de referencia de la habitación (mundo) (ci)
    const Corners &nominal_corners = room.corners;

    // 4) Transformar las esquinas nominales al marco del robot
    //    Regla: de room -> robot usamos la inversa de la pose actual (Cambio de marco de referencia)
    Corners nominal_in_robot = room.transform_corners_to(robot_pose.inverse());

    // 5) Aplicar el método húngaro para emparejar esquinas medidas con nominales (ci, mi)
    const double MAX_CORNER_DIFF = 400.0; // umbral en mm (ajustable)
    Match matches = hungarian.match(measured_corners, nominal_in_robot, MAX_CORNER_DIFF);

    if (matches.empty()) {
        qInfo() << "No matches after Hungarian";
        return;
    }

    // 6) Construir matrices W y b
    size_t M = matches.size();
    if (M < 2) {
        qInfo() << "Not enough matches to estimate pose";
        return;
    }

    Eigen::MatrixXd W(2 * M, 3);
    Eigen::VectorXd b(2 * M);

    for (size_t i = 0; i < M; ++i)
    {
        const auto &[meas_c, nom_c, _] = matches[i];
        const auto &[p_meas, __, ___]  = meas_c;
        const auto &[p_nom, ____, _____] = nom_c;

        double mx = p_meas.x();
        double my = p_meas.y();
        double cx = p_nom.x();
        double cy = p_nom.y();

        b(2 * i)     = cx - mx;
        b(2 * i + 1) = cy - my;

        W.block<1, 3>(2 * i, 0)     << 1.0, 0.0, -my;
        W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0,  mx;
    }

    // 7) Resolver por mínimos cuadrados con pseudoinversa: \ r = (WᵀW)^(-1) Wᵀ b \
    Eigen::Matrix3d WT_W = W.transpose() * W;
    Eigen::Vector3d WT_b = W.transpose() * b;
    Eigen::Vector3d r = WT_W.inverse() * WT_b;

    std::cout << "Estimated r = [x=" << r(0) << ", y=" << r(1) << ", phi=" << r(2) << "]" << std::endl;
    qInfo() << "--------------------";

    if (r.array().isNaN().any()) {
        qWarning() << "Invalid (NaN) values in pose correction";
        return;
    }

    // 8) Actualizar la pose del robot con los nuevos valores estimados
    robot_pose.translate(Eigen::Vector2d(r(0), r(1)));
    robot_pose.rotate(r(2));

    // 9) Actualizar el dibujo del robot en el entorno
    robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
    double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
    robot_room_draw->setRotation(angle);

    qInfo() << "Pose updated -> x:" << robot_pose.translation().x()
            << " y:" << robot_pose.translation().y()
            << " theta:" << angle;
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

void SpecificWorker::new_target_slot(QPointF p) {
	qInfo() << "World coordinates" << p;
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

