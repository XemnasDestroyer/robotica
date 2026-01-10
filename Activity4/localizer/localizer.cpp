// localised is set to false

void SpecificWorker::compute()
{
    RoboCompLidar3D::TPoints data = read_data();
    data = door_detector.filter_points(data);
    draw_local_doors(&viewer->scene);


    // compute corners and center
    const auto &[corners, lines] = room_detector.compute_corners(data, &viewer->scene);
    const auto center_opt = center_estimator.estimate(data);
    draw_lidar(data, center_opt, &viewer->scene);


    // update robot pose
    float max_match_error = -1;
    //Match match;
    if (localised)
    {
        if (const auto res = update_robot_pose(current_room, corners, robot_pose, true); res.has_value())
        {
            robot_pose = res.value().first;
            max_match_error = res.value().second;
            time_series_plotter->addDataPoint(match_error_graph,max_match_error);
        }
    }


    // Process state machine
    RetVal ret_val = process_state(data, corners, &viewer->scene, &viewer_room->scene);
    auto [st, adv, rot] = ret_val;
    state = st;


    // Send movements commands to the robot constrained by the match_error
    if (not pushButton_stop->isChecked())
        move_robot(adv, rot, max_match_error);

    draw_nominal_room(current_room, &viewer_room->scene);
    //print_room_data();
    draw_robot_in_viewer(localised, robot_pose, robot_room_draw);
    update_gui(adv, rot);


    last_time = std::chrono::high_resolution_clock::now();;
}

SpecificWorker::RetVal SpecificWorker::localise(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    // initialise robot pose at origin. Necessary to reser pose accumulation
    robot_pose.setIdentity();
    robot_pose.translate(Eigen::Vector2f(0.0,0.0));
    localised = false;


    // if error high but not at room centre, go to centering step
    // compute mean of LiDAR points as room center estimate


    if(const auto center = center_estimator.estimate(points); center.has_value())
    {
        if (center.value().norm() > params.RELOCAL_CENTER_EPS )
            return{STATE::GOTO_ROOM_CENTER, 0.0f, 0.0f};


        // If close enough to center -> stop and move to TURN
        if (center.value().norm() < params.RELOCAL_CENTER_EPS )
            return {STATE::TURN, 0.0f, 0.0f};
    }
    qWarning() << __FUNCTION__ << "Not able to estimate room center from walls, continue localising.";
    return {STATE::LOCALISE, 0.0f, 0.0f};
}
