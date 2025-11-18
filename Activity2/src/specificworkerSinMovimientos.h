#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H
#ifdef emit
#undef emit     
#endif
#include <execution>
#include "movements.h"
//Para paralelizar filter_isolated_points()

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
    SpecificWorker(const ConfigLoader &configLoader, TuplePrx tprx, bool startup_check);

    /**
     * \brief Destructor for SpecificWorker.
     */
    ~SpecificWorker() override;

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

private:
    // Movimiento con el que comienza
    State current_state = State::SPIRAL;
    /**
     * @brief Flag indicating whether startup checks are enabled.
     */
    bool startup_check_flag;

    // Gráficos y entorno
    QRectF dimensions;
    AbstractGraphicViewer *viewer = nullptr;
    QGraphicsPolygonItem *robot_polygon = nullptr;
    AbstractGraphicViewer *viewer_room = nullptr;
    Eigen::Affine2d robot_pose;
    rc::Room_Detector room_detector;
    rc::Hungarian hungarian;
    QGraphicsPolygonItem *robot_room_draw = nullptr;

    // Movimientos
    Movements movimientos;

    // Estos métodos nos lo da pablo ya hechos en el doc
    void draw_lidar(const RoboCompLidar3D::TPoints &filtered_points, QGraphicsScene *scene);

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
    std::expected<int, std::string> closest_lidar_index_to_given_angle(const RoboCompLidar3D::TPoints &points, float angle);

    std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints &points);

    // Filter isolated points: keep only points with at least one neighbour within distance d (mm)
    RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);
};

#endif // SPECIFICWORKER_H




