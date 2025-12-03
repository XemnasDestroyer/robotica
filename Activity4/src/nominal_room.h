#pragma once
#include <QPointF>
#include <QRectF>
#include <Eigen/Dense>
#include <vector>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/sliding_window.hpp>

#include "src/common_types.h"

  struct NominalRoom
        {
            float width; //  mm
            float length;
            explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners  corners_ = {}) :
                width(width_), length(length_)
            {};
            [[nodiscard]] Corners corners() const
            {
                // compute corners from width and length
                return {
                    {QPointF{-width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, length/2.f}, 0.f, 0.f},
                    {QPointF{-width/2.f, length/2.f}, 0.f, 0.f}
                };
            }

            // Devuelve los distintos muros que hay en la habitación según las esquinas que haya.
            [[nodiscard]] Walls walls() const
            {
                Walls walls_;
                auto cs = corners();
                cs.push_back(cs[0]);
                for (const auto &[i, c_] : cs | iter::sliding_window(2) | iter::enumerate)
                {
                    const auto &[c1, _, __] = c_[0];
                    const auto &[c2, ___, ____] = c_[1];
                    auto w = Eigen::ParametrizedLine<float, 2>::Through(Eigen::Vector2f{c1.x(), c1.y()}, Eigen::Vector2f{c2.x(), c2.y()});
                    walls_.emplace_back(i, w);
                }
            };

            // Devuelve el muro más cercano al punto point
            [[nodiscard]] Wall point_to_wall(const Eigen::Vector2f &point) const
            {
                // return the closest wall to p
                const auto ws = walls();
                const auto min = std::ranges::min_element(ws, [point](const auto &w1, const auto &w2)
                {
                    const auto &[i, w1_] = w1;
                    const auto &[i2, w2_] = w2;
                    return w1_.distance(point) < w2_.distance(point);
                });
                return *min;
            }

            // Devuelve la proyección de un punto a un muro dado
            [[nodiscard]] float distance_from_wall(const Wall &wall, const Eigen::Vector2f &p) const
            {
                // return the distance from first corner to point p
                const auto &[i, w] = wall;
                const auto proj = w.projection(p);
                return proj.norm();
            }

            [[nodiscard]] QRectF rect() const
            {
                return QRectF{-width/2.f, -length/2.f, width, length};
            }
            [[nodiscard]] Corners transform_corners_to(const Eigen::Affine2d &transform) const  // for room to robot pass the inverse of robot_pose
            {
                Corners transformed_corners;
                for(const auto &[p, _, __] : corners())
                {
                    auto ep = Eigen::Vector2d{p.x(), p.y()};
                    Eigen::Vector2d tp = transform * ep;
                    transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
                }
                return transformed_corners;
            }
        };