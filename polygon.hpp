/*
 * Copyright 2020 Casey Sanchez
 */

#pragma once

#include <limits>

#include "graph.hpp"

namespace Mandoline
{
    class Polygon : public Graph
    {
    public:
        Polygon();
        Polygon(Graph const &graph);
        Polygon(std::vector<Eigen::Vector2d> const &vertices, std::vector<Eigen::Array2i> const &edges);
        Polygon(std::initializer_list<Eigen::Vector2d> const &vertices, std::initializer_list<Eigen::Array2i> const &edges);
        Polygon(std::vector<Eigen::Vector2d> const &vertices);
        Polygon(std::initializer_list<Eigen::Vector2d> const &vertices);

        bool IsPointInside(Eigen::Vector2d const &point) const;
        bool IsPointOnEdge(Eigen::Vector2d const &point, Eigen::Array2i const &edge, double const epsilon = 1e-9) const;

        Eigen::Vector2d Min() const;
        Eigen::Vector2d Max() const;
        Eigen::Vector2d Centroid() const;

        // Generates a regular n-sided polygon
        static Polygon Regular(uint32_t const &n_sides, double const &radius);
    };
}