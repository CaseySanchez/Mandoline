/*
 * Copyright 2020 Casey Sanchez
 */

#pragma once

#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>

namespace Mandoline
{
    class Graph
    {
        std::vector<Eigen::Vector2d> m_vertices;
        std::vector<Eigen::Array2i> m_edges;

    public:
        Graph();
        Graph(std::vector<Eigen::Vector2d> const &vertices, std::vector<Eigen::Array2i> const &edges);
        Graph(std::initializer_list<Eigen::Vector2d> const &vertices, std::initializer_list<Eigen::Array2i> const &edges);
        Graph(std::vector<Eigen::Vector2d> const &vertices, bool const &loop);
        Graph(std::initializer_list<Eigen::Vector2d> const &vertices, bool const &loop);

        std::vector<Eigen::Vector2d> Vertices() const;
        std::vector<Eigen::Array2i> Edges() const;

        std::vector<std::array<Eigen::Vector2d, 2>> Segments() const;
        std::vector<Eigen::Hyperplane<double, 2>> Lines() const;

        std::array<Eigen::Vector2d, 2> Segment(Eigen::Array2i const &edge) const;
        Eigen::Hyperplane<double, 2> Line(Eigen::Array2i const &edge) const;
    };
}