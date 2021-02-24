/*
 * Copyright 2020 Casey Sanchez
 */

#include "graph.hpp"

namespace Mandoline
{
    Graph::Graph()
    {
    }

    Graph::Graph(std::vector<Eigen::Vector2d> const &vertices, std::vector<Eigen::Array2i> const &edges) : m_vertices(vertices), m_edges(edges)
    {
    }

    Graph::Graph(std::initializer_list<Eigen::Vector2d> const &vertices, std::initializer_list<Eigen::Array2i> const &edges) : Graph(std::vector<Eigen::Vector2d>{ vertices }, std::vector<Eigen::Array2i>{ edges })
    {
    }

    std::vector<Eigen::Vector2d> Graph::Vertices() const
    {
        return m_vertices;
    }

    std::vector<Eigen::Array2i> Graph::Edges() const
    {
        return m_edges;
    }

    std::vector<std::array<Eigen::Vector2d, 2>> Graph::Segments() const
    {
        std::vector<std::array<Eigen::Vector2d, 2>> segments;

        std::transform(std::cbegin(m_edges), std::cend(m_edges), std::back_inserter(segments),
            [this](Eigen::Array2i const &edge) -> std::array<Eigen::Vector2d, 2> {
                return Segment(edge);
            });

        return segments;
    }

    std::vector<Eigen::Hyperplane<double, 2>> Graph::Lines() const
    {
        std::vector<Eigen::Hyperplane<double, 2>> lines;

        std::transform(std::cbegin(m_edges), std::cend(m_edges), std::back_inserter(lines),
            [this](Eigen::Array2i const &edge) -> Eigen::Hyperplane<double, 2> {
                return Line(edge);
            });

        return lines;
    }

    std::array<Eigen::Vector2d, 2> Graph::Segment(Eigen::Array2i const &edge) const
    {
        return { m_vertices.at(edge[0]), m_vertices.at(edge[1]) };
    }

    Eigen::Hyperplane<double, 2> Graph::Line(Eigen::Array2i const &edge) const
    {
        auto const &[vertex_a, vertex_b] = Segment(edge);

        return Eigen::Hyperplane<double, 2>::Through(vertex_a, vertex_b);
    }

    uint32_t Graph::Intersections(Eigen::Vector2d const &point) const
    {
        std::vector<Eigen::Array2i> const &edges = Edges();

        uint32_t intersections = 0;

        for (Eigen::Array2i const &edge : edges) {
            auto const &[vertex_a, vertex_b] = Segment(edge);

            if ((vertex_a[1] > point[1]) != (vertex_b[1] > point[1]) && (point[0] < (vertex_b[0] - vertex_a[0]) * (point[1] - vertex_a[1]) / (vertex_b[1] - vertex_a[1]) + vertex_a[0])) {
                intersections++;
            }
        }

        return intersections;
    }

    bool Graph::IsPointInside(Eigen::Vector2d const &point) const
    {
        return Intersections(point) % 2 != 0;
    }

    bool Graph::IsPointOnEdge(Eigen::Vector2d const &point, Eigen::Array2i const &edge, double const epsilon) const
    {
        auto const &[vertex_a, vertex_b] = Segment(edge);

        double const norm = (vertex_a - point).norm() + (point - vertex_b).norm() - (vertex_a - vertex_b).norm();

        return std::abs(norm) < epsilon;
    }

    Eigen::Vector2d Graph::Min() const
    {
        std::vector<Eigen::Vector2d> const &vertices = Vertices();

        Eigen::Vector2d min;

        min[0] = std::numeric_limits<double>::infinity();
        min[1] = std::numeric_limits<double>::infinity();

        for (Eigen::Vector2d const &vertex : vertices) {
            min[0] = std::min(min[0], vertex[0]);
            min[1] = std::min(min[1], vertex[1]);
        }

        return min;
    }

    Eigen::Vector2d Graph::Max() const
    {
        std::vector<Eigen::Vector2d> const &vertices = Vertices();

        Eigen::Vector2d max;

        max[0] = -std::numeric_limits<double>::infinity();
        max[1] = -std::numeric_limits<double>::infinity();

        for (Eigen::Vector2d const &vertex : vertices) {
            max[0] = std::max(max[0], vertex[0]);
            max[1] = std::max(max[1], vertex[1]);
        }

        return max;
    }

    Eigen::Vector2d Graph::Centroid() const
    {
        return (Min() + Max()) * 0.5;
    }
}