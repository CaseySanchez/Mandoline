/*
 * Copyright 2020 Casey Sanchez
 */

#include "polygon.hpp"

namespace Mandoline
{
    Polygon::Polygon() : Graph()
    {
    }

    Polygon::Polygon(Graph const &graph) : Graph(graph)
    {
    }

    Polygon::Polygon(std::vector<Eigen::Vector2d> const &vertices, std::vector<Eigen::Array2i> const &edges) : Graph(vertices, edges)
    {
    }

    Polygon::Polygon(std::initializer_list<Eigen::Vector2d> const &vertices, std::initializer_list<Eigen::Array2i> const &edges) : Polygon(std::vector<Eigen::Vector2d>{ vertices }, std::vector<Eigen::Array2i>{ edges })
    {
    }

    Polygon::Polygon(std::vector<Eigen::Vector2d> const &vertices) : Graph(vertices, true)
    {
    }

    Polygon::Polygon(std::initializer_list<Eigen::Vector2d> const &vertices) : Polygon(std::vector<Eigen::Vector2d>{ vertices })
    {
    }

    bool Polygon::IsPointInside(Eigen::Vector2d const &point) const
    {
        std::vector<Eigen::Array2i> const &edges = Edges();

        bool inside = false;

        for (Eigen::Array2i const &edge : edges) {
            auto const &[vertex_a, vertex_b] = Segment(edge);

            if ((vertex_a[1] > point[1]) != (vertex_b[1] > point[1]) && (point[0] < (vertex_b[0] - vertex_a[0]) * (point[1] - vertex_a[1]) / (vertex_b[1] - vertex_a[1]) + vertex_a[0])) {
                inside = !inside;
            }
        }

        return inside;
    }

    bool Polygon::IsPointOnEdge(Eigen::Vector2d const &point, Eigen::Array2i const &edge, double const epsilon) const
    {
        auto const &[vertex_a, vertex_b] = Segment(edge);

        double const norm = (vertex_a - point).norm() + (point - vertex_b).norm() - (vertex_a - vertex_b).norm();

        return std::abs(norm) < epsilon;
    }

    Eigen::Vector2d Polygon::Min() const
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

    Eigen::Vector2d Polygon::Max() const
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

    Eigen::Vector2d Polygon::Centroid() const
    {
        return (Min() + Max()) * 0.5;
    }

    // Generates a regular n-sided polygon
    Polygon Polygon::Regular(uint32_t const &n_sides, double const &radius)
    {
        if (n_sides < 3) {
            throw std::domain_error("n_sides < 3");
        }

        std::vector<Eigen::Vector2d> out_vertices;

        double const tau = 2.0 * 3.14159;
        double const step = tau / static_cast<double>(n_sides);

        for (uint32_t n = 0; n < n_sides; ++n) {
            double theta = step * (static_cast<double>(n) + 0.5);

            out_vertices.emplace_back(std::cos(-theta) * radius, std::sin(-theta) * radius);
        }

        return Polygon(out_vertices);
    }
}