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

    // Creates a graph given an ordered vector of vertices
    // Edges are generated as pairs of vertex indices
    // V = { v0, v1, v2 }
    // E = { { 0, 1 }, { 1, 2 } } if loop = false
    //   = { { 0, 1 }, { 1, 2 }, { 2, 0 } } if loop = true
    Graph::Graph(std::vector<Eigen::Vector2d> const &vertices, bool const &loop) : m_vertices(vertices)
    {
        for (uint32_t index = 0; index < vertices.size() - 1; index++) {
            m_edges.emplace_back(index, index + 1);
        }

        if (loop) {
            m_edges.emplace_back(vertices.size() - 1, 0);
        }
    }

    Graph::Graph(std::initializer_list<Eigen::Vector2d> const &vertices, bool const &loop) : Graph(std::vector<Eigen::Vector2d>{ vertices }, loop)
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
}