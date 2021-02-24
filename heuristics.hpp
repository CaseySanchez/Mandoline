/*
 * Copyright 2020 Casey Sanchez
 */

#pragma once

#include <set>
#include <vector>
#include <initializer_list>
#include <iostream>

#include "graph.hpp"

namespace Mandoline
{
    // Generates edges of a path from vertices
    // e.g. Given vertices 0, 1, 2, the following edges are created:
    // { 0, 1 }, { 1, 2 }
    class Path
    {
        std::vector<Eigen::Vector2d> m_vertices;

    public:
        Path(std::vector<Eigen::Vector2d> const &vertices);
        Path(std::initializer_list<Eigen::Vector2d> const &vertices);

        void Compute(Graph &output);
    };

    // Generates edges of a polygon from vertices
    // e.g. Given vertices 0, 1, 2, the following edges are created:
    // { 0, 1 }, { 1, 2 }, { 2, 0 }
    class Polygon
    {
        std::vector<Eigen::Vector2d> m_vertices;

    public:
        Polygon(std::vector<Eigen::Vector2d> const &vertices);
        Polygon(std::initializer_list<Eigen::Vector2d> const &vertices);

        void Compute(Graph &output);
    };

    // Generates a regular n-sided polygon
    class Regular
    {
        uint32_t m_sides;
        double m_radius;

    public:
        Regular(uint32_t const &sides, double const &radius);

        void Compute(Graph &output);
    };

    class Bezier
    {
        std::vector<Eigen::Vector2d> m_vertices;
        double m_bezier_gain;

    public:
        Bezier(std::vector<Eigen::Vector2d> const &vertices, double const &bezier_gain = 1.0);

        void Compute(Graph &output);

    private:
        // TODO: Max of 20!, need a more efficient method for calculating binomial coefficients
        uint64_t Factorial(uint64_t const &n);
    };

    // Applies an affine transformation to the graph.
    class Transform
    {
        Graph m_graph;
        Eigen::Affine2d m_transform;

    public:
        Transform(Graph const &graph, Eigen::Affine2d const &transform);

        void Compute(Graph &output);
    };

    // Merges the graphs contained within the list to a single graph.
    class Merge
    {
        std::vector<Graph> m_graphs;

    public:
        Merge(Graph const &graph_lhs, Graph const &graph_rhs);
        Merge(std::initializer_list<Graph> const &graphs);
        Merge(std::vector<Graph> const &graphs);

        void Compute(Graph &output);
    };

    // Reverses the ordering of vertices in the graph.
    // This changes edges from { a, b } to { b, a } which effects the edge normals.
    class Invert
    {
        Graph m_graph;

    public:
        Invert(Graph const &graph);

        void Compute(Graph &output);
    };

    // Extrudes the edges of the graph by the specified distance.
    class Extrude
    {
        Graph m_graph;
        double m_distance;

    public:
        Extrude(Graph const &graph, double const &distance);

        void Compute(Graph &output);
    };

    class Slice
    {
        Graph m_graph;
        double m_spacing;

    public:
        Slice(Graph const &graph, double const &spacing);

        void Compute(Graph &output);

    private:
        void Segment(std::vector<Eigen::Vector2d> &output_vertices, std::vector<Eigen::Array2i> &output_edges);
        void Connect(std::vector<Eigen::Vector2d> &output_vertices, std::vector<Eigen::Array2i> &output_edges);
    };

    // Creates vertices & edges at the intersections of the two graphs
    class Fragment
    {
        Graph m_graph_lhs;
        Graph m_graph_rhs;

    public:
        Fragment(Graph const &graph_lhs, Graph const &graph_rhs);
        
        void Compute(Graph &output);
    };

    // Removes edges where the midpoint falls within the polygon
    class Difference
    {
        Graph m_graph_lhs;
        Graph m_graph_rhs;

    public:
        Difference(Graph const &graph_lhs, Graph const &graph_rhs);
        
        void Compute(Graph &output);
    };

    // Unionizes two polygons
    class Union
    {
        Graph m_graph_lhs;
        Graph m_graph_rhs;
        
    public:
        Union(Graph const &graph_lhs, Graph const &graph_rhs);

        void Compute(Graph &output);
    };
}