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
        Graph m_graph_lhs;
        Graph m_graph_rhs;

    public:
        Merge(Graph const &graph_lhs, Graph const &graph_rhs);

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