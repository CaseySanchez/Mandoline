/*
 * Copyright 2020 Casey Sanchez
 */

#pragma once

#include <set>
#include <vector>
#include <initializer_list>
#include <iostream>

#include "graph.hpp"
#include "polygon.hpp"

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
        std::vector<Graph> m_graphs;

    public:
        Merge(std::vector<Graph> const &graphs);
        Merge(std::initializer_list<Graph> const &graphs);

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
        Polygon m_polygon;
        double m_spacing;

    public:
        Slice(Polygon const &polygon, double const &spacing);

        void Compute(Graph &output);

    private:
        void Segment(std::vector<Eigen::Vector2d> &out_vertices, std::vector<Eigen::Array2i> &out_edges);
        void Connect(std::vector<Eigen::Vector2d> &out_vertices, std::vector<Eigen::Array2i> &out_edges);
    };
}