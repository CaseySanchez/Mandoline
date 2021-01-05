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
<<<<<<< HEAD
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
=======
    // Applies an affine transformation to the polygon.
    class Transform
    {
        Polygon m_polygon;
        Eigen::Affine2d m_transform;

    public:
        Transform(Polygon const &polygon, Eigen::Affine2d const &transform);

        void Compute(Polygon &output);
    };

    // Merges the polygons contained within the list to a single polygon.
    class Merge
    {
        std::vector<Polygon> m_polygons;

    public:
        Merge(std::vector<Polygon> const &polygons);
        Merge(std::initializer_list<Polygon> const &polygons);

        void Compute(Polygon &output);
    };

    // Extrudes the edges of the polygon by the specified distance.
    class Extrude
    {
        Polygon m_polygon;
        double m_distance;

    public:
        Extrude(Polygon const &polygon, double const &distance);

        void Compute(Polygon &output);
    };

    // Reverses the ordering of vertices in the polygon.
    // This changes edges from { a, b } to { b, a } which effects the edge normals.
    class Invert
    {
        Polygon m_polygon;

    public:
        Invert(Polygon const &polygon);

        void Compute(Polygon &output);
>>>>>>> 852ca8a3d89f18267859ca1be967e5ca0833c2c8
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