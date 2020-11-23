#include <string>
#include <iostream>
#include <cairommconfig.h>
#include <cairomm/cairomm.h>
#include <cairomm/context.h>
#include <cairomm/surface.h>
#include <cairomm/fontface.h>
#include <cmath>
#include <chrono>

#include "graph.hpp"
#include "polygon.hpp"
#include "heuristics.hpp"

int main(int argc, char **argv)
{
    auto then = std::chrono::high_resolution_clock::now();

    Mandoline::Polygon polygon1;
    Mandoline::Polygon polygon2;
    Mandoline::Graph graph;
    Mandoline::Polygon padded_polygon;

    Mandoline::Polygon p1 { { -400.0, -400.0 }, { -250.0, 0.0 }, { -400.0, 400.0 }, { 400.0, 400.0 }, { 400.0, -400.0} };
    Mandoline::Polygon p2 { Mandoline::Polygon::Regular(4, 75.0) };
    Mandoline::Polygon p3 { Mandoline::Polygon::Regular(20, 50.0) };

    Eigen::Affine2d root = Eigen::Affine2d::Identity() * Eigen::Translation2d(720, 450);

    Mandoline::Transform(p1, root).Compute(p1);
    Mandoline::Transform(p2, root * Eigen::Translation2d(-125.0, -125.0)).Compute(p2);
    Mandoline::Transform(p3, root * Eigen::Translation2d(150.0, 150.0) * Eigen::Rotation2Dd(3.14159 * 0.5)).Compute(p3);

    Mandoline::Merge({ p1, p2, p3 }).Compute(polygon1);

    Mandoline::Extrude(p1, -30.0).Compute(p1);
    Mandoline::Extrude(p2, 30.0).Compute(p2);
    Mandoline::Extrude(p3, 30.0).Compute(p3);

    Mandoline::Merge({ p1, p2, p3 }).Compute(polygon2);

    Mandoline::Slice(polygon2, 10.0).Compute(graph);

    auto now = std::chrono::high_resolution_clock::now();

    std::cout << "Duration (microseconds): " << std::chrono::duration_cast<std::chrono::microseconds>(now - then).count() << std::endl;

    auto surface = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32, 1440, 900);
    auto context = Cairo::Context::create(surface);

    context->set_source_rgba(1.0, 1.0, 1.0, 1.0);
    context->paint();

    context->set_line_width(5.0);
    context->set_source_rgba(0.0, 0.0, 0.0, 1.0);

    for (auto [vertex_a, vertex_b] : polygon1.Segments()) {
        context->move_to(vertex_a.x(), vertex_a.y());
        context->line_to(vertex_b.x(), vertex_b.y());
        context->stroke();
    }

    context->set_line_width(2.0);
    context->set_source_rgba(0.0, 0.2, 1.0, 1.0);

    for (auto [vertex_a, vertex_b] : polygon2.Segments()) {
        context->move_to(vertex_a.x(), vertex_a.y());
        context->line_to(vertex_b.x(), vertex_b.y());
        context->stroke();
    }

    context->set_line_width(3.0);
    context->set_source_rgba(239.0 / 256.0, 163.0 / 256.0, 31.0 / 256.0, 1.0);

    int i = 0;

    auto font = Cairo::ToyFontFace::create("Bitstream Charter", Cairo::FontSlant::FONT_SLANT_NORMAL, Cairo::FontWeight::FONT_WEIGHT_BOLD);

    for (auto [vertex_a, vertex_b] : graph.Segments()) {
        context->move_to(vertex_a.x(), vertex_a.y());
        context->line_to(vertex_b.x(), vertex_b.y());
        context->stroke();

        Eigen::Vector2d v = (vertex_a + vertex_b) * 0.5 + Eigen::Hyperplane<double, 2>::Through(vertex_a, vertex_b).normal() * 10.0;

        context->move_to(v.x(), v.y());
        context->show_text(std::to_string(++i).c_str());

        std::string filename = "image" + std::to_string(i) + ".png";

        surface->write_to_png(filename.c_str());
    }

    return 0;
}