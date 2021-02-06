#include <string>
#include <iostream>
#include <cmath>
#include <chrono>

#include <cairomm/cairomm.h>
#include <cairomm/context.h>
#include <cairomm/surface.h>
#include <cairomm/fontface.h>

#include "graph.hpp"
#include "heuristics.hpp"

int main(int argc, char **argv)
{
    Mandoline::Graph polygon1 { Mandoline::Graph::Regular(20, 200.0) };
    Mandoline::Graph polygon2 { Mandoline::Graph::Regular(20, 200.0) };
    Mandoline::Graph polygon3 { Mandoline::Graph::Regular(20, 200.0) };

    Eigen::Affine2d root = Eigen::Affine2d::Identity() * Eigen::Translation2d(720, 450);

    Mandoline::Transform(polygon1, root * Eigen::Translation2d(100.0, 0.0)).Compute(polygon1);
    Mandoline::Transform(polygon2, root * Eigen::Translation2d(-150.0, -150.0)).Compute(polygon2);
    Mandoline::Transform(polygon3, root * Eigen::Translation2d(-100.0, 100.0)).Compute(polygon3);

    Mandoline::Graph polygon_union;

    Mandoline::Union(polygon1, polygon2).Compute(polygon_union);
    Mandoline::Union(polygon_union, polygon3).Compute(polygon_union);

    Mandoline::Graph polygon_slice;

    Mandoline::Slice(polygon_union, 20.0).Compute(polygon_slice);
    
    auto surface = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32, 1440, 900);
    auto context = Cairo::Context::create(surface);
    auto font = Cairo::ToyFontFace::create("Bitstream Charter", Cairo::FontSlant::FONT_SLANT_NORMAL, Cairo::FontWeight::FONT_WEIGHT_BOLD);

    context->set_source_rgba(1.0, 1.0, 1.0, 1.0);
    context->paint();

    context->set_line_width(5.0);

    int i = 0;

    for (auto [vertex_a, vertex_b] : polygon_slice.Segments()) {
        context->set_source_rgba(1.0, 0.0, 0.0, 1.0);
        context->move_to(vertex_a.x(), vertex_a.y());
        context->line_to(vertex_b.x(), vertex_b.y());
        context->stroke();

        Eigen::Vector2d v = (vertex_a + vertex_b) * 0.5 + Eigen::Hyperplane<double, 2>::Through(vertex_a, vertex_b).normal() * 10.0;

        context->set_source_rgba(0.0, 0.0, 1.0, 1.0);
        context->move_to(v.x(), v.y());
        context->show_text(std::to_string(++i).c_str());
    }

    surface->write_to_png("slice.png");
    
    return 0;
}
