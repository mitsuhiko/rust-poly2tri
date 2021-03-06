#include <vector>
#include <poly2tri.h>

// include poly2tri here
#include <common/shapes.cc>
#include <sweep/advancing_front.cc>
#include <sweep/cdt.cc>
#include <sweep/sweep.cc>
#include <sweep/sweep_context.cc>


extern "C" {

    void *p2t_polyline_new()
    {
        return reinterpret_cast<void *>(new std::vector<p2t::Point *>);
    }

    void p2t_polyline_free(void *polyline)
    {
        std::vector<p2t::Point *> *vec =
            reinterpret_cast<std::vector<p2t::Point *> *>(polyline);
        delete vec;
    }

    void p2t_polyline_add_point(void *polyline, double x, double y)
    {
        std::vector<p2t::Point *> *vec =
            reinterpret_cast<std::vector<p2t::Point *> *>(polyline);
        vec->push_back(new p2t::Point(x, y));
    }

    int p2t_polyline_get_point(
        void *polyline, size_t idx, double *x_out, double *y_out)
    {
        std::vector<p2t::Point *> *vec =
            reinterpret_cast<std::vector<p2t::Point *> *>(polyline);
        if (idx < vec->size()) {
            p2t::Point *p = (*vec)[idx];
            *x_out = p->x;
            *y_out = p->y;
            return 0;
        } else {
            return 1;
        }
    }

    size_t p2t_polyline_size(const void *polyline)
    {
        const std::vector<p2t::Point *> *vec =
            reinterpret_cast<const std::vector<p2t::Point *> *>(polyline);
        return vec->size();
    }

    void *p2t_cdt_new(void *polyline)
    {
        std::vector<p2t::Point *> *vec =
            reinterpret_cast<std::vector<p2t::Point *> *>(polyline);
        p2t::CDT *cdt = new p2t::CDT(*vec);
        delete vec;
        return reinterpret_cast<void *>(cdt);
    }

    void p2t_cdt_free(void *cdt)
    {
        p2t::CDT *c = reinterpret_cast<p2t::CDT *>(cdt);
        delete c;
    }

    void p2t_cdt_add_hole(void *cdt, void *polyline)
    {
        std::vector<p2t::Point *> *vec =
            reinterpret_cast<std::vector<p2t::Point *> *>(polyline);
        p2t::CDT *c = reinterpret_cast<p2t::CDT *>(cdt);
        c->AddHole(*vec);
        delete vec;
    }

    void p2t_cdt_add_point(void *cdt, double x, double y)
    {
        p2t::CDT *c = reinterpret_cast<p2t::CDT *>(cdt);
        c->AddPoint(new p2t::Point(x, y));
    }

    void p2t_cdt_triangulate(void *cdt)
    {
        p2t::CDT *c = reinterpret_cast<p2t::CDT *>(cdt);
        c->Triangulate();
    }

    void *p2t_cdt_get_triangles(void *cdt)
    {
        p2t::CDT *c = reinterpret_cast<p2t::CDT *>(cdt);
        auto vec = new std::vector<p2t::Triangle *>(c->GetTriangles());
        auto first_p = (*vec)[0];
        return reinterpret_cast<void *>(vec);
    }

    size_t p2t_triangles_count(void *triangles)
    {
        std::vector<p2t::Triangle *> *t =
            reinterpret_cast<std::vector<p2t::Triangle *> *>(triangles);
        return t->size();
    }

    void p2t_triangles_free(void *triangles)
    {
        std::vector<p2t::Triangle *> *t =
            reinterpret_cast<std::vector<p2t::Triangle *> *>(triangles);
        delete t;
    }

    const void *p2t_triangles_get_triangle(
        void *triangles, size_t idx)
    {
        std::vector<p2t::Triangle *> *t =
            reinterpret_cast<std::vector<p2t::Triangle *> *>(triangles);
        return reinterpret_cast<void *>((*t)[idx]);
    }

    void p2t_triangle_get_point(
        const void *triangle, size_t idx, double *x_out, double *y_out)
    {
        p2t::Triangle *t = (p2t::Triangle *)triangle;
        const p2t::Point *point = t->GetPoint(idx);
        *x_out = point->x;
        *y_out = point->y;
    }
}
