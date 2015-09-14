//! This library implements a wrapper for the poly2tri C++ implementation
//! of the constrained Delaunay triangulation.
extern crate libc;

use std::mem;
use libc::{c_void, c_int, size_t};


#[link(name="poly2tri")]
extern "C" {
    fn p2t_polyline_new() -> *mut c_void;
    fn p2t_polyline_free(polygon: *mut c_void);
    fn p2t_polyline_add_point(polygon: *mut c_void, x: f64, y: f64);
    fn p2t_polyline_get_point(polygon: *mut c_void, idx: size_t,
                              x_out: *mut f64, y_out: *mut f64) -> c_int;
    fn p2t_polyline_size(polygon: *const c_void) -> size_t;

    fn p2t_cdt_new(polygon: *mut c_void) -> *mut c_void;
    fn p2t_cdt_free(cdt: *mut c_void);
    fn p2t_cdt_triangulate(cdt: *mut c_void);
    fn p2t_cdt_get_triangles(cdt: *mut c_void) -> *mut c_void;
    fn p2t_cdt_add_hole(cdt: *mut c_void, polygon: *mut c_void);
    fn p2t_cdt_add_point(cdt: *mut c_void, x: f64, y: f64);

    fn p2t_triangles_free(triangles: *mut c_void);
    fn p2t_triangles_count(triangles: *mut c_void) -> size_t;
    fn p2t_triangles_get_triangle(triangles: *const c_void, idx: size_t) -> *const c_void;

    fn p2t_triangle_get_point(triangle: *const c_void, idx: size_t,
                              x_out: *mut f64, y_out: *mut f64);
}

/// Basic polygon interface
pub struct Polygon {
    ll: *mut c_void,
}

/// The Constrained Delaunay triangulation state
pub struct CDT {
    ll: *mut c_void,
}

/// An immutable vector of triangles
pub struct TriangleVec {
    ll: *mut c_void,

    // hold on to the cdt until we no longer need the triangle vector
    #[allow(dead_code)]
    cdt: CDT,
}

/// A single triangle
#[derive(Copy, Clone, PartialEq)]
pub struct Triangle {
    pub points: [[f64; 2]; 3],
}


/// This struct implements a polygon.  Note that this is currently
/// a wrapper around a c library and as such it's bubbles through
/// some of the underlying implementation details such as panicking
/// on duplicated points.
impl Polygon {

    /// Creates a new empty polygon.
    pub fn new() -> Polygon {
        unsafe {
            Polygon {
                ll: p2t_polyline_new()
            }
        }
    }

    /// Creates a polygon from an iterator of points.
    pub fn from_iterator<'a, I>(points: I) -> Polygon
        where I: Iterator<Item=&'a [f64; 2]>
    {
        let mut rv = Polygon::new();
        for point in points {
            rv.add_point(point[0], point[1]);
        }
        rv
    }

    /// Adds a single point to the polygon.  These points must not
    /// be repeated!
    pub fn add_point(&mut self, x: f64, y: f64) {
        unsafe {
            p2t_polyline_add_point(self.ll, x, y);
        }
    }

    /// Looks up a point at a certain index.
    pub fn get_point(&self, idx: usize) -> [f64; 2] {
        unsafe {
            let mut x: f64 = 0.0;
            let mut y: f64 = 0.0;
            if p2t_polyline_get_point(self.ll, idx as size_t, &mut x, &mut y) == 0 {
                [x, y]
            } else {
                panic!("Out of range");
            }
        }
    }

    /// Returns the size of the polygon (number of points).
    pub fn size(&self) -> usize {
        unsafe {
            p2t_polyline_size(self.ll) as usize
        }
    }
}

impl Drop for Polygon {

    fn drop(&mut self) {
        unsafe {
            p2t_polyline_free(self.ll);
        }
    }
}


/// This holds the basic state of the constrained Delaunay triangulation
/// algorithm.  Once the triangulation was kicked off the triangles are
/// returned and this no longer has a purpose.
impl CDT {

    /// Creates a new constrained Delaunay triangulation from a polygon.
    pub fn new(polygon: Polygon) -> CDT {
        unsafe {
            let rv = CDT {
                ll: p2t_cdt_new(polygon.ll)
            };
            mem::forget(polygon);
            rv
        }
    }

    /// Adds a hole into the CDT.
    pub fn add_hole(&mut self, polygon: Polygon) {
        unsafe {
            p2t_cdt_add_hole(self.ll, polygon.ll);
            mem::forget(polygon);
        }
    }

    /// Adds a steiner point to the CDT.
    pub fn add_steiner_point(&mut self, x: f64, y: f64) {
        unsafe {
            p2t_cdt_add_point(self.ll, x, y);
        }
    }

    /// Triangulates the polygon.
    pub fn triangulate(self) -> TriangleVec {
        unsafe {
            p2t_cdt_triangulate(self.ll);
            let ll = p2t_cdt_get_triangles(self.ll);
            TriangleVec {
                cdt: self,
                ll: ll,
            }
        }
    }
}

impl Drop for CDT {

    fn drop(&mut self) {
        unsafe {
            p2t_cdt_free(self.ll);
        }
    }
}


/// Implements a read only interface to a triangle vector.
impl TriangleVec {

    /// The number of triangles.
    pub fn size(&self) -> usize {
        unsafe {
            p2t_triangles_count(self.ll) as usize
        }
    }

    /// Returns a copy of the triangle at a certain point.
    pub fn get_triangle(&self, idx: usize) -> Triangle {
        assert!(idx < self.size(), "Out of range");
        let mut p0 = [0.0; 2];
        let mut p1 = [0.0; 2];
        let mut p2 = [0.0; 2];
        unsafe {
            let tri = p2t_triangles_get_triangle(self.ll, idx as size_t);
            p2t_triangle_get_point(tri, 0, &mut p0[0], &mut p0[1]);
            p2t_triangle_get_point(tri, 1, &mut p1[0], &mut p1[1]);
            p2t_triangle_get_point(tri, 2, &mut p2[0], &mut p2[1]);
        }
        Triangle {
            points: [p0, p1, p2]
        }
    }
}

impl Drop for TriangleVec {

    fn drop(&mut self) {
        unsafe {
            p2t_triangles_free(self.ll);
        }
    }
}

/// Triangulates an iterator of points.  This is a shortcut
/// for the most common operation.
pub fn triangulate_points<'a, I>(points: I) -> TriangleVec
    where I: Iterator<Item=&'a [f64; 2]>
{
    CDT::new(Polygon::from_iterator(points)).triangulate()
}


#[test]
fn test_basics() {
    let mut polygon = Polygon::new();
    let min = 0.01f64;
    let max = 2.50f64;
    polygon.add_point(min, min);
    polygon.add_point(min, max);
    polygon.add_point(max, max);
    polygon.add_point(max, min);
    assert_eq!(polygon.size(), 4);

    let cdt = CDT::new(polygon);
    let triangles = cdt.triangulate();
    assert_eq!(triangles.size(), 2);

    let tri = triangles.get_triangle(0);
    assert_eq!(tri.points[0], [min, max]);
    assert_eq!(tri.points[1], [max, min]);
    assert_eq!(tri.points[2], [max, max]);

    let tri = triangles.get_triangle(1);
    assert_eq!(tri.points[0], [min, max]);
    assert_eq!(tri.points[1], [min, min]);
    assert_eq!(tri.points[2], [max, min]);
}

#[test]
fn test_iter() {
    let points : [[f64; 2]; 4] = [[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]];
    let triangles = triangulate_points(points.into_iter());
    assert_eq!(triangles.size(), 2);
}
