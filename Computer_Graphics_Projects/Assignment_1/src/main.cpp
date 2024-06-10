////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <string>
#include <array>
#include <deque>
#include <sstream>
#include <stdio.h>
// Shortcut to avoid  everywhere, DO NOT USE IN .h
using namespace Eigen;
////////////////////////////////////////////////////////////////////////////////

const std::string root_path = DATA_DIR;

// Computes the determinant of the matrix whose columns are the vector u and v
double inline det(const Vector2d &u, const Vector2d &v)
{
    // TODO
    Matrix2d m;
    m << u, v;
    return m.determinant();
}

// Return true iff [a,b] intersects [c,d]
bool intersect_segment(const Vector2d &a, const Vector2d &b, const Vector2d &c, const Vector2d &d)
{
    // TODO
    // vector u = b-a and v = d-c, x= b-c, y = a-c, 
    //if det(u,x)>0 && det(u,y)>0 || det(u,x)<0 && det(u,y)<0
    //then no intersection return false otherwise return true
    Vector2d u = b - a;
    Vector2d v = d - c;
    Vector2d w = a - c;

    double det_u_v = det(u, v);
    double det_u_w = det(u, w);
    double det_v_w = det(v, w);

    if (det_u_v == 0) {
        if (det_u_w == 0 || det_v_w == 0) {

            double t0 = w.dot(u) / u.dot(u);
            double t1 = w.dot(v) / v.dot(v);
            return (t0 >= 0 && t0 <= 1) || (t1 >= 0 && t1 <= 1);
        } else {
            return false; 
        }
    } else {
        double t = det_v_w / det_u_v;
        double s = det_u_w / det_u_v;
        return (t >= 0 && t <= 1) && (s >= 0 && s <= 1);
    }
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const std::vector<Vector2d> &poly, const Vector2d &query)
{
    // 1. skip for now: Compute bounding box and set coordinate of a point outside the polygon
    // TODO
    //Vector2d outside(800, 800);
    double min_x = poly[0].x(), min_y = poly[0].y();
    double max_x = poly[0].x(), max_y = poly[0].y();

    for (const auto &p : poly) {
        min_x = std::min(min_x, p.x());
        min_y = std::min(min_y, p.y());
        max_x = std::max(max_x, p.x());
        max_y = std::max(max_y, p.y());
    }

    Vector2d outside(max_x + 1, min_y); // A point outside the polygon

    // 2. Cast a ray from the query point to the 'outside' point, count number of intersections
    // TODO
    //if num of intersections even is even then the point is outside otherwise it is inside
    int intersections = 0;
    for (size_t i = 0; i < poly.size(); i++) {
        size_t j = (i + 1) % poly.size();
        if (intersect_segment(poly[i], poly[j], query, outside)) {
            intersections++;
        }
    }
    return intersections % 2 != 0;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Vector2d> load_xyz(const std::string &filename)
{
   std::vector<Vector2d> points;
    std::ifstream in(filename);

    if (in.is_open()) {
        std::string line;
        while (std::getline(in, line)) {
            std::istringstream iss(line);
            double x, y;
            if (iss >> x >> y) {
                Vector2d point(x, y);
                points.push_back(point);
            }
        }
        in.close();
    } else {
        std::cout << "error" << std::endl;
    }

    return points;
}

void save_xyz(const std::string &filename, const std::vector<Vector2d> &points)
{
    // TODO
    std::ofstream file(filename);

    if (file.is_open()) {
        for (const auto &point : points) {
            file << point.x() << " " << point.y() << " 0\n";
        }
        file.close();
    } else {
        std::cout << "error" << std::endl;
    }

}

std::vector<Vector2d> load_obj(const std::string &filename)
{
    std::ifstream in(filename);
    std::vector<Vector2d> points;
    std::vector<Vector2d> poly;
    char key;
    while (in >> key)
    {
        if (key == 'v')
        {
            double x, y, z;
            in >> x >> y >> z;
            points.push_back(Vector2d(x, y));
        }
        else if (key == 'f')
        {
            std::string line;
            std::getline(in, line);
            std::istringstream ss(line);
            int id;
            while (ss >> id)
            {
                poly.push_back(points[id - 1]);
            }
        }
    }
    return poly;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    const std::string points_path = root_path + "/points.xyz";
    const std::string poly_path = root_path + "/polygon.obj";
    const std::string output_path = root_path + "/output.xyz";
    std::vector<Vector2d> points = load_xyz(points_path);
    // for (const auto& point : points) {
    // std::cout << "Point: (" << point.x() << ", " << point.y() << ")" << std::endl;   }

    ////////////////////////////////////////////////////////////////////////////////
    //Point in polygon
    std::vector<Vector2d> poly = load_obj(poly_path);
    std::vector<Vector2d> result;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (is_inside(poly, points[i]))
        {
            result.push_back(points[i]);
        }
    }
    save_xyz("output.xyz", result);    

    return 0;
}
