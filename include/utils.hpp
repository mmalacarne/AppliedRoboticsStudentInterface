#ifndef PATH_H
#define PATH_H

#include <vector>
#include <cmath>
#include <cstddef>
// A configuration of the robot along the path, represented by x, y, orientation and curvature
struct Pose 
{
  float s, x, y, theta, kappa;

  Pose(float s, float x, float y, float theta, float kappa):
    s(s), x(x), y(y), theta(theta), kappa(kappa)
  {}

  Pose(): 
    Pose(0, 0, 0, 0, 0)
  {}

  float distance(float _x, float _y)
  {
    return std::hypot(x-_x, y-_y);
  }
};

// A sequence of sampled robot configurations composing a (discretization of the) path
struct Path 
{
  std::vector<Pose> points;
  
  Path(std::vector<Pose> const & points):
    points(points)
  {}

  Path()
  {}
  
  bool empty() { return points.empty(); }
  size_t size() { return points.size(); }
  void setPoints(const std::vector<Pose>& points) { this->points = points; }
};

struct Point 
{
  float x, y;

  Point(float x, float y):
    x(x), y(y)
  {}

  Point(): 
    Point(0, 0)
  {}

  // Operator overload in order to use this struct/type in std::map
  bool operator<(const Point& p) const
  {
    double x = static_cast<double>(this->x);
    double y = static_cast<double>(this->y);

    double px = static_cast<double>(p.x);
    double py = static_cast<double>(p.y);

    double distance = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    double distnace_p = std::sqrt(std::pow(px, 2) + std::pow(py, 2));

    return (distance < distnace_p);
  }

  // Operator overload for comparison
  bool operator==(const Point& p) const
  {
    return (this->x == p.x && this->y == p.y);
  }

  bool operator!=(const Point& p) const
  {
    return (this->x != p.x || this->y != p.y);
  }

};


typedef std::vector<Point> Polygon;

void getBaricenter(const Polygon& obj, Point& baricenter);

void getBaricenter(const Polygon& obj, double& x, double& y);


#endif 
