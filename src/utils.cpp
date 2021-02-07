#include "utils.hpp"

/*!
* Retrieves the baricenter of the provided polygon.
* @param[in]  	obj     	Polygon with unknown baricenter.
* @param[out] 	baricenter  Baricenter point.
*/
void getBaricenter(const Polygon& obj, Point& baricenter){
  double b_x, b_y;

  for (const auto& pt: obj){
    b_x += static_cast<double>(pt.x);
    b_y += static_cast<double>(pt.y);
  }

  b_x /= static_cast<double>(obj.size());
  b_y /= static_cast<double>(obj.size());

  baricenter = Point(static_cast<float>(b_x), static_cast<float>(b_y));
}

/*!
* Retrieves the baricenter of the provided polygon.
* @param[in]  	obj     Polygon with unknown baricenter.
* @param[out] 	x 		Baricenter x coord.
* @param[out] 	y  		Baricenter y coord.
*/
void getBaricenter(const Polygon& obj, double& x, double& y){
  x = 0.;
  y = 0.;

  for (const auto& pt: obj){
    x += static_cast<double>(pt.x);
    y += static_cast<double>(pt.y);
  }

  x /= static_cast<double>(obj.size());
  y /= static_cast<double>(obj.size());
}