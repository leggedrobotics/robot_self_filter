#include <cmath>

#include <tf2/LinearMath/Vector3.h>

#include "robot_self_filter/bodies.h"
#include "robot_self_filter/shapes.h"

int main()
{
  robot_self_filter::shapes::Sphere shape(1.0);
  robot_self_filter::bodies::Sphere body(&shape);
  body.setPadding(0.25);
  return body.containsPoint(tf2::Vector3(1.2, 0.0, 0.0)) &&
         !body.containsPoint(tf2::Vector3(1.3, 0.0, 0.0)) ? 0 : 1;
}
