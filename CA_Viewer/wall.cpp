#include "wall.h"

#define thickness 0.02


Wall::Wall(const QVector3D& normal, const QVector3D& point) :
  _normal(normal), _point(point)
{
  _normal.normalize();
}
