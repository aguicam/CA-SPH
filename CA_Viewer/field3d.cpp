#include "field3d.h"

Field3D::Field3D() :
  _xRes(0), _yRes(0), _zRes(0), _data(NULL)
{
}

Field3D::Field3D(int xRes, int yRes, int zRes) :
  _xRes(xRes), _yRes(yRes), _zRes(zRes), _cellCount(xRes*yRes*zRes)
{
  _data = new partVect[_xRes * _yRes * _zRes];
}

Field3D::~Field3D()
{
  if (_data) delete[] _data;
}
