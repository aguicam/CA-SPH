#ifndef FIELD3D_H
#define FIELD3D_H

#include <iostream>
#include <Particle.h>
using namespace std;

class Field3D
{
    typedef vector<Particle*> partVect;

public:
    Field3D();
    Field3D(int xRes, int yRes, int zRes);
     virtual ~Field3D();

    inline partVect& operator()(int x, int y, int z) {
      return _data[x + y*_xRes + z*_xRes*_yRes];
    }

    int xRes() const { return _xRes; }
    int yRes() const { return _yRes; }
    int zRes() const { return _zRes; }
    int cellCount() const { return _cellCount; }
    partVect* data() const { return _data; }

  private:

    int _xRes;
    int _yRes;
    int _zRes;
    int _cellCount;

    partVect* _data;
};

#endif // FIELD3D_H
