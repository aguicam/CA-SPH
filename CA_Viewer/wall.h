#ifndef WALL_H
#define WALL_H
#include <glm/glm.hpp>
#include <QVector3D>

class Wall
{
public:
    Wall(const QVector3D& normal, const QVector3D& point);


    QVector3D _normal;
    QVector3D _point;
};

#endif // WALL_H
