#ifndef _GEOMETRYP_H
#define _GEOMETRYP_H

#include <QVector3D>
#include <cmath>

struct GeometryP{
    virtual void setPosition(const QVector3D& newPos) = 0;
    virtual bool isInside(const QVector3D& point) = 0;
};

struct Plane : public GeometryP {
    QVector3D normal;
	float dconst;
	Plane(){};
	~Plane() {};
    Plane(const QVector3D& point, const QVector3D& normalVect);
    Plane(const QVector3D& point0, const QVector3D& point1, const QVector3D& point2);

    void setPosition(const QVector3D& newPos);
    void setNormal(const QVector3D& newNorm);
    bool isInside(const QVector3D& point);
    float distPoint2Plane(const QVector3D& point);
    QVector3D closestPointInPlane(const QVector3D& point);
    QVector3D intersecSegment(QVector3D point1, QVector3D point2);
    QVector3D intersecPos(QVector3D p1,float bounce);
    QVector3D intersecVelocity(QVector3D p1, QVector3D v1, float bounce, float isVerlet, float dt);
    void setPosAndNorm(const QVector3D& newPos,const QVector3D& newNorm);
};	

struct TriangleP : public Plane {
    QVector3D vertex1, vertex2, vertex3;
    float area;
    TriangleP();
    virtual ~TriangleP();
    TriangleP(const QVector3D& point0, const QVector3D& point1, const QVector3D& point2);
    void setNewVertices(const QVector3D& point0, const QVector3D& point1, const QVector3D& point2);
    bool isInsideArea(const QVector3D& point);
    bool intersecSegment(const QVector3D& point1, const QVector3D& point2, QVector3D& pTall);
    QVector3D intersecSegmentP(const QVector3D& point1,const QVector3D& point2);
};

struct Sphere : public GeometryP {
    QVector3D center;
    float radi;
    Sphere();
    virtual ~Sphere();
    Sphere(const QVector3D& point, const float& radious);
    void setPosition(const QVector3D& newPos);
    bool isInside(const QVector3D& point);
    bool intersecSegment(const QVector3D& point1, const QVector3D& point2, QVector3D& pTall);
    float distPointToCenter(const QVector3D& point);
    QVector3D intersectingPointSphere(const QVector3D& pa,const QVector3D& pn );
};

#endif
