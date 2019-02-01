//#pragma once
#include "GeometryP.h"
#include <iostream>

//****************************************************
// Plane
//****************************************************

Plane::Plane(const QVector3D& point, const QVector3D& normalVect){
    normal = normalVect.normalized();
    dconst = -QVector3D::dotProduct(point, normal);
}

Plane::Plane(const QVector3D& point0, const QVector3D& point1, const QVector3D& point2){
    QVector3D v1 = point1 - point0;
    QVector3D v2 = point2 - point0;
    normal = (QVector3D::crossProduct(v1,v2) ).normalized();
    dconst = -QVector3D::dotProduct(point0, normal);
}

void Plane::setPosition(const QVector3D& newPos){
    dconst = -QVector3D::dotProduct(newPos, normal);
}
void Plane::setNormal(const QVector3D& newNorm){

    normal=newNorm.normalized();
  //  std::cout<<" The normal is "<<normal.x<< " "<< normal.y<< " " <<normal.z<<" the d is "<<dconst<<std::endl;
}
void Plane::setPosAndNorm(const QVector3D& newPos,const QVector3D& newNorm){
    normal = newNorm.normalized();
    dconst = -QVector3D::dotProduct(newPos, normal);
}


bool Plane::isInside(const QVector3D& point){
	float dist;
    dist = QVector3D::dotProduct(point, normal) + dconst;
	if (dist > 1.e-7)
		return false;
	else
		return true;
}

float Plane::distPoint2Plane(const QVector3D& point){
	float dist;
    return dist = QVector3D::dotProduct(point, normal) + dconst;
}

QVector3D Plane::closestPointInPlane(const QVector3D& point){
    QVector3D closestP;
    float r = (-dconst - QVector3D::dotProduct(point, normal));
	return closestP = point + r*normal;
}

QVector3D Plane::intersecSegment(QVector3D point1,QVector3D point2){
    float r = (-dconst - QVector3D::dotProduct(point1, normal)) / QVector3D::dotProduct((point2 - point1), normal);
    return (1 - r)*point1 + r*point2;
}

QVector3D Plane::intersecPos(QVector3D p1,float bounce){
return p1-(1+bounce)*(QVector3D::dotProduct(p1,normal)+dconst)*normal;
}

QVector3D Plane::intersecVelocity(QVector3D p1, QVector3D v1, float bounce, float isVerlet, float dt){

    if(isVerlet){
       // intersecSegment(p1,intersecPos(p1,bounce))
        return (intersecPos(p1,bounce)-p1)/dt;
    }
    else{
        return v1-(1+bounce)*QVector3D::dotProduct(v1,normal)*normal;
    }
}


//****************************************************
// Triangle
//****************************************************
TriangleP::TriangleP(){}
TriangleP::~TriangleP(){}

TriangleP::TriangleP(const QVector3D& point0, const QVector3D& point1, const QVector3D& point2){
    vertex1=point0;
    vertex2=point1;
    vertex3=point2;
    normal= QVector3D::crossProduct(vertex2-vertex1,vertex3-vertex1).normalized();
    dconst= -QVector3D::dotProduct(point0, normal);
    area= 0.5*QVector3D::crossProduct(vertex2-vertex1,vertex3-vertex2).length();
}


void TriangleP::setNewVertices(const QVector3D& point0, const QVector3D& point1, const QVector3D& point2){
    vertex1=point0;
    vertex2=point1;
    vertex3=point2;
    normal=QVector3D::crossProduct(vertex2-vertex1,vertex3-vertex1).normalized();
    dconst= -QVector3D::dotProduct(point0, normal);
    area= 0.5*QVector3D::crossProduct(vertex2-vertex1,vertex3-vertex2).length();
}

bool TriangleP::isInsideArea(const QVector3D& point){
    float areaT1,areaT2,areaT3,dist;
    areaT1=0.5*QVector3D::crossProduct(vertex2-point,vertex3-vertex2).length();
    areaT2=0.5*QVector3D::crossProduct(point-vertex1,vertex3-point).length();
    areaT3=0.5*QVector3D::crossProduct(vertex2-vertex1,point-vertex2).length();

    dist = areaT1+areaT2+areaT3-area;
    if (dist > 1.e-7)
        return false;
    else
        return true;
}

QVector3D TriangleP::intersecSegmentP(const QVector3D& point1,const QVector3D& point2){
    float r = (-dconst - QVector3D::dotProduct(point1, normal)) / QVector3D::dotProduct((point2 - point1), normal);
    return (1 - r)*point1 + r*point2;
}

//****************************************************
// Sphere
//****************************************************
Sphere::Sphere(){}
Sphere::~Sphere(){}

Sphere::Sphere(const QVector3D& point, const float& radious){
    center=point;
    radi=radious;
}
void Sphere::setPosition(const QVector3D& newPos){
    center=newPos;
}

float Sphere::distPointToCenter(const QVector3D& point){
    return (point-center).length();
}

bool Sphere::isInside(const QVector3D& point){
    if(distPointToCenter(point)>radi)return false;
    else return true;
}

QVector3D Sphere::intersectingPointSphere(const QVector3D& pa,const QVector3D& pn ){
    QVector3D v= pn-pa;
    float a = QVector3D::dotProduct(v,v);
    float b = QVector3D::dotProduct(2.0f*v,pa-center);
    float c = QVector3D::dotProduct(center,center)+QVector3D::dotProduct(pa,pa)-2.0*QVector3D::dotProduct(pa,center)-radi*radi;

    float lambda1 = (-b+sqrt(b*b-4*a*c))/(2*a);
    float lambda2 = (-b-sqrt(b*b-4*a*c))/(2*a);
    if(lambda1>=0 && lambda1<=1){
        return (1-lambda1)*pa+lambda1*pn;
    }
    if(lambda2>=0 && lambda2<=1){
        return (1-lambda2)*pa+lambda2*pn;
    }

}
