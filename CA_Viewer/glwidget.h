#ifndef GLWIDGET_H
#define GLWIDGET_H


#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include "trianglemesh.h"
#include <QGLShaderProgram>
#include <GeometryP.h>
#include <Particle.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <math.h>
#include <QOpenGLTimerQuery>
#include <wall.h>
#include <field3d.h>

#define GAS_STIFFNESS 3.0 //20.0 // 461.5  // Nm/kg is gas constant of water vapor
#define REST_DENSITY 998.29 // kg/m^3 is rest density of water particle
#define PARTICLE_MASS 0.02 // kg
#define VISCOSITY 3.5 // 5.0 // 0.00089 // Ns/m^2 or Pa*s viscosity of water
#define SURFACE_TENSION 0.0728 // N/m
#define SURFACE_THRESHOLD 7.065
#define KERNEL_PARTICLES 20.0

#define WALL_K 10000.0 // wall spring constant
#define WALL_DAMPING -0.9

#define GRAVITY_ACCELERATION -9.80665
#define BOX_SIZE 0.4


class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{

public:
	GLWidget(QWidget *parent);
	~GLWidget();

	void loadMesh(const QString &filename);
    void loadSphere(const QString &filename);
	void setPolygonMode(bool bFill);
    void changeHomework(int index);
    void changeHomeworkState(int index);


    void setXPosr(float a);
    void setYPosr(float a);
    void setZPosr(float a);


    void initParticleSpring();
    void change_ke(float new_ke);
    void change_kd(float new_kd);
    void change_keB(float new_keB);
    void change_kdB(float new_kdB);
    void RunFluidSimulation(float dt);
    void playPause();
    void restart();

protected:
	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();

	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);

private:
	void setProjection(float aspect);
	void setModelview();
    QMatrix4x4 getModelview();
    void resetParticle(int k);

    void renderParticleSystem();
    void renderParticleSpring();
    void initParticleSystem();
    void collisionSphere(int k, QVector3D prevPos);
    void collisionPlane(int k, QVector3D prevPos);
    void collisionTriangle(int k, QVector3D prevPos);
    void UpdateGrid();
    double Wpoly6(double radiusSquared);
    void Wpoly6Gradient(QVector3D& diffPosition, double radiusSquared, QVector3D& gradient);
    double Wpoly6Laplacian(double radiusSquared);
    void WspikyGradient(QVector3D &diffPosition, double radiusSquared, QVector3D &gradient);
    double WviscosityLaplacian(double radiusSquared);
    void CalculateFluidAcceleration(float dt);



private:
	bool bPolygonFill;
	float angleX, angleY, distance;
    QPoint lastMousePos;


    int homework;
    float initLenghtH;
    float initLenghtV;
    float initDiag;
    int HW2state=0;
    int xParticles;
    int yParticles;

    //QOpenGLShaderProgram *program;
    QGLShaderProgram *program;
	TriangleMesh mesh;

    TriangleMesh room;
    TriangleMesh rightWall;
    TriangleMesh floor;

    bool SemiEuler=true;

    //Particles
    Particle p;
    std::vector<Particle> particlesI;
    std::vector<Particle*> particles;
    int numParticles;
    QVector3D initPPos;


    float dt = 0.03f;
    //Planes
    std::vector<Plane> planes;


    //Spheres
    std::vector<Sphere> spheres;

    std::vector<std::vector<bool>> springForceM;

    std::vector<Wall> _walls;
    Field3D* grid;
    QVector3D gridSize;
    int counter=0;

    double h = 0.0457;

    bool pause;
    bool ri=false;

public slots:
    void resetScene();
    void changeSolver(bool b);

    void changeFountain(bool b);

    QVector3D wallRPos;
    float dir=-1.0f;

};

#endif // GLWIDGET_H
