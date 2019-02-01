#include "glwidget.h"
#include <iostream>
#include <QApplication>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <plyreader.h>
#include <random>

#define rand01() ((float)std::rand()/RAND_MAX)


using namespace std;




const float rotationFactor = 0.5f;
const float maxRotationCamera = 75.0f;
const float minDistanceCamera = 1.0f;
const float maxDistanceCamera = 50.0f;


GLWidget::GLWidget(QWidget *parent) : QOpenGLWidget(parent), bPolygonFill(true), angleX(45.0f), angleY(60.0f), distance(2.0f)
{
    program = NULL;
}

GLWidget::~GLWidget()
{
    if(program) delete program;
}


void GLWidget::initializeGL()
{
    initializeOpenGLFunctions();

    program = new QGLShaderProgram();
    program->addShaderFromSourceFile(QGLShader::Vertex, ":/shaders/simpleshader.vert");
    program->addShaderFromSourceFile(QGLShader::Fragment, ":/shaders/simpleshader.frag");

    program->link();
    if(!program->isLinked())
    {
        cout << "Shader program has not linked" << endl << endl << "Log: " << endl << endl << program->log().toStdString();
        QApplication::quit();
    }
    program->bind();

    mesh.buildCube();
    room.buildRoom();
    floor.buildFloor();
    rightWall.buildRight();


    if(!mesh.init(program))
    {
        cout << "Could not create vbo" << endl;
        QApplication::quit();
    }
    if(!room.init(program))
    {
        cout << "Could not create vbo" << endl;
        QApplication::quit();
    }
    if(!floor.init(program))
    {
        cout << "Could not create vbo" << endl;
        QApplication::quit();
    }
    if(!rightWall.init(program))
    {
        cout << "Could not create vbo" << endl;
        QApplication::quit();
    }

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);


    loadSphere(QString("sphere.ply"));

    initParticleSystem();
}

void GLWidget::resizeGL(int w, int h)
{
    glViewport(0,0,w,h);
    setProjection((float)w/h);
    setModelview();
}

void GLWidget::paintGL()
{

        renderParticleSystem();


    update();
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    lastMousePos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    // Rotation
    if(event->buttons() & Qt::LeftButton)
    {
        angleX += rotationFactor * (event->y() - lastMousePos.y());
        angleX = max(-maxRotationCamera, min(angleX, maxRotationCamera));
        angleY += rotationFactor * (event->x() - lastMousePos.x());
    }
    // Zoom
    if(event->buttons() & Qt::RightButton)
    {
        distance += 0.01f * (event->y() - lastMousePos.y());
        distance = max(minDistanceCamera, min(distance, maxDistanceCamera));
    }

    lastMousePos = event->pos();

    makeCurrent();
    setModelview();
    doneCurrent();
    update();
}

void GLWidget::setProjection(float aspect)
{
    QMatrix4x4 projectionMatrix;

    projectionMatrix.perspective(60, aspect, 0.01, 100.0);
    program->bind();
    program->setUniformValue("projection", projectionMatrix);
    program->release();
}

void GLWidget::setModelview()
{
    QMatrix4x4 modelviewMatrix;

    modelviewMatrix.translate(0, 0, -distance);
    modelviewMatrix.rotate(angleX, 1.0f, 0.0f, 0.0f);
    modelviewMatrix.rotate(angleY, 0.0f, 1.0f, 0.0f);
    program->bind();
    program->setUniformValue("modelview", modelviewMatrix);
    program->setUniformValue("normalMatrix", modelviewMatrix.normalMatrix());
    program->release();
}

QMatrix4x4 GLWidget::getModelview()
{
    QMatrix4x4 modelviewMatrix;

    modelviewMatrix.translate(0, 0, -distance);
    modelviewMatrix.rotate(angleX, 1.0f, 0.0f, 0.0f);
    modelviewMatrix.rotate(angleY, 0.0f, 1.0f, 0.0f);
    //	program->bind();
    //	program->setUniformValue("modelview", modelviewMatrix);
    //	program->setUniformValue("normalMatrix", modelviewMatrix.normalMatrix());
    //	program->release();
    return modelviewMatrix;

}

void GLWidget::setPolygonMode(bool bFill)
{
    bPolygonFill = bFill;

    makeCurrent();
    if(bFill)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    doneCurrent();
    update();
}

void GLWidget::loadMesh(const QString &filename)
{
    PLYReader reader;

    mesh.destroy();
    reader.readMesh(filename, mesh);
    makeCurrent();
    if(!mesh.init(program))
    {
        cout << "Could not create vbo" << endl;
        QApplication::quit();
    }
    doneCurrent();
    update();
}
void GLWidget::loadSphere(const QString &filename)
{
    PLYReader reader;

    mesh.destroy();
    reader.readMesh(filename, mesh);
    makeCurrent();
    if(!mesh.init(program))
    {
        cout << "Could not create vbo" << endl;
        QApplication::quit();
    }
    doneCurrent();
}




void GLWidget::resetScene(){
    particles.clear();
    // p.setPosition(QVector3D(0.0f,15.0f,0.0f));
    //  p.setVelocity(QVector3D(0.0f,0.0f,0.0f));
    // update();
}
void GLWidget::changeSolver(bool b){
    SemiEuler=b;
}


//RENDERING FUNCTIONS
void GLWidget::renderParticleSystem(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program->bind();
    program->setUniformValue("bLighting", bPolygonFill);
    program->setUniformValue("color", QVector4D(1, 0.0, 0.0, 1.0));

    QMatrix4x4 model = getModelview();
    model = getModelview();


    program->setUniformValue("color", QVector4D(0.8, 0.8, 0.8, 1.0));
    //QMatrix4x4 model = getModelview();
    model = getModelview();

//    model.scale(1);
//    program->setUniformValue("modelview", model);
//    program->setUniformValue("normalMatrix", model.normalMatrix());
//    room.render(*this);


    //RENDER PARTICLES

    for(int k= 0;k<(int)particles.size();k++){

        model = getModelview();
        QVector3D currentParticlePos = particles[k]->getCurrentPosition();

        model.translate(currentParticlePos.x(),currentParticlePos.y()+gridSize.y()/2.0,currentParticlePos.z());
        model.scale(0.05);//0.05
        program->setUniformValue("modelview", model);
        program->setUniformValue("normalMatrix", model.normalMatrix());
        program->setUniformValue("color", QVector4D(0.8,0.7,0.1, 1.0));
        mesh.render(*this);

    }

    setModelview();
    if(!pause){
        RunFluidSimulation(0.01);
    }


    program->release();


    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    program->bind();
    program->setUniformValue("bLighting", bPolygonFill);
    model = getModelview();
    program->setUniformValue("color", QVector4D(0.8, 0.8, 0.8, 1.0));

    model.scale(1.5,3,1.5);
    program->setUniformValue("modelview", model);
    program->setUniformValue("normalMatrix", model.normalMatrix());
    room.render(*this);

    model = getModelview();
    model.scale(1.5,3,1.5);
    model.translate(_walls[3]._point-QVector3D(gridSize.x()/2.0,0.0,0.0));
    program->setUniformValue("modelview", model);
    program->setUniformValue("normalMatrix", model.normalMatrix());
    rightWall.render(*this);
    program->release();



    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);




}

//INITIALIZING FUNCTIONS
void GLWidget::initParticleSystem(){
    particles.clear();
    particlesI.clear();
    spheres.clear();
    planes.clear();
    numParticles = 500;
    Particle ps(0.0f, 8.0f, 0.0f);
    p=ps;
    p.addForce(0, -9.8f, 0);
    for(int i = 0; i < numParticles; i++){
        p.setPosition(QVector3D(0.05*i,0.0,-3.0));
        particlesI.push_back(p);
    }


    gridSize = QVector3D (1.5*BOX_SIZE, BOX_SIZE, 1.5*BOX_SIZE);

    int gridXRes = (int)ceil(gridSize.x()/h);
    int gridYRes = (int)ceil(gridSize.y()/h);
    int gridZRes = (int)ceil(gridSize.z()/h);


    _walls.clear();

    _walls.push_back(Wall(QVector3D(0,0,1), QVector3D(0,0,-gridSize.z()/2.0))); // back
    _walls.push_back(Wall(QVector3D(0,0,-1), QVector3D(0,0,gridSize.z()/2.0))); // front
    _walls.push_back(Wall(QVector3D(1,0,0), QVector3D(-gridSize.x()/2.0,0,0)));     // left
    _walls.push_back(Wall(QVector3D(-1,0,0), QVector3D(gridSize.x()/2.0,0,0)));     // right
    _walls.push_back(Wall(QVector3D(0,1,0), QVector3D(0,-gridSize.y()/2.0,0))); // bottom

    wallRPos=QVector3D(-gridSize.x()/2.0,0,0);

    grid = new Field3D(gridXRes, gridYRes, gridZRes);

    vector<Particle*>& firstGridCell = (*grid)(0,0,0);

    int count = 0;
    for (double y = -gridSize.y()/2.0; y < gridSize.y()/2.0; y+= h/2.0) {
      for (double x = -gridSize.x()/2.0; x < -gridSize.x()/4.0; x += h/2.0) {
        for (double z = -gridSize.z()/2.0; z < gridSize.z()/2.0; z+= h/2.0) {
            Particle* ps = new  Particle(x, y, z);
            ps->id = count;
            ps->setPreviousPosition(ps->getCurrentPosition());
            particles.push_back(ps);
            count++;
            firstGridCell.push_back(ps);
        }
      }
    }

    UpdateGrid();



}

void GLWidget::playPause(){
    pause=!pause;
    cout<<"changed pause to: "<<pause<<endl;
}
void GLWidget::restart(){
    initParticleSystem();
}

void GLWidget::UpdateGrid(){
    for (unsigned int x = 0; x < (*grid).xRes(); x++) {
        for (unsigned int y = 0; y < (*grid).yRes(); y++) {
          for (unsigned int z = 0; z < (*grid).zRes(); z++) {

            vector<Particle*>& particles = (*grid)(x,y,z);

            for (int p = 0; p < particles.size(); p++) {
                Particle* particle = particles[p];

                double nx = (particle->getCurrentPosition().x()+ BOX_SIZE/2.0)/h + 0.0000001;
                double ny = (particle->getCurrentPosition().y()+ BOX_SIZE/2.0)/h + 0.0000001;
                double nz = (particle->getCurrentPosition().z()+ BOX_SIZE/2.0)/h + 0.0000001;

                int newGridCellX = std::floor(nx);
                int newGridCellY = std::floor(ny);
                int newGridCellZ = std::floor(nz);

                if (newGridCellX < 0)
                    newGridCellX = 0;
                else if (newGridCellX >= (*grid).xRes())
                    newGridCellX = (*grid).xRes() - 1;
                if (newGridCellY < 0)
                    newGridCellY = 0;
                else if (newGridCellY >= (*grid).yRes())
                    newGridCellY = (*grid).yRes() - 1;
                if (newGridCellZ < 0)
                    newGridCellZ = 0;
                else if (newGridCellZ >= (*grid).zRes())
                    newGridCellZ = (*grid).zRes() - 1;

                if (x != newGridCellX || y != newGridCellY || z != newGridCellZ) {
                    (*grid)(newGridCellX, newGridCellY, newGridCellZ).push_back(particle);
                    particles[p] = particles.back();
                    particles.pop_back();
                    p--;
                }
            }
          }
        }
    }
}
double GLWidget::Wpoly6(double radiusSquared) {

  static double coefficient = 315.0/(64.0*M_PI*pow(h,9));
  static double hSquared = h*h;

  return coefficient * pow(hSquared-radiusSquared, 3);
}

void GLWidget::Wpoly6Gradient(QVector3D &diffPosition, double radiusSquared, QVector3D &gradient) {

  static double coefficient = -945.0/(32.0*M_PI*pow(h,9));
  static double hSquared = h*h;

  gradient = coefficient * pow(hSquared-radiusSquared, 2) * diffPosition;
}

double GLWidget::Wpoly6Laplacian(double radiusSquared) {

  static double coefficient = -945.0/(32.0*M_PI*pow(h,9));
  static double hSquared = h*h;

  return coefficient * (hSquared-radiusSquared) * (3.0*hSquared - 7.0*radiusSquared);
}

void GLWidget::WspikyGradient(QVector3D& diffPosition, double radiusSquared, QVector3D& gradient) {  //

  static double coefficient = -45.0/(M_PI*pow(h,6));

  double radius = sqrt(radiusSquared);

  gradient = coefficient * pow(h-radius, 2) * diffPosition / radius;
}


double GLWidget::WviscosityLaplacian(double radiusSquared) {

  static double coefficient = 45.0/(M_PI*pow(h,6));

  double radius = sqrt(radiusSquared);

  return coefficient * (h - radius);
}

void GLWidget::RunFluidSimulation(float dt){
   if(grid == nullptr) return;
   counter++;

   if(counter>100){
  //     cout<<counter<<" Wall pos: "<<_walls[3]._point.x()<<" "<<_walls[3]._point.y()<<" "<<_walls[3]._point.z()<<endl;
   _walls[3]._point +=dir*QVector3D(0.01,0.0,0.0);
  // cout<<counter<<" "<<dir<<" Wall pos: "<<_walls[3]._point.x()<<" "<<_walls[3]._point.y()<<" "<<_walls[3]._point.z()<<endl;
   }
   QVector3D w=_walls[3]._point;
   //if(w.x()>gridSize.x()/2.0){counter=0;}
   if(w.x()>gridSize.x()/2.0){
       _walls[3]._point +=-QVector3D(0.01,0.0,0.0);
       dir=-1.0f;
       counter=0;
   }
   if(w.x()<=0.12){
       dir=1.0f;
   }
 //   cout<<counter<<" Wall pos: "<<_walls[3]._point.x()<<" "<<_walls[3]._point.y()<<" "<<_walls[3]._point.z()<<endl;

    CalculateFluidAcceleration(dt);

    for (unsigned int gridCellIndex = 0; gridCellIndex < (*grid).cellCount(); gridCellIndex++) {

        vector<Particle*>& particles = (*grid).data()[gridCellIndex];

        for (unsigned int p = 0; p < particles.size(); p++) {

          Particle* particle = particles[p];

          QVector3D newPosition = particle->getCurrentPosition() + particle->getVelocity()*dt + particle->m_acceleration*dt*dt;
          QVector3D newVelocity = (newPosition - particle->getCurrentPosition()) / dt;

          particle->setPreviousPosition(particle->getCurrentPosition());
          particle->setPosition(newPosition);
          particle->setVelocity(newVelocity);

        }
      }


      UpdateGrid();

}
void GLWidget::CalculateFluidAcceleration(float dt){
    //Update particle density and preasure
    for (int x = 0; x < (*grid).xRes(); x++) {
        for (int y = 0; y < (*grid).yRes(); y++) {
            for (int z = 0; z < (*grid).zRes(); z++) {

                vector<Particle*>& particles = (*grid)(x,y,z);

                for (int p = 0; p < particles.size(); p++) {
                    Particle* particle = particles[p];
                    particle->m_density = 0;

                    //Neighborhoods iterations!
                    for (int offsetX = -1; offsetX <= 1; offsetX++) {
                        if (x+offsetX < 0) continue;
                        if (x+offsetX >= (*grid).xRes()) break;

                        for (int offsetY = -1; offsetY <= 1; offsetY++) {
                            if ( y+offsetY < 0) continue;
                            if ( y+offsetY >= (*grid).yRes()) break;


                            for (int offsetZ = -1; offsetZ <= 1; offsetZ++) {
                                if ( z+offsetZ < 0) continue;
                                if ( z+offsetZ >= (*grid).zRes()) break;

                                vector<Particle*>& neighborGridCellParticles = (*grid)(x+offsetX, y+offsetY, z+offsetZ);

                                for (int i = 0; i < neighborGridCellParticles.size(); i++) {

                                    Particle* neighborParticle = neighborGridCellParticles[i];

                                    QVector3D diffPosition = particle->getCurrentPosition() - neighborParticle->getCurrentPosition();

                                    double radiusSquared = QVector3D::dotProduct(diffPosition, diffPosition);

                                    if (radiusSquared < h*h){
                                        particle->m_density += Wpoly6(radiusSquared);
                                    }
                                }
                            }
                        }
                    }

                    particle->m_density *= PARTICLE_MASS;
                    particle->m_pressure = GAS_STIFFNESS * (particle->m_density - REST_DENSITY);
                }
            }
        }
    }

    // COMPUTE FORCES FOR ALL PARTICLES
    for (int x = 0; x < (*grid).xRes(); x++) {
        for (int y = 0; y < (*grid).yRes(); y++) {
            for (int z = 0; z < (*grid).zRes(); z++) {

                vector<Particle*>& particles = (*grid)(x,y,z);

                for (int p = 0; p < particles.size(); p++) {
                    Particle* particle = particles[p];

                    QVector3D f_pressure, f_viscosity, f_surface,
                              colorFieldNormal;

                    QVector3D f_gravity=  QVector3D(0.0,-9.8,0.0) * particle->m_density;


                    double colorFieldLaplacian;

                    for (int offsetX = -1; offsetX <= 1; offsetX++) {
                        if (x+offsetX < 0) continue;
                        if (x+offsetX >= (*grid).xRes()) break;

                        for (int offsetY = -1; offsetY <= 1; offsetY++) {
                            if (y+offsetY < 0) continue;
                            if (y+offsetY >= (*grid).yRes()) break;

                            for (int offsetZ = -1; offsetZ <= 1; offsetZ++) {
                                if (z+offsetZ < 0) continue;
                                if (z+offsetZ >= (*grid).zRes()) break;

                                vector<Particle*>& neighborGridCellParticles = (*grid)(x+offsetX, y+offsetY, z+offsetZ);

                                for (int i = 0; i < neighborGridCellParticles.size(); i++) {

                                    Particle* neighborParticle = neighborGridCellParticles[i];

                                    QVector3D diffPosition = particle->getCurrentPosition() - neighborParticle->getCurrentPosition();

                                    double radiusSquared = QVector3D::dotProduct(diffPosition, diffPosition);

                                    if (radiusSquared < h*h){
                                        QVector3D poly6Gradient, spikyGradient;

                                        Wpoly6Gradient(diffPosition, radiusSquared, poly6Gradient);

                                        WspikyGradient(diffPosition, radiusSquared, spikyGradient);

                                        if (particle->id != neighborParticle->id) {
                                          f_pressure += (particle->m_pressure/pow(particle->m_density,2)+neighborParticle->m_pressure/pow(neighborParticle->m_density,2))*spikyGradient;

                                          f_viscosity += (neighborParticle->getVelocity() - particle->getVelocity()) * WviscosityLaplacian(radiusSquared) / neighborParticle->m_density;

                                        }


                                        colorFieldNormal += poly6Gradient / neighborParticle->m_density;

                                        colorFieldLaplacian += Wpoly6Laplacian(radiusSquared) / neighborParticle->m_density;
                                    }

                                }
                            }
                        }
                    }

                    f_pressure *= -PARTICLE_MASS * particle->m_density;

                    f_viscosity *= VISCOSITY * PARTICLE_MASS;

                    colorFieldNormal *= PARTICLE_MASS;


                    //particle.normal = -1.0 * colorFieldNormal;

                    colorFieldLaplacian *= PARTICLE_MASS;


                    // surface tension force

                    double colorFieldNormalMagnitude = colorFieldNormal.length();

                    if (colorFieldNormalMagnitude > SURFACE_THRESHOLD) {

                        //particle.flag() = true;
                        f_surface = -SURFACE_TENSION * colorFieldNormal / colorFieldNormalMagnitude * colorFieldLaplacian;

                    }


                    particle->m_acceleration = (f_pressure + f_viscosity + f_surface + f_gravity) / particle->m_density;


                    //fixme wall collision!!!

                    for (unsigned int i = 0; i < _walls.size(); i++) {

                        Wall& wall = _walls[i];

                        double d = QVector3D::dotProduct(wall._point - particle->getCurrentPosition(), wall._normal) + 0.01; // particle radius

                        if (d > 0.0) {

                          particle->m_acceleration += WALL_K * wall._normal * d;
                          particle->m_acceleration += WALL_DAMPING * QVector3D::dotProduct(particle->getVelocity(), wall._normal) * wall._normal;
                        }
                      }
                }
            }
        }
    }

}
