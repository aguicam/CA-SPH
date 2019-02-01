#ifndef _PARTICLE_H
#define _PARTICLE_H

#include <glm\glm.hpp>
#include <QMatrix4x4>
#include <math.h>

class Particle
{
public:
	enum class UpdateMethod : std::int8_t { EulerOrig, EulerSemi, Verlet };

	Particle();
	Particle(const float& x, const float& y, const float& z);
//	Particle(QVector3D pos, QVector3D vel, float bouncing = 1.0f, bool fixed = false, int lifetime = -1, QVector3D force = QVector3D(0, 0, 0));
	~Particle();
	//setters
	void setPosition(const float& x, const float& y, const float& z);
    void setPosition(QVector3D pos);
	void setPreviousPosition(const float& x, const float& y, const float& z);
    void setPreviousPosition(QVector3D pos);
	void setVelocity(const float& x, const float& y, const float& z);
    void setVelocity(QVector3D vel);
	void setForce(const float& x, const float& y, const float& z);
    void setForce(QVector3D force);
	void setBouncing(float bouncing);
	void setLifetime(float lifetime);
	void setFixed(bool fixed);


	//getters
    QVector3D getCurrentPosition();
    QVector3D getPreviousPosition();
    QVector3D getForce();
    QVector3D getVelocity();

	float getBouncing();
	float getLifetime();
	bool isFixed();

	//other
    void addForce(QVector3D force);
	void addForce(const float& x, const float& y, const float& z);
	void updateParticle(const float& dt, UpdateMethod method = UpdateMethod::EulerOrig);

    int id=0;
    QVector3D m_acceleration;
    float m_density;
    float m_pressure;

  //  QVector4D color;
private:
    QVector3D m_currentPosition;
    QVector3D m_previousPosition;
    QVector3D m_force;
    QVector3D m_velocity;

	float m_bouncing;
	float m_lifetime;
	bool  m_fixed;

};

#endif
