#include "Particle.h"


Particle::Particle()
{
}

Particle::Particle(const float& x, const float& y, const float& z) :
m_previousPosition(0, 0, 0), m_velocity(0, 0, 0), m_force(0, 0, 0), m_bouncing(1), m_lifetime(50), m_fixed(false)
{
    m_currentPosition= QVector3D(x,y,z);

}

/*
Particle::Particle(QVector3D pos, QVector3D vel, float bouncing, bool fixed, int lifetime, QVector3D force) :
m_currentPosition(pos), m_previousPosition(pos), m_force(force), m_velocity(vel), m_bouncing(bouncing), m_lifetime(lifetime), m_fixed(fixed)
{
}
*/

Particle::~Particle()
{
}

//setters
void Particle::setPosition(const float& x, const float& y, const float& z)
{
    QVector3D pos(x,y,z);
	m_currentPosition =  pos;
}
void Particle::setPosition(QVector3D pos)
{
	m_currentPosition = pos;
}

void Particle::setPreviousPosition(const float& x, const float& y, const float& z)
{
    QVector3D pos(x, y, z);
	m_previousPosition = pos;
}

void Particle::setPreviousPosition(QVector3D pos)
{
	m_previousPosition = pos;
}

void Particle::setForce(const float& x, const float& y, const float& z)
{
    QVector3D force(x, y, z);
	m_force = force;
}

void Particle::setForce(QVector3D force)
{
	m_force = force;
}

void Particle::addForce(const float& x, const float& y, const float& z)
{
    QVector3D force(x,y,z);
	m_force += force;
}

void Particle::addForce(QVector3D force)
{
	m_force += force;
}

void Particle::setVelocity(const float& x, const float& y, const float& z)
{
    QVector3D vel(x,y,z);
	m_velocity = vel;
}

void Particle::setVelocity(QVector3D vel)
{
	m_velocity = vel;
}

void Particle::setBouncing(float bouncing)
{
	m_bouncing = bouncing;
}

void Particle::setLifetime(float lifetime)
{
	m_lifetime = lifetime;
}

void Particle::setFixed(bool fixed)
{
	m_fixed = fixed;
}


//getters

QVector3D Particle::getCurrentPosition()
{
	return m_currentPosition;
}

QVector3D Particle::getPreviousPosition()
{
	return m_previousPosition;
}

QVector3D Particle::getForce()
{
	return m_force;
}

QVector3D Particle::getVelocity()
{
	return m_velocity;
}

float Particle::getBouncing()
{
	return m_bouncing;
}

float Particle::getLifetime()
{
	return m_lifetime;
}

bool Particle::isFixed()
{
	return m_fixed;
}

void Particle::updateParticle(const float& dt, UpdateMethod method)
{
	if (!m_fixed & m_lifetime > 0)
	{
		switch (method)
		{
		case UpdateMethod::EulerOrig:
		{
			m_previousPosition = m_currentPosition;
			m_currentPosition += m_velocity*dt;
			m_velocity += m_force*dt;
		}
			break;
		case UpdateMethod::EulerSemi:
		{
            m_previousPosition = m_currentPosition;
            m_velocity += m_force*dt;
            m_currentPosition += m_velocity*dt;

		}
			break;
		case UpdateMethod::Verlet:
		{
            QVector3D prev=m_currentPosition;
            m_currentPosition += 0.99f*(m_currentPosition-m_previousPosition)+m_force*dt*dt;
            if(std::isnan(m_currentPosition.x())){
                int i=0;
            }
            m_previousPosition =prev;
            m_velocity = (m_currentPosition-m_previousPosition)/dt;

		}
			break;
		}
	}
	return;
}
