#ifndef DEF_BULLET_RIGIDS
	#define	DEF_BULLET_RIGIDS

#include "btBulletDynamicsCommon.h"
#include "vector.h"

struct RigidParams
{
	double spacing;
	double halfExtent;
	double numShellParticles;
};


class BulletFluidWrapper
{
public:


	int m_numShellParticles;

	Vector3DF* m_shellParticles;

	btRigidBody* m_body;

	btDynamicsWorld*		m_dynamicsWorld;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	void InitShell( double radius, double spacing );
	void InitPhysics( RigidParams& params );
	void RunPhysics(double dT);


};


#endif