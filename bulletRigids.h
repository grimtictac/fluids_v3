#ifndef DEF_BULLET_RIGIDS
	#define	DEF_BULLET_RIGIDS

#include "btBulletDynamicsCommon.h"



class BulletFluidWrapper
{
public:

	btRigidBody* m_body;

	btDynamicsWorld*		m_dynamicsWorld;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	void InitPhysics();
	void RunPhysics(double dT);


};


#endif