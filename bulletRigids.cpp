#include "bulletRigids.h"

#define START_POS_X -0
#define START_POS_Y -15
#define START_POS_Z -0

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.


void BulletFluidWrapper::InitPhysics()
{
	//setTexturing(true);
	//setShadows(true);

	//setCameraDistance(btScalar(SCALING*50.));

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
	
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		float start_x = 0; // START_POS_X - ARRAY_SIZE_X/2;
		float start_y = 15; //START_POS_Y;
		float start_z = 0; //START_POS_Z - ARRAY_SIZE_Z/2;

		//for (int k=0;k<ARRAY_SIZE_Y;k++)
		//{
		//	for (int i=0;i<ARRAY_SIZE_X;i++)
		//	{
		//		for(int j = 0;j<ARRAY_SIZE_Z;j++)
		//		{
		//			startTransform.setOrigin(SCALING*btVector3(
		//								btScalar(2.0*i + start_x),
		//								btScalar(20+2.0*k + start_y),
		//								btScalar(2.0*j + start_z)));


					startTransform.setOrigin(SCALING*btVector3(
										btScalar(start_x),
										btScalar(start_y),
										btScalar(start_z)));


					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					m_body = new btRigidBody(rbInfo);

					m_dynamicsWorld->addRigidBody(m_body);
	//			}
	//		}
	//	}
	}

}


void BulletFluidWrapper::RunPhysics(double dT)
{
	//m_body->applyCentralForce(btVector3(10,10,20));
	m_body->applyForce(btVector3(0,1,0),btVector3(0,0,1) );

	m_dynamicsWorld->stepSimulation( dT );
}