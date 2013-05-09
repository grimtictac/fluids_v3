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


void BulletFluidWrapper::InitPhysics( RigidParams& params )
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

		btScalar	mass(0.1f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		float start_x = 0; // START_POS_X - ARRAY_SIZE_X/2;
		float start_y = 45; //START_POS_Y;
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
	}

	InitShell( params.halfExtent, params.spacing );

	params.numShellParticles = m_numShellParticles;
}

void BulletFluidWrapper::InitShell( double halfExtent, double spacing )
{
/*
	double extent = halfExtent * 2;

	spacing/=25;


	int halfWidthParticleCount = int(floor(extent/spacing));

	m_numShellParticles = pow( double(halfWidthParticleCount) + 1, 3 );
	m_shellParticles = new Vector3DF[m_numShellParticles];


	Vector3DF pos;
	int n, p;
	float dx, dy, dz, x, y, z;
	int cntx, cnty, cntz;
	cntx = ceil( extent / spacing );
	cntz = ceil( extent / spacing );
	int cnt = cntx * cntz;
	int xp, yp, zp, c2;
	float odd;
		

	dx = extent;
	dy = extent;
	dz = extent;
	
	c2 = cnt/2;

	int i=0;

	for (float y = -halfExtent; y <= halfExtent; y += spacing ) {	
		for (int xz=0; xz < cnt; xz++ ) {
			
			x = -halfExtent + (xz % int(cntx))*spacing;
			z = -halfExtent + (xz / int(cntx))*spacing;

			m_shellParticles[i++] = Vector3DF( x , y , z );
		}
	}

*/

	//int halfWidthParticleCount = int(floor(halfExtent/spacing));

	//m_numShellParticles = pow( double(2*halfWidthParticleCount) + 1, 3 );
	//m_shellParticles = new Vector3DF[m_numShellParticles];

	//int i=0;

	//for(double x=-halfExtent; x<halfExtent; x+=spacing)	
	//	for(double y=-halfExtent; y<halfExtent; y+=spacing)
	//		for(double z=-halfExtent; z<halfExtent; z+=spacing) m_shellParticles[i++] = Vector3DF( x , y , z );		
	//

	m_numShellParticles = 100;
    m_shellParticles = new Vector3DF[m_numShellParticles];


	for(int i=0; i<m_numShellParticles; i++)
	{
		float x = (float(rand()%1000)/40000)-10; //sin(float(i)) +  
		float y = (float(rand()%1000)/30000)-10;
		float z = (float(rand()%1000)/20000)-10; //cos(float(i)) + 


		m_shellParticles[i] = Vector3DF( x , y , z );
	}
}

void BulletFluidWrapper::RunPhysics(double dT)
{
	//m_body->applyCentralForce(btVector3(10,10,20));
	
	static float bouy = 0;
	bouy += 0.01;
	
	//m_body->applyForce(btVector3(0,0, 2),btVector3(1,0,0) );
	//m_body->applyForce(btVector3(0,0,-2),btVector3( 0.1,0,0) );

	m_dynamicsWorld->stepSimulation( dT );
}