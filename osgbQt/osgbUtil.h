#ifndef OSGBUTILS_H
#define OSGBUTILS_H
#include "Utils.h"

enum CollisionTypes {
	COL_GATE = 0x1 << 0,
	COL_WALL = 0x1 << 1,
	COL_DEFAULT = 0x1 << 2,
};

int DefaultCollisionFlags;

unsigned int defaultCollidesWith(COL_GATE | COL_WALL | COL_DEFAULT);

btCollisionWorld* initCollision()
{
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

	btVector3 worldAabbMin(-10000, -10000, -10000);
	btVector3 worldAabbMax(10000, 10000, 10000);
	btBroadphaseInterface* inter = new btAxisSweep3(worldAabbMin, worldAabbMax, 1000);

	btCollisionWorld* collisionWorld = new btCollisionWorld(dispatcher, inter, collisionConfiguration);

	return(collisionWorld);
}


btDynamicsWorld* initPhysics()
{
	btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher * dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

	btVector3 worldAabbMin(-10000, -10000, -10000);
	btVector3 worldAabbMax(10000, 10000, 10000);
	btBroadphaseInterface * inter = new btAxisSweep3(worldAabbMin, worldAabbMax, 1000);

	btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, inter, solver, collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0, 0, 9.8));
	
	return(dynamicsWorld);
}


osg::Geode* osgBox(const osg::Vec3& center, const osg::Vec3& halfLengths)
{
	osg::Vec3 l(halfLengths * 2.);
	osg::Box* box = new osg::Box(center, l.x(), l.y(), l.z());
	osg::ShapeDrawable* shape = new osg::ShapeDrawable(box);
	shape->setColor(osg::Vec4(1., 1., 1., 1.));
	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(shape);
	return(geode);
}


osg::MatrixTransform*  makeControllObj(btDynamicsWorld* bw, btRigidBody* shakeBody, string path, osgbDynamics::MotionState * tshakeMotion)
{
	osg::MatrixTransform* root = new osg::MatrixTransform;
	//const std::string fileName("objs/cup/tea_cup_4.skp/tea_cup_4.obj");
	const std::string fileName(path);
	osg::Node* node = osgDB::readNodeFile(fileName);
	if (node == NULL)
	{
		osg::notify(osg::FATAL) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the osgBullet data directory." << std::endl;
		exit(0);
	}
	root->addChild(node);

	btCollisionShape* cs = osgbCollision::btBoxCollisionShapeFromOSG(node);

	//tshakeMotion = new osgbDynamics::MotionState();
	tshakeMotion->setTransform(root);
	btScalar mass(1.0);
	btVector3 inertia(0, 0, 0);
	btRigidBody::btRigidBodyConstructionInfo rb(mass, tshakeMotion, cs, inertia);
    shakeBody = new btRigidBody(rb);
	//cout << "collisionFlags:" << shakeBody->getCollisionFlags() << endl;
	shakeBody->setCollisionFlags(shakeBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	//shakeBody->setCollisionFlags(shakeBody->getCollisionFlags());// | btCollisionObject::CF_STATIC_OBJECT);
	bw->addRigidBody(shakeBody, COL_DEFAULT, defaultCollidesWith);
	
	//osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
	//cr->_sceneGraph = root;
	//cr->_shapeType = BOX_SHAPE_PROXYTYPE;
	//cr->_mass = 1.f;
	//cr->_restitution = 1.f;
	//btRigidBody* body = osgbDynamics::createRigidBody(cr.get(), cs);
	////body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	//bw->addRigidBody(body);

	
	return(root);
}

osg::MatrixTransform*  makeObj(btDynamicsWorld* bw, string path)
{
	osg::MatrixTransform* root = new osg::MatrixTransform;
	//const std::string fileName("objs/cup/tea_cup_4.skp/tea_cup_4.obj");
	const std::string fileName(path);
	osg::Node* node = osgDB::readNodeFile(fileName);
	if (node == NULL)
	{
		osg::notify(osg::FATAL) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the osgBullet data directory." << std::endl;
		exit(0);
	}
	root->addChild(node);

	btCollisionShape* cs = osgbCollision::btBoxCollisionShapeFromOSG(node);

	osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
	cr->_sceneGraph = root;
	cr->_shapeType = BOX_SHAPE_PROXYTYPE;
	cr->_mass = 1.f;
	cr->_restitution = 1.f;
	btRigidBody* body = osgbDynamics::createRigidBody(cr.get(), cs);
	bw->addRigidBody(body, COL_DEFAULT, defaultCollidesWith);
	//body->setWorldTransform();
	return(root);
}

#endif