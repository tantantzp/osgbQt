#include "Utils.h"
#include "PickModelHandler.h"

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





/*
int main( int argc,
char * argv[] )
{
btCollisionWorld* collisionWorld = initCollision();


// MoveManipulator* mm = new MoveManipulator;
//mm->setCollisionWorld(collisionWorld);
ref_ptr<PickModelHandler> picker = new PickModelHandler;
picker->setCollisionWorld(collisionWorld);

osg::ref_ptr< osg::Group > root = createScene( collisionWorld, picker.get());

root->addChild(picker->getOrCreateSelectionBox());

osgViewer::Viewer viewer;
viewer.setUpViewInWindow( 10, 30, 800, 600 );
viewer.setCameraManipulator( new osgGA::TrackballManipulator() );
viewer.addEventHandler( picker.get() );
viewer.setSceneData( root.get() );


bool lastColState = false;
while( !viewer.done() )
{
// collisionWorld->performDiscreteCollisionDetection();

// detectCollision( lastColState, collisionWorld );

viewer.frame();
}

return( 0 );
}
*/

