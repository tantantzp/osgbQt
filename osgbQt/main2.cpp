//#include "Utils.h"
//#include "PickModelHandler.h"
//#include "ShakeManipulator.h"
//#include "osgbUtil.h"
//
//
//
////osg::Group* createScene(btCollisionWorld* cw, PickModelHandler *picker)
////{
////	osg::ref_ptr< osg::Group > root = new osg::Group;
////
////	// Create a static box
////	// osg::Geode* geode = new osg::Geode;
////	// geode->addDrawable( osgwTools::makeBox( osg::Vec3( .5, .5, .5 ) ) );
////
////	ref_ptr<Node> model1 = osgDB::readNodeFile("D:/ProgramLib/objs/chair/chair_3.skp/chair_3.obj");  //;cow.osg");
////	//ref_ptr<Node> model1 = osgDB::readNodeFile("cow.osg");
////	Matrix transMatrix1 = osg::Matrix::translate(-20., 0., 0.);
////	ref_ptr<MatrixTransform> trans1 = new MatrixTransform(transMatrix1);
////	trans1->addChild(model1.get());
////	root->addChild(trans1.get());
////	btCollisionObject* btBoxObject1 = new btCollisionObject;
////
////
////	btBoxObject1->setCollisionShape(osgbCollision::btConvexHullCollisionShapeFromOSG(model1.get()));
////	//btBoxObject1->setCollisionShape(osgbCollision::btBoxCollisionShapeFromOSG(model1.get()));
////	//btBoxObject1->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
////	btBoxObject1->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
////	btBoxObject1->setWorldTransform(osgbCollision::asBtTransform(transMatrix1));
////	cw->addCollisionObject(btBoxObject1);
////
////	picker->insertObjPair(trans1.get(), btBoxObject1);
////
////	// Create a box we can drag around with the mouse
////	//geode = new osg::Geode;
////	//geode->addDrawable( osgwTools::makeBox( osg::Vec3( .5, .5, .5 ) ) );
////	ref_ptr<Node> model2 = osgDB::readNodeFile("D:/ProgramLib/objs/chair/chair_7.skp/chair_7.obj"); 
////	//ref_ptr<Node> model2 = osgDB::readNodeFile("cow.osg");
////	Matrix transMatrix2 = osg::Matrix::translate(20, 0., 0.);
////	ref_ptr<MatrixTransform> trans2 = new MatrixTransform(transMatrix2);
////	trans2->addChild(model2.get());
////	root->addChild(trans2.get());
////
////	btCollisionObject* btBoxObject2 = new btCollisionObject;
////
////
////	btBoxObject2->setCollisionShape(osgbCollision::btConvexHullCollisionShapeFromOSG(model2.get()));
////	//btBoxObject2->setCollisionShape( osgbCollision::btBoxCollisionShapeFromOSG( model2.get()));
////	btBoxObject2->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
////	btBoxObject2->setWorldTransform(osgbCollision::asBtTransform(transMatrix2));
////	cw->addCollisionObject(btBoxObject2);
////
////	picker->insertObjPair(trans2.get(), btBoxObject2);
////	//btCollisionObjectArray btObjArray = cw->getCollisionObjectArray();
////	// mm->setCollisionObject( btBoxObject );
////	//mm->setMatrixTransform( trans2.get() );
////
////	return(root.release());
////}
//
//class ViewerWidget : public QWidget
//{
//public:
//	ViewerWidget() : QWidget()
//	{
//		osgQt::GraphicsWindowQt* gw = createGraphicsWindow(0, 0, 500, 500);
//		if (gw)
//		{
//			cout << "Valid GraphicsWindowQt" << endl;
//			QVBoxLayout* layout = new QVBoxLayout;
//			layout->addWidget(gw->getGLWidget());
//			setLayout(layout);
//		}
//
//		//_collisionWorld = initCollision();
//		//_picker = new PickModelHandler;	
//		//_picker->setCollisionWorld(_collisionWorld)
//		//_root = createScene(_collisionWorld, _picker);
//		//_root->addChild(_picker->getOrCreateSelectionBox());
//
//		_bulletWorld = initPhysics();
//		_root = new osg::Group;
//
//		int dieNum = 2;
//
//
//		for (int i = 0; i < dieNum; i++) {
//			MatrixTransform *tmp = makeObj(_bulletWorld, "dice.osg");
//			_root->addChild(tmp);
//
//
//		}
//		string objPath = "objs/cup/tea_cup_4.skp/tea_cup_4.obj";
//		_root->addChild(makeObj(_bulletWorld, objPath));
//
//		const osg::Vec4 plane(0., 0., -200., 0.);
//
//		_root->addChild(osgbDynamics::generateGroundPlane(plane, _bulletWorld,
//			NULL, COL_DEFAULT, defaultCollidesWith));
//
//		/* BEGIN: Create environment boxes */
//		float xDim(10.);
//		float yDim(10.);
//		float zDim(10.);
//		float thick(.1);
//
//		osg::MatrixTransform* shakeBox = new osg::MatrixTransform;
//		btCompoundShape* cs = new btCompoundShape;
//		{ // floor -Z (far back of the shake cube)
//			osg::Vec3 halfLengths(xDim*.5, yDim*.5, thick*.5);
//			osg::Vec3 center(0., 0., zDim*.5);
//			shakeBox->addChild(osgBox(center, halfLengths));
//			btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
//			btTransform trans; trans.setIdentity();
//			trans.setOrigin(osgbCollision::asBtVector3(center));
//			cs->addChildShape(trans, box);
//		}
//		{ // top +Z (invisible, to allow user to see through; no OSG analogue
//			osg::Vec3 halfLengths(xDim*.5, yDim*.5, thick*.5);
//			osg::Vec3 center(0., 0., -zDim*.5);
//			//shakeBox->addChild( osgBox( center, halfLengths ) );
//			btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
//			btTransform trans; trans.setIdentity();
//			trans.setOrigin(osgbCollision::asBtVector3(center));
//			cs->addChildShape(trans, box);
//		}
//		{ // left -X
//			osg::Vec3 halfLengths(thick*.5, yDim*.5, zDim*.5);
//			osg::Vec3 center(-xDim*.5, 0., 0.);
//			shakeBox->addChild(osgBox(center, halfLengths));
//			btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
//			btTransform trans; trans.setIdentity();
//			trans.setOrigin(osgbCollision::asBtVector3(center));
//			cs->addChildShape(trans, box);
//		}
//		{ // right +X
//			osg::Vec3 halfLengths(thick*.5, yDim*.5, zDim*.5);
//			osg::Vec3 center(xDim*.5, 0., 0.);
//			shakeBox->addChild(osgBox(center, halfLengths));
//			btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
//			btTransform trans; trans.setIdentity();
//			trans.setOrigin(osgbCollision::asBtVector3(center));
//			cs->addChildShape(trans, box);
//		}
//		{ // bottom of window -Y
//			osg::Vec3 halfLengths(xDim*.5, thick*.5, zDim*.5);
//			osg::Vec3 center(0., -yDim*.5, 0.);
//			shakeBox->addChild(osgBox(center, halfLengths));
//			btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
//			btTransform trans; trans.setIdentity();
//			trans.setOrigin(osgbCollision::asBtVector3(center));
//			cs->addChildShape(trans, box);
//		}
//		{ // bottom of window -Y
//			osg::Vec3 halfLengths(xDim*.5, thick*.5, zDim*.5);
//			osg::Vec3 center(0., yDim*.5, 0.);
//			shakeBox->addChild(osgBox(center, halfLengths));
//			btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
//			btTransform trans; trans.setIdentity();
//			trans.setOrigin(osgbCollision::asBtVector3(center));
//			cs->addChildShape(trans, box);
//		}
//		/* END: Create environment boxes */
//
//		osgbDynamics::MotionState * shakeMotion = new osgbDynamics::MotionState();
//		shakeMotion->setTransform(shakeBox);
//		btScalar mass(1.0);
//		btVector3 inertia(0, 0, 0);
//		btRigidBody::btRigidBodyConstructionInfo rb(mass, shakeMotion, cs, inertia);
//		btRigidBody* shakeBody = new btRigidBody(rb);
//
//		DefaultCollisionFlags = shakeBody->getCollisionFlags();
//
//		shakeBody->setCollisionFlags(shakeBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
//		//shakeBody->setCollisionFlags(shakeBody->getCollisionFlags() | btCollisionObject::CF_CHARACTER_OBJECT);// | btCollisionObject::CF_STATIC_OBJECT);
//		shakeBody->setActivationState(DISABLE_DEACTIVATION);
//
//
//
//		_bulletWorld->addRigidBody(shakeBody, COL_DEFAULT, defaultCollidesWith);
//		_root->addChild(shakeBox);
//
//		osgbDynamics::MotionState * tshakeMotion = new osgbDynamics::MotionState();
//		btRigidBody* rigidBody;
//		MatrixTransform * tTrans;
//		tTrans = makeControllObj(_bulletWorld, rigidBody, "dice.osg", tshakeMotion);
//		_root->addChild(tTrans);
//		osg::Camera* camera = _viewer.getCamera();
//		camera->setGraphicsContext(gw);
//
//		const osg::GraphicsContext::Traits* traits = gw->getTraits();
//
//		camera->setClearColor(osg::Vec4(0.2, 0.2, 0.6, 1.0));
//		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
//		camera->setProjectionMatrixAsPerspective(40., 1., 1., 50.);
//
//		_viewer.setSceneData(_root);
//		_viewer.addEventHandler(new ShakeManipulator(shakeMotion, _bulletWorld, rigidBody, _root, tTrans));
//		//_viewer.addEventHandler(new osgbInteraction::DragHandler(
//		//	_bulletWorld, _viewer.getCamera()));
//
//		//_viewer.addEventHandler(_picker);
//		osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
//		tb->setHomePosition(osg::Vec3(0, 0, -20), osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0));
//		_viewer.setCameraManipulator(tb);
//
//		// Use single thread here to avoid known issues under Linux
//		_viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
//
//		connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));
//		_timer.start(10);
//	}
//	~ViewerWidget()
//	{
//		delete _bulletWorld;
//		//delete _collisionWorld;
//		//delete _picker;
//	}
//
//	osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name = "", bool windowDecoration = false)
//	{
//		osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
//		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
//		traits->windowName = name;
//		traits->windowDecoration = windowDecoration;
//		traits->x = x;
//		traits->y = y;
//		traits->width = w;
//		traits->height = h;
//		traits->doubleBuffer = true;
//		traits->alpha = ds->getMinimumNumAlphaBits();
//		traits->stencil = ds->getMinimumNumStencilBits();
//		traits->sampleBuffers = ds->getMultiSamples();
//		traits->samples = ds->getNumMultiSamples();
//
//		return new osgQt::GraphicsWindowQt(traits.get());
//	}
//
//protected:
//	virtual void paintEvent(QPaintEvent* event)
//	{
//		const double currSimTime = _viewer.getFrameStamp()->getSimulationTime();
//		double elapsed(currSimTime - prevSimTime);
//		if (_viewer.getFrameStamp()->getFrameNumber() < 3)
//			elapsed = 1. / 180.;
//		//osg::notify( osg::ALWAYS ) << elapsed / 3. << ", " << 1./180. << std::endl;
//		_bulletWorld->stepSimulation(elapsed, 3, elapsed / 3.);
//		prevSimTime = currSimTime;
//
//		_viewer.frame();
//	}
//
//	osgViewer::Viewer _viewer;
//	QTimer _timer;
//	//btCollisionWorld* _collisionWorld;
//	btDynamicsWorld* _bulletWorld;
//	//PickModelHandler* _picker;
//	Group* _root;
//
//	double prevSimTime = 0.;
//};
//
//
////int main(int argc, char** argv)
////{
////	QApplication app(argc, argv);
////
////	osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::SingleThreaded;
////	ViewerWidget* widget = new ViewerWidget();
////
////	widget->setGeometry(100, 100, 800, 600);
////	widget->show();
////	return app.exec();
////}
