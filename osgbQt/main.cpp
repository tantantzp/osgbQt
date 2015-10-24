#include "Utils.h"
#include "PickModelHandler.h"
//#include "ShakeManipulator.h"
#include "osgbUtil.h"



void  createScene(Group* root, btCollisionWorld* cw, PickModelHandler *picker)
{
	// Create a static box
	// osg::Geode* geode = new osg::Geode;
	// geode->addDrawable( osgwTools::makeBox( osg::Vec3( .5, .5, .5 ) ) );

	ref_ptr<Node> model1 = osgDB::readNodeFile("D:/ProgramLib/objs/chair/chair_3.skp/chair_3.obj");  //;cow.osg");
	//ref_ptr<Node> model1 = osgDB::readNodeFile("cow.osg");
	Matrix transMatrix1 = osg::Matrix::translate(-20., 0., 0.);
	ref_ptr<MatrixTransform> trans1 = new MatrixTransform(transMatrix1);
	trans1->addChild(model1.get());
	root->addChild(trans1.get());
	btCollisionObject* btBoxObject1 = new btCollisionObject;


	btBoxObject1->setCollisionShape(osgbCollision::btConvexHullCollisionShapeFromOSG(model1.get()));
	//btBoxObject1->setCollisionShape(osgbCollision::btBoxCollisionShapeFromOSG(model1.get()));
	//btBoxObject1->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
	btBoxObject1->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
	btBoxObject1->setWorldTransform(osgbCollision::asBtTransform(transMatrix1));
	cw->addCollisionObject(btBoxObject1);

	picker->insertObjPair(trans1.get(), btBoxObject1);

	// Create a box we can drag around with the mouse
	//geode = new osg::Geode;
	//geode->addDrawable( osgwTools::makeBox( osg::Vec3( .5, .5, .5 ) ) );
	ref_ptr<Node> model2 = osgDB::readNodeFile("D:/ProgramLib/objs/chair/chair_7.skp/chair_7.obj"); 
	//ref_ptr<Node> model2 = osgDB::readNodeFile("cow.osg");
	Matrix transMatrix2 = osg::Matrix::translate(20, 0., 0.);
	ref_ptr<MatrixTransform> trans2 = new MatrixTransform(transMatrix2);
	trans2->addChild(model2.get());
	root->addChild(trans2.get());

	btCollisionObject* btBoxObject2 = new btCollisionObject;


	btBoxObject2->setCollisionShape(osgbCollision::btConvexHullCollisionShapeFromOSG(model2.get()));
	//btBoxObject2->setCollisionShape( osgbCollision::btBoxCollisionShapeFromOSG( model2.get()));
	btBoxObject2->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
	btBoxObject2->setWorldTransform(osgbCollision::asBtTransform(transMatrix2));
	cw->addCollisionObject(btBoxObject2);

	picker->insertObjPair(trans2.get(), btBoxObject2);
	//btCollisionObjectArray btObjArray = cw->getCollisionObjectArray();
	// mm->setCollisionObject( btBoxObject );
	//mm->setMatrixTransform( trans2.get() );

}

class MyViewerWidget : public QWidget
{
public:
	MyViewerWidget() : QWidget()
	{
		osgQt::GraphicsWindowQt* gw = createGraphicsWindow(0, 0, 500, 500);
		if (gw)
		{
			cout << "Valid GraphicsWindowQt" << endl;
			QVBoxLayout* layout = new QVBoxLayout;
			layout->addWidget(gw->getGLWidget());
			setLayout(layout);
		}

		_collisionWorld = initCollision();
		_root = new osg::Group;
		cout << "before picker" << endl;
		_picker = new PickModelHandler(_collisionWorld, _root);
		cout << "after picker" << endl;
		
		createScene(_root, _collisionWorld, _picker);		



		_viewer.setSceneData(_root);		
	
		//*BEGIN: set camera*/
		osg::Camera* camera = _viewer.getCamera();
		const osg::GraphicsContext::Traits* traits = gw->getTraits();
		camera->setGraphicsContext(gw);
		camera->setClearColor(osg::Vec4(0.2, 0.2, 0.6, 1.0));
		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		camera->setProjectionMatrixAsPerspective(40., 1., 1., 50.);
		//*END: set camera */


		_viewer.addEventHandler(_picker);
		osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
		tb->setHomePosition(osg::Vec3(0, 0, -150), osg::Vec3(0, 0, -100), osg::Vec3(0, -1, 0));
		_viewer.setCameraManipulator(tb);
		
		// Use single thread here to avoid known issues under Linux
		_viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
		
		connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));
		_timer.start(10);
	}
	~MyViewerWidget()
	{
		
		delete _collisionWorld;
		delete _picker;
	}

	osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name = "", bool windowDecoration = false)
	{
		osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->windowName = name;
		traits->windowDecoration = windowDecoration;
		traits->x = x;
		traits->y = y;
		traits->width = w;
		traits->height = h;
		traits->doubleBuffer = true;
		traits->alpha = ds->getMinimumNumAlphaBits();
		traits->stencil = ds->getMinimumNumStencilBits();
		traits->sampleBuffers = ds->getMultiSamples();
		traits->samples = ds->getNumMultiSamples();

		return new osgQt::GraphicsWindowQt(traits.get());
	}

protected:
	virtual void paintEvent(QPaintEvent* event)
	{
		//const double currSimTime = _viewer.getFrameStamp()->getSimulationTime();
		//double elapsed(currSimTime - prevSimTime);
		//if (_viewer.getFrameStamp()->getFrameNumber() < 3)
		//	elapsed = 1. / 180.;
		////osg::notify( osg::ALWAYS ) << elapsed / 3. << ", " << 1./180. << std::endl;
		//_bulletWorld->stepSimulation(elapsed, 3, elapsed / 3.);
		//prevSimTime = currSimTime;
		
		_viewer.frame();
	}

	osgViewer::Viewer _viewer;
	QTimer _timer;
	btCollisionWorld* _collisionWorld;
	//btDynamicsWorld* _bulletWorld;
	PickModelHandler* _picker;
	Group* _root;

	double prevSimTime = 0.;
};


int main(int argc, char** argv)
{
	QApplication app(argc, argv);

   
    MyViewerWidget* widget = new MyViewerWidget();

	widget->setGeometry(100, 100, 800, 600);
	widget->show();
	return app.exec();
}
