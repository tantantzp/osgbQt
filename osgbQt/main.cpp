#include "Utils.h"
#include "PickModelHandler.h"
#include "osgbUtil.h"


osg::Camera* createCamera(int x, int y, int w, int h)
{
	osg::DisplaySettings* ds =
		osg::DisplaySettings::instance().get();
	osg::ref_ptr<osg::GraphicsContext::Traits> traits =
		new osg::GraphicsContext::Traits;
	traits->windowDecoration = false;
	traits->x = x;
	traits->y = y;
	traits->width = w;
	traits->height = h;
	traits->doubleBuffer = true;
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setGraphicsContext(
		new osgQt::GraphicsWindowQt(traits.get()));
	camera->setClearColor(osg::Vec4(0.2, 0.2, 0.6, 1.0));
	camera->setViewport(new osg::Viewport(
		0, 0, traits->width, traits->height));
	camera->setProjectionMatrixAsPerspective(
		30.0f, static_cast<double>(traits->width) /
		static_cast<double>(traits->height), 1.0f, 10000.0f);
	return camera.release();
}


osg::Group* createScene(btCollisionWorld* cw, PickModelHandler *picker)
{
	osg::ref_ptr< osg::Group > root = new osg::Group;

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

	return(root.release());
}

class MyViewerWidget : public QWidget
{
public:
	MyViewerWidget(osg::Camera* camera) : QWidget()
	{
		_collisionWorld = initCollision();

		_picker = new PickModelHandler;	
		_picker->setCollisionWorld(_collisionWorld);

		_root = createScene(_collisionWorld, _picker);
		_root->addChild(_picker->getOrCreateSelectionBox());

		_viewer.setCamera(camera);
		_viewer.setSceneData(_root);
		_viewer.addEventHandler(_picker);
		_viewer.setCameraManipulator(new osgGA::TrackballManipulator());
		//_viewer.setUpViewInWindow(10, 30, 800, 600);


		// Use single thread here to avoid known issues under Linux
		_viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
		
		osgQt::GraphicsWindowQt* gw = dynamic_cast<osgQt::GraphicsWindowQt*>(camera->getGraphicsContext());
		if (gw)
		{
			cout << "Valid GraphicsWindowQt" << endl;
			QVBoxLayout* layout = new QVBoxLayout;
			layout->addWidget(gw->getGLWidget());
			setLayout(layout);
		}

		connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));
		_timer.start(40);
	}
	~MyViewerWidget()
	{
		delete _collisionWorld;
		delete _picker;
	}

protected:
	virtual void paintEvent(QPaintEvent* event)
	{
		_viewer.frame();
	}

	osgViewer::Viewer _viewer;
	QTimer _timer;
	btCollisionWorld* _collisionWorld;
	PickModelHandler* _picker;
	Group* _root;
};


class ViewerWidget : public QWidget
{
public:
	ViewerWidget(osg::Camera* camera, osg::Node* scene) : QWidget()
	{
		_viewer.setCamera(camera);
		_viewer.setSceneData(scene);
		_viewer.addEventHandler(new osgViewer::StatsHandler);
		_viewer.setCameraManipulator(new osgGA::TrackballManipulator);
		// Use single thread here to avoid known issues under Linux
		_viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
		
		osgQt::GraphicsWindowQt* gw = dynamic_cast<osgQt::GraphicsWindowQt*>(camera->getGraphicsContext());
		if (gw)
		{
			cout << "Valid GraphicsWindowQt" << endl;
			QVBoxLayout* layout = new QVBoxLayout;
			layout->addWidget(gw->getGLWidget());
			setLayout(layout);
		}

		connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));
		_timer.start(40);
	}


protected:
	virtual void paintEvent(QPaintEvent* event)
	{
		_viewer.frame();

	}

	osgViewer::Viewer _viewer;
	QTimer _timer;
};

int main(int argc, char** argv)
{
	QApplication app(argc, argv);
	osg::Camera* camera = createCamera(50, 50, 640, 480);

	MyViewerWidget* widget = new MyViewerWidget(camera);

    //osg::Node* scene = osgDB::readNodeFile("cow.osg");
	//ViewerWidget* widget = new ViewerWidget(camera, scene);
	widget->setGeometry(100, 100, 800, 600);
	widget->show();
	return app.exec();
}
