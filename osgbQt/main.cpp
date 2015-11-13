#include "Utils.h"
#include "MyManipulator.h"
#include "PickModelHandler.h"
#include "osgbUtil.h"

Node* createLight( ) //Node* model)
{
	//ComputeBoundsVisitor cbbv;
	//model->accept(cbbv);
	//BoundingBox bb = cbbv.getBoundingBox();
	Vec4 lightpos(0., -1000., 0., 1.);
	//lightpos.set(bb.center().x(), bb.center().y(), bb.zMax() + bb.radius()*2.0f, 1.0f);
	LightSource* ls = new LightSource();
	ls->getLight()->setPosition(lightpos);
	ls->getLight()->setAmbient(Vec4(0.6, 0.6, 0.6, 1.0));
	ls->getLight()->setDiffuse(Vec4(0.8, 0.8, 0.8, 1.0));
	ls->getLight()->setLightNum(3);;

	return ls;
}


class MyViewerWidget : public QWidget
{
public:
	MyViewerWidget() : QWidget()
	{
		gw = createGraphicsWindow(0, 0, 500, 500);

		_collisionWorld = initCollision();
		_root = new osg::Group;
		_shadowScene = new osgShadow::ShadowedScene();
		
		
		_shadowScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
		_shadowScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);
		osgShadow::ShadowTexture * st = new osgShadow::ShadowTexture();
		_shadowScene->setShadowTechnique(st);


		cout << "before picker" << endl;
		//_picker = new PickModelHandler(_collisionWorld, _root, &_viewer);
		_picker = new PickModelHandler(_collisionWorld, _shadowScene, &_viewer);
		cout << "after picker" << endl;

		float widthX = 250., widthZ = 250., heightY = 100.;
		_picker->addGround(widthX, widthZ, heightY);


		for (int i = 0; i < 2; i++) {
			string strobj1 = "D:/ProgramLib/objs/chair/chair_17.skp/chair_17.obj";
			Vec3d pos1(-40., 0., 0.);
			_picker->addOneObj(strobj1, pos1);
		}
		for (int i = 0; i < 2; i++) {
			string strobj2 = "D:/ProgramLib/objs/chair/chair_8.skp/chair_8.obj";
			Vec3d pos2(40., 0., 0.);
			_picker->addOneObj(strobj2, pos2);
		}

		for (int i = 0; i < 1; i++) {
			string strobj = "D:/ProgramLib/objs/desk/bedroom_143/11.obj";
			Vec3d pos(0., 0., 80.);
			_picker->addOneObj(strobj, pos);
		}

		for (int i = 0; i < 1; i++) {
			string strobj = "D:/ProgramLib/objs/bed/bed_1.skp/bed_1.obj";
			Vec3d pos(80., 0., 80.);
			_picker->addOneObj(strobj, pos);
		}

		//createScene(_root, _collisionWorld, _picker);		

		//*set scenedata
		_shadowScene->addChild(createLight());
		_root->addChild(_shadowScene);
		_viewer.setSceneData(_root);		
	
		//*BEGIN: set camera*/
		osg::Camera* camera = _viewer.getCamera();
		setCamera(camera);
		//*END: set camera */	
		_manipulator->setPickModelHandler(_picker);

		//* add event handler
		_viewer.addEventHandler(_picker);

		// Use single thread here to avoid known issues under Linux
		_viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
		if (gw)
		{
			cout << "Valid GraphicsWindowQt" << endl;
			QVBoxLayout* layout = new QVBoxLayout;
			layout->addWidget(gw->getGLWidget());
			setLayout(layout);
		}

		connect(&_timer, SIGNAL(timeout()), this, SLOT(repaint()));
		connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));

		_timer.start(40);
	}
	~MyViewerWidget()
	{
		
		delete _collisionWorld;
		delete _picker;
	}
	void setCamera(Camera* camera){
		const osg::GraphicsContext::Traits* traits = gw->getTraits();
		camera->setGraphicsContext(gw);
		camera->setClearColor(osg::Vec4(0.2, 0.2, 0.6, 1.0));
		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		camera->setProjectionMatrixAsPerspective(40., 1., 1., 50.);
		//osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
		_manipulator = new MyManipulator(&_viewer);
		_manipulator->setHomePosition(osg::Vec3(0, -30, 300), osg::Vec3(0, 0, 150), osg::Vec3(0, -1, 0));
		_viewer.setCameraManipulator(_manipulator);
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
		//traits->alpha = ds->getMinimumNumAlphaBits();
		//traits->stencil = ds->getMinimumNumStencilBits();
		//traits->sampleBuffers = ds->getMultiSamples();
		//traits->samples = ds->getNumMultiSamples();

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
	MyManipulator* _manipulator;
	Group* _root;
	osgShadow::ShadowedScene* _shadowScene;
	osgQt::GraphicsWindowQt* gw;
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
