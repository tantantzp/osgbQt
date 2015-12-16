#include "Utils.h"
#include "MyManipulator.h"
#include "PickModelHandler.h"
#include "osgbUtil.h"

bool DOUBLEVIEW = false;
bool STEREO = true;

Node* createLight( ) //Node* model)
{

	Vec4 lightpos(0, -1000., 0., 1.);
	LightSource* ls = new LightSource();
	ls->getLight()->setPosition(lightpos);
	ls->getLight()->setAmbient(Vec4(0.9, 0.9, 0.9, 1.0));
	ls->getLight()->setDiffuse(Vec4(0.8, 0.8, 0.8, 1.0));
	ls->getLight()->setLightNum(1);

	return ls;
}





class MyViewerWidget : public QWidget
{
public:
	MyViewerWidget() : QWidget()
	{
		gw = createGraphicsWindow(0, 0, 1000, 1000);
		if (gw)
		{
			cout << "Valid GraphicsWindowQt" << endl;
			QVBoxLayout* layout = new QVBoxLayout;
			layout->addWidget(gw->getGLWidget());
			setLayout(layout);
		}


		_collisionWorld = initCollision();
		_root = new osg::Group;
		_rootLeft = new osg::Group;
		_rootRight = new osg::Group;
		_shadowScene = new osgShadow::ShadowedScene();

		_viewLeft = new osgViewer::Viewer;
		_viewRight = new osgViewer::Viewer;

		
		//_viewer.addView(_viewLeft);
		//_viewer.addView(_viewRight);

		//viewLeft viewRight
		{
			_shadowScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
			_shadowScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);
			osgShadow::ShadowMap * sm = new osgShadow::ShadowMap();
			_shadowScene->setShadowTechnique(sm);
			cout << "before picker" << endl;
			//_picker = new PickModelHandler(_collisionWorld, _root, &_viewer);
			_picker = new PickModelHandler(_collisionWorld, _shadowScene, _viewLeft, _viewRight, _rootLeft, _rootRight);
		
			cout << "after picker" << endl;

			float widthX = 300., widthZ = 300., heightY = 150.;
			_picker->addGround(widthX, widthZ, heightY);



			for (int i = 0; i < 2; i++) {
				string strobj1 = "D:/ProgramLib/objs/chair/chair_17.skp/chair_17.obj";
				Vec3d pos1(-40., 70., 0.);
				_picker->addOneObj(strobj1, pos1);
			}
			for (int i = 0; i < 2; i++) {
				string strobj2 = "D:/ProgramLib/objs/chair/chair_8.skp/chair_8.obj";
				Vec3d pos2(40., 70., 0.);
				_picker->addOneObj(strobj2, pos2);
			}


			string strobj = "D:/ProgramLib/objs/desk/bedroom_143/11.obj";
			Vec3d pos(100., 70., 80.);
			_picker->addOneObj(strobj, pos);

		

			strobj = "D:/ProgramLib/objs/table/dining_table_55.skp/dining_table_55.obj";
			Vec3d pos1(0., 70., 80.);
			_picker->addOneObj(strobj, pos);

			

			//*set scenedata
			_shadowScene->addChild(createLight());
			_root->addChild(_shadowScene);

			_rootLeft->addChild(_root);
			_rootRight->addChild(_root);
			_viewLeft->setSceneData(_rootLeft);
			_viewRight->setSceneData(_rootRight);

			setCamera();

			//_picker->addBackground("wall3.jpg", "wall3.jpg");


			_manipulator->setPickModelHandler(_picker);
			_manipulator2->setPickModelHandler(_picker);

			if (STEREO)
			{
				DisplaySettings *dis = new osg::DisplaySettings();
				dis->setStereo(true);
				//dis->setStereoMode(DisplaySettings::HORIZONTAL_SPLIT); // QUAD_BUFFER,
				//dis->setStereoMode(DisplaySettings::QUAD_BUFFER); 
				float eyeSeperation = 0.01f;
				dis->setEyeSeparation(eyeSeperation);
				_viewLeft->setDisplaySettings(dis);
				_viewRight->setDisplaySettings(dis);
			}

			//* add event handler
			_viewLeft->addEventHandler(_picker);
			_viewRight->addEventHandler(_picker);

		}


		// Use single thread here to avoid known issues under Linux
		_viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
		_viewLeft->setThreadingModel(osgViewer::Viewer::SingleThreaded);
		_viewRight->setThreadingModel(osgViewer::Viewer::SingleThreaded);

		connect(&_timer, SIGNAL(timeout()), this, SLOT(repaint()));
		connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));

		_timer.start(40);
	}
	~MyViewerWidget()
	{
		delete _collisionWorld;
		//delete _picker;
	}
	void setCamera()
	{
		osg::Camera* camera = _viewLeft->getCamera();
		const osg::GraphicsContext::Traits* traits = gw->getTraits();
		camera->setGraphicsContext(gw);
		
		camera->setClearColor(osg::Vec4(0.5, 0.5, 0.5, 0.0));

		if (DOUBLEVIEW)
		{
			camera->setViewport(new osg::Viewport(0, 0, traits->width / 2, traits->height));
		}
		else
		{
			camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		}
		camera->setProjectionMatrixAsPerspective(40., 1., 1., 50.);

		_manipulator = new MyManipulator(_viewLeft);
		_manipulator->setHomePosition(osg::Vec3(0, -50, 500), osg::Vec3(0, 0, 0), osg::Vec3(0, -1, 0));
		
		_viewLeft->setCameraManipulator(_manipulator);


		osg::Camera* camera2 = _viewRight->getCamera();
	
		camera2->setGraphicsContext(gw);

		camera2->setClearColor(osg::Vec4(0.5, 0.5, 0.5, 0.0));
		if (DOUBLEVIEW)
		{
			camera2->setViewport(new osg::Viewport(traits->width / 2, 0, traits->width / 2, traits->height));
		}
		else
		{
			camera2->setViewport(new osg::Viewport(0, 0, 10, 10));
		}
		camera2->setProjectionMatrixAsPerspective(40., 1., 1., 50.);

		_manipulator2 = new MyManipulator(_viewRight);
		_manipulator2->setHomePosition(osg::Vec3(0, -50, 500), osg::Vec3(0, 0, 0), osg::Vec3(0, -1, 0));

		_viewRight->setCameraManipulator(_manipulator2);
		_manipulator2->performCameraTranslate(_manipulator->_eyeDistance, 0);

		_manipulator->setManipulator(_manipulator2);
		_manipulator2->setManipulator(_manipulator);

	}

	//void addSlaveCamera()
	//{
	//	const osg::GraphicsContext::Traits* traits = gw->getTraits();
	//	Camera* cameraClient = new Camera();
	//	cameraClient->setGraphicsContext(gw);
	//	cameraClient->setViewport(new Viewport(0, 0, traits->width / 3, traits->height / 3));
	//	_viewer.addSlave(cameraClient);
	//	
	//}

	void addBackground(string imgpath)
	{
		if (backgroundCamera.get() != NULL)
		{
			_root->removeChild(backgroundCamera.get());
		}

		backgroundCamera = createHUDBg(imgpath);
		_root->addChild(backgroundCamera);


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

		return new osgQt::GraphicsWindowQt(traits.get());
	}

protected:
	virtual void paintEvent(QPaintEvent* event)
	{

		_frameCount = (_frameCount + 1) % 20; 
		if (_frameCount == 0)
		{
			_picker->addBackground("wall3.jpg", "wall3.jpg");
		}

		//_viewer.frame();
		_viewLeft->frame();
	}

	osgViewer::Viewer _viewer;
	//osgViewer::CompositeViewer _viewer;


	ref_ptr<osgViewer::Viewer> _viewLeft;
	ref_ptr<osgViewer::Viewer> _viewRight;

	QTimer _timer;
	btCollisionWorld* _collisionWorld;
	ref_ptr<PickModelHandler> _picker;

	ref_ptr<MyManipulator> _manipulator;
	ref_ptr<MyManipulator> _manipulator2;
	ref_ptr<Group> _root;
	ref_ptr<Group> _rootLeft;
	ref_ptr<Group> _rootRight;
	ref_ptr<Group> _noShadowGroup;
	ref_ptr<osgShadow::ShadowedScene> _shadowScene;
	osgQt::GraphicsWindowQt* gw;
	double prevSimTime = 0.;
	int _frameCount = 0;
	ref_ptr<Camera> backgroundCamera;

};


int main(int argc, char** argv)
{
	osg::ArgumentParser arguments(&argc, argv);

	arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() + " is the standard OpenSceneGraph example which loads and visualises 3d models.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options] filename ...");
	QApplication app(argc, argv);

   
    MyViewerWidget* widget = new MyViewerWidget();

	widget->setGeometry(100, 100, 800, 600);
	widget->show();
	//widget->showFullScreen();
	return app.exec();
}
