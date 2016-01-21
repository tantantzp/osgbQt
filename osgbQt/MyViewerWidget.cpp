#include "MyViewerWidget.h"

#ifdef DEF_DOUBLEVIEW
    bool DOUBLEVIEW = true;
#else
    bool DOUBLEVIEW = false;
#endif

#ifdef DEF_STEREO
	bool STEREO = true;
#else
	bool STEREO = false;
#endif
	
#ifdef DEF_ISWIN
	bool ISWIN = true;
#else
	bool ISWIN = false;
#endif





Node* createLight() //Node* model)
{

	Vec4 lightpos(0, -1000., 0., 1.);
	LightSource* ls = new LightSource();
	ls->getLight()->setPosition(lightpos);
	ls->getLight()->setAmbient(Vec4(0.9, 0.9, 0.9, 1.0));
	ls->getLight()->setDiffuse(Vec4(0.8, 0.8, 0.8, 1.0));
	ls->getLight()->setLightNum(1);

	return ls;
}


MyViewerWidget::MyViewerWidget() :
#ifdef QUAD_BUFFER_TEST
QGLWidget(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer | QGL::Rgba | QGL::StencilBuffer | QGL::AlphaChannel | QGL::StereoBuffers))
#else
QGLWidget()
#endif
{
	if (!ISWIN)
	{



#ifdef QUAD_WINDOW_EMBEDDED
		{
			_gw = new osgViewer::GraphicsWindowEmbedded(0, 0, width(), height());
		}
#else
		{
		_gw = createGraphicsWindow(0, 0, 1000, 1000);


		if (_gw)
		{
			cout << "Valid GraphicsWindowQt" << endl;
			QVBoxLayout* layout = new QVBoxLayout;
			layout->addWidget(_gw->getGLWidget());

			setLayout(layout);
		}
	}
#endif
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

		setCamera();

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


		_picker->addBackground("wall3.jpg", "wall2.jpg");

		_manipulator->setPickModelHandler(_picker);
		_manipulator2->setPickModelHandler(_picker);

		if (STEREO)
		{
			//DisplaySettings *dis = new osg::DisplaySettings();
			_displaySetting = osg::DisplaySettings::instance();  //new osg::DisplaySettings();

			_displaySetting->setStereo(true);
			
			//if (ISQUAD)
#ifdef QUAD_BUFFER_TEST
			_displaySetting->setStereoMode(DisplaySettings::QUAD_BUFFER);
#else
			//_displaySetting->setStereoMode(DisplaySettings::HORIZONTAL_SPLIT); // QUAD_BUFFER,
#endif
			//float eyeSeperation = 0.01f;
			//_displaySetting->setEyeSeparation(eyeSeperation);

			_viewLeft->setDisplaySettings(_displaySetting);
			_viewRight->setDisplaySettings(_displaySetting);
		}

		//* add event handler
		_viewLeft->addEventHandler(_picker);
		_viewRight->addEventHandler(_picker);

	}
	// Use single thread here to avoid known issues under Linux
	//_viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
	_viewLeft->setThreadingModel(osgViewer::Viewer::SingleThreaded);
	_viewRight->setThreadingModel(osgViewer::Viewer::SingleThreaded);

	if (ISWIN)
	{
		//_viewer.realize();
		//_viewer.getWindows(_windows);

		_viewLeft->realize();
		_viewLeft->getWindows(_windows);

		_pWin = dynamic_cast<osgViewer::GraphicsWindowWin32*>(_windows[0]);
		_pWin->setWindowRectangleImplementation(0, 0, width(), height());
		_isFirstFrame = true;
		startTimer(20);
	}
	else
	{
		connect(&_timer, SIGNAL(timeout()), this, SLOT(repaint()));
		connect(&_timer, SIGNAL(timeout()), this, SLOT(update()));
		_timer.start(40);

	}




}
MyViewerWidget::~MyViewerWidget()
{
	delete _collisionWorld;
	//delete _picker;
}
void MyViewerWidget::setCamera()
{
	osg::Camera* camera = _viewLeft->getCamera();
	camera->setClearColor(osg::Vec4(0.5, 0.5, 0.5, 0.0));
	camera->setCullMaskLeft(CameraMaskLeft);
	camera->setCullMaskRight(CameraMaskRight);

	if (!ISWIN)
	{
		camera->setGraphicsContext(_gw);
		const osg::GraphicsContext::Traits* traits = _gw->getTraits();

		if (DOUBLEVIEW)
		{
			camera->setViewport(new osg::Viewport(0, 0, traits->width / 2, traits->height));
		}
		else
		{
			camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		}
	}

	camera->setProjectionMatrixAsPerspective(40., 1., 1., 50.);

	_manipulator = new MyManipulator(_viewLeft);
	_manipulator->setHomePosition(osg::Vec3(0, 0, 500), osg::Vec3(0, 0, 0), osg::Vec3(0, -1, 0));

	_viewLeft->setCameraManipulator(_manipulator);


	osg::Camera* camera2 = _viewRight->getCamera();



	camera2->setClearColor(osg::Vec4(0.5, 0.5, 0.5, 0.0));
	if (!ISWIN)
	{
		const osg::GraphicsContext::Traits* traits = _gw->getTraits();
		camera2->setGraphicsContext(_gw);
		if (DOUBLEVIEW)
		{
			camera2->setViewport(new osg::Viewport(traits->width / 2, 0, traits->width / 2, traits->height));
		}
		else
		{
			camera2->setViewport(new osg::Viewport(0, 0, 10, 10));
		}
	}
	camera2->setProjectionMatrixAsPerspective(40., 1., 1., 50.);

	_manipulator2 = new MyManipulator(_viewRight);
	_manipulator2->setHomePosition(osg::Vec3(0, 0, 500), osg::Vec3(0, 0, 0), osg::Vec3(0, -1, 0));

	_viewRight->setCameraManipulator(_manipulator2);
	_manipulator2->performCameraTranslate(_manipulator->_eyeDistance, 0);

	_manipulator->setManipulator(_manipulator2);
	_manipulator2->setManipulator(_manipulator);

}

//void addSlaveCamera()
//{
//	const osg::GraphicsContext::Traits* traits = _gw->getTraits();
//	Camera* cameraClient = new Camera();
//	cameraClient->setGraphicsContext(_gw);
//	cameraClient->setViewport(new Viewport(0, 0, traits->width / 3, traits->height / 3));
//	_viewer.addSlave(cameraClient);
//	
//}

void MyViewerWidget::addBackground(string imgpath)
{
	if (backgroundCamera.get() != NULL)
	{
		_root->removeChild(backgroundCamera.get());
	}

	backgroundCamera = createHUDBg(imgpath);
	_root->addChild(backgroundCamera);


}

void MyViewerWidget::myFrame()
{
	//_frameCount = (_frameCount + 1) % 20;
	//if (_frameCount == 0)
	//{
	//float t = rand() % 10;
	//
	//float mov = 0.01;
	//if (t > 3)
	//{
	//	mov = 0.;
	//}
	//_manipulator->performCameraRotate(mov, 0.);
	//_manipulator->setOrientation();
	//}
	//_viewer.frame();

	bool flag = false;
	if (_manipulator->_isRotate > 0 )
	{
		float dx = _manipulator->_myDx;
		float dy = _manipulator->_myDy;
		_manipulator->performCameraRotate(dx, dy);

		_manipulator->setOrientation();
		_picker->setAxis();

		//_picker->addBackground("wall3.jpg", "wall2.jpg");
		_picker->addBackground();
		_manipulator->_isRotate = 0;

		flag = true;
	}

	if (_manipulator->_isTranslate > 0)
	{
		float dx = _manipulator->_myDx;
		float dy = _manipulator->_myDy;
		_manipulator->performCameraTranslate(dx, dy);

		_manipulator->setOrientation();
		_picker->setAxis();


		_picker->addBackground();
		_manipulator->_isTranslate = 0;

		flag = true;
	}

	_viewLeft->frame();

	
	_frameCount = (_frameCount + 1) % 2;
	if (_frameCount == 0)
	{
		cv::Mat img1 = cv::imread("wall2.jpg");
		cv::Mat img2 = cv::imread("wall3.jpg");
		_picker->addBackground(img1, img2);
		//_picker->setBackgroundImg("wall3.jpg", "wall2.jpg");
	}


}
osgQt::GraphicsWindowQt* MyViewerWidget::createGraphicsWindow(int x, int y, int w, int h, const std::string& name, bool windowDecoration)
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

#ifdef QUAD_WINDOW_EMBEDDED
osgViewer::GraphicsWindow* MyViewerWidget::getGraphicsWindow()
{
	return _gw.get();
}

const osgViewer::GraphicsWindow* MyViewerWidget::getGraphicsWidow()const
{
	return _gw.get();
}
void MyViewerWidget::resizeGL(int width, int height)
{
	_gw->getEventQueue()->windowResize(0, 0, width, height);
	_gw->resized(0, 0, width, height);
}
void MyViewerWidget::resizeEvent(QResizeEvent * event)
{
	if (ISWIN)
	{
		_pWin->setWindowRectangleImplementation(0, 0, width(), height());
	}
}
void  MyViewerWidget::keyPressEvent(QKeyEvent* event)
{
	_gw->getEventQueue()->keyPress((osgGA::GUIEventAdapter::KeySymbol) event->key());
}

void  MyViewerWidget::keyReleaseEvent(QKeyEvent* event)
{
	_gw->getEventQueue()->keyRelease((osgGA::GUIEventAdapter::KeySymbol)event->key());
}

void  MyViewerWidget::mousePressEvent(QMouseEvent* event)
{
	int button = 0;
	switch (event->button())
	{
	case(Qt::LeftButton) :
		button = 1;
		break;
	case (Qt::MidButton) :
		button = 2;
		break;
	case (Qt::RightButton) :
		button = 3;
		break;
	case (Qt::NoButton) :
		button = 0;
		break;
	default:
		button = 0;
		break;

	}

	_gw->getEventQueue()->mouseButtonPress(event->x(), event->y(), button);
		

}

void MyViewerWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
	int button = 0;
	switch (event->button())
	{
	case(Qt::LeftButton) :
		button = 1;
		break;
	case (Qt::MidButton) :
		button = 2;
		break;
	case (Qt::RightButton) :
		button = 3;
		break;
	case (Qt::NoButton) :
		button = 0;
		break;
	default:
		button = 0;
		break;

	}

	_gw->getEventQueue()->mouseDoubleButtonPress(event->x(), event->y(), button);
}

void  MyViewerWidget::mouseReleaseEvent(QMouseEvent* event)
{
	int button = 0;
	switch (event->button())
	{
	case(Qt::LeftButton) :
		button = 1;
		break;
	case(Qt::MidButton) :
		button = 2;
		break;
	case(Qt::RightButton) :
		button = 3;
		break;
	case(Qt::NoButton) :
		button = 0;
		break;
	default:
		button = 0;
		break;
	}
	_gw->getEventQueue()->mouseButtonRelease(event->x(), event->y(), button);
}

void   MyViewerWidget::mouseMoveEvent(QMouseEvent* event)
{
	_gw->getEventQueue()->mouseMotion(event->x(), event->y());

}




#endif

void MyViewerWidget::timerEvent(QTimerEvent *event)
{
	if (ISWIN)
	{
		if (_isFirstFrame == true)
		{
			//::SetParent(_pWin->getHWND(), winId());
			::SetParent(_pWin->getHWND(), HWND());
			//_viewer.frame();
			_viewLeft->frame();
			_displaySetting->setStereo(false);

			_isFirstFrame = false;
		}

		//_viewer.frame();
		myFrame();
	}
}


#ifdef QUAD_WINDOW_EMBEDDED
void MyViewerWidget::paintGL()
{
	myFrame();

}
#else
void MyViewerWidget::paintEvent(QPaintEvent* event)
{
	myFrame();
	//_frameCount = (_frameCount + 1) % 20;
	//if (_frameCount == 0)
	//{
	//	_picker->setAxis();
	//	_picker->addBackground("wall3.jpg", "wall3.jpg");
	//}

	////_viewer.frame();
	//_viewLeft->frame();
}
#endif

