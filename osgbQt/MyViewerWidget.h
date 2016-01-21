#ifndef MYVIEWERWIDGET_H
#define MYVIEWERWIDGET_H

#include "Utils.h"
#include "MyManipulator.h"
#include "PickModelHandler.h"
#include "osgbUtil.h"
#include <QLayout>
//#include "clientmainwindow.h"
#include <QApplication>
//#include "CmConsoleWindow.h"
#include <osgViewer/api/Win32/GraphicsWindowWin32>




Node* createLight();//Node* model)

class MyViewerWidget : public QGLWidget
{
public:
	MyViewerWidget();
	~MyViewerWidget();
	void setCamera();

	//void addSlaveCamera()
	//{
	//	const osg::GraphicsContext::Traits* traits = _gw->getTraits();
	//	Camera* cameraClient = new Camera();
	//	cameraClient->setGraphicsContext(_gw);
	//	cameraClient->setViewport(new Viewport(0, 0, traits->width / 3, traits->height / 3));
	//	_viewer.addSlave(cameraClient);
	//	
	//}
	void updateBackground(cv::Mat& img1, cv::Mat& img2);
	void addBackground(string imgpath);
	void myFrame();
	osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name = "", bool windowDecoration = false);

#ifdef QUAD_WINDOW_EMBEDDED
	osgViewer::GraphicsWindow* getGraphicsWindow();

	const osgViewer::GraphicsWindow* getGraphicsWidow()const;
	void resizeGL(int width, int height);
	void resizeEvent(QResizeEvent * event);
	void  MyViewerWidget::keyPressEvent(QKeyEvent* event);
	void  MyViewerWidget::keyReleaseEvent(QKeyEvent* event);
	void  MyViewerWidget::mousePressEvent(QMouseEvent* event);
	void MyViewerWidget::mouseDoubleClickEvent(QMouseEvent* event);
	void  MyViewerWidget::mouseReleaseEvent(QMouseEvent* event);
	void   MyViewerWidget::mouseMoveEvent(QMouseEvent* event);
#endif

protected:

	void timerEvent(QTimerEvent *event);


#ifdef QUAD_WINDOW_EMBEDDED
	virtual void paintGL();
#else
	virtual void paintEvent(QPaintEvent* event);
#endif
	osgViewer::Viewer _viewer;
	//osgViewer::CompositeViewer _viewer;


	ref_ptr<osgViewer::Viewer> _viewLeft;
	ref_ptr<osgViewer::Viewer> _viewRight;

	QTimer _timer;
	btCollisionWorld* _collisionWorld;

	ref_ptr<Group> _root;
	ref_ptr<Group> _rootLeft;
	ref_ptr<Group> _rootRight;
	ref_ptr<Group> _noShadowGroup;
	ref_ptr<osgShadow::ShadowedScene> _shadowScene;

#ifdef QUAD_WINDOW_EMBEDDED
	osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _gw;
#else
	osgQt::GraphicsWindowQt* _gw;
#endif
	double prevSimTime = 0.;
	int _frameCount = 0;
	ref_ptr<Camera> backgroundCamera;
	ref_ptr<osg::DisplaySettings>       _displaySetting;

	osgViewer::GraphicsWindowWin32*     _pWin;
	osgViewer::ViewerBase::Windows      _windows;
	bool                               _isFirstFrame;

public:
	ref_ptr<PickModelHandler> _picker;

	ref_ptr<MyManipulator> _manipulator;
	ref_ptr<MyManipulator> _manipulator2;

};
#endif //MYVIEWERWIDGET_H