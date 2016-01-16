#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>
#include <set>

#include <GL/glut.h>
#include <osg/MatrixTransform>
#include <osg/LOD>
#include <osg/ProxyNode>
#include <osg/NodeVisitor>
#include <osg/io_utils>
#include <osg/Switch>
#include <osg/Group>
#include <osg/Geode>
#include <osgDB/ReadFile>
#include <osg/Drawable>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/TriangleFunctor>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/PolygonMode>
#include <osg/Fog>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/Texture2D>
#include <osg/BlendFunc>
#include <osg/Program>
#include <osg/LineWidth>
#include <osg/Camera>
#include <osg/ComputeBoundsVisitor>
#include <osg/DrawPixels>

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>
#include <osgGA/GUIEventHandler>

#include <osgwTools/Shapes.h>
#include <osgwTools/Version.h>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowVolume>
#include <osgShadow/ShadowTexture>
#include <osgShadow/ShadowMap>
#include <osgShadow/SoftShadowMap>

#include <osgUtil/Simplifier>
#include <osgUtil/LineSegmentIntersector>

#include <btBulletCollisionCommon.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbInteraction/DragHandler.h>

#include <btBulletDynamicsCommon.h>

#include <QtCore/QtCore>
#include <QtGui/QtGui>
#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QApplication>
#include <osgQt/GraphicsWindowQt>
#include <osgViewer/api/Win32/GraphicsWindowWin32>


#include <opencv2/opencv.hpp>

using namespace std;
using namespace osg;
using namespace osgGA;
using namespace osgDB;
//using namespace cv;

const int MODELSIZEMIN = 15;  //auto scaling when load the obj, if the radius of obj < MODELSIZEMIN, scale to MODELSIZEMIN
const int MODELSIZEMAX = 40;

const float PICKRANGE = 10.;  //pixel

const int ReceivesShadowTraversalMask = 0x1;
const int CastsShadowTraversalMask = 0x2;
const int NoShadowTraversalMask = 0x4;

const int ChooseBoxMask = 0x2;
const int AxisMask = 0x2;

const int NodeMaskLeft = 0x8;
const int NodeMaskRight = 0x10;
const int CameraMaskLeft = 0xB;
const int CameraMaskRight = 0x13;

static float BACKGROUNDWIDTH = 720.;
static float BACKGROUNDHEIGHT = 1280.;
static float BACKGROUNDDEEP = 2200.;
static float BACKGROUNDEYEDIS = 10.;





//#define DEF_DOUBLEVIEW
//#define QUAD_BUFFER_TEST
//#define QUAD_WINDOW_EMBEDDED

#define DEF_STEREO

//#define DEF_ISWIN

//#define DEF_AXIS



#pragma comment(lib, "osg.lib")
#pragma comment(lib, "osgViewer.lib")
#pragma comment(lib, "osgText.lib")
#pragma comment(lib, "osgUtil.lib")
#pragma comment(lib, "osgGA.lib")
#pragma comment(lib, "osgDB.lib")
#pragma comment(lib, "osgQt.lib")
#pragma comment(lib, "OpenThreads.lib")
#pragma comment(lib, "osgWidget.lib")
#pragma comment(lib, "Qt5Core.lib")
#pragma comment(lib, "Qt5Gui.lib")
#pragma comment(lib, "Qt5OpenGL.lib")
#pragma comment(lib, "qtmain.lib")
#pragma comment(lib, "Qt5Widgets.lib")
#pragma comment(lib, "osgbCollision.lib")
#pragma comment(lib, "osgFX.lib")

#endif
