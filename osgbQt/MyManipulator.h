
#ifndef MYMANIPULATOR_H
#define MYMANIPULATOR_H

#include "Utils.h"
#include <osgGA/StandardManipulator>
#include <osgGA/OrbitManipulator>
#include <osg/BoundsChecking>
#include "PickModelHandler.h"

/** MyManipulator is base class for camera control based on focal center,
distance from the center, and orientation of distance vector to the eye.
This is the base class for trackball style manipulators.*/
class MyManipulator : public OrbitManipulator
{
	typedef OrbitManipulator inherited;

public:

	MyManipulator(osgViewer::Viewer* view = NULL, int flags = DEFAULT_SETTINGS);
	MyManipulator(const MyManipulator& om,
		const osg::CopyOp& copyOp = osg::CopyOp::SHALLOW_COPY);

	META_Object(osgGA, MyManipulator);

	virtual void setByMatrix(const osg::Matrixd& matrix);
	virtual void setByInverseMatrix(const osg::Matrixd& matrix);
	virtual osg::Matrixd getMatrix() const;
	virtual osg::Matrixd getInverseMatrix() const;

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);

public:

	virtual bool handleResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleMouseMove(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleMouseDrag(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleMousePush(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleMouseRelease(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleKeyDown(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleKeyUp(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleMouseWheel(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);

	virtual bool performMovement(float tx, float ty, int function); // 1:rotate camera 2:translate camera 
    bool performCameraRotate(const double dx, const double dy);
	bool performCameraTranslate(const double dx, const double dy);
	bool performCameraZoom(const double zoomFactor);

	void flushMouseEventStack();

	void doRotate(Quat& rotation, const double yaw, const double pitch,const Vec3d& localUp);
	void rotateTrackball(const float px0, const float py0, const float px1, const float py1, const float scale);
	int setOrientation();

	void rotateWithFixedVertical(const float dx, const float dy, const Vec3f& up);
	void setPickModelHandler(PickModelHandler* handler) 
	{
		_myHandler = handler;
		setOrientation();
	};

	//osg::Vec3d _center;
	//osg::Quat  _rotation;
	//double     _distance;

	int _orientation;
	PickModelHandler* _myHandler;
	osgViewer::Viewer* _view;
//	osg::ref_ptr< const osgGA::GUIEventAdapter > _ga_t1;
//	osg::ref_ptr< const osgGA::GUIEventAdapter > _ga_t0;
	Vec2d _oldPoint;
	Vec2d _curPoint;
	double _zoomFactor;
};

/// Constructor.
MyManipulator::MyManipulator(osgViewer::Viewer* view, int flags)
: inherited(flags)
{
	setVerticalAxisFixed(false);
	_orientation = -1; 
	_allowThrow = false;
	_view = view;
	_oldPoint = _curPoint = Vec2d(0., 0.);
	_zoomFactor = 0.1;
}


/// Constructor.
MyManipulator::MyManipulator(const MyManipulator& om, const CopyOp& copyOp)
: osg::Object(om, copyOp),
inherited(om, copyOp)
{
	_orientation = -1;
	_allowThrow = false;
	_view = om._view;
	_oldPoint = _curPoint = Vec2d(0., 0.);
	_zoomFactor = 0.1;
}

int MyManipulator::setOrientation()
{
	Vec3d tvec1 = _rotation * Vec3d(1., 0., 0.);
	double tx = tvec1.x();
	double ty = tvec1.y();
	double tz = tvec1.z();
	int orientation;


	if (abs(tx) > abs(tz)) {
		if (tx > 0)    //"+X" 
		{
			orientation = 3;
		}
		else // "-X" 
		{
			orientation = 1;
		}
	}
	else {
		if (tz > 0) //"+Z" 
		{
			orientation = 2;
		}
		else //"-Z" 
		{
			orientation = 0;
		}
	}

	if (orientation != _orientation)
	{
		_orientation = orientation;
		_myHandler->setOrientation(orientation);
	}
	return orientation;
}

/** Set the position of the manipulator using a 4x4 matrix.*/
void MyManipulator::setByMatrix(const osg::Matrixd& matrix)
{
	_center = osg::Vec3d(0., 0., -_distance) * matrix;
	_rotation = matrix.getRotate();

	// fix current rotation
	if (getVerticalAxisFixed())
		fixVerticalAxis(_center, _rotation, true);
}


/** Set the position of the manipulator using a 4x4 matrix.*/
void MyManipulator::setByInverseMatrix(const osg::Matrixd& matrix)
{
	setByMatrix(osg::Matrixd::inverse(matrix));
}


/** Get the position of the manipulator as 4x4 matrix.*/
osg::Matrixd MyManipulator::getMatrix() const
{
	return osg::Matrixd::translate(0., 0., _distance) *
		osg::Matrixd::rotate(_rotation) *
		osg::Matrixd::translate(_center);
}


/** Get the position of the manipulator as a inverse matrix of the manipulator,
typically used as a model view matrix.*/
osg::Matrixd MyManipulator::getInverseMatrix() const
{
	return osg::Matrixd::translate(-_center) *
		osg::Matrixd::rotate(_rotation.inverse()) *
		osg::Matrixd::translate(0.0, 0.0, -_distance);
}


/** Handles events. Returns true if handled, false otherwise.*/
bool MyManipulator::handle(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
	switch (ea.getEventType())
	{

//	case GUIEventAdapter::FRAME:
//		return handleFrame(ea, us);

	case GUIEventAdapter::RESIZE:
		return handleResize(ea, us);

	default:
		break;
	}

	if (ea.getHandled())
		return false;

	switch (ea.getEventType())
	{
	case GUIEventAdapter::MOVE:
		return handleMouseMove(ea, us);

	case GUIEventAdapter::DRAG:
		return handleMouseDrag(ea, us);

	case GUIEventAdapter::PUSH:
		return handleMousePush(ea, us);

	case GUIEventAdapter::RELEASE:
		return handleMouseRelease(ea, us);

	case GUIEventAdapter::KEYDOWN:
		return handleKeyDown(ea, us);

	case GUIEventAdapter::KEYUP:
		return handleKeyUp(ea, us);

	case GUIEventAdapter::SCROLL:
		if (_flags & PROCESS_MOUSE_WHEEL)
			return handleMouseWheel(ea, us);
		else
			return false;

	default:
		return false;
	}
}




/// Handles GUIEventAdapter::RESIZE event.
bool MyManipulator::handleResize(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
	init(ea, us);
	us.requestRedraw();
	return true;
}


/// Handles GUIEventAdapter::MOVE event.
bool MyManipulator::handleMouseMove(const GUIEventAdapter& /*ea*/, GUIActionAdapter& /*us*/)
{
	return false;
}

/** Performs trackball rotation based on two points given, for example,
by mouse pointer on the screen.

Scale parameter is useful, for example, when manipulator is thrown.
It scales the amount of rotation based, for example, on the current frame time.*/
void MyManipulator::rotateTrackball(const float px0, const float py0,
	const float px1, const float py1, const float scale)
{
	osg::Vec3d axis;
	float angle;

	float dx = px1 - px0;
	float dy = py1 - py0;
	
	if (abs(dx) > abs(dy)) {

		trackball(axis, angle, px0 + (px1 - px0)*scale, py0 , px0, py0);
	}
	else
	{
		trackball(axis, angle, px0 , py0 + (py1 - py0)*scale, px0, py0);
	}
	//trackball(axis, angle, px0 + (px1 - px0)*scale, py0 + (py1 - py0)*scale, px0, py0);

	Quat new_rotate;
	new_rotate.makeRotate(angle, axis);

	_rotation = _rotation * new_rotate;
}



/** Reset the internal GUIEvent stack.*/
void MyManipulator::flushMouseEventStack()
{
	_oldPoint = Vec2d(-10., -10.);
	_curPoint = Vec2d(-10., -10.);
}


bool MyManipulator::performMovement(float tx, float ty, int function)
{
	_oldPoint = _curPoint;
	_curPoint = Vec2d(tx, ty);
	//cout << "_curPoint " << _curPoint.x() <<" "<<_curPoint.y() << endl;
	bool ret = false;
	if (_curPoint.x() < -2. || _curPoint.y() < -2. || _oldPoint.x() < -2. || _oldPoint.y() < -2.
		|| (function != 1 && function != 2))
	{
		ret = false;
	}
	else
	{
		float dx = _curPoint.x() - _oldPoint.x();
		float dy = _curPoint.y() - _oldPoint.y();

		if (dx == 0. && dy == 0.)
			ret =  false;
		if (function == 1)
		{
			ret = performCameraRotate(dx, dy);
		}
		else if (function == 2)
		{
			ret = performCameraTranslate(dx, dy);
		}

		ret = false;
	}

	if (ret)
	{
		_view->requestRedraw();
	}
	_view->requestContinuousUpdate(false);
	setOrientation();

	return ret;
}


/** Update rotation by yaw and pitch.
*
*  localUp parameter defines either camera's "UP" vector
*  that will be preserved during rotation, or it can be zero (0,0,0) to specify
*  that camera's "UP" vector will be not preserved and free rotation will be made.*/
void MyManipulator::doRotate(Quat& rotation, const double yaw, const double pitch,
	const Vec3d& localUp)
{
	bool verticalAxisFixed = (localUp != Vec3d(0., 0., 0.));

	// fix current rotation
	if (verticalAxisFixed)
		fixVerticalAxis(rotation, localUp, true);

	// rotations
	Quat rotateYaw(-yaw, verticalAxisFixed ? localUp : rotation * Vec3d(0., 1., 0.));
	Quat rotatePitch;
	Quat newRotation;
	Vec3d cameraRight(rotation * Vec3d(1., 0., 0.));

	double my_dy = pitch;
	int i = 0;

	do {

		// rotations
		rotatePitch.makeRotate(my_dy, cameraRight);
		newRotation = rotation * rotateYaw * rotatePitch;

		// update vertical axis
		if (verticalAxisFixed)
			fixVerticalAxis(newRotation, localUp, false);

		// check for viewer's up vector to be more than 90 degrees from "up" axis
		Vec3d newCameraUp = newRotation * Vec3d(0., 1., 0.);
		if (newCameraUp * localUp > 0.)
		{

			// apply new rotation
			rotation = newRotation;
			return;

		}

		my_dy /= 2.;
		if (++i == 20)
		{
			rotation = rotation * rotateYaw;
			return;
		}

	} while (true);
}

/** Performs rotation horizontally by dx parameter and vertically by dy parameter,
while keeping UP vector given by up parameter.*/
void MyManipulator::rotateWithFixedVertical(const float dx, const float dy, const Vec3f& up)
{
	if (abs(dx) > abs(dy)) {
		//rotateYawPitch(_rotation, dx, 0, up);
	    doRotate(_rotation, dx, 0, up);
	}
	else
	{
		doRotate(_rotation, 0, dy, up);
	}
}


// doc in parent
bool  MyManipulator::performCameraRotate(const double dx, const double dy)
{
	// rotate camera
	rotateWithFixedVertical(dx, dy, Vec3f(0, -1, 0));

	return true;
}


// doc in parent
bool  MyManipulator::performCameraTranslate(const double dx, const double dy)
{
	//float scale = -0.3f * _distance * getThrowScale(eventTimeDelta);
	//panModel(dx*scale, dy*scale);
	panModel(-dx * 100, -dy * 100);
	return true;
}
bool MyManipulator::performCameraZoom(const double zoomFactor)
{
	zoomModel(zoomFactor, true);
	_view->requestRedraw();
	_view->requestContinuousUpdate(false);
	return true;
}


/// Handles GUIEventAdapter::DRAG event.
bool MyManipulator::handleMouseDrag(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
	float tx = ea.getXnormalized();
	float ty = ea.getYnormalized();
	unsigned int buttonMask = ea.getButtonMask();
	int function = 0;
	if (buttonMask == GUIEventAdapter::LEFT_MOUSE_BUTTON)
		function = 1;
	else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
		function = 2;


	performMovement(tx, ty, function);
	//_myHandler->addBackground("wall1.jpg");
	//_thrown = false

	return true;
}




/// Handles GUIEventAdapter::PUSH event.
bool MyManipulator::handleMousePush(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
	flushMouseEventStack();

	float tx = ea.getXnormalized();
	float ty = ea.getYnormalized();
	unsigned int buttonMask = ea.getButtonMask();
	int function = 0;
	if (buttonMask == GUIEventAdapter::LEFT_MOUSE_BUTTON)
		function = 1;
	else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
		function = 2;


	performMovement(tx, ty, function);
	
	//_thrown = false;
	return true;
}


/// Handles GUIEventAdapter::RELEASE event.
bool MyManipulator::handleMouseRelease(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
	float tx = ea.getXnormalized();
	float ty = ea.getYnormalized();
	unsigned int buttonMask = ea.getButtonMask();
	int function = 0;
	if (buttonMask == GUIEventAdapter::LEFT_MOUSE_BUTTON)
		function = 1;
	else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
		function = 2;

	performMovement(tx, ty, function);

	flushMouseEventStack();

	return true;
}


/// Handles GUIEventAdapter::KEYDOWN event.
bool MyManipulator::handleKeyDown(const GUIEventAdapter& ea, GUIActionAdapter& us)
{

	switch (ea.getKey())
	{
	//case GUIEventAdapter::KEY_Space:

	//	flushMouseEventStack();
	//	_thrown = false;
	//	home(ea, us);
	//	return true;
	//	break;
	case GUIEventAdapter::KEY_Up:
		performCameraTranslate(0, -0.1);
		return true;
		break;
	case GUIEventAdapter::KEY_Down:
		performCameraTranslate(0, 0.1);
		return true;
		break;
	case GUIEventAdapter::KEY_Left:
		performCameraTranslate(0.1, 0);
		return true;
		break;
	case GUIEventAdapter::KEY_Right:
		performCameraTranslate(-0.1, 0);
		return true;
		break;
	case 'p':
	case 'P':
		cout << "P::" << endl;

		Vec3d tvec1 = _rotation * Vec3d(1., 0., 0.);
		double tx = tvec1.x();
		double tz = tvec1.z();
		if (abs(tx) > abs(tz)) {
			if (tx > 0) cout << "+X" << endl;
			else cout << "-X" << endl;
		}
		else {
			if (tz > 0) cout << "+Z" << endl;
			else cout << "-Z" << endl;
		}



		return true;
		break;

	}

	//if (ea.getKey() == GUIEventAdapter::KEY_Space)
	//{
	//	flushMouseEventStack();
	//	_thrown = false;
	//	home(ea, us);
	//	return true;
	//}


	return false;
}


/// Handles GUIEventAdapter::KEYUP event.
bool MyManipulator::handleKeyUp(const GUIEventAdapter& /*ea*/, GUIActionAdapter& /*us*/)
{
	return false;
}



// doc in parent
bool MyManipulator::handleMouseWheel(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
	osgGA::GUIEventAdapter::ScrollingMotion sm = ea.getScrollingMotion();

	//// handle centering
	//if (_flags & SET_CENTER_ON_WHEEL_FORWARD_MOVEMENT)
	//{
	//	if (((sm == GUIEventAdapter::SCROLL_DOWN && _wheelZoomFactor > 0.)) ||
	//		((sm == GUIEventAdapter::SCROLL_UP   && _wheelZoomFactor < 0.)))
	//	{
	//		if (getAnimationTime() <= 0.)
	//		{
	//			// center by mouse intersection (no animation)
	//			setCenterByMousePointerIntersection(ea, us);
	//		}
	//		else
	//		{
	//			// start new animation only if there is no animation in progress
	//			if (!isAnimating())
	//				startAnimationByMousePointerIntersection(ea, us);
	//		}
	//	}
	//}

	switch (sm)
	{
		// mouse scroll up event
		case GUIEventAdapter::SCROLL_UP:
		{
			performCameraZoom(_zoomFactor);
			return true;
		}

			// mouse scroll down event
		case GUIEventAdapter::SCROLL_DOWN:
		{
			// perform zoom
			performCameraZoom(-_zoomFactor);

			return true;
		}

		// unhandled mouse scrolling motion
	    default:
		    return false;
	}
}


#endif /* OSGGA_ORBIT_MANIPULATOR */
