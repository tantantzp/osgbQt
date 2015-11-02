
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

	MyManipulator(int flags = DEFAULT_SETTINGS);
	MyManipulator(const MyManipulator& om,
		const osg::CopyOp& copyOp = osg::CopyOp::SHALLOW_COPY);

	META_Object(osgGA, MyManipulator);

	virtual void setByMatrix(const osg::Matrixd& matrix);
	virtual void setByInverseMatrix(const osg::Matrixd& matrix);
	virtual osg::Matrixd getMatrix() const;
	virtual osg::Matrixd getInverseMatrix() const;

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);

public:

	virtual bool handleFrame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleMouseMove(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleMouseDrag(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleMousePush(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleMouseRelease(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleKeyDown(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleKeyUp(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	virtual bool handleMouseWheel(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
	
	virtual bool performMovementLeftMouseButton(const double eventTimeDelta, const double dx, const double dy);
	virtual bool performMovementMiddleMouseButton(const double eventTimeDelta, const double dx, const double dy);
	virtual bool performMovementRightMouseButton(const double eventTimeDelta, const double dx, const double dy);
	
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
};

/// Constructor.
MyManipulator::MyManipulator(int flags)
: inherited(flags)
{
	setVerticalAxisFixed(false);
	_orientation = -1; 

	_allowThrow = false;
}


/// Constructor.
MyManipulator::MyManipulator(const MyManipulator& om, const CopyOp& copyOp)
: osg::Object(om, copyOp),
inherited(om, copyOp)
{
	_orientation = -1;
	_allowThrow = false;
}

int MyManipulator::setOrientation()
{
	Vec3d tvec1 = _rotation * Vec3d(1., 0., 0.);
	double tx = tvec1.x();
	double ty = tvec1.y();
	double tz = tvec1.z();
	int orientation;

	//if (abs(ty) > abs(tx) && abs(ty) > abs(tz))
	//{
	//	if (ty > 0) orientation = 4;
	//	else orientation = 5;

	//}
	//else 
	//{
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
	//}

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

	case GUIEventAdapter::FRAME:
		return handleFrame(ea, us);

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


/// Handles GUIEventAdapter::FRAME event.
bool MyManipulator::handleFrame(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
	double current_frame_time = ea.getTime();

	_delta_frame_time = current_frame_time - _last_frame_time;
	_last_frame_time = current_frame_time;

	if (_thrown && performMovement())
	{
		us.requestRedraw();
	}

	if (_animationData && _animationData->_isAnimating)
	{
		performAnimationMovement(ea, us);
	}

	return false;
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
bool  MyManipulator::performMovementLeftMouseButton(const double eventTimeDelta, const double dx, const double dy)
{
	// rotate camera
	//if (getVerticalAxisFixed())
	    rotateWithFixedVertical(dx, dy, Vec3f(0, -1, 0));
	//else
	//	rotateTrackball(_ga_t0->getXnormalized(), _ga_t0->getYnormalized(),
	//	_ga_t1->getXnormalized(), _ga_t1->getYnormalized(),
	//	getThrowScale(eventTimeDelta));
	return true;
}


// doc in parent
bool  MyManipulator::performMovementMiddleMouseButton(const double eventTimeDelta, const double dx, const double dy)
{
	// pan model
	//float scale = -0.3f * _distance * getThrowScale(eventTimeDelta);
	//panModel(dx*scale, dy*scale);
	//return true;
	return false;
}


// doc in parent
bool  MyManipulator::performMovementRightMouseButton(const double eventTimeDelta, const double dx, const double dy)
{
	// zoom model
	//zoomModel(dy * getThrowScale(eventTimeDelta), true);
	//return true;

	float scale = -0.3f * _distance * getThrowScale(eventTimeDelta);
	panModel(dx*scale, dy*scale);
	return true;
}


/// Handles GUIEventAdapter::DRAG event.
bool MyManipulator::handleMouseDrag(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
	addMouseEvent(ea);

	if (performMovement())
		us.requestRedraw();

	us.requestContinuousUpdate(false);
	_thrown = false;
	setOrientation();

	return true;
}




/// Handles GUIEventAdapter::PUSH event.
bool MyManipulator::handleMousePush(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
	flushMouseEventStack();
	addMouseEvent(ea);

	if (performMovement())
		us.requestRedraw();

	us.requestContinuousUpdate(false);
	_thrown = false;

	setOrientation();

	return true;
}


/// Handles GUIEventAdapter::RELEASE event.
bool MyManipulator::handleMouseRelease(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
	if (ea.getButtonMask() == 0)
	{

		double timeSinceLastRecordEvent = _ga_t0.valid() ? (ea.getTime() - _ga_t0->getTime()) : DBL_MAX;
		if (timeSinceLastRecordEvent > 0.02)
			flushMouseEventStack();

		if (isMouseMoving())
		{

			if (performMovement() && _allowThrow)
			{
				us.requestRedraw();
				us.requestContinuousUpdate(true);
				_thrown = true;
			}

			return true;
		}
	}
	setOrientation();
	flushMouseEventStack();
	addMouseEvent(ea);
	if (performMovement())
		us.requestRedraw();
	us.requestContinuousUpdate(false);
	_thrown = false;

	return true;
}


/// Handles GUIEventAdapter::KEYDOWN event.
bool MyManipulator::handleKeyDown(const GUIEventAdapter& ea, GUIActionAdapter& us)
{

	switch (ea.getKey())
	{
		//rotate around Y axis
	case GUIEventAdapter::KEY_Space:

		flushMouseEventStack();
		_thrown = false;
		home(ea, us);
		return true;
		break;
	case GUIEventAdapter::KEY_Up:
		cout << "up" << endl;
		panModel(0, 10);
		return true;
		break;
	case GUIEventAdapter::KEY_Down:
		cout << "down" << endl;
		panModel(0, -10);
		return true;
		break;
	case GUIEventAdapter::KEY_Left:
		cout << "left" << endl;
		panModel(-10, 0);
		return true;
		break;
	case GUIEventAdapter::KEY_Right:
		cout << "right" << endl;
		panModel(10, 0);
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

	// handle centering
	if (_flags & SET_CENTER_ON_WHEEL_FORWARD_MOVEMENT)
	{

		if (((sm == GUIEventAdapter::SCROLL_DOWN && _wheelZoomFactor > 0.)) ||
			((sm == GUIEventAdapter::SCROLL_UP   && _wheelZoomFactor < 0.)))
		{

			if (getAnimationTime() <= 0.)
			{
				// center by mouse intersection (no animation)
				setCenterByMousePointerIntersection(ea, us);
			}
			else
			{
				// start new animation only if there is no animation in progress
				if (!isAnimating())
					startAnimationByMousePointerIntersection(ea, us);

			}

		}
	}

	switch (sm)
	{
		// mouse scroll up event
	case GUIEventAdapter::SCROLL_UP:
	{
		// perform zoom
		zoomModel(_wheelZoomFactor, true);
		us.requestRedraw();
		us.requestContinuousUpdate(isAnimating() || _thrown);
		return true;
	}

		// mouse scroll down event
	case GUIEventAdapter::SCROLL_DOWN:
	{
		// perform zoom
		zoomModel(-_wheelZoomFactor, true);
		us.requestRedraw();
		us.requestContinuousUpdate(isAnimating() || _thrown);
		return true;
}

		// unhandled mouse scrolling motion
	default:
		return false;
	}
}


#endif /* OSGGA_ORBIT_MANIPULATOR */
