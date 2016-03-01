#include "MyManipulator.h"

/** MyManipulator is base class for camera control based on focal center,
distance from the center, and orientation of distance vector to the eye.
This is the base class for trackball style manipulators.*/

//// Constructor.
MyManipulator::MyManipulator(osgViewer::View* view, int flags)
: inherited(flags)
{
	setVerticalAxisFixed(false);
	_orientation = -1;
	_allowThrow = false;
	_view = view;
	_oldPoint = _curPoint = Vec2d(0., 0.);
	_zoomFactor = 0.1;
	_maxZoom = 0.6;
	_currentZoom = 0;

#ifndef USER2
	_myClientNum = 0;
#else
	_myClientNum = 1;
#endif
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
	_maxZoom = 0.6;
	_currentZoom = 0;

#ifndef USER2
	_myClientNum = 0;
#else
	_myClientNum = 1;
#endif
}
void MyManipulator::setCameraMatrix()
{
	_myHandler->setCameraMatrix(getMatrix());

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
	_myHandler->updateCameraVec();
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

		trackball(axis, angle, px0 + (px1 - px0)*scale, py0, px0, py0);
	}
	else
	{
		trackball(axis, angle, px0, py0 + (py1 - py0)*scale, px0, py0);
	}
	//trackball(axis, angle, px0 + (px1 - px0)*scale, py0 + (py1 - py0)*scale, px0, py0);

	Quat new_rotate;
	new_rotate.makeRotate(angle, axis);

	_rotation = _rotation * new_rotate;
}



/** Reset the internal GUIEvent stack.*/
void MyManipulator::flushMouseEventStack(int clientNum)
{
	if (clientNum == 0)
	{
		_oldPoint = Vec2d(-10., -10.);
		_curPoint = Vec2d(-10., -10.);
	}

	if (clientNum == 1)
	{
		_oldPoint2 = Vec2d(-10., -10.);
		_curPoint2 = Vec2d(-10., -10.);
	}

}


bool MyManipulator::performMovement(float tx, float ty, int function, int clientNum)
{
	if (clientNum == 0)
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
				ret = false;
			if (function == 1)
			{
				_myDx += dx ;
				_myDy += dy ;

				_isRotate = 1;
				ret = true;
				//ret = performCameraRotate(_myDx, _myDy);
			}
			else if (function == 2)
			{

				_myDx += dx ;
				_myDy += dy ;

				_isTranslate = 1;
				ret = true;
				//ret = performCameraTranslate(_myDx, _myDy);
			}
		}

		//if (ret)
		//	_view->requestRedraw();
		//setOrientation();

		return ret;
	}
	else if (clientNum == 1)
	{
		_oldPoint2 = _curPoint2;
		_curPoint2 = Vec2d(tx, ty);
		//cout << "_curPoint " << _curPoint.x() <<" "<<_curPoint.y() << endl;
		bool ret = false;
		if (_curPoint2.x() < -2. || _curPoint2.y() < -2. || _oldPoint2.x() < -2. || _oldPoint2.y() < -2.
			|| (function != 1 && function != 2))
		{
			ret = false;
		}
		else
		{
			float dx = _curPoint2.x() - _oldPoint2.x();
			float dy = _curPoint2.y() - _oldPoint2.y();

			if (dx == 0. && dy == 0.)
				ret = false;
			if (function == 1)
			{
				_myDx2 += dx ;
				_myDy2 += dy ;
				_isRotate2 = 1;
				ret = true;
				//ret = performCameraRotate(_myDx, _myDy);
			}
			else if (function == 2)
			{
				_myDx2 += dx ;
				_myDy2 += dy ;
				_isTranslate2 = 1;
				ret = true;
				//ret = performCameraTranslate(_myDx, _myDy);
			}
		}

		//if (ret)
		//	_view->requestRedraw();
		//setOrientation();

		return ret;
	}
	cout << "clientNum is invalid in performMovement!" << endl;
	return false;
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

	//if (verticalAxisFixed)
	//	fixVerticalAxis(rotation, localUp, true);

	// rotations
	Quat rotateYaw(-yaw, verticalAxisFixed ? localUp : rotation * Vec3d(0., 1., 0.));
	Quat rotatePitch;
	Quat newRotation;
	Vec3d cameraRight(rotation * Vec3d(1., 0., 0.));
	//cout << "rotateYaw:" << rotateYaw << endl;
	//cout << "cameraRight:"<< cameraRight << endl;
	double my_dy = pitch;
	int i = 0;

	do {

		// rotations
		rotatePitch.makeRotate(my_dy, cameraRight);
		newRotation = rotation * rotateYaw * rotatePitch;

		// update vertical axis
		//if (verticalAxisFixed)
		//	fixVerticalAxis(newRotation, localUp, false);

		// check for viewer's up vector to be more than 90 degrees from "up" axis
		Vec3d newCameraUp = newRotation * Vec3d(0., 1., 0.);

		//if (newCameraUp * localUp > 0.)
		//{
			// apply new rotation
			rotation = newRotation;
			return;
		//}

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
		_curdx += dx;
		//_curdy += dy;
	}
	else
	{
		doRotate(_rotation, 0, dy, up);
		//_curdx += dx;
		_curdy += dy;
	}
	cout << "_curdx:" << _curdx << endl;
	cout << "_curdy:" << _curdy << endl;
}


// doc in parent
bool  MyManipulator::performCameraRotate(const double dx, const double dy)
{
	// rotate camera
	//setCenter(Vec3d(0, 0, 0));
	rotateWithFixedVertical(dx, dy, Vec3f(0, -1, 0));




//	panModel(0., 0., _currentZoom);
	//panModel(_currentPanX, _currentPanY, _currentZoom);
	//panModel(curCenter.x(), curCenter.y(), curCenter.z());
	//if (dx > 0.)
	//    rotateWithFixedVertical(0.05, 0., Vec3f(0, -1, 0));
	//else
	//	rotateWithFixedVertical(-0.05, 0., Vec3f(0, -1, 0));
	//_myHandler->addBackground("wall3.jpg", "wall3.jpg");
	return true;
}
// doc in parent
bool  MyManipulator::performCameraRotateBack()
{
	// rotate camera
	//setCenter(Vec3d(0, 0, 0));
	doRotate(_rotation, -_curdx, 0, Vec3f(0, -1, 0));
	doRotate(_rotation, 0,-_curdy, Vec3f(0, -1, 0));
	_curdx = 0;
	_curdy = 0;
	cout << "_curdx:" << _curdx << endl;
	cout << "_curdy:" << _curdy << endl;
	setOrientation();

	//	panModel(0., 0., _currentZoom);
	//panModel(_currentPanX, _currentPanY, _currentZoom);
	//panModel(curCenter.x(), curCenter.y(), curCenter.z());
	//if (dx > 0.)
	//    rotateWithFixedVertical(0.05, 0., Vec3f(0, -1, 0));
	//else
	//	rotateWithFixedVertical(-0.05, 0., Vec3f(0, -1, 0));
	//_myHandler->addBackground("wall3.jpg", "wall3.jpg");
	return true;
}

// doc in parent
bool  MyManipulator::performCameraTranslate(const double dx, const double dy)
{
	//float scale = -0.3f * _distance * getThrowScale(eventTimeDelta);
	//panModel(dx*scale, dy*scale);
	panModel(-dx * 100, -dy * 100);
	_currentPanX += -dx * 100;
	_currentPanY += -dy * 100;
	return true;
}
bool MyManipulator::performCameraZoom(const double zoomFactor)
{

	//if (_currentZoom + zoomFactor > _maxZoom || _currentZoom + zoomFactor < (-_maxZoom))
	//{
	//	cout << "zoom too far!!" << endl;
	//}
	//else
	{
		zoomModel(zoomFactor, true);
		//panModel(0, 0, zoomFactor * 500);
		_currentZoom += zoomFactor * 500;
		_myHandler->addBackground();
		_view->requestRedraw();
		_view->requestContinuousUpdate(false);
	}	

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


	performMovement(tx, ty, function,_myClientNum);
	//_manipulator2->performCameraTranslate(-_eyeDistance, 0);
	//_manipulator2->performMovement(tx, ty, function, _myClientNum);
	//_manipulator2->performCameraTranslate(_eyeDistance, 0);
	//_myHandler->addBackground("wall1.jpg");
	//_thrown = false

	return true;
}




/// Handles GUIEventAdapter::PUSH event.
bool MyManipulator::handleMousePush(const GUIEventAdapter& ea, GUIActionAdapter& us)
{
	flushMouseEventStack(_myClientNum);
	//_manipulator2->flushMouseEventStack(_myClientNum);
	float tx = ea.getXnormalized();
	float ty = ea.getYnormalized();
	unsigned int buttonMask = ea.getButtonMask();
	int function = 0;
	if (buttonMask == GUIEventAdapter::LEFT_MOUSE_BUTTON)
		function = 1;
	else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
		function = 2;


	performMovement(tx, ty, function, _myClientNum);

	//_manipulator2->performCameraTranslate(-_eyeDistance, 0);
	//_manipulator2->performMovement(tx, ty, function, _myClientNum);
	//_manipulator2->performCameraTranslate(_eyeDistance, 0);
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

	performMovement(tx, ty, function, _myClientNum);
	//_manipulator2->performCameraTranslate(-_eyeDistance, 0);
	//_manipulator2->performMovement(tx, ty, function, _myClientNum);
	//_manipulator2->performCameraTranslate(_eyeDistance, 0);
	flushMouseEventStack(_myClientNum);
	//_manipulator2->flushMouseEventStack(_myClientNum);
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
		//_manipulator2->performCameraTranslate(0, -_eyeDistance);
		return true;
		break;
	case GUIEventAdapter::KEY_Down:
		performCameraTranslate(0, 0.1);
		//_manipulator2->performCameraTranslate(0, _eyeDistance);
		return true;
		break;
	case GUIEventAdapter::KEY_Left:
		performCameraTranslate(0.1, 0);
		//_manipulator2->performCameraTranslate(_eyeDistance, 0);
		return true;
		break;
	case GUIEventAdapter::KEY_Right:
		performCameraTranslate(-0.1, 0);
		//_manipulator2->performCameraTranslate(-_eyeDistance, 0);
		return true;
		break;
	case 'p':
	case 'P':
		cout << "P::" << endl;
		cout << "rotate back" << endl;
		performCameraRotateBack();
		return true;
		break;

		//performCameraRotate(0.05f, 0.0);
		
		//Vec3d tvec1 = _rotation * Vec3d(1., 0., 0.);
		//double tx = tvec1.x();
		//double tz = tvec1.z();
		//if (abs(tx) > abs(tz)) {
		//	if (tx > 0) cout << "+X" << endl;
		//	else cout << "-X" << endl;
		//}
		//else {
		//	if (tz > 0) cout << "+Z" << endl;
		//	else cout << "-Z" << endl;
		//}



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
		//_manipulator2->performCameraTranslate(-_eyeDistance, 0);
		//_manipulator2->performCameraZoom(_zoomFactor);
		//_manipulator2->performCameraTranslate(_eyeDistance, 0);
		return true;
	}

		// mouse scroll down event
	case GUIEventAdapter::SCROLL_DOWN:
	{
		// perform zoom
		performCameraZoom(-_zoomFactor);
	// _manipulator2->performCameraTranslate(-_eyeDistance, 0);
	// _manipulator2->performCameraZoom(-_zoomFactor);
	// _manipulator2->performCameraTranslate(_eyeDistance, 0);
		return true;
	}

		// unhandled mouse scrolling motion
	default:
		return false;
	}
}