#include "Utils.h"
#include "osgbUtil.h"



class ShakeManipulator : public osgGA::GUIEventHandler
{
public:
	ShakeManipulator(osgbDynamics::MotionState* motion, btDynamicsWorld* bulletWorld, btRigidBody* rigidBody = NULL, Group* root = NULL, MatrixTransform* tTrans = NULL)
		: _motion(motion), _bw(bulletWorld), _rbody(rigidBody), _root(root), _trans(tTrans)
	{}
	void setRigidBody(btRigidBody* rigidBody) {
		_rbody = rigidBody;
	}

	//btDynamicsWorld* dw, osg::Camera* scene )
	//	: _dw(dw),
	//	_scene(scene),
	//	_constraint(NULL),
	//	_constrainedMotionState(NULL),
	//	_pt(NULL)
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
	{
		//const bool ctrl((ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) != 0);
		//if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH)
		//{
		//	if (!ctrl ||
		//		((ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) == 0))
		//		return(false);
		//	osg::notify(osg::WARN) << "DragHandler:pick." << std::endl;
		//	const bool picked = pick(ea.getXnormalized(), ea.getYnormalized());
		//	if (picked)
		//		_constraint->getRigidBodyA().activate(true);
		//	return(picked);
		//}
		//else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
		//{
		//	osg::notify(osg::WARN) << "DragHandler: DRAG." << std::endl;
		//	if ((!ctrl) || (_constraint == NULL) ||
		//		((ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) == 0))
		//		return(false);
		//	osg::Vec4d farPointNDC = osg::Vec4d(ea.getXnormalized(), ea.getYnormalized(), 1., 1.);
		//	osg::Matrix p = _scene->getProjectionMatrix();
		//	double zNear, zFar, fovy, aspect;
		//	p.getPerspective(fovy, aspect, zNear, zFar);
		//	osg::Vec4d farPointCC = farPointNDC * zFar;
		//	p.invert(p);
		//	osg::Matrix v = _scene->getViewMatrix();
		//	v.invert(v);
		//	osg::Vec4d farPointWC = farPointCC * p * v;
		//	osg::Vec3d look, at, up;
		//	_scene->getViewMatrixAsLookAt(look, at, up);
		//	// Intersect ray with plane.
		//	// TBD. Stolen from osgWorks' MxCore::intersectPlaneRay(), which really should be in some math library somewhere.
		//	osg::Vec3d planeNormal = osg::Vec3d(_dragPlane[0], _dragPlane[1], _dragPlane[2]);
		//	const osg::Vec3d vDir = osg::Vec3(farPointWC[0], farPointWC[1], farPointWC[2]) - look;
		//	const double dotVd = vDir * planeNormal;
		//	if (dotVd == 0.)
		//	{
		//		osg::notify(osg::WARN) << "DragHandler: No plane intersection." << std::endl;
		//		return(false);
		//	}
		//	double length = -(planeNormal * look + _dragPlane[3]) / dotVd;
		//	osg::Vec3 pointOnPlane = look + (vDir * length);
		//	osg::notify(osg::DEBUG_FP) << "  OSG point " << pointOnPlane << std::endl;
		//	if (_pt != NULL)
		//		_pt->pause(true);
		//	osg::Matrix ow2bw;
		//	if (_constrainedMotionState != NULL)
		//		ow2bw = _constrainedMotionState->computeOsgWorldToBulletWorld();
		//	osg::Vec3d bulletPoint = pointOnPlane * ow2bw;
		//	osg::notify(osg::DEBUG_FP) << "    bullet point " << bulletPoint << std::endl;
		//	_constraint->setPivotB(osgbCollision::asBtVector3(bulletPoint));
		//	if (_pt != NULL)
		//		_pt->pause(false);
		//	return(true);
		//}
		//else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE)
		//{
		//	if (_constraint == NULL)
		//		return(false);
		//	if (_pt != NULL)
		//		_pt->pause(true);
		//	_dw->removeConstraint(_constraint);
		//	if (_pt != NULL)
		//		_pt->pause(false);
		//	delete _constraint;
		//	_constraint = NULL;
		//	_constrainedMotionState = NULL;
		//	return(true);
		//}

		switch (ea.getEventType())
		{
		case GUIEventAdapter::KEYUP:
		{
			if (ea.getKey() == GUIEventAdapter::KEY_Space)
			{
				btTransform trans; trans.setIdentity();
				_motion->setWorldTransform(trans);

				return true;
			}

			return false;
		}

		case GUIEventAdapter::PUSH:
		{
			_lastX = ea.getXnormalized();
			_lastY = ea.getYnormalized();

			btTransform world;
			_motion->getWorldTransform(world);
			btVector3 o = world.getOrigin();
			o[2] = 0.25;
			world.setOrigin(o);
			_motion->setWorldTransform(world);

			return true;
		}
		case GUIEventAdapter::DRAG:
		{
			btVector3 move;

			move[0] = _lastX - ea.getXnormalized();
			move[1] = ea.getYnormalized() - _lastY;
			move[2] = 0.;
			move *= 10.;

			btTransform moveTrans; moveTrans.setIdentity();
			moveTrans.setOrigin(move);
			btTransform world;
			_motion->getWorldTransform(world);
			btTransform netTrans = moveTrans * world;
			btVector3 o = netTrans.getOrigin();
			o[2] = 0.;
			netTrans.setOrigin(o);

			_motion->setWorldTransform(netTrans);

			_lastX = ea.getXnormalized();
			_lastY = ea.getYnormalized();

			return true;
		}
		case  GUIEventAdapter::KEYDOWN:
		{
			btTransform world;
			_motion->getWorldTransform(world);
			Vec3 scaleVec = _motion->getScale();
			Matrix matrix = osgbCollision::asOsgMatrix(world);
			bool scaleFlag = false;
			bool rotateFlag = false;
			switch (ea.getKey())
			{
			case 'm':
			case 'M':
				scaleVec = scaleVec * 1.02;
				scaleFlag = true;
				break;
			case 'n':
			case 'N':
				scaleVec = scaleVec * 0.98;
				scaleFlag = true;
				break;
			case 'k':
			case 'K':
				matrix *= Matrix::rotate(-0.2f, X_AXIS);
				rotateFlag = true;
				break;
			case 'l':
			case 'L':
				matrix *= Matrix::rotate(0.2f, X_AXIS);
				rotateFlag = true;
				break;
			case 'b':
			case 'B':
				//btRigidBody* tmp = _rbody;
				_root->removeChild(_trans);
				if (_rbody == NULL) cout << "NULL" << endl;
				
				//_bw->removeRigidBody(_rbody);
				//_rbody->setCollisionFlags(DefaultCollisionFlags);
				
				//_bw->addRigidBody(_rbody);
				//_root->addChild(_trans);
				break;
			case 'c':
			case 'C':
				//btRigidBody* tmp = _rbody;
				//_root->addChild(makeObj(_bw, "dice.osg"));

				//_rbody->setCollisionFlags(DefaultCollisionFlags);
			
				//_bw->addRigidBody(_rbody);
				//_root->addChild(_trans);
				break;
			}
		    

			if (scaleFlag) {
				_motion->setScale(scaleVec);
				_motion->setWorldTransform(world);
				return true;
			}
			if (rotateFlag) {
				_motion->setWorldTransform(osgbCollision::asBtTransform(matrix));
			}

			return true;
		}
			
		default:
			break;
		}
		return false;
	}

protected:
	btDynamicsWorld* _bw;
	Group* _root;
	osgbDynamics::MotionState* _motion;
	osgbDynamics::MotionState* _motion2;
	btRigidBody* _rbody;
	MatrixTransform * _trans;
	float _lastX, _lastY;
};