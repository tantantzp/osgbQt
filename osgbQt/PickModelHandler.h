#ifndef PICKMODELHANDLER_H
#define PICKMODELHANDLER_H

#include "Utils.h"


class PickModelHandler : public osgGA::GUIEventHandler
{
public:
	PickModelHandler() : _selectionBox(0), _lastModel(0), _selectCollisionObj(0), _collisionWorld(0){
		objMap.clear();
		transStep = 1.0f;
	}
	Node *getOrCreateSelectionBox();
	void  PickModelHandler::detectCollision(bool& colState, btCollisionWorld* cw);
	virtual bool handle(const osgGA::GUIEventAdapter &, osgGA::GUIActionAdapter &);

	void setCollisionObject(btCollisionObject* co) { _selectCollisionObj = co; }
	void setMatrixTransform(osg::MatrixTransform* sBox) { _selectionBox = sBox; }
	void setCollisionWorld(btCollisionWorld* btcw) { _collisionWorld = btcw; }
	void insertObjPair(MatrixTransform* transObj, btCollisionObject* collisionObj)
	{
		objMap.insert(make_pair(transObj, collisionObj));
	}
protected:
	ref_ptr<MatrixTransform> _selectionBox;		// bounding box of the selected model;
	ref_ptr<MatrixTransform> _lastModel;
	btCollisionObject*  _selectCollisionObj;
	btCollisionWorld* _collisionWorld;


	map<MatrixTransform *, btCollisionObject *> objMap;

	double _lastX, _lastY;
	bool _colState;
	float transStep;
};

Node *PickModelHandler::getOrCreateSelectionBox()
{
	if (!_selectionBox)
	{
		ref_ptr<ShapeDrawable> boxDrawable = new ShapeDrawable(new Box(Vec3(), 1.0f));
		boxDrawable->setColor(Vec4(1.0f, 0.0f, 0.0f, 0.0f));	// red color
		ref_ptr<Geode> geode = new Geode;
		geode->addDrawable(boxDrawable.get());
		_selectionBox = new MatrixTransform;
		_selectionBox->setNodeMask(0);		//hide this box
		_selectionBox->addChild(geode.get());
		StateSet *ss = _selectionBox->getOrCreateStateSet();
		ss->setMode(GL_LIGHTING, StateAttribute::OFF);
		ss->setAttributeAndModes(new PolygonMode(PolygonMode::FRONT_AND_BACK, PolygonMode::LINE));
	}

	return _selectionBox.get();
}

void  PickModelHandler::detectCollision(bool& colState, btCollisionWorld* cw)
{
	colState = false;
	unsigned int numManifolds = cw->getDispatcher()->getNumManifolds();
	if ((numManifolds == 0))
	{
		//osg::notify(osg::ALWAYS) << "No collision." << std::endl;
		colState = false;
	}
	else {
		for (unsigned int i = 0; i < numManifolds; i++)
		{
			btPersistentManifold* contactManifold = cw->getDispatcher()->getManifoldByIndexInternal(i);
			unsigned int numContacts = contactManifold->getNumContacts();
			for (unsigned int j = 0; j<numContacts; j++)
			{
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				if ((pt.getDistance() <= 0.f) )
				{
					// grab these values for the contact normal arrows:
					osg::Vec3 pos = osgbCollision::asOsgVec3(pt.getPositionWorldOnA()); // position of the collision on object A
					osg::Vec3 normal = osgbCollision::asOsgVec3(pt.m_normalWorldOnB); // returns a unit vector
					float pen = pt.getDistance(); //penetration depth

					osg::Quat q;
					q.makeRotate(osg::Vec3(0, 0, 1), normal);

					osg::notify(osg::ALWAYS) << "Collision detected." << std::endl;

					/*osg::notify(osg::ALWAYS) << "\tPosition: " << pos << std::endl;
					osg::notify(osg::ALWAYS) << "\tNormal: " << normal << std::endl;
					osg::notify(osg::ALWAYS) << "\tPenetration depth: " << pen << std::endl;*/
					//osg::notify( osg::ALWAYS ) << q.w() <<","<< q.x() <<","<< q.y() <<","<< q.z() << std::endl;
					colState = true;
				}
				else if ((pt.getDistance() > 0.f) )
				{
					//osg::notify(osg::ALWAYS) << "No collision." << std::endl;
					colState = false;
				}
			}
		}
	}
}

bool PickModelHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	// only be called when 	the user is holding the Ctrl key and releasing the left mouse button
	if (ea.getEventType() == GUIEventAdapter::RELEASE &&
		ea.getButton() == GUIEventAdapter::LEFT_MOUSE_BUTTON &&
		(ea.getModKeyMask() & GUIEventAdapter::MODKEY_CTRL))
	{

		osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
		if (viewer)
		{

			ref_ptr<osgUtil::LineSegmentIntersector> intersector = 
				new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, ea.getX(), ea.getY());
			osgUtil::IntersectionVisitor iv(intersector.get());
			iv.setTraversalMask(~0x1);    //avoid choosing the Node Mask
			viewer->getCamera()->accept(iv);

			// update last selected model.
			if (intersector->containsIntersections())
			{
				const osgUtil::LineSegmentIntersector::Intersection &result = *(intersector->getIntersections().begin());
				BoundingBox bb = result.drawable->getBound();
				//Vec3 worldCenter = bb.center() * computeLocalToWorld(result.nodePath);

				_selectionBox->setNodeMask(0x1);  //avoid choosing the Node Mask
				Matrix mat = Matrix::scale(bb.xMax() - bb.xMin(), bb.yMax() - bb.yMin(), bb.zMax() - bb.zMin())/* * Matrix::translate(worldCenter)*/;
				//_selectionBox->setMatrix(mat);

				// _lastModel
				const NodePath &np = result.nodePath;	//NodePath: Group(root)-->MatrixTransform(trans)-->Group(?)-->Geode(model)
				int nSize = np.size();

				bool flag = false;
				for (int i = nSize - 1; i >= 0; i--)
				{
					cout << np[i]->className() << endl;

				    int xmax = INT_MIN, xmin = INT_MAX, ymax INT_MIN, ymin = INT_MAX, zmax INT_MIN, zmin = INT_MAX;
					bool hasgeoflag = false;
					if (strcmp(np[i]->className(), "MatrixTransform") == 0)
					{
						_lastModel = dynamic_cast<MatrixTransform *>(np[i]);	
						int childNum = _lastModel->getNumChildren();


						for (int j = 0; j < childNum; j++) {
							if (strcmp(_lastModel->getChild(j)->className(), "Group") == 0) {
								Group*  tgroup = dynamic_cast<Group *>(_lastModel->getChild(j));
								int groupChildNum = tgroup->getNumChildren();
								for (int k = 0; k < groupChildNum; k++) {
									Node* tmpNode = tgroup->getChild(k);
									if (strcmp(tmpNode->className(), "Geode") == 0) {
										Geode*  tgeo = dynamic_cast<Geode *>(tmpNode);
										int drawNum = tgeo->getNumDrawables();
										if (drawNum > 0) {
											BoundingBox tbb = tgeo->getDrawable(0)->getBound();
											if (tbb.xMax() > xmax) xmax = tbb.xMax();
											if (tbb.xMin() < xmin) xmin = tbb.xMin();
											if (tbb.yMax() > ymax) ymax = tbb.yMax();
											if (tbb.yMin() < ymin) ymin = tbb.yMin();
											if (tbb.zMax() > zmax) zmax = tbb.zMax();
											if (tbb.zMin() < zmin) zmin = tbb.zMin();
											hasgeoflag = true;
										}
									 }
							     }
						    }
						}
						
						cout << "hasgeoflag:  " << flag << endl;
						if (hasgeoflag) 
							mat = Matrix::scale(xmax - xmin, ymax - ymin, zmax - zmin);
						else {
							BoundingSphere tbb = _lastModel->getBound();
							//cout << tbb.radius() << "  " << tbb.radius2() << "  " << tbb.center() << endl;
							float radius = tbb.radius() * 0.6;
							mat = Matrix::scale(2 * radius, 2 * radius, 2 * radius);
						}



						_selectCollisionObj = objMap[_lastModel];
        				mat *= _lastModel->getMatrix();
						flag = true;
						//cout << "flag true" << endl;
						break;
					}
				}
				//if (nSize >= 3)
				//{
				//	//???why is 'nSize - 3'. 
				//	
				//	_lastModel = dynamic_cast<MatrixTransform *>(np[nSize - 3]);
				//	
				//	_selectCollisionObj = objMap[_lastModel];
				//	mat *= _lastModel->getMatrix();
				//}
				if (!flag)
				{
					_lastModel = NULL;
					_selectCollisionObj = NULL;
				}

			   _selectionBox->setMatrix(mat);
				
			}
			else
			{
				_selectionBox->setNodeMask(0); //hide the box
				_lastModel = NULL;
				
			}
		}
	}
	else if (ea.getEventType() == GUIEventAdapter::KEYDOWN && _lastModel != NULL && _selectionBox != NULL)
	{
		// if we want to rotate/scale around its own center,
		// we should first translate to the world origin
		// and then perform the rotate/scale. Finally translate back.

		// Only rotation around the Z_Axis is allowed in this demo\

		Matrix oriMatrix = _lastModel->getMatrix();
		Matrix oriSmatrix = _selectionBox->getMatrix();

		Matrix matrix = _lastModel->getMatrix();
		Matrix smatrix = _selectionBox->getMatrix();

		//Vec3d oritmpTrans = matrix.getTrans();
		//Quat oritmpQuat = matrix.getRotate();
		//Matrix oritMatrix;
		//oritMatrix *= Matrix::translate(oritmpTrans);
		//oritMatrix *= Matrix::rotate(oritmpQuat);

		Vec3d transVec1 = matrix.getTrans();
		Vec3d transVec2 = smatrix.getTrans();
		// transVec3 = GetWorldTransform();
		matrix *= Matrix::translate(-transVec1);
		smatrix *= Matrix::translate(-transVec2);

		//bool rotateflag = false;
		switch (ea.getKey())
		{
		case 'k':
		case 'K':
			matrix *= Matrix::rotate(-0.1f, Z_AXIS);
			smatrix *= Matrix::rotate(-0.1f, Z_AXIS);
			break;
		case 'l':
		case 'L':
			matrix *= Matrix::rotate(0.1f, Z_AXIS);
			smatrix *= Matrix::rotate(0.1f, Z_AXIS);
			break;
		case 'm':
		case 'M':
			matrix *= Matrix::scale(1.1f, 1.1f, 1.1f);
			smatrix *= Matrix::scale(1.1f, 1.1f, 1.1f);
			break;
		case 'n':
		case 'N':
			matrix *= Matrix::scale(0.9f, 0.9f, 0.9f);
			smatrix *= Matrix::scale(0.9f, 0.9f, 0.9f);
			break;
		case 'd':
		case 'D':
			transVec1 += Vec3d(transStep, 0.0f, 0.0f);
			transVec2 += Vec3d(transStep, 0.0f, 0.0f);
			break;
		case 'a':
		case 'A':
			transVec1 += Vec3d(-transStep, 0.0f, 0.0f);
			transVec2 += Vec3d(-transStep, 0.0f, 0.0f);
			break;
		case 'w':
		case 'W':
			transVec1 += Vec3d(0.0f, 0.0f, transStep);
			transVec2 += Vec3d(0.0f, 0.0f, transStep);
			break;
		case 's':
		case 'S':
			transVec1 += Vec3d(-0.0f, 0.0f, -transStep);
			transVec2 += Vec3d(-0.0f, 0.0f, -transStep);
			break;
		case 'x':
		case 'X':
			transVec1 += Vec3d(0.0f, transStep, 0.0f);
			transVec2 += Vec3d(0.0f, transStep, 0.0f);
			break;
		case 'c':
		case 'C':
			transVec1 += Vec3d(-0.0f, -transStep, 0.0f);
			transVec2 += Vec3d(-0.0f, -transStep, 0.0f);
			break;
		default:
			break;
		}
		//if (rotateflag)
		//{
		matrix *= Matrix::translate(transVec1);
		smatrix *= Matrix::translate(transVec2);
		_lastModel->setMatrix(matrix);
		_selectionBox->setMatrix(smatrix);
		btCollisionObject* tmpColObj = _selectCollisionObj;
		_collisionWorld->removeCollisionObject(_selectCollisionObj);
		_selectCollisionObj = new btCollisionObject;
		_selectCollisionObj->setCollisionShape(osgbCollision::btConvexHullCollisionShapeFromOSG(_lastModel.get()));
		_selectCollisionObj->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
		_collisionWorld->addCollisionObject(_selectCollisionObj);
		_collisionWorld->performDiscreteCollisionDetection();
		detectCollision(_colState, _collisionWorld);
		if (_colState == true)
		{
			_lastModel->setMatrix(oriMatrix);
			_selectionBox->setMatrix(oriSmatrix);
			_collisionWorld->removeCollisionObject(_selectCollisionObj);
			delete _selectCollisionObj;
			_selectCollisionObj = tmpColObj;
			_collisionWorld->addCollisionObject(_selectCollisionObj);
			_colState = false;
		}
		else
		{
			delete tmpColObj;
			objMap[_lastModel] = _selectCollisionObj;
		}
	    //}
	//	else
		
		//{
		//	bool isScale = false;
		//	btVector3 scaleFactor(1, 1, 1);
		//	btVector3 unscaleFactor(1, 1, 1);
		//	switch (ea.getKey())
		//	{
		//	case 'k':
		//	case 'K':
		//		matrix *= Matrix::rotate(-0.1f, Z_AXIS);
		//		smatrix *= Matrix::rotate(-0.1f, Z_AXIS);
		//		break;
		//	case 'l':
		//	case 'L':
		//		matrix *= Matrix::rotate(0.1f, Z_AXIS);
		//		smatrix *= Matrix::rotate(0.1f, Z_AXIS);
		//		break;
		//	case 'm':
		//	case 'M':
		//		matrix *= Matrix::scale(1.1f, 1.1f, 1.1f);
		//		smatrix *= Matrix::scale(1.1f, 1.1f, 1.1f);
		//		isScale = true;
		//		scaleFactor.setValue(1.1, 1.1, 1.1);
		//		unscaleFactor.setValue(1 / 1.1, 1 / 1.1, 1 / 1.1);
		//		break;
		//	case 'n':
		//	case 'N':
		//		matrix *= Matrix::scale(0.9f, 0.9f, 0.9f);
		//		smatrix *= Matrix::scale(0.9f, 0.9f, 0.9f);
		//		isScale = true;
		//		scaleFactor.setValue(0.9, 0.9, 0.9);
		//		unscaleFactor.setValue(1 / 0.9, 1 / 0.9, 1 / 0.9);
		//		break;
		//	case 'd':
		//	case 'D':
		//		transVec1 += Vec3d(0.2f, 0.0f, 0.0f);
		//		transVec2 += Vec3d(0.2f, 0.0f, 0.0f);
		//		break;
		//	case 'a':
		//	case 'A':
		//		transVec1 += Vec3d(-0.2f, 0.0f, 0.0f);
		//		transVec2 += Vec3d(-0.2f, 0.0f, 0.0f);
		//		break;
		//	case 'w':
		//	case 'W':
		//		transVec1 += Vec3d(0.0f, 0.0f, 0.2f);
		//		transVec2 += Vec3d(0.0f, 0.0f, 0.2f);
		//		break;
		//	case 's':
		//	case 'S':
		//		transVec1 += Vec3d(-0.0f, 0.0f, -0.2f);
		//		transVec2 += Vec3d(-0.0f, 0.0f, -0.2f);
		//		break;
		//	default:
		//		break;
		//	}
		//	matrix *= Matrix::translate(transVec1);
		//	smatrix *= Matrix::translate(transVec2);
		//	//Vec3d tmpTrans = matrix.getTrans();
		//	//Quat tmpQuat = matrix.getRotate();
		//	//Matrix tMatrix;
		//	//tMatrix *= Matrix::translate(tmpTrans);
		//	//tMatrix *= Matrix::rotate(tmpQuat);

		//	_lastModel->setMatrix(matrix);
		//	_selectionBox->setMatrix(smatrix);
		//	if (isScale) {
		//		cout << "scale" << endl;
		//		_selectCollisionObj->getCollisionShape()->setLocalScaling(scaleFactor);
		//		_collisionWorld->updateSingleAabb(_selectCollisionObj);
		//		
		//	}
		//	else {
		//		_selectCollisionObj->setWorldTransform(osgbCollision::asBtTransform(matrix));

		//	}
		//	_collisionWorld->performDiscreteCollisionDetection();
		//	detectCollision(_colState, _collisionWorld);
		//	if (_colState == true)
		//	{
		//		_lastModel->setMatrix(oriMatrix);
		//		_selectionBox->setMatrix(oriSmatrix);
		//		if (isScale) {
		//			cout << "unscale" << endl;
		//			_selectCollisionObj->getCollisionShape()->setLocalScaling(unscaleFactor);
		//			_collisionWorld->updateSingleAabb(_selectCollisionObj);
		//		}
		//		else {
		//			_selectCollisionObj->setWorldTransform(osgbCollision::asBtTransform(oriMatrix));
		//		}
		//		_colState = false;
		//	}
		//}

	}

	return false;
}







#endif
