#ifndef PICKMODELHANDLER_H
#define PICKMODELHANDLER_H

#include "Utils.h"


class PickModelHandler : public osgGA::GUIEventHandler
{
public:
	PickModelHandler(btCollisionWorld* collisionWorld, Group* root) : _selectionBox(0), _lastModel(0), _selectCollisionObj(0) {
		setCollisionWorld(collisionWorld);
		objMap.clear();
		transStep = 1.0f;
		root->addChild(getOrCreateSelectionBox());
		_root = root;
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
	void translateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d transVec);
	void rotateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Matrix rotMatrix);
	void scaleOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d scaleVec);
	void handleKeyEvent(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
	void handlePickEvent(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
protected:
	ref_ptr<MatrixTransform> _selectionBox;		// bounding box of the selected model;
	ref_ptr<MatrixTransform> _lastModel;
	btCollisionObject*  _selectCollisionObj;
	btCollisionWorld* _collisionWorld;
	Group* _root;

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
				if ((pt.getDistance() <= -1.5f) )
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
				else if ((pt.getDistance() > -1.5f) )
				{
					//osg::notify(osg::ALWAYS) << "No collision." << std::endl;
					colState = false;
				}
			}
		}
	}
}


void PickModelHandler::translateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d transVec)
{
	Matrix oriMatrix =  model->getMatrix();
	Matrix oriSmatrix = box->getMatrix();
	btTransform oriBtTrans = collisionObj->getWorldTransform();
	btVector3 oriBtScale = collisionObj->getCollisionShape()->getLocalScaling();

	Matrix matrix = model->getMatrix();
	Matrix smatrix = box->getMatrix();
	btTransform btTrans = collisionObj->getWorldTransform();
	btVector3 btScale = collisionObj->getCollisionShape()->getLocalScaling();

	Matrix tmpBtMatrix = osgbCollision::asOsgMatrix(btTrans);
	Vec3d transVec3 = tmpBtMatrix.getTrans();


	matrix *= Matrix::translate(transVec);
	smatrix *= Matrix::translate(transVec);
	tmpBtMatrix *= Matrix::translate(transVec);
	//btTrans *= osgbCollision::asBtTransform(Matrix::translate(transVec));

	model->setMatrix(matrix);
	box->setMatrix(smatrix);
	//collisionObj->setWorldTransform(btTrans);
	collisionObj->setWorldTransform(osgbCollision::asBtTransform(tmpBtMatrix));


	_collisionWorld->performDiscreteCollisionDetection();
	detectCollision(_colState, _collisionWorld);

	if (_colState == true)
	{
		model->setMatrix(oriMatrix);
		box->setMatrix(oriSmatrix);
		collisionObj->setWorldTransform(oriBtTrans);
		_colState = false;
	}
	return;
}

void PickModelHandler::rotateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Matrix rotMatrix)
{
	Matrix oriMatrix = model->getMatrix();
	Matrix oriSmatrix = box->getMatrix();
	btTransform oriBtTrans = collisionObj->getWorldTransform();
	btVector3 oriBtScale = collisionObj->getCollisionShape()->getLocalScaling();

	Matrix matrix = model->getMatrix();
	Matrix smatrix = box->getMatrix();
	btTransform btTrans = collisionObj->getWorldTransform();
	btVector3 btScale = collisionObj->getCollisionShape()->getLocalScaling();

	Matrix tmpBtMatrix = osgbCollision::asOsgMatrix(btTrans);

	Vec3d transVec1 = matrix.getTrans();
	Vec3d transVec2 = smatrix.getTrans();
	Vec3d transVec3 = tmpBtMatrix.getTrans();

	//in order to rotate around the obj itself
	//first translate to the center
	matrix *= Matrix::translate(-transVec1);
	smatrix *= Matrix::translate(-transVec2);
	tmpBtMatrix *= Matrix::translate(-transVec3);

	//rotate
	matrix *= rotMatrix;
	smatrix *= rotMatrix;
	tmpBtMatrix *= rotMatrix;
	//btTrans *= osgbCollision::asBtTransform(rotMatrix);

	//translate back
	matrix *= Matrix::translate(transVec1);
	smatrix *= Matrix::translate(transVec2);
	tmpBtMatrix *= Matrix::translate(transVec3);

	btTrans *= osgbCollision::asBtTransform(rotMatrix);
	model->setMatrix(matrix);
	box->setMatrix(smatrix);
	//collisionObj->setWorldTransform(btTrans);

	collisionObj->setWorldTransform(osgbCollision::asBtTransform(tmpBtMatrix));

	_collisionWorld->performDiscreteCollisionDetection();
	detectCollision(_colState, _collisionWorld);

	if (_colState == true)
	{
		model->setMatrix(oriMatrix);
		box->setMatrix(oriSmatrix);
		collisionObj->setWorldTransform(oriBtTrans);
		_colState = false;
	}
	return;

}
void PickModelHandler::scaleOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d scaleVec)
{
	Matrix oriMatrix = model->getMatrix();
	Matrix oriSmatrix = box->getMatrix();
	btTransform oriBtTrans = collisionObj->getWorldTransform();
	btVector3 oriBtScale = collisionObj->getCollisionShape()->getLocalScaling();

	Matrix matrix = model->getMatrix();
	Matrix smatrix = box->getMatrix();
	btTransform btTrans = collisionObj->getWorldTransform();
	btVector3 btScale = collisionObj->getCollisionShape()->getLocalScaling();


	Vec3d transVec1 = matrix.getTrans();
	Vec3d transVec2 = smatrix.getTrans();

	matrix *= Matrix::translate(-transVec1);
	smatrix *= Matrix::translate(-transVec2);

	//scale
	float sx = scaleVec.x(), sy = scaleVec.y(), sz = scaleVec.z();
	matrix *= Matrix::scale(sx, sy, sz);
	smatrix *= Matrix::scale(sx, sy, sz);
	btVector3 scaleFactor(sx, sy, sz);

	matrix *= Matrix::translate(transVec1);
	smatrix *= Matrix::translate(transVec2);

	cout << "scale" << endl;
	btVector3 tscaleVec = btScale * scaleFactor;
	if (float(tscaleVec.x()) > 1.6) {
		cout << "too big" << endl;
	}
	else if (float(tscaleVec.x()) < 0.5) {
		cout << "too small" << endl;
	}
	else {
		model->setMatrix(matrix);
		box->setMatrix(smatrix);
		collisionObj->getCollisionShape()->setLocalScaling(tscaleVec);
		collisionObj->setWorldTransform(btTrans);
		_collisionWorld->updateSingleAabb(collisionObj);


		_collisionWorld->performDiscreteCollisionDetection();
		detectCollision(_colState, _collisionWorld);
		if (_colState == true)
		{
			model->setMatrix(oriMatrix);
			box->setMatrix(oriSmatrix);

			cout << "unscale" << endl;
			collisionObj->getCollisionShape()->setLocalScaling(oriBtScale);
			collisionObj->setWorldTransform(oriBtTrans);
			_collisionWorld->updateSingleAabb(collisionObj);

		}
	}
	return;
}

bool PickModelHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	// only be called when 	the user is holding the Ctrl key and releasing the left mouse button
	if (ea.getEventType() == GUIEventAdapter::RELEASE &&
		ea.getButton() == GUIEventAdapter::LEFT_MOUSE_BUTTON &&
		(ea.getModKeyMask() & GUIEventAdapter::MODKEY_CTRL))
	{
		handlePickEvent(ea, aa);
	}
	else if (ea.getEventType() == GUIEventAdapter::KEYDOWN && _lastModel != NULL && _selectionBox != NULL)
	{
		handleKeyEvent(ea, aa);
	}

	return false;
}


void  PickModelHandler::handlePickEvent(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
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

void PickModelHandler::handleKeyEvent(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	// if we want to rotate/scale around its own center,
	// we should first translate to the world origin
	// and then perform the rotate/scale. Finally translate back.

	{
		bool isScale = false;
		bool isRotate = false;
		bool isTranslate = false;
		Vec3d scaleFactor(1, 1, 1);
		Matrix rotMatrix;
		Vec3d transVec;

		switch (ea.getKey())
		{
		//rotate around Y axis
		case 'k':    
		case 'K':
			rotMatrix = Matrix::rotate(-0.1f, Y_AXIS);;
			isRotate = true;
			break;
		case 'l':
		case 'L':
			rotMatrix = Matrix::rotate(0.1f, Y_AXIS);;
			isRotate = true;
			break;
		
	    //scale object
		case 'm':
		case 'M':
			scaleFactor.set(1.1f, 1.1f, 1.1f);
			isScale = true;
			break;
		case 'n':
		case 'N':
			scaleFactor.set(0.9f, 0.9f, 0.9f);
			isScale = true;
			break;
		//move along x(right/left)
		case 'd':
		case 'D':
			transVec = Vec3d(transStep, 0.0f, 0.0f);
			isTranslate = true;
			break;
		case 'a':
		case 'A':
			transVec = Vec3d(-transStep, 0.0f, 0.0f);
			isTranslate = true;
			break;
	   //move along y(up/down)
		case 'w':
		case 'W':
			transVec = Vec3d(0.0f, transStep, 0.0f);
			
			isTranslate = true;
			break;
		case 's':
		case 'S':
			transVec = Vec3d(0.0f, -transStep, 0.0f);
			isTranslate = true;
			break;
		case 'x':
		case 'X':
			transVec = Vec3d(0.0f, 0.0f, transStep);
			isTranslate = true;
			break;
		case 'c':
		case 'C':
			transVec = Vec3d(0.0f, 0.0f, -transStep);
			isTranslate = true;
			break;
		default:
			break;
		}

		if (isTranslate) {
			translateOneObj(_lastModel, _selectionBox, _selectCollisionObj, transVec);
		}
		else if (isRotate) {
			rotateOneObj(_lastModel, _selectionBox, _selectCollisionObj, rotMatrix);
		}
		else if (isScale) {
			scaleOneObj(_lastModel, _selectionBox, _selectCollisionObj, scaleFactor);

		}
	}
}






#endif
