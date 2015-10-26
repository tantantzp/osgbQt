#ifndef PICKMODELHANDLER_H
#define PICKMODELHANDLER_H

#include "Utils.h"
#include "osgbUtil.h"


class PickModelHandler : public osgGA::GUIEventHandler
{
public:
	PickModelHandler(btCollisionWorld* collisionWorld, Group* root, osgViewer::Viewer* view) : _selectionBox(0), _lastModel(0), _selectCollisionObj(0) {
		setCollisionWorld(collisionWorld);
		_objMap.clear();

		root->addChild(getOrCreateSelectionBox());
		_root = root;
		_view = view;
		_hasGround = false;
		_transStep = 3.0f;

		_groundWidthX = 0;
		_groundWidthZ = 0;
		_groundHeightY = 0;
	}
	Node *getOrCreateSelectionBox();
	void  PickModelHandler::detectCollision(bool& colState, btCollisionWorld* cw);
	virtual bool handle(const osgGA::GUIEventAdapter &, osgGA::GUIActionAdapter &);

	void setCollisionObject(btCollisionObject* co) { _selectCollisionObj = co; }
	void setMatrixTransform(osg::MatrixTransform* sBox) { _selectionBox = sBox; }
	void setCollisionWorld(btCollisionWorld* btcw) { _collisionWorld = btcw; }
	void insertObjPair(MatrixTransform* transObj, btCollisionObject* collisionObj)
	{
		_objMap.insert(make_pair(transObj, collisionObj));
	}

	//handle scene
	void addGround(float widthX, float widthZ, float heightY);
	bool addOneObj(string objPath, Vec3d initPos);
	bool addOneObj(MatrixTransform * matrixTrans);
	bool doAddObj(MatrixTransform * matrixTrans);

protected:
	bool translateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d transVec);
	bool rotateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Matrix rotMatrix);
	bool scaleOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d scaleVec);
	void handleKeyEvent(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
	void handlePickEvent(float clickX, float clickY);//const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
	bool deleteOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj);
	bool duplicateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj);


protected:
	ref_ptr<MatrixTransform> _selectionBox;		// bounding box of the selected model;
	ref_ptr<MatrixTransform> _lastModel;
	btCollisionObject*  _selectCollisionObj;
	btCollisionWorld* _collisionWorld;
	Group* _root;
	osgViewer::Viewer* _view;


	map<MatrixTransform *, btCollisionObject *> _objMap;

	double _lastX, _lastY;
	double _groundWidthX, _groundWidthZ, _groundHeightY;
	bool _colState;
	float _transStep;
	bool _hasGround;
};

bool PickModelHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
	// only be called when 	the user is holding the Ctrl key and releasing the left mouse button
	if (ea.getEventType() == GUIEventAdapter::RELEASE &&
		ea.getButton() == GUIEventAdapter::LEFT_MOUSE_BUTTON &&
		(ea.getModKeyMask() & GUIEventAdapter::MODKEY_CTRL))
	{
		float clickX = ea.getX(), clickY = ea.getY();

		handlePickEvent(clickX, clickY);
	}
	else if (ea.getEventType() == GUIEventAdapter::KEYDOWN && _lastModel != NULL && _selectionBox != NULL)
	{
		handleKeyEvent(ea, aa);
	}

	return false;
}

bool  PickModelHandler::doAddObj(MatrixTransform * trans)
{
	_root->addChild(trans);
	btCollisionObject* btBoxObject = new btCollisionObject;
	Node* model = trans->getChild(0);
	Matrix transMatrix = trans->getMatrix();
	btBoxObject->setCollisionShape(osgbCollision::btConvexHullCollisionShapeFromOSG(model));
	//btBoxObject1->setCollisionShape(osgbCollision::btBoxCollisionShapeFromOSG(model1.get()));

	btBoxObject->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
	btBoxObject->setWorldTransform(osgbCollision::asBtTransform(transMatrix));
	_collisionWorld->addCollisionObject(btBoxObject);

	this->insertObjPair(trans, btBoxObject);

	bool collisionFlag = true;

	_collisionWorld->performDiscreteCollisionDetection();
	detectCollision(collisionFlag, _collisionWorld);

	if (collisionFlag) {
		Vec3d initVec = transMatrix.getTrans();
		Vec3d transVec[4] = { Vec3d(30., 0, 0), Vec3d(0, 0, 30.), Vec3d(-30., 0, 0), Vec3d(0, 0, -30.) }; // Vec3d(0, -30., 0)

		for (int k = 1; k < 20; k++)
		{
			bool canbreak = false;
			for (int i = 0; i < 4; i++){

				Vec3d tmp = transVec[i] * k;
				if (tmp.x() < _groundWidthX  && tmp.y()  < _groundHeightY && tmp.z() < _groundWidthZ
					&& tmp.x() > -_groundWidthX  && tmp.y()  > -_groundHeightY && tmp.z() > -_groundWidthZ) {
					if (translateOneObj(trans, NULL, btBoxObject, tmp)) {
						collisionFlag = false;
						canbreak = true;
						break;
					}
				}
			}
			if (canbreak) break;
		}
	}

	if (collisionFlag) {
		cout << "collision happen, add obj failed!" << endl;
		_root->removeChild(trans);
		_collisionWorld->removeCollisionObject(btBoxObject);
		return false;
	}
	else {


		return true;
	}


}

bool PickModelHandler::addOneObj(string objPath, Vec3d initPos){
	//ref_ptr<Node> model1 = osgDB::readNodeFile("D:/ProgramLib/objs/chair/chair_3.skp/chair_3.obj");  //;cow.osg");
	ref_ptr<Node> model = osgDB::readNodeFile(objPath);  
	Matrix transMatrix = osg::Matrix::translate(initPos.x(), initPos.y(), initPos.z());
	ref_ptr<MatrixTransform> trans = new MatrixTransform(transMatrix);
	trans->addChild(model.get());
	return doAddObj(trans.get());
}

bool PickModelHandler::addOneObj(MatrixTransform * matrixTrans) {
	ref_ptr<MatrixTransform> trans = new MatrixTransform(*matrixTrans, CopyOp::DEEP_COPY_ALL);
	return doAddObj(trans.get());
}



void PickModelHandler::addGround(float widthX, float widthZ, float heightY)
{
	if (_hasGround){
		cout << "already has ground" << endl;
		return;
	}
	/* BEGIN: Create environment boxes */
	_groundWidthX = widthX;
	_groundWidthZ = widthZ;
	_groundHeightY = heightY;

	float xDim(widthX);
	float yDim(heightY);
	float zDim(widthZ);
	float thick(.1);
	osg::MatrixTransform* shakeBox = new osg::MatrixTransform;
	btCompoundShape* cs = new btCompoundShape;
	{ // floor -Z
		osg::Vec3 halfLengths(xDim, yDim, thick);
	//	osg::Vec3 halfLengths(xDim*.5, yDim*.5, thick*.5);
		osg::Vec3 center(0., 0., zDim);
		//shakeBox->addChild(osgBox(center, halfLengths));
		btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(osgbCollision::asBtVector3(center));
		cs->addChildShape(trans, box);
	}
	{ // top +Z 
		osg::Vec3 halfLengths(xDim, yDim, thick);
		osg::Vec3 center(0., 0., -zDim);
		shakeBox->addChild( osgBox( center, halfLengths ) );
		btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(osgbCollision::asBtVector3(center));
		cs->addChildShape(trans, box);
	}
	{ // left -X
		osg::Vec3 halfLengths(thick, yDim, zDim);
		osg::Vec3 center(-xDim, 0., 0.);
		shakeBox->addChild(osgBox(center, halfLengths));
		btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(osgbCollision::asBtVector3(center));
		cs->addChildShape(trans, box);
	}
	{ // right +X
		osg::Vec3 halfLengths(thick, yDim, zDim);
		osg::Vec3 center(xDim, 0., 0.);
		shakeBox->addChild(osgBox(center, halfLengths));
		btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(osgbCollision::asBtVector3(center));
		cs->addChildShape(trans, box);
	}
	{ //  -Y  roof
		osg::Vec3 halfLengths(xDim, thick, zDim);
		osg::Vec3 center(0., -yDim, 0.);
		//shakeBox->addChild(osgBox(center, halfLengths));
		btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(osgbCollision::asBtVector3(center));
		cs->addChildShape(trans, box);
	}
	{ //  -Y floor
		osg::Vec3 halfLengths(xDim, thick, zDim);
		osg::Vec3 center(0., yDim, 0.);
		shakeBox->addChild(osgBox(center, halfLengths));
		btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(osgbCollision::asBtVector3(center));
		cs->addChildShape(trans, box);
	}
	/* END: Create environment boxes */
	shakeBox->setNodeMask(0x1);  //avoid choosing the Node Mask
	_root->addChild(shakeBox);
	btCollisionObject* btground = new btCollisionObject;
	btground->setCollisionShape(cs);
	btground->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
	//btground->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
	//btground->setWorldTransform(osgbCollision::asBtTransform(transMatrix1));
	_collisionWorld->addCollisionObject(btground);
	_hasGround = true;
}

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

bool PickModelHandler::deleteOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj) 
{
	if (model != NULL && box != NULL && collisionObj != NULL){
		_root->removeChild(model);
		_collisionWorld->removeCollisionObject(collisionObj);
		
		_lastModel = NULL;
		_selectCollisionObj = NULL;
		_selectionBox->setNodeMask(0); //hide the box

		//delete model;
		delete collisionObj;

		return true;
	}
	else {
		cout << "delete nothing!" << endl;
		return false;
	}
	return false;
}
bool PickModelHandler::duplicateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj)
{
	if (model != NULL && box != NULL && collisionObj != NULL){
		
		

		//delete model;
		delete collisionObj;

		return true;
	}
	else {
		cout << "duplicate nothing!" << endl;
		return false;
	}
	return false;


}

bool PickModelHandler::translateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d transVec)
{
	Matrix oriSmatrix, smatrix;


	Matrix oriMatrix =  model->getMatrix();
	if (box != NULL)  oriSmatrix = box->getMatrix();
	btTransform oriBtTrans = collisionObj->getWorldTransform();
	btVector3 oriBtScale = collisionObj->getCollisionShape()->getLocalScaling();

	Matrix matrix = model->getMatrix();
	if (box != NULL)  smatrix = box->getMatrix();
	btTransform btTrans = collisionObj->getWorldTransform();
	btVector3 btScale = collisionObj->getCollisionShape()->getLocalScaling();

	Matrix tmpBtMatrix = osgbCollision::asOsgMatrix(btTrans);
	Vec3d transVec3 = tmpBtMatrix.getTrans();


	matrix *= Matrix::translate(transVec);
	if (box != NULL)  smatrix *= Matrix::translate(transVec);
	tmpBtMatrix *= Matrix::translate(transVec);
	//btTrans *= osgbCollision::asBtTransform(Matrix::translate(transVec));

	model->setMatrix(matrix);
	if (box != NULL)  box->setMatrix(smatrix);
	//collisionObj->setWorldTransform(btTrans);
	collisionObj->setWorldTransform(osgbCollision::asBtTransform(tmpBtMatrix));


	_collisionWorld->performDiscreteCollisionDetection();
	detectCollision(_colState, _collisionWorld);

	if (_colState == true)
	{
		model->setMatrix(oriMatrix);
		if (box != NULL)  box->setMatrix(oriSmatrix);
		collisionObj->setWorldTransform(oriBtTrans);
		_colState = false;
		return false;
	}
	else return true;
}

bool PickModelHandler::rotateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Matrix rotMatrix)
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
		return false;
	}
	else return true;

}
bool PickModelHandler::scaleOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d scaleVec)
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
	if (float(tscaleVec.x()) > 2) {
		cout << "too big" << endl;
		return false;
	}
	else if (float(tscaleVec.x()) < 0.3) {
		cout << "too small" << endl;
		return false;
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
			return false;
		}
		else return true;
	}
	return false;
}



void  PickModelHandler::handlePickEvent(float clickX, float clickY )//const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{

	//osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
	osgViewer::Viewer *viewer = _view;
	if (viewer)
	{

		ref_ptr<osgUtil::LineSegmentIntersector> intersector =
			new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, clickX, clickY);//ea.getX(), ea.getY());
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



					_selectCollisionObj = _objMap[_lastModel];
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
		bool isDuplicate = false;
		bool isDelete = false;
		Vec3d scaleFactor(1, 1, 1);
		Matrix rotMatrix;
		Vec3d transVec;

		switch (ea.getKey())
		{
		//rotate around Y axis
		case 'k':    
		case 'K':
			rotMatrix = Matrix::rotate(-0.2f, Y_AXIS);;
			isRotate = true;
			break;
		case 'l':
		case 'L':
			rotMatrix = Matrix::rotate(0.2f, Y_AXIS);;
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
			transVec = Vec3d(-_transStep, 0.0f, 0.0f);
			isTranslate = true;
			break;
		case 'a':
		case 'A':
			transVec = Vec3d(_transStep, 0.0f, 0.0f);
			isTranslate = true;
			break;
	   //move along y(up/down)
		case 'w':
		case 'W':
			transVec = Vec3d(0.0f, -_transStep, 0.0f);
			isTranslate = true;
			break;
		case 's':
		case 'S':
			transVec = Vec3d(0.0f, _transStep, 0.0f);
			isTranslate = true;
			break;
		case 'x':
		case 'X':
			transVec = Vec3d(0.0f, 0.0f, _transStep);
			isTranslate = true;
			break;
		case 'c':
		case 'C':
			transVec = Vec3d(0.0f, 0.0f, -_transStep);
			isTranslate = true;
			break;
		case 'u':  //duplicate
		case 'U':
			isDuplicate = true;
			break;
		case 'i':  //delete
		case 'I':
			isDelete = true;
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

		else if (isDuplicate) {
			addOneObj(_lastModel);

		}
		else if (isDelete) {
			deleteOneObj(_lastModel, _selectionBox, _selectCollisionObj);
		}
	}
}






#endif
