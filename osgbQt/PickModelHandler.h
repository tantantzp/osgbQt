#ifndef PICKMODELHANDLER_H
#define PICKMODELHANDLER_H

#include "Utils.h"
#include "osgbUtil.h"
#include "MyManipulator.h"

#define PIE 3.14f

float degreeToPie(float degree)
{
	float res = degree * PIE / 180.;
	return res;
}

StateSet* createTextureState(Image* img)
{
	StateSet* stateset;
	if (img)
	{
		cout << "wall image!" << endl;
		Texture2D* texture = new Texture2D;
		texture->setWrap(Texture2D::WRAP_S, Texture2D::REPEAT);
		texture->setWrap(Texture2D::WRAP_T, Texture2D::REPEAT);
		//texture->setFilter(Texture::MIN_FILTER, Texture::LINEAR);
		//texture->setFilter(Texture::MAG_FILTER, Texture::NEAREST);

		texture->setImage(img);
		//TexGen* texgen = new TexGen;
		//texgen->setMode(TexGen::OBJECT_LINEAR);
		//texgen->setMode(TexGen::SPHERE_MAP);
		//texgen->setPlane(TexGen::S, Plane(0., 0., 1., 0.));

		stateset = new StateSet;
		stateset->setTextureAttributeAndModes(0, texture, StateAttribute::ON);

		//stateset->setTextureAttributeAndModes(0, texgen, StateAttribute::ON);
		//stateset->setTextureMode(0, GL_TEXTURE_GEN_S, StateAttribute::ON | StateAttribute::OVERRIDE);
		//stateset->setTextureMode(0, GL_TEXTURE_GEN_T, StateAttribute::ON | StateAttribute::OVERRIDE);
	}
	return stateset;
}


class PickModelHandler : public osgGA::GUIEventHandler
{
public:
	PickModelHandler(btCollisionWorld* collisionWorld, Group* root, osgViewer::Viewer* view) {
		_selectionBoxVec.clear();
		_lastModelVec.clear();
		_selectCollisionObjVec.clear();
		setCollisionWorld(collisionWorld);
		_objMap.clear();
		_root = root;
		_view = view;

		_selectNum = 0;
		getOrCreateSelectionBox(_selectNum);

		_hasGround = false;
		_transStep = 10.0f;

		_groundWidthX = 0;
		_groundWidthZ = 0;
		_groundHeightY = 0;
		_orientation = 0;

		_leftVec = Vec3d(-1., 0., 0.);
		_rightVec = Vec3d(1., 0., 0.);
		_forwardVec = Vec3d(0., 0., 1.);
		_backwardVec = Vec3d(0., 0., -1.);
		_upVec = Vec3d(0., -1., 0.);
		_downVec = Vec3d(0., 1., 0.);
		_forwardDegree = 90.;

	}
	MatrixTransform *getOrCreateSelectionBox(unsigned int index);
	void  PickModelHandler::detectCollision(bool& colState, btCollisionWorld* cw);
	virtual bool handle(const osgGA::GUIEventAdapter &, osgGA::GUIActionAdapter &);

	void setCollisionObject(btCollisionObject* co, unsigned int index) { _selectCollisionObjVec[index] = co; }
	//void setMatrixTransform(osg::MatrixTransform* sBox) { _selectionBox = sBox; }
	void setCollisionWorld(btCollisionWorld* btcw) { _collisionWorld = btcw; }
	void insertObjPair(MatrixTransform* transObj, btCollisionObject* collisionObj){_objMap.insert(make_pair(transObj, collisionObj));}

	//handle scene
	void addGround(float widthX, float widthZ, float heightY);
	bool addOneObj(string objPath, Vec3d initPos);
	MatrixTransform* duplicateOneObj(MatrixTransform * matrixTrans);
	bool doAddObj(MatrixTransform * matrixTrans);
	void chooseOneMatrixTransform(MatrixTransform* lastmodel);
	void clearAllchoose();
	void setOrientation(int orientation);

protected:
	bool translateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d transVec);
	bool rotateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Matrix rotMatrix);
	bool scaleOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d scaleVec);
	bool rotateMultipleObj(vector<MatrixTransform*> modelVec, vector<MatrixTransform*> boxVec, vector<btCollisionObject*> collisionObjVec, Matrix rotMatrix);

	void handleKeyEvent(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
	void handlePickEvent(float clickX, float clickY);//const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
	//bool deleteOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj);
	bool deleteOneObj(unsigned int index);
	//bool duplicateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj);
	bool permutateRound(vector<MatrixTransform*> model, vector<MatrixTransform*> box, vector<btCollisionObject*> collisionObj);
	bool permutateRow(vector<MatrixTransform*> modelVec, vector<MatrixTransform*> boxVec, vector<btCollisionObject*> collisionObjVec);
	bool scaleGap(vector<MatrixTransform*> modelVec, vector<MatrixTransform*> boxVec, vector<btCollisionObject*> collisionObjVec, bool );

protected:
	vector< MatrixTransform* > _selectionBoxVec;		// bounding box of the selected model;
	vector< MatrixTransform* > _lastModelVec;
	vector< btCollisionObject* > _selectCollisionObjVec;
	btCollisionWorld* _collisionWorld;
	Group* _root;
	osgViewer::Viewer* _view;

	map<MatrixTransform *, btCollisionObject *> _objMap;


	double _lastX, _lastY;
	double _groundWidthX, _groundWidthZ, _groundHeightY;
	bool _colState;
	float _transStep;
	bool _hasGround;
	unsigned int _selectNum;

	Geode* wallBox[6];
	//orientation and corresponding direction Vec3
	int _orientation; 
	Vec3f _leftVec, _rightVec, _forwardVec, _backwardVec, _upVec, _downVec;
	float _forwardDegree;
	
	//degree


};


void  PickModelHandler::setOrientation(int orientation)
{
	_orientation = orientation;
	cout << "ori:" << orientation << endl;
	//hideWall
	for (int i = 0; i < 5; i++)
	{
		if (i == orientation) wallBox[i]->setNodeMask(0);
		else wallBox[i]->setNodeMask(1);

	}

	_upVec = Vec3d(0., -1., 0.);
	_downVec = Vec3d(0., 1., 0.);
	switch (orientation)
	{
	case(0) :
		_leftVec = Vec3d(0., 0., 1.);
		_rightVec = Vec3d(0., 0., -1.);
		_forwardVec = Vec3d(-1., 0., 0.);
		_backwardVec = Vec3d(1., 0., 0.);
		_forwardDegree = 180.;
		break;
	case(1) :
		_leftVec = Vec3d(1., 0., 0.);
		_rightVec = Vec3d(-1., 0., 0.);
		_forwardVec = Vec3d(0., 0., 1.);
		_backwardVec = Vec3d(0., 0., -1.);
		_forwardDegree = 90.;
		break;
	case(2) :
		_leftVec = Vec3d(0., 0., -1.);
		_rightVec = Vec3d(0., 0., 1.);
		_forwardVec = Vec3d(1., 0., 0.);
		_backwardVec = Vec3d(-1., 0., 0.);
		_forwardDegree = 0.;
		break;
	case(3) :
		_leftVec = Vec3d(-1., 0., 0.);
		_rightVec = Vec3d(1., 0., 0.);
		_forwardVec = Vec3d(0., 0., -1.);
		_backwardVec = Vec3d(0., 0., 1.);
		_forwardDegree = 270.;
		break;
	default:
		break;
	}


}

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
	else if (ea.getEventType() == GUIEventAdapter::KEYDOWN && _selectNum > 0 )//_lastModel != NULL && _selectionBox != NULL)
	{
		handleKeyEvent(ea, aa);
	}

	return false;
}

bool  PickModelHandler::doAddObj(MatrixTransform * trans)
{
	cout << "do add obj" << endl;
	_root->addChild(trans);
	btCollisionObject* btBoxObject = new btCollisionObject;
	Node* model = trans->getChild(0);
	Matrix transMatrix = trans->getMatrix();
	//btBoxObject->setCollisionShape(osgbCollision::btConvexTriMeshCollisionShapeFromOSG(model));
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
		Vec3d transVec[2];
	
		transVec[0] = _leftVec * 30;
		transVec[1] = _rightVec * 30;

		for (int k = 1; k < 20; k++)
		{
			bool canbreak = false;
			for (int i = 0; i < 2; i++){

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
	if (model.get() == NULL) {
		cout << "read node file:" << objPath << "  failed" << endl;
		return false;

	}
	Matrix transMatrix = osg::Matrix::translate(initPos.x(), initPos.y(), initPos.z());
	ref_ptr<MatrixTransform> trans = new MatrixTransform(transMatrix);
	trans->addChild(model.get());
	trans->setNodeMask(CastsShadowTraversalMask);
	return doAddObj(trans.get());
}

MatrixTransform* PickModelHandler::duplicateOneObj(MatrixTransform * matrixTrans)
{
	if (matrixTrans == NULL) {
		return NULL;
	}
	MatrixTransform* trans = new MatrixTransform(*matrixTrans, CopyOp::DEEP_COPY_ALL);
	bool flag = doAddObj(trans);
	//chooseOneMatrixTransform(trans.get());
	if (flag) return trans;
	else return NULL;
	
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
	float thick(2);

	Image* wallImage = osgDB::readImageFile("wall2.jpg");
	Image* groundImage = osgDB::readImageFile("ground.jpg");
	StateSet* wallStateSet;
	StateSet* groundStateSet;
	wallStateSet = createTextureState(wallImage);
	groundStateSet = createTextureState(groundImage);


	osg::MatrixTransform* shakeBox = new osg::MatrixTransform;
	btCompoundShape* cs = new btCompoundShape;
	{ // left -X
		osg::Vec3 halfLengths(thick, yDim, zDim);
		osg::Vec3 center(-xDim, 0., 0.);

		wallBox[0] = osgBox(center, halfLengths);
		if (wallImage) wallBox[0]->setStateSet(wallStateSet);
		
		wallBox[0]->setNodeMask(0x1);
		shakeBox->addChild(wallBox[0]);

		btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(osgbCollision::asBtVector3(center));
		cs->addChildShape(trans, box);
	}
	{ // -Z
		osg::Vec3 halfLengths(xDim, yDim, thick);
	//	osg::Vec3 halfLengths(xDim*.5, yDim*.5, thick*.5);
		osg::Vec3 center(0., 0., zDim);
		wallBox[1] = osgBox(center, halfLengths);
		if (wallImage) wallBox[1]->setStateSet(wallStateSet);
		wallBox[1]->setNodeMask(0x1);
		shakeBox->addChild(wallBox[1]);
		btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(osgbCollision::asBtVector3(center));
		cs->addChildShape(trans, box);
	}
	{ // right +X
		osg::Vec3 halfLengths(thick, yDim, zDim);
		osg::Vec3 center(xDim, 0., 0.);
		wallBox[2] = osgBox(center, halfLengths);
		if (wallImage) wallBox[2]->setStateSet(wallStateSet);
		wallBox[2]->setNodeMask(0x1);
		shakeBox->addChild(wallBox[2]);

		btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(osgbCollision::asBtVector3(center));
		cs->addChildShape(trans, box);
	}
	{ // +Z  
		osg::Vec3 halfLengths(xDim, yDim, thick);
		osg::Vec3 center(0., 0., -zDim);
		wallBox[3] = osgBox(center, halfLengths);
		if (wallImage) wallBox[3]->setStateSet(wallStateSet);
		wallBox[3]->setNodeMask(0x1);
		shakeBox->addChild(wallBox[3]);
		btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(osgbCollision::asBtVector3(center));
		cs->addChildShape(trans, box);
	}
	{ //  -Y floor
		osg::Vec3 halfLengths(xDim, thick, zDim);
		osg::Vec3 center(0., yDim, 0.);
		wallBox[4] = osgBox(center, halfLengths);
		if (groundImage) wallBox[4]->setStateSet(groundStateSet);
		wallBox[4]->setNodeMask(0x1);
		shakeBox->addChild(wallBox[4]);
		btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
		btTransform trans; trans.setIdentity();
		trans.setOrigin(osgbCollision::asBtVector3(center));
		cs->addChildShape(trans, box);
	}
	//{ //  -Y floor2
	//	osg::Vec3 halfLengths(xDim, thick, zDim);
	//	osg::Vec3 center(0., yDim + 8., 0.);
	//	wallBox[5] = osgBox(center, halfLengths);
	//	wallBox[5]->setNodeMask(0x4);
	//	shakeBox->addChild(wallBox[5]);
	//	btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
	//	btTransform trans; trans.setIdentity();
	//	trans.setOrigin(osgbCollision::asBtVector3(center));
	//	cs->addChildShape(trans, box);
	//}
	//{ //  -Y  roof
	//	osg::Vec3 halfLengths(xDim, thick, zDim);
	//	osg::Vec3 center(0., -yDim, 0.);
	//	//shakeBox->addChild(osgBox(center, halfLengths));
	//	btBoxShape* box = new btBoxShape(osgbCollision::asBtVector3(halfLengths));
	//	btTransform trans; trans.setIdentity();
	//	trans.setOrigin(osgbCollision::asBtVector3(center));
	//	cs->addChildShape(trans, box);
	//}

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

MatrixTransform *PickModelHandler::getOrCreateSelectionBox(unsigned int index)
{
	if (index >= _selectionBoxVec.size()) {
		ref_ptr<ShapeDrawable> boxDrawable = new ShapeDrawable(new Box(Vec3(), 1.0f));
		boxDrawable->setColor(Vec4(0.3f, 0.3f, 0.3f, 0.0f));	
		ref_ptr<Geode> geode = new Geode;
		geode->addDrawable(boxDrawable.get());
		MatrixTransform* selectionBox = new MatrixTransform;
		selectionBox->setNodeMask(0);		//hide this box
		selectionBox->addChild(geode.get());
		StateSet *ss = selectionBox->getOrCreateStateSet();
		ss->setMode(GL_LIGHTING, StateAttribute::OFF);
		ss->setAttributeAndModes(new PolygonMode(PolygonMode::FRONT_AND_BACK, PolygonMode::LINE));
		_selectionBoxVec.push_back(selectionBox);
		_root->addChild(_selectionBoxVec[index]); //add selection Box to root
		return _selectionBoxVec[index];
	}
	else return _selectionBoxVec[index];
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

bool PickModelHandler::deleteOneObj(unsigned int index) 
{
	MatrixTransform* model = _lastModelVec[index];
	MatrixTransform* box = _selectionBoxVec[index];
	btCollisionObject* collisionObj = _selectCollisionObjVec[index];

	if (model != NULL && box != NULL && collisionObj != NULL){
		_root->removeChild(model);
		_collisionWorld->removeCollisionObject(collisionObj);
		
		_lastModelVec[index] = NULL;
		_selectCollisionObjVec[index] = NULL;
		_selectionBoxVec[index]->setNodeMask(0); //hide the box

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
//bool PickModelHandler::duplicateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj)
//{
//	if (model != NULL && box != NULL && collisionObj != NULL){
//		//delete model;
//		delete collisionObj;
//
//		return true;
//	}
//	else {
//		cout << "duplicate nothing!" << endl;
//		return false;
//	}
//	return false;
//
//
//}

bool PickModelHandler::translateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d transVec)
{
	if (model == NULL || collisionObj == NULL ) {
		return false;
	}
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
	if (model == NULL || collisionObj == NULL || box == NULL) {
		return false;
	}
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

	//matrix *= Matrix::translate(_backwardVec * 30);
	//smatrix *= Matrix::translate(_backwardVec * 30);
	//tmpBtMatrix *= Matrix::translate(_backwardVec * 30);


	//rotate
	matrix *= rotMatrix;
	smatrix *= rotMatrix;
	tmpBtMatrix *= rotMatrix;
	//btTrans *= osgbCollision::asBtTransform(rotMatrix);

	//translate back
	//matrix *= Matrix::translate(-_backwardVec * 30);
	//smatrix *= Matrix::translate(-_backwardVec * 30);
	//tmpBtMatrix *= Matrix::translate(-_backwardVec * 30);

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
	if (model == NULL || collisionObj == NULL || box == NULL) {
		return false;
	}
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

bool PickModelHandler::rotateMultipleObj(vector<MatrixTransform*> modelVec, vector<MatrixTransform*> boxVec, vector<btCollisionObject*> collisionObjVec, Matrix rotMatrix)
{
	if (modelVec.size() > boxVec.size() || modelVec.size() > collisionObjVec.size())
	{
		cout << "model:" << modelVec.size() << " boxVec: " << boxVec.size() << " collisionObj: " << collisionObjVec.size() << endl;
		cout << "rotate multipleObj error: model ,box and collisionObj size don't match" << endl;
		return false;
	}

	int Num = 0;
	float Radius = 0;

	Vec3d modelCenter(0., 0., 0.), boxCenter(0., 0., 0.), collisionObjCenter(0., 0., 0.);

	vector<Matrix> modelOriMatrixVec;
	vector<Matrix> boxOriMatrixVec;
	vector<Matrix> collisionOriMatrixVec;
	for (int i = 0; i < modelVec.size(); i++)
	{
		MatrixTransform* model = modelVec[i];
		MatrixTransform* box = boxVec[i];
		btCollisionObject* collisionObj = collisionObjVec[i];
		if (model == NULL || collisionObj == NULL || box == NULL) {
			continue;
		}

		modelOriMatrixVec.push_back(model->getMatrix());
		boxOriMatrixVec.push_back(box->getMatrix());
		btTransform oriBtTrans = collisionObj->getWorldTransform();
		//btVector3 oriBtScale = collisionObj->getCollisionShape()->getLocalScaling();
		collisionOriMatrixVec.push_back(osgbCollision::asOsgMatrix(oriBtTrans));

		Vec3d transVec1 = modelOriMatrixVec[Num].getTrans();
		Vec3d transVec2 = boxOriMatrixVec[Num].getTrans();
		Vec3d transVec3 = collisionOriMatrixVec[Num].getTrans();

		modelCenter += transVec1;
		boxCenter += transVec2;
		collisionObjCenter += transVec3;

		BoundingSphere tbb = model->getBound();
		float tradius = tbb.radius() * 0.6;
		if (tradius > Radius) Radius = tradius;

		Num++;
	}
	modelCenter /= (float)Num;
	boxCenter /= (float)Num;
	collisionObjCenter /= (float)Num;
	//modelCenter += _backwardVec * 60;
	//boxCenter += _backwardVec * 60;
	//collisionObjCenter += _backwardVec * 60;


	bool collisionFlag = false;
	int tmpN = 0;
	for (int i = 0; i < modelVec.size(); i++)
	{
		MatrixTransform* model = modelVec[i];
		MatrixTransform* box = boxVec[i];
		btCollisionObject* collisionObj = collisionObjVec[i];
		if (model == NULL || collisionObj == NULL || box == NULL) {
			continue;
		}

		Matrix matrix = modelOriMatrixVec[tmpN];
		Matrix smatrix = boxOriMatrixVec[tmpN];
		Matrix btMatrix = collisionOriMatrixVec[tmpN];

		//Vec3d transVec1 = matrix.getTrans();
		//Vec3d transVec2 = smatrix.getTrans();
		//Vec3d transVec3 = btMatrix.getTrans();

		//translate to center
		matrix *= Matrix::translate(-modelCenter);
		smatrix *= Matrix::translate(-boxCenter);
		btMatrix *= Matrix::translate(-collisionObjCenter);

		//rotate
		matrix *= rotMatrix;
		//smatrix *= rotMatrix;
		btMatrix *= rotMatrix;

		//translate back
		matrix *= Matrix::translate(modelCenter);
		smatrix *= Matrix::translate(boxCenter);
		btMatrix *= Matrix::translate(collisionObjCenter);

		model->setMatrix(matrix);
		box->setMatrix(smatrix);
		collisionObj->setWorldTransform(osgbCollision::asBtTransform(btMatrix));

		tmpN++;
	}

	_collisionWorld->performDiscreteCollisionDetection();
	detectCollision(_colState, _collisionWorld);

	if (_colState)
	{
		cout << "collision happen during row permutation" << endl;
		int tmpN2 = 0;

		for (int i = 0; i < modelVec.size(); i++)
		{
			MatrixTransform* model = modelVec[i];
			MatrixTransform* box = boxVec[i];
			btCollisionObject* collisionObj = collisionObjVec[i];
			if (model == NULL || collisionObj == NULL || box == NULL) {
				continue;
			}

			model->setMatrix(modelOriMatrixVec[tmpN2]);
			box->setMatrix(boxOriMatrixVec[tmpN2]);
			collisionObj->setWorldTransform(osgbCollision::asBtTransform(collisionOriMatrixVec[tmpN2]));
			tmpN2++;
		}
		_colState = false;
		return false;
	}
	else
	{
		cout << "permutaion success!" << endl;
		clearAllchoose();
		int tmpN2 = 0;
		for (int i = 0; i < modelVec.size(); i++)
		{
			MatrixTransform* model = modelVec[i];
			if (model == NULL) continue;
			chooseOneMatrixTransform(model);
			tmpN2++;
		}
		return true;
	}


}
bool PickModelHandler::scaleGap(vector<MatrixTransform*> modelVec, vector<MatrixTransform*> boxVec, vector<btCollisionObject*> collisionObjVec, bool isInceaseGap)
{
	if (modelVec.size() > boxVec.size() || modelVec.size() > collisionObjVec.size())
	{
		cout << "model:" << modelVec.size() << " boxVec: " << boxVec.size() << " collisionObj: " << collisionObjVec.size() << endl;
		cout << "scale gap error: model ,box and collisionObj size don't match" << endl;
		return false;
	}

	int Num = 0;
	float Radius = 0;

	Vec3d modelCenter(0., 0., 0.), boxCenter(0., 0., 0.), collisionObjCenter(0., 0., 0.);

	vector<Matrix> modelOriMatrixVec;
	vector<Matrix> boxOriMatrixVec;
	vector<Matrix> collisionOriMatrixVec;
	for (int i = 0; i < modelVec.size(); i++)
	{
		MatrixTransform* model = modelVec[i];
		MatrixTransform* box = boxVec[i];
		btCollisionObject* collisionObj = collisionObjVec[i];
		if (model == NULL || collisionObj == NULL || box == NULL) {
			continue;
		}

		modelOriMatrixVec.push_back(model->getMatrix());
		boxOriMatrixVec.push_back(box->getMatrix());
		btTransform oriBtTrans = collisionObj->getWorldTransform();
		//btVector3 oriBtScale = collisionObj->getCollisionShape()->getLocalScaling();
		collisionOriMatrixVec.push_back(osgbCollision::asOsgMatrix(oriBtTrans));

		Vec3d transVec1 = modelOriMatrixVec[Num].getTrans();
		Vec3d transVec2 = boxOriMatrixVec[Num].getTrans();
		Vec3d transVec3 = collisionOriMatrixVec[Num].getTrans();

		modelCenter += transVec1;
		boxCenter += transVec2;
		collisionObjCenter += transVec3;

		BoundingSphere tbb = model->getBound();
		float tradius = tbb.radius() * 0.6;
		if (tradius > Radius) Radius = tradius;

		Num++;
	}
	modelCenter /= (float)Num;
	boxCenter /= (float)Num;
	collisionObjCenter /= (float)Num;
	//modelCenter += _backwardVec * 60;
	//boxCenter += _backwardVec * 60;
	//collisionObjCenter += _backwardVec * 60;


	bool collisionFlag = false;
	int tmpN = 0;
	for (int i = 0; i < modelVec.size(); i++)
	{
		MatrixTransform* model = modelVec[i];
		MatrixTransform* box = boxVec[i];
		btCollisionObject* collisionObj = collisionObjVec[i];
		if (model == NULL || collisionObj == NULL || box == NULL) {
			continue;
		}

		int gapDistance;
		if(isInceaseGap) gapDistance = Radius * 0.5;
		else gapDistance = -Radius * 0.5;

		Matrix matrix = modelOriMatrixVec[tmpN];
		Matrix smatrix = boxOriMatrixVec[tmpN];
		Matrix btMatrix = collisionOriMatrixVec[tmpN];

		Vec3d transVec1 = matrix.getTrans();
		Vec3d transVec2 = smatrix.getTrans();
		Vec3d transVec3 = btMatrix.getTrans();

		Matrix transMatrix;
		Vec3d dirVec = (transVec1 - modelCenter);
		dirVec *= 0.01;
		//dirVec.normalize();

		transMatrix = Matrix::translate(dirVec * gapDistance);

		matrix *= transMatrix;
		smatrix *= transMatrix;
		btMatrix *= transMatrix;

		model->setMatrix(matrix);
		box->setMatrix(smatrix);
		collisionObj->setWorldTransform(osgbCollision::asBtTransform(btMatrix));

		tmpN++;
	}

	_collisionWorld->performDiscreteCollisionDetection();
	detectCollision(_colState, _collisionWorld);

	if (_colState)
	{
		cout << "collision happen during row permutation" << endl;
		int tmpN2 = 0;

		for (int i = 0; i < modelVec.size(); i++)
		{
			MatrixTransform* model = modelVec[i];
			MatrixTransform* box = boxVec[i];
			btCollisionObject* collisionObj = collisionObjVec[i];
			if (model == NULL || collisionObj == NULL || box == NULL) {
				continue;
			}

			model->setMatrix(modelOriMatrixVec[tmpN2]);
			box->setMatrix(boxOriMatrixVec[tmpN2]);
			collisionObj->setWorldTransform(osgbCollision::asBtTransform(collisionOriMatrixVec[tmpN2]));
			tmpN2++;
		}
		_colState = false;
		return false;
	}
	else
	{
		cout << "permutaion success!" << endl;
		clearAllchoose();
		int tmpN2 = 0;
		for (int i = 0; i < modelVec.size(); i++)
		{
			MatrixTransform* model = modelVec[i];
			if (model == NULL) continue;
			chooseOneMatrixTransform(model);
			tmpN2++;
		}
		return true;
	}


}

bool PickModelHandler::permutateRow(vector<MatrixTransform*> modelVec, vector<MatrixTransform*> boxVec, vector<btCollisionObject*> collisionObjVec)
{
	if (modelVec.size() > boxVec.size() || modelVec.size() > collisionObjVec.size())
	{
		cout << "model:" << modelVec.size() << " boxVec: " << boxVec.size() << " collisionObj: " << collisionObjVec.size() << endl;
		cout << "permutate row error: model ,box and collisionObj size don't match" << endl;
		return false;
	}

	int Num = 0;
	float Radius = 0;

	Vec3d modelCenter(0., 0., 0.), boxCenter(0., 0., 0.), collisionObjCenter(0., 0., 0.);

	vector<Matrix> modelOriMatrixVec;
	vector<Matrix> boxOriMatrixVec;
	vector<Matrix> collisionOriMatrixVec;
	for (int i = 0; i < modelVec.size(); i++)
	{
		MatrixTransform* model = modelVec[i];
		MatrixTransform* box = boxVec[i];
		btCollisionObject* collisionObj = collisionObjVec[i];
		if (model == NULL || collisionObj == NULL || box == NULL) {
			continue;
		}

		modelOriMatrixVec.push_back(model->getMatrix());
		boxOriMatrixVec.push_back(box->getMatrix());
		btTransform oriBtTrans = collisionObj->getWorldTransform();
		//btVector3 oriBtScale = collisionObj->getCollisionShape()->getLocalScaling();
		collisionOriMatrixVec.push_back(osgbCollision::asOsgMatrix(oriBtTrans));

		Vec3d transVec1 = modelOriMatrixVec[Num].getTrans();
		Vec3d transVec2 = boxOriMatrixVec[Num].getTrans();
		Vec3d transVec3 = collisionOriMatrixVec[Num].getTrans();

		modelCenter += transVec1;
		boxCenter += transVec2;
		collisionObjCenter += transVec3;

		BoundingSphere tbb = model->getBound();
		float tradius = tbb.radius();
		if (tradius > Radius) Radius = tradius;


		Num++;
	}
	//float pDegree = 360.0 / (float)Num;
	float pDistance = Radius * 1.5;
	//Radius *= 1.5;
	modelCenter /= (float)Num;
	boxCenter /= (float)Num;
	collisionObjCenter /= (float)Num;
	//modelCenter += _backwardVec * 60;
	//boxCenter += _backwardVec * 60;
	//collisionObjCenter += _backwardVec * 60;


	for (int k = 0; k < 3; k++)
	{

		cout << "try time" << k << endl;
		bool collisionFlag = false;
		int tmpN = 0;
		for (int i = 0; i < modelVec.size(); i++)
		{
			MatrixTransform* model = modelVec[i];
			MatrixTransform* box = boxVec[i];
			btCollisionObject* collisionObj = collisionObjVec[i];
			if (model == NULL || collisionObj == NULL || box == NULL) {
				continue;
			}

			int tDistance = pDistance * ((tmpN + 1) / 2);
			//int tDegree = pDegree * tmpN;
			//Matrix initRotMatrix = Matrix::rotate(degreeToPie((_forwardDegree)), _upVec);
            //Matrix rotMatrix = Matrix::rotate(degreeToPie((-tDegree)), _upVec);
			Matrix transMatrix;
			if ((tmpN % 2) == 1)
			{
				transMatrix = Matrix::translate(_rightVec * tDistance);
			}
			else
			{
				transMatrix = Matrix::translate(_leftVec * tDistance);
			}


			Matrix matrix = modelOriMatrixVec[tmpN];
			Matrix smatrix = boxOriMatrixVec[tmpN];
			Matrix btMatrix = collisionOriMatrixVec[tmpN];

			Vec3d transVec1 = matrix.getTrans();
			Vec3d transVec2 = smatrix.getTrans();
			Vec3d transVec3 = btMatrix.getTrans();

			Quat initRot;
			initRot.makeRotate(degreeToPie((_forwardDegree)), _upVec);
			Vec3 boxScale = smatrix.getScale();

			//rotate to initial orientation
			matrix *= Matrix::translate(-transVec1);
			smatrix *= Matrix::translate(-transVec2);
			btMatrix *= Matrix::translate(-transVec3);

			matrix.setRotate(initRot);
			smatrix.setRotate(initRot);
			smatrix *= Matrix::scale(boxScale);
			btMatrix.setRotate(initRot);

			matrix *= Matrix::translate(transVec1);
			smatrix *= Matrix::translate(transVec2);
			btMatrix *= Matrix::translate(transVec3);


			//row permutation

			matrix *= Matrix::translate(-transVec1);
			smatrix *= Matrix::translate(-transVec2);
			btMatrix *= Matrix::translate(-transVec3);

			matrix *= transMatrix;
			smatrix *= transMatrix;
			btMatrix *= transMatrix;

			//translate back to center

			matrix *= Matrix::translate(modelCenter);
			smatrix *= Matrix::translate(boxCenter);
			btMatrix *= Matrix::translate(collisionObjCenter);

			model->setMatrix(matrix);
			box->setMatrix(smatrix);
			collisionObj->setWorldTransform(osgbCollision::asBtTransform(btMatrix));

			tmpN++;
		}

		_collisionWorld->performDiscreteCollisionDetection();
		detectCollision(_colState, _collisionWorld);

		if (_colState)
		{
			cout << "collision happen during row permutation" << endl;
			int tmpN2 = 0;

			for (int i = 0; i < modelVec.size(); i++)
			{
				MatrixTransform* model = modelVec[i];
				MatrixTransform* box = boxVec[i];
				btCollisionObject* collisionObj = collisionObjVec[i];
				if (model == NULL || collisionObj == NULL || box == NULL) {
					continue;
				}

				model->setMatrix(modelOriMatrixVec[tmpN2]);
				box->setMatrix(boxOriMatrixVec[tmpN2]);
				collisionObj->setWorldTransform(osgbCollision::asBtTransform(collisionOriMatrixVec[tmpN2]));
				tmpN2++;
			}
			_colState = false;
			modelCenter += _upVec * 60;
			boxCenter += _upVec * 60;
			collisionObjCenter += _upVec * 60;
		}
		else
		{
			cout << "permutaion success!" << endl;
			clearAllchoose();
			int tmpN2 = 0;
			for (int i = 0; i < modelVec.size(); i++)
			{
				MatrixTransform* model = modelVec[i];
				if (model == NULL) continue;
				chooseOneMatrixTransform(model);
				tmpN2++;
			}
			return true;
		}

	}
	return false;
}

bool PickModelHandler::permutateRound(vector<MatrixTransform*> modelVec, vector<MatrixTransform*> boxVec, vector<btCollisionObject*> collisionObjVec)
{
	if (modelVec.size() > boxVec.size() || modelVec.size() > collisionObjVec.size())
	{
		cout << "model:" << modelVec.size() << " boxVec: " << boxVec.size() <<" collisionObj: " << collisionObjVec.size() << endl;
		cout << "permutate round error: model ,box and collisionObj size don't match" << endl;
		return false;
	}

	int Num = 0;
	float Radius = 0;

	Vec3d modelCenter(0., 0., 0.), boxCenter(0., 0., 0.), collisionObjCenter(0., 0., 0.);

	vector<Matrix> modelOriMatrixVec;
	vector<Matrix> boxOriMatrixVec;
	vector<Matrix> collisionOriMatrixVec;
	for (int i = 0; i < modelVec.size(); i++)
	{
		MatrixTransform* model = modelVec[i];
		MatrixTransform* box = boxVec[i];
		btCollisionObject* collisionObj = collisionObjVec[i];
		if (model == NULL || collisionObj == NULL || box == NULL) {
			continue;
		}

		modelOriMatrixVec.push_back(model->getMatrix());
		boxOriMatrixVec.push_back(box->getMatrix());
		btTransform oriBtTrans = collisionObj->getWorldTransform();
		//btVector3 oriBtScale = collisionObj->getCollisionShape()->getLocalScaling();
		collisionOriMatrixVec.push_back(osgbCollision::asOsgMatrix(oriBtTrans));

		Vec3d transVec1 = modelOriMatrixVec[Num].getTrans();
		Vec3d transVec2 = boxOriMatrixVec[Num].getTrans();
		Vec3d transVec3 = collisionOriMatrixVec[Num].getTrans();

		modelCenter += transVec1;
		boxCenter += transVec2;
		collisionObjCenter += transVec3;
		


		BoundingSphere tbb = model->getBound();
		float tradius = tbb.radius() * 0.6;
		if (tradius > Radius) Radius = tradius;


		Num++; 
	}
	float pDegree = 360.0 / (float)Num;
	Radius += Num * 10.;
	modelCenter /= (float)Num;
	boxCenter /= (float)Num;
	collisionObjCenter /= (float)Num;
	modelCenter += _backwardVec * 60;
	boxCenter += _backwardVec * 60;
	collisionObjCenter += _backwardVec * 60;
	

	for (int k = 0; k < 3; k++)
	{

		cout << "try time" << k << endl;
		bool collisionFlag = false;
		int tmpN = 0;
		for (int i = 0; i < modelVec.size(); i++)
		{
			MatrixTransform* model = modelVec[i];
			MatrixTransform* box = boxVec[i];
			btCollisionObject* collisionObj = collisionObjVec[i];
			if (model == NULL || collisionObj == NULL || box == NULL) {
				continue;
			}
			int tDegree = pDegree * tmpN;
			//Matrix initRotMatrix = Matrix::rotate(degreeToPie((_forwardDegree)), _upVec);

			Matrix rotMatrix = Matrix::rotate(degreeToPie((-tDegree)), _upVec);

			Matrix matrix = modelOriMatrixVec[tmpN];
			Matrix smatrix = boxOriMatrixVec[tmpN];
			Matrix btMatrix = collisionOriMatrixVec[tmpN];

			Vec3d transVec1 = matrix.getTrans();
			Vec3d transVec2 = smatrix.getTrans();
			Vec3d transVec3 = btMatrix.getTrans();

			Quat initRot;
			initRot.makeRotate(degreeToPie((_forwardDegree)), _upVec);
			Vec3 boxScale = smatrix.getScale();

			//rotate to initial orientation
			matrix *= Matrix::translate(-transVec1);
			smatrix *= Matrix::translate(-transVec2);
			btMatrix *= Matrix::translate(-transVec3);

			matrix.setRotate(initRot);
			smatrix.setRotate(initRot);
			smatrix *= Matrix::scale(boxScale);
			btMatrix.setRotate(initRot);

			matrix *= Matrix::translate(transVec1);
			smatrix *= Matrix::translate(transVec2);
			btMatrix *= Matrix::translate(transVec3);


			//round permutation

			matrix *= Matrix::translate(-transVec1);
			smatrix *= Matrix::translate(-transVec2);
			btMatrix *= Matrix::translate(-transVec3);

			matrix *= Matrix::translate(_backwardVec * Radius);
			smatrix *= Matrix::translate(_backwardVec * Radius);
			btMatrix *= Matrix::translate(_backwardVec * Radius);

			//rotate
			matrix *= rotMatrix;
			//smatrix *= rotMatrix;
			btMatrix *= rotMatrix;

			//translate back
			matrix *= Matrix::translate(-_backwardVec * Radius);
			smatrix *= Matrix::translate(-_backwardVec * Radius);
			btMatrix *= Matrix::translate(-_backwardVec * Radius);

			matrix *= Matrix::translate(modelCenter);
			smatrix *= Matrix::translate(boxCenter);
			btMatrix *= Matrix::translate(collisionObjCenter);

			model->setMatrix(matrix);
			box->setMatrix( smatrix);
			collisionObj->setWorldTransform(osgbCollision::asBtTransform(btMatrix));

			tmpN++;

			//_collisionWorld->performDiscreteCollisionDetection();
			//detectCollision(_colState, _collisionWorld);
			//if (_colState)
			//{
			//	collisionFlag = true;
			//	break;
			//}

		}

		_collisionWorld->performDiscreteCollisionDetection();
		detectCollision(_colState, _collisionWorld);

		if (_colState)
		{
			cout << "collision happen during permutation" << endl;
			int tmpN2 = 0;
		
			for (int i = 0; i < modelVec.size(); i++)
			{
				MatrixTransform* model = modelVec[i];
				MatrixTransform* box = boxVec[i];
				btCollisionObject* collisionObj = collisionObjVec[i];
				if (model == NULL || collisionObj == NULL || box == NULL) {
					continue;
				}
			
				model->setMatrix(modelOriMatrixVec[tmpN2]);
				box->setMatrix(boxOriMatrixVec[tmpN2]);
				collisionObj->setWorldTransform(osgbCollision::asBtTransform(collisionOriMatrixVec[tmpN2]));
				tmpN2++;
			}
			_colState = false;
			modelCenter += _upVec * 60;
			boxCenter += _upVec * 60;
			collisionObjCenter += _upVec * 60;
		}
		else
		{
			cout << "permutaion success!" << endl;
			clearAllchoose();
			int tmpN2 = 0;
			for (int i = 0; i < modelVec.size(); i++)
			{
				MatrixTransform* model = modelVec[i];
				if (model == NULL) continue;
				
				chooseOneMatrixTransform(model);
				tmpN2++;
			}
			

			return true;
		}
	}
	return false;
}

void PickModelHandler::chooseOneMatrixTransform(MatrixTransform* lastModel)
{
	
	bool isalreadyChosed = false;
	for (int i = 0; i < _selectNum; i++) {
		if (lastModel == _lastModelVec[i]) {
			cout << "already chosed!" << endl;
			isalreadyChosed = true;
			_lastModelVec[i] = NULL;   //lazy delete without changing the selectNum
			_selectCollisionObjVec[i] = NULL;
			_selectionBoxVec[i]->setNodeMask(0);
			break;
		}
	}
	if (isalreadyChosed) return;

	int xmax = INT_MIN, xmin = INT_MAX, ymax INT_MIN, ymin = INT_MAX, zmax INT_MIN, zmin = INT_MAX;
	bool hasgeoflag = false;
	Matrix mat;
	int childNum = lastModel->getNumChildren();

	for (int j = 0; j < childNum; j++) {
		if (strcmp(lastModel->getChild(j)->className(), "Group") == 0) {
			Group*  tgroup = dynamic_cast<Group *>(lastModel->getChild(j));
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

	if (hasgeoflag)
		mat = Matrix::scale(xmax - xmin, ymax - ymin, zmax - zmin);
	else {
		BoundingSphere tbb = lastModel->getBound();
		//cout << tbb.radius() << "  " << tbb.radius2() << "  " << tbb.center() << endl;
		float radius = tbb.radius() * 0.6;
		mat = Matrix::scale(2 * radius, 2 * radius, 2 * radius);
	}

	
	_lastModelVec.push_back(lastModel);
	_selectCollisionObjVec.push_back(_objMap[lastModel]);
	mat *= lastModel->getMatrix();

	if (_selectNum >= _selectionBoxVec.size()) {
		MatrixTransform * tbox = getOrCreateSelectionBox(_selectNum);
		tbox->setNodeMask(0x1);
		tbox->setMatrix(mat);
	}
	else {
		_selectionBoxVec[_selectNum]->setMatrix(mat);
		_selectionBoxVec[_selectNum]->setNodeMask(0x1);
	}
	
	_selectNum++; //increment the selected obj 

	return;
}

void PickModelHandler::clearAllchoose()
{
	cout << "clear all choose" << endl;
	_selectNum = 0;
	for (int i = 0; i < _selectionBoxVec.size(); i++) {
		_selectionBoxVec[i]->setNodeMask(0); //hide all the box
	}
	_lastModelVec.clear();
	_selectCollisionObjVec.clear();
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
			Matrix mat = Matrix::scale(bb.xMax() - bb.xMin(), bb.yMax() - bb.yMin(), bb.zMax() - bb.zMin())/* * Matrix::translate(worldCenter)*/;

			// _lastModel
			const NodePath &np = result.nodePath;	//NodePath: Group(root)-->MatrixTransform(trans)-->Group(?)-->Geode(model)
			int nSize = np.size();

			bool flag = false;
			MatrixTransform * lastModel;

			for (int i = nSize - 1; i >= 0; i--)
			{
				cout << np[i]->className() << endl;
				if (strcmp(np[i]->className(), "MatrixTransform") == 0)
				{
					MatrixTransform * lastModel = dynamic_cast<MatrixTransform *>(np[i]);
					//bool isalreadyChosed = false;
					//for (int i = 0; i < _selectNum; i++) {
					//	if (lastModel == _lastModelVec[i]) {
					//		cout << "already chosed!" << endl;
					//		isalreadyChosed = true;
					//		_lastModelVec[i] = NULL;   //lazy delete without changing the selectNum
					//		_selectCollisionObjVec[i] = NULL;
					//		_selectionBoxVec[i]->setNodeMask(0);
					//		break;
					//	}
					//}
					//if (isalreadyChosed) break;
					chooseOneMatrixTransform(lastModel);
					break;
				}
			}
		}
		else
		{
			cout << "choose nothing" << endl;
			clearAllchoose();
			//_selectNum = 0;
			//for (int i = 0; i < _selectionBoxVec.size(); i++) {
			//	_selectionBoxVec[i]->setNodeMask(0); //hide all the box
			//}
			//_lastModelVec.clear();
			//_selectCollisionObjVec.clear();

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
		bool isPermutateRound = false;
		bool isPermutateRow = false;
		bool isPermutateCol = false;
		bool isInceaseGap = false;
		bool isRotateMultiple = false;

		Vec3d scaleFactor(1, 1, 1);
		Matrix rotMatrix;

		Vec3d transVec;

		switch (ea.getKey())
		{
		//rotate around Y axis
		case 'k':    
		case 'K':
			rotMatrix = Matrix::rotate(degreeToPie(45), _upVec);
			isRotate = true;
			break;
		case 'l':
		case 'L':
			rotMatrix = Matrix::rotate(degreeToPie(-45), _upVec);
			isRotate = true;
			break;
		case 'h':
		case 'H':
			rotMatrix = Matrix::rotate(degreeToPie(45), _upVec);
			isRotateMultiple = true;
			break;
		case 'j':
		case 'J':
			rotMatrix = Matrix::rotate(degreeToPie(-45), _upVec);
			isRotateMultiple = true;
			break;
	    //scale object
		case 'm':
		case 'M':
			scaleFactor.set(1.1f, 1.1f, 1.1f);
			isScale = true;
			isInceaseGap = true;
			break;
		case 'n':
		case 'N':
			scaleFactor.set(0.9f, 0.9f, 0.9f);
			isScale = true;
			isInceaseGap = false;
			break;
		//move along x(right/left)
		case 'd':
		case 'D':  //right
			transVec = _rightVec * _transStep;
			isTranslate = true;

			break;
		case 'a':
		case 'A':
			transVec = _leftVec * _transStep;
			isTranslate = true;
			break;
	   //move along y(up/down)
		case 'w':
		case 'W':
			transVec = _upVec * _transStep;
			isTranslate = true;
			break;
		case 's':
		case 'S':
			transVec = _downVec * _transStep;
			isTranslate = true;
			break;
		case 'x':
		case 'X':
			transVec = _forwardVec * _transStep;
			isTranslate = true;
			break;
		case 'c':
		case 'C':
			transVec = _backwardVec * _transStep;
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
		case 'R':
		case 'r':
			isPermutateRound = true;
			break;
		case 'T':
		case 't':
			isPermutateRow = true;
			break;
		default:
			break;
		}

		if (isTranslate) {
			for (int i = 0; i < _selectNum; i++) {
				translateOneObj(_lastModelVec[i], _selectionBoxVec[i], _selectCollisionObjVec[i], transVec);
			}
		}
		else if (isRotate) {
			int tmp = _selectNum;
			bool flag = true;
			int i = 0;
			for (i = 0; i < _selectNum; i++) {
				flag = rotateOneObj(_lastModelVec[i], _selectionBoxVec[i], _selectCollisionObjVec[i], rotMatrix);
				if (!flag) {
					break;
				}
			}
			if (tmp >= 2 && !flag)
			{
				scaleGap(_lastModelVec, _selectionBoxVec, _selectCollisionObjVec, true);
				scaleGap(_lastModelVec, _selectionBoxVec, _selectCollisionObjVec, true);
				scaleGap(_lastModelVec, _selectionBoxVec, _selectCollisionObjVec, true);

				for (; i < _selectNum; i++) {
					rotateOneObj(_lastModelVec[i], _selectionBoxVec[i], _selectCollisionObjVec[i], rotMatrix);
					
				}

			}

		}
		else if (isRotateMultiple) {
			int tmp = _selectNum;
			if (tmp < 2)
			{
				cout << "too few objs to rotate" << endl;
			}
			else
			{
				rotateMultipleObj(_lastModelVec, _selectionBoxVec, _selectCollisionObjVec, rotMatrix);
			}
		}
		else if (isScale) {
			int tmp = _selectNum;
			if (tmp < 2)
			{
				for (int i = 0; i < _selectNum; i++) {
					scaleOneObj(_lastModelVec[i], _selectionBoxVec[i], _selectCollisionObjVec[i], scaleFactor);
				}
			}
			else
			{
				scaleGap(_lastModelVec, _selectionBoxVec, _selectCollisionObjVec, isInceaseGap);
			}

		}

		else if (isDuplicate) {
			int tmp = _selectNum;
			int tNum = 0;
			for (int i = 0; i < tmp; i++) {
				if (_lastModelVec[i] == NULL) continue;
				tNum++;
				MatrixTransform* tmatrix = duplicateOneObj(_lastModelVec[i]);
				if (tmatrix != NULL) {
					chooseOneMatrixTransform(tmatrix);
				}
				break;
			}
		}
		else if (isDelete) {
			for (unsigned int i = 0; i < _selectNum; i++) {
				deleteOneObj(i);
			//	deleteOneObj(_lastModelVec[i], _selectionBoxVec[i], _selectCollisionObjVec[i]);
			}
		}
		else if (isPermutateRound) {
			int tmp = _selectNum;
			if (tmp < 2)
			{
				cout << "too few objects are selected";
			}
			else
			{
				permutateRound(_lastModelVec, _selectionBoxVec, _selectCollisionObjVec);
			}
		}
		else if (isPermutateRow) {
			int tmp = _selectNum;
			if (tmp < 2)
			{
				cout << "too few objects are selected";
			}
			else
			{
				permutateRow(_lastModelVec, _selectionBoxVec, _selectCollisionObjVec);
			}
		}
	}
}

#endif
