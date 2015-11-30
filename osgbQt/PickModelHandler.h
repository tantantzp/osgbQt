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

osg::Camera* createHUDBg(std::string imagePath){
	osg::ref_ptr<osg::Camera>camera = new osg::Camera;
	camera->setProjectionMatrixAsOrtho2D(0, 800, 0, 600);
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setRenderOrder(osg::Camera::PRE_RENDER);
	camera->setViewport(0, 0, 1300, 800);
	camera->setClearMask(GL_DEPTH_BUFFER_BIT);
	camera->setAllowEventFocus(false);
	camera->setViewMatrix(osg::Matrix::identity());

	osg::ref_ptr<osg::Geode>geode = new osg::Geode;
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr<osg::Geometry>gm = new osg::Geometry;

	//亚入顶点
	osg::ref_ptr<osg::Vec3Array>vertex = new osg::Vec3Array;
	vertex->push_back(osg::Vec3(0, 0, 0));
	vertex->push_back(osg::Vec3(800, 0, 0));
	vertex->push_back(osg::Vec3(800, 600, 0));
	vertex->push_back(osg::Vec3(0, 600, 0));
	gm->setVertexArray(vertex);

	//压入法线

	//纹理坐标
	osg::ref_ptr<osg::Vec2Array>coord = new osg::Vec2Array;
	coord->push_back(osg::Vec2(0, 0));
	coord->push_back(osg::Vec2(1, 0));
	coord->push_back(osg::Vec2(1, 1));
	coord->push_back(osg::Vec2(0, 1));
	gm->setTexCoordArray(0, coord);
	gm->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));

	osg::ref_ptr<osg::Image>image = osgDB::readImageFile(imagePath);
	if (!image.valid()){
		//    std::cout<"read image faild "<<std::endl;
		return camera.release();
	}
	//  osg::ref_ptr<osg::Texture2D>t2d=new osg::Texture2D;
	osg::Texture2D*t2d = new osg::Texture2D;
	t2d->setImage(0, image);


	gm->getOrCreateStateSet()->setTextureAttributeAndModes(0, t2d, osg::StateAttribute::ON);
	geode->addDrawable(gm);
	camera->addChild(geode);
	return camera.release();
}

class PickModelHandler : public osgGA::GUIEventHandler
{
public:
	PickModelHandler(btCollisionWorld* collisionWorld, Group* root, osgViewer::Viewer* view, Group* noShadow = NULL);
	MatrixTransform *getOrCreateSelectionBox(unsigned int index, int clientNum);
	void  PickModelHandler::detectCollision(bool& colState, btCollisionWorld* cw);
	virtual bool handle(const osgGA::GUIEventAdapter &, osgGA::GUIActionAdapter &);

	//void setCollisionObject(btCollisionObject* co, unsigned int index) { _selectCollisionObjVec[index] = co; }

	//void setMatrixTransform(osg::MatrixTransform* sBox) { _selectionBox = sBox; }
	void setCollisionWorld(btCollisionWorld* btcw) { _collisionWorld = btcw; }
	void insertObjPair(MatrixTransform* transObj, btCollisionObject* collisionObj){_objMap.insert(make_pair(transObj, collisionObj));}
	void clearObjPair(){ _objMap.clear(); }
	//handle scene
	void setOrientation(int orientation);
	void createBillboardTree(Image* image, float deep = 800.0);
	void createBackboard(Image* image, float deep = 1000.0);
public:
	void addGround(float widthX, float widthZ, float heightY);
	bool addOneObj(string objPath, Vec3d initPos);

	//API
	void popFromHistoryAPI();
	void translateAPI(Vec3 transVec, int clientNum); // clientNum = 0 or = 1
	void rotateAPI(Matrix rotMatrix, int clientNum);
	void rotateAllAPI(Matrix rotMatrix, int clientNum);
	void duplicateAPI(int clientNum);
	void deleteAPI(int clientNum);
	void scaleAPI(Vec3 scaleFactor, int clientNum);
	void permutateRoundAPI(int clientNum);
	void permutateRowAPI(int clientNum);
	int getSelectNumAPI(int clientNum);
	int pickAPI(float clickX, float clickY, int clientNum); //return the index of the chosed obj, the normalized x and y(with left down corner to be (-1, -1) and right up corner tobe (1, 1);
	bool chooseAPI(int index, int clientNum);  //use the index returned by calling pickAPI function to choose an object
	void addBackground(string imagePath);
	//void addBackground(Camera* _camera);

protected:

	bool doAddObj(MatrixTransform * matrixTrans, bool isDetectCollision = true);
	void handleKeyEvent(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
	int handlePickEvent(float clickX, float clickY, int clientNum);//const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
	bool chooseOneMatrixTransform(int index, int clientNum);
	bool chooseOneMatrixTransform(MatrixTransform* lastmodel, int index, int clientNum);
	bool chooseOneMatrixTransformWithPush(int index, int clientNum);
	void clearAllchoose(int clientNum);

	void pushToHistory();
	void popFromHistory();

	MatrixTransform* duplicateOneObj(MatrixTransform * matrixTrans);
	bool translateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d transVec);
	bool rotateOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Matrix rotMatrix);
	bool scaleOneObj(MatrixTransform* model, MatrixTransform* box, btCollisionObject* collisionObj, Vec3d scaleVec);
	bool deleteOneObj(unsigned int index);
	//bool rotateMultipleObj(vector<MatrixTransform*> modelVec, vector<MatrixTransform*> boxVec, vector<btCollisionObject*> collisionObjVec, Matrix rotMatrix, int clientNum);
	//bool permutateRound(vector<MatrixTransform*> model, vector<MatrixTransform*> box, vector<btCollisionObject*> collisionObj, int clientNum);
	//bool permutateRow(vector<MatrixTransform*> modelVec, vector<MatrixTransform*> boxVec, vector<btCollisionObject*> collisionObjVec, int clientNum);
	//bool scaleGap(vector<MatrixTransform*> modelVec, vector<MatrixTransform*> boxVec, vector<btCollisionObject*> collisionObjVec, bool , int clientNum);

	bool rotateMultipleObj(Matrix rotMatrix, int clientNum);
	bool permutateRound(int clientNum);
	bool permutateRow(int clientNum);
	bool scaleGap(bool, int clientNum);

protected:
	vector< vector< ref_ptr<MatrixTransform> > > _historyModelVec;
	vector< vector<int> > _historySelectIndexVec;
	int _maxHistoryLength;

	vector< ref_ptr<MatrixTransform> > _allModelVec;
	vector< btCollisionObject* > _allCollisionObjVec;
	vector< ref_ptr<MatrixTransform> > _allSelectBoxVec;		// bounding box of the selected model;
	vector< int > _allSelectIndexVec; //not selected:-1, select by client1: 0, select by client2: 1

	//unsigned int _allNum;

	//vector< int > _selectIndexVec[2];   //client1 selection vector
	//vector< int > _selectIndexVec2;  //client2 selection vector s


	//
	//vector< MatrixTransform* > _lastModelVec;
	//vector< btCollisionObject* > _selectCollisionObjVec;
	btCollisionWorld* _collisionWorld;
	Group* _root;
	Group* _noShadowGroup;
	osgViewer::Viewer* _view;

	map<MatrixTransform *, btCollisionObject *> _objMap;


	double _lastX, _lastY;
	double _groundWidthX, _groundWidthZ, _groundHeightY;
	bool _colState;
	float _transStep;
	bool _hasGround;
	//unsigned int _selectNum;

	Geode* wallBox[6];
	//orientation and corresponding direction Vec3
	int _orientation; 
	Vec3f _leftVec, _rightVec, _forwardVec, _backwardVec, _upVec, _downVec;
	float _forwardDegree;
	
	//background
	string _imagePath;
	ref_ptr<Billboard> backBoard;
	ref_ptr<Geode> geoBackboard;
	int _imgIndex = 0;
	int _maxImg = 2;

};
PickModelHandler::PickModelHandler(btCollisionWorld* collisionWorld, Group* root, osgViewer::Viewer* view, Group* noShadow) {
	//_selectionBoxVec.clear();
	//_lastModelVec.clear();
	//_selectCollisionObjVec.clear();
	//cout << "init step 1" << endl;
	setCollisionWorld(collisionWorld);
	_allModelVec.clear();
	_allCollisionObjVec.clear();
	_allSelectBoxVec.clear();
	_allSelectIndexVec.clear();


	_historyModelVec.clear();
	_historySelectIndexVec.clear();

	_maxHistoryLength = 20;

	_objMap.clear();
	_root = root;
	_view = view;
	//_allNum = 0;
	//_selectNum = 0;

	//getOrCreateSelectionBox(0, 0);

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

	_noShadowGroup = noShadow;
}

void PickModelHandler::createBackboard(Image* image, float deep)
{
	float tDeep = deep;
	float tLength = 500.0;
	Vec3f eye;
	Vec3f center;
	Vec3f up;

	_view->getCamera()->getViewMatrixAsLookAt(eye, center, up);
	Vec3f direction = center - eye;
	direction.normalize();
	up.normalize();
	Vec3f left = direction ^ up;
	left.normalize();

	Vec3f boardCenter = eye + direction * deep;


	Vec3f point1 = boardCenter + up * tLength + left * tLength;
	Vec3f point2 = boardCenter + up * tLength - left * tLength;
	Vec3f point3 = boardCenter - up * tLength - left * tLength;
	Vec3f point4 = boardCenter - up * tLength + left * tLength;
	ref_ptr<Geometry> geometry = new Geometry();



	ref_ptr<Vec3Array> v = new Vec3Array();
	v->push_back(point1);
	v->push_back(point2);
	v->push_back(point3);
	v->push_back(point4);
	//v->push_back(Vec3(-tLength, tDeep, -tLength));
	//v->push_back(Vec3(tLength, tDeep, -tLength));
	//v->push_back(Vec3(tLength, tDeep, tLength));
	//v->push_back(Vec3(-tLength, tDeep, tLength));

	geometry->setVertexArray(v.get());

	//set normal
	//ref_ptr<Vec3Array> normal = new Vec3Array();
	//normal->push_back(Vec3(1.0, 0.0, 0.0) ^ Vec3(0.0, 0.0, 1.0));

	//geometry->setNormalArray(normal.get());
	//geometry->setNormalBinding(Geometry::BIND_OVERALL);

	//set texture coordinate
	ref_ptr<Vec2Array> vt = new Vec2Array();
	vt->push_back(Vec2(0.0, 0.0));
	vt->push_back(Vec2(1.0, 0.0));
	vt->push_back(Vec2(1.0, 1.0));
	vt->push_back(Vec2(0.0, 1.0));

	geometry->setTexCoordArray(0, vt.get());

	//draw rectangle
	geometry->addPrimitiveSet(new DrawArrays(PrimitiveSet::QUADS, 0, 4));


	if (image)
	{
		ref_ptr<Texture2D> texture = new Texture2D();
		texture->setWrap(Texture2D::WRAP_S, Texture2D::REPEAT);
		texture->setWrap(Texture2D::WRAP_T, Texture2D::REPEAT);
		texture->setImage(image);

		ref_ptr<StateSet> stateset = new osg::StateSet();
		stateset->setTextureAttributeAndModes(0, texture, StateAttribute::ON);
		//stateset->setMode(GL_BLEND, StateAttribute::OFF);
		stateset->setMode(GL_LIGHTING, StateAttribute::OFF);

		geometry->setStateSet(stateset.get());
	}

	if (geoBackboard.get() != NULL)
	{
		_root->removeChild(geoBackboard.get());
	}

	geoBackboard = new Geode();
	

	geoBackboard->addDrawable(geometry.get());
	geoBackboard->setNodeMask(0x1);
	//geoBackboard->setNodeMask(NoShadowTraversalMask);

	_root->addChild(geoBackboard.get());
	//_noShadowGroup->addChild(geoBackboard.get());

}


void PickModelHandler::createBillboardTree(Image* image, float deep)
{
	ref_ptr<Geometry> geometry = new Geometry();

	float tLength = 800.0;
	float tDeep = deep;
	//float tDeep = 800.0;

	ref_ptr<Vec3Array> v = new Vec3Array();
	v->push_back(Vec3(-tLength, tDeep, -tLength));
	v->push_back(Vec3(tLength, tDeep, -tLength));
	v->push_back(Vec3(tLength, tDeep, tLength));
	v->push_back(Vec3(-tLength, tDeep, tLength));

	geometry->setVertexArray(v.get());

	//set normal
	ref_ptr<Vec3Array> normal = new Vec3Array();
	normal->push_back(Vec3(1.0, 0.0, 0.0) ^ Vec3(0.0, 0.0, 1.0));

	geometry->setNormalArray(normal.get());
	geometry->setNormalBinding(Geometry::BIND_OVERALL);

	//set texture coordinate
	ref_ptr<Vec2Array> vt = new Vec2Array();
	vt->push_back(Vec2(0.0, 0.0));
	vt->push_back(Vec2(1.0, 0.0));
	vt->push_back(Vec2(1.0, 1.0));
	vt->push_back(Vec2(0.0, 1.0));

	geometry->setTexCoordArray(0, vt.get());

	//draw rectangle
	geometry->addPrimitiveSet(new DrawArrays(PrimitiveSet::QUADS, 0, 4));


	if (image)
	{
		ref_ptr<Texture2D> texture = new Texture2D();
		texture->setWrap(Texture2D::WRAP_S, Texture2D::REPEAT);
		texture->setWrap(Texture2D::WRAP_T, Texture2D::REPEAT);
		texture->setImage(image);

		ref_ptr<StateSet> stateset = new osg::StateSet();
		stateset->setTextureAttributeAndModes(0, texture, StateAttribute::ON);
		//stateset->setMode(GL_BLEND, StateAttribute::ON);
		stateset->setMode(GL_LIGHTING, StateAttribute::OFF);

		geometry->setStateSet(stateset.get());
	}

	if (backBoard.get() != NULL)
	{
		_root->removeChild(backBoard.get());
	}

	backBoard = new Billboard();
	backBoard->setMode(Billboard::POINT_ROT_EYE);
	
	backBoard->addDrawable(geometry.get(), Vec3(0.0, 0.0, 0.0f));

	//ref_ptr<Billboard> billboard2 = new Billboard();
	//billboard2->setMode(Billboard::AXIAL_ROT);
	//billboard2->setAxis(Vec3(0.0, 0.0, 1.0));
	//billboard2->addDrawable(geometry.get(), Vec3(10.0, 0.0, 0.0));

	//ref_ptr<Group> billboard = new Group();
	//billboard->addChild(billboard1.get());
	//billboard->addChild(billboard2.get());

	_root->addChild(backBoard.get());
	//return billboard.get();
}



void  PickModelHandler::setOrientation(int orientation)
{
	_orientation = orientation;
	cout << "ori:" << orientation << endl;
	//hideWall
	for (int i = 0; i < 4; i++)
	{
		if (i == orientation || i == (orientation + 2) % 4)
			wallBox[i]->setNodeMask(0x0);
		else wallBox[i]->setNodeMask(0x1);
	
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

		handlePickEvent(clickX, clickY, 0);
	}
	else if (ea.getEventType() == GUIEventAdapter::RELEASE &&
		ea.getButton() == GUIEventAdapter::RIGHT_MOUSE_BUTTON &&
		(ea.getModKeyMask() & GUIEventAdapter::MODKEY_CTRL))
	{
		float clickX = ea.getX(), clickY = ea.getY();

		handlePickEvent(clickX, clickY, 1);
	}
	else if (ea.getEventType() == GUIEventAdapter::KEYDOWN )//_lastModel != NULL && _selectionBox != NULL)
	{
		handleKeyEvent(ea, aa);
	}

	return false;
}

void PickModelHandler::pushToHistory()
{
	//cout << "push to history" << endl;
	//vector< ref_ptr<MatrixTransform> > tModelVec;
	////vector< btCollisionObject* > tCollisionVec;
	//
	//tModelVec.clear();
	////tCollisionVec.clear();
	//cout << "history Size:" << _historyModelVec.size() << endl;
	//cout << "all model size:" << _allModelVec.size() << endl;


	//for (int i = 0; i < _allModelVec.size(); i++)
	//{
	//	MatrixTransform* trans;
	//	//btCollisionObject* btBoxObject;
	//	if (_allModelVec[i] == NULL)
	//	{
	//		trans = NULL;
	//		//btBoxObject = NULL;
	//		tModelVec.push_back(trans);
	//		//tCollisionVec.push_back(btBoxObject);
	//	}
	//	else
	//	{
	//	   // trans = new MatrixTransform(*_allModelVec[i], CopyOp::DEEP_COPY_ALL);
	//		trans = new MatrixTransform(*_allModelVec[i], CopyOp::SHALLOW_COPY );
	//		tModelVec.push_back(trans);
	//	}
	//}
	//vector<int> tHisVec;
	//for (int i = 0; i < _selectIndexVec.size(); i++)
	//{
	//	tHisVec.push_back(_selectIndexVec[i]);
	//}
	//if (_historyModelVec.size() == _maxHistoryLength)
	//{
	//	_historyModelVec.erase(_historyModelVec.begin());
	//	_historySelectIndexVec.erase(_historySelectIndexVec.begin());
	//}
	//_historyModelVec.push_back(tModelVec);
	////_historyCollisionObjVec.push_back(tCollisionVec);
	//_historySelectIndexVec.push_back(tHisVec);

}
void PickModelHandler::popFromHistory()
{
	cout << "pop from history is currently removed" << endl;
	//if (_historyModelVec.size() > 0 && _historySelectIndexVec.size() > 0)
	//{
	//	cout << "pop from history, history size:" << _historyModelVec.size() << endl;
	//	vector< ref_ptr<MatrixTransform> > tModelVec = _historyModelVec[_historyModelVec.size() - 1];
	//	//vector< btCollisionObject* > tCollisionVec = _historyCollisionObjVec[_historyCollisionObjVec.size() - 1];
	//	vector< int > tIndexVec = _historySelectIndexVec[_historySelectIndexVec.size() - 1];
	//	_historyModelVec.pop_back();
	//	//_historyCollisionObjVec.pop_back();
	//	_historySelectIndexVec.pop_back();
	//	clearAllchoose();

	//	for (int i = 0; i < _allModelVec.size(); i++)
	//	{
	//		if (_allModelVec[i] != NULL)
	//		    _root->removeChild(_allModelVec[i]);
	//	}
	//	for (int i = 0; i < _allCollisionObjVec.size(); i++)
	//	{
	//		if (_allCollisionObjVec[i] != NULL)
	//	     	_collisionWorld->removeCollisionObject(_allCollisionObjVec[i]);
	//	}
	//	_allModelVec.clear();
	//	_allCollisionObjVec.clear();
	//	clearObjPair();

	//	_allNum = 0;
	//	cout << "remove all old objs" << endl;
	//	cout << "add size:" << tModelVec.size() << endl;
	//	for (int i = 0; i < tModelVec.size(); i++)
	//	{
	//		cout << "add new obj" << endl;
	//		doAddObj(tModelVec[i], false);
	//	}

	//	//_allModelVec = tModelVec;
	//	//_allCollisionObjVec = tCollisionVec;
	//	//_selectIndexVec = tIndexVec;
	//	
	//	for (int i = 0; i < tIndexVec.size(); i++)
	//	{
	//		if (tIndexVec[i] >= 0);
	//		    chooseOneMatrixTransform(tIndexVec[i]);
	//	}

	//}
	//else
	//{
	//	cout << "no history" << endl;
	//}


}

bool  PickModelHandler::doAddObj(MatrixTransform * trans, bool isDetectCollision)
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


	if (isDetectCollision)
	{
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

			_allModelVec.push_back(trans);
			_allCollisionObjVec.push_back(btBoxObject);
			_allSelectBoxVec.push_back(NULL);
			_allSelectIndexVec.push_back(-1);
			//_allNum++;

			//cout << "all model num: " << _allNum << endl;
			cout << "all model vec size:" << _allModelVec.size() << " " << _allCollisionObjVec.size() << endl;
			return true;
		}
	}
	else
	{
		_allModelVec.push_back(trans);
		_allCollisionObjVec.push_back(btBoxObject);
		//_allNum++;

		//cout << "all model num: " << _allNum << endl;
		cout << "all model vec size:" << _allModelVec.size() << " " << _allCollisionObjVec.size() << endl;
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
	pushToHistory();
	bool res = doAddObj(trans.get());
	return res;
}

MatrixTransform* PickModelHandler::duplicateOneObj(MatrixTransform * matrixTrans)
{
	if (matrixTrans == NULL) {
		return NULL;
	}
	//MatrixTransform* trans = new MatrixTransform(*matrixTrans, CopyOp::DEEP_COPY_ALL);
	MatrixTransform* trans = new MatrixTransform(*matrixTrans, CopyOp::SHALLOW_COPY);
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

MatrixTransform *PickModelHandler::getOrCreateSelectionBox(unsigned int index, int clientNum)
{
	if (clientNum != 0 && clientNum != 1)
	{
		cout << "clientNum error:0 or 1" << endl;
		return NULL;
	}

	//if (index < _allSelectBoxVec.size()  && _allSelectBoxVec[index] == NULL)
	//{
		ref_ptr<ShapeDrawable> boxDrawable = new ShapeDrawable(new Box(Vec3(), 1.0f));

		if (clientNum == 0)
		{
			boxDrawable->setColor(Vec4(.9f, .9f, .9f, 0.0f));
		}
		else if (clientNum == 1)
		{
			boxDrawable->setColor(Vec4(0.3f, 0.6f, 0.3f, 0.0f));
		}
		ref_ptr<Geode> geode = new Geode;
		geode->addDrawable(boxDrawable.get());
		ref_ptr<MatrixTransform> selectionBox = new MatrixTransform;
		//selectionBox->setNodeMask(0);		//hide this box
		selectionBox->addChild(geode.get());
		ref_ptr<StateSet> ss = selectionBox->getOrCreateStateSet();
		ss->setMode(GL_LIGHTING, StateAttribute::OFF);
		ss->setAttributeAndModes(new PolygonMode(PolygonMode::FRONT_AND_BACK, PolygonMode::LINE));

		_allSelectBoxVec[index] = selectionBox;
		_root->addChild(selectionBox); //add selection Box to root
		return selectionBox;

	//}
	//else
	//{
	//	ref_ptr<MatrixTransform> selectionBox = _allSelectBoxVec[index];
	//	ref_ptr<Geode> geode = selectionBox->getChild

	//}
	

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
	if (index > _allModelVec.size())
	{
		cout << "index out of border in deleteOneObj" << endl;
		return false;
	}

	ref_ptr<MatrixTransform> model = _allModelVec[index];
	ref_ptr<MatrixTransform> box = _allSelectBoxVec[index];
	btCollisionObject* collisionObj = _allCollisionObjVec[index];

	if (model != NULL && box != NULL && collisionObj != NULL){
		_root->removeChild(model);
		_collisionWorld->removeCollisionObject(collisionObj);
		_root->removeChild(box);

		_allModelVec[index] = NULL;
		_allCollisionObjVec[index] = NULL;
		_allSelectBoxVec[index] = NULL;
		//_allSelectBoxVec[index]->setNodeMask(0);

		return true;
	}
	else {
		cout << "delete nothing!" << endl;
		return false;
	}
	return false;
}

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

bool PickModelHandler::rotateMultipleObj( Matrix rotMatrix, int clientNum)
{

	vector<MatrixTransform*> modelVec;
	vector<MatrixTransform*> boxVec;
	vector<btCollisionObject*> collisionObjVec;
	for (int i = 0; i < _allModelVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			modelVec.push_back(_allModelVec[i]);
			boxVec.push_back(_allSelectBoxVec[i]);
			collisionObjVec.push_back(_allCollisionObjVec[i]);
		}
	}
	if (modelVec.size() < 2)
	{
		cout << "too few obj" << endl;
		return false;
	}

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
		//clearAllchoose(clientNum);
		//cout << "rechoose" << endl;

		for (int i = 0; i < _allSelectIndexVec.size(); i++) {
			if (_allSelectIndexVec[i] == clientNum)
			{
				_allSelectBoxVec[i]->setNodeMask(0);
				_allSelectIndexVec[i] = -1;
				chooseOneMatrixTransform(i, clientNum);
			}
		}

		//for (int i = 0; i < modelVec.size(); i++)
		//{
		//	MatrixTransform* model = modelVec[i];
		//	if (model == NULL) continue;
		//	chooseOneMatrixTransform(model,  clientNum);
		//	tmpN2++;
		//}
		return true;
	}


}
bool PickModelHandler::scaleGap(bool isInceaseGap, int clientNum)
{
	vector<MatrixTransform*> modelVec;
	vector<MatrixTransform*> boxVec;
	vector<btCollisionObject*> collisionObjVec;
	for (int i = 0; i < _allModelVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			modelVec.push_back(_allModelVec[i]);
			boxVec.push_back(_allSelectBoxVec[i]);
			collisionObjVec.push_back(_allCollisionObjVec[i]);
		}
	}
	if (modelVec.size() < 2)
	{
		cout << "too few obj" << endl;
		return false;
	}
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


		for (int i = 0; i < _allSelectIndexVec.size(); i++) {
			if (_allSelectIndexVec[i] == clientNum)
			{
				_allSelectBoxVec[i]->setNodeMask(0);
				_allSelectIndexVec[i] = -1;
				chooseOneMatrixTransform(i, clientNum);
			}
		}

		//clearAllchoose();
		//int tmpN2 = 0;
		//for (int i = 0; i < modelVec.size(); i++)
		//{
		//	MatrixTransform* model = modelVec[i];
		//	if (model == NULL) continue;
		//	chooseOneMatrixTransform(model);
		//	tmpN2++;
		//}
		return true;
	}


}

bool PickModelHandler::permutateRow(int clientNum)
{
	vector<MatrixTransform*> modelVec;
	vector<MatrixTransform*> boxVec;
	vector<btCollisionObject*> collisionObjVec;
	for (int i = 0; i < _allModelVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			modelVec.push_back(_allModelVec[i]);
			boxVec.push_back(_allSelectBoxVec[i]);
			collisionObjVec.push_back(_allCollisionObjVec[i]);
		}
	}
	if (modelVec.size() < 2)
	{
		cout << "too few obj" << endl;
		return false;
	}
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

			for (int i = 0; i < _allSelectIndexVec.size(); i++) {
				if (_allSelectIndexVec[i] == clientNum)
				{
					_allSelectBoxVec[i]->setNodeMask(0);
					_allSelectIndexVec[i] = -1;
					chooseOneMatrixTransform(i, clientNum);
				}
			}

			//clearAllchoose();
			//int tmpN2 = 0;
			//for (int i = 0; i < modelVec.size(); i++)
			//{
			//	MatrixTransform* model = modelVec[i];
			//	if (model == NULL) continue;
			//	chooseOneMatrixTransform(model);
			//	tmpN2++;
			//}
			return true;
		}

	}
	return false;
}

bool PickModelHandler::permutateRound(int clientNum)
{
	vector<MatrixTransform*> modelVec;
	vector<MatrixTransform*> boxVec;
	vector<btCollisionObject*> collisionObjVec;

	for (int i = 0; i < _allModelVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			modelVec.push_back(_allModelVec[i]);
			boxVec.push_back(_allSelectBoxVec[i]);
			collisionObjVec.push_back(_allCollisionObjVec[i]);
		}
	}
	if (modelVec.size() < 2)
	{
		cout << "too few obj" << endl;
		return false;
	}
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


			for (int i = 0; i < _allSelectIndexVec.size(); i++) {
				if (_allSelectIndexVec[i] == clientNum)
				{
					_allSelectBoxVec[i]->setNodeMask(0);
					_allSelectIndexVec[i] = -1;
					chooseOneMatrixTransform(i, clientNum);
				}
			}

			//clearAllchoose();
			//int tmpN2 = 0;
			//for (int i = 0; i < modelVec.size(); i++)
			//{
			//	MatrixTransform* model = modelVec[i];
			//	if (model == NULL) continue;
			//	
			//	chooseOneMatrixTransform(model);
			//	tmpN2++;
			//}
			

			return true;
		}
	}
	return false;
}

bool PickModelHandler::chooseOneMatrixTransform(int index, int clientNum)
{
	if (clientNum != 0 && clientNum != 1)
	{
		cout << "clientNum error:0 or 1" << endl;
		return false;
	}
	if (index == -1)
	{
		clearAllchoose(clientNum);
		return false;
	}
	if (index < -1 || index >= _allModelVec.size())
	{
		cout << "index out of border in chooseOneMatrixTransformWithPush" << endl;
		return false;
	}

	if (index >= 0 && index < _allModelVec.size())
	{
		MatrixTransform* model = _allModelVec[index];
		if (model == NULL) return false;
		bool res = chooseOneMatrixTransform(model, index, clientNum);
		//addBackground(_imagePath);
		return res;
	}
	else
	{
		cout << "invalid index in chooseOneMatrixTransform" << endl;
		return false;
	}

}


bool PickModelHandler::chooseOneMatrixTransformWithPush(int index, int clientNum)
{
	if (clientNum != 0 && clientNum != 1)
	{
		cout << "clientNum error:0 or 1" << endl;
		return false;
	}
	pushToHistory();
	if (index == -1)
	{
		clearAllchoose(clientNum);
		return false;
	}
	if (index < -1 || index >= _allModelVec.size())
	{
		cout << "index out of border in chooseOneMatrixTransformWithPush" << endl;
		return false;
	}

	if (index >= 0 && index < _allModelVec.size())
	{
		MatrixTransform* model = _allModelVec[index];
		if (model == NULL) return false;
		return chooseOneMatrixTransform(model,index, clientNum);
	}
	else
	{
		cout << "invalid index in chooseOneMatrixTransform" << endl;
		return false;
	}

}

bool PickModelHandler::chooseOneMatrixTransform(MatrixTransform* lastModel, int index, int clientNum)
{
	if (clientNum != 0 && clientNum != 1)
	{
		cout << "clientNum error:0 or 1" << endl;
		return false;
	}

	bool isalreadyChosed = false;
	
	bool isChosedByOppositeSide = false;
	bool isChosedByMySide = false;
	int oppositeNum = 1 - clientNum;
	if (_allSelectIndexVec[index] == oppositeNum)//first check :is chosed by opposite client
	{
		isChosedByOppositeSide = true;
		_root->removeChild(_allSelectBoxVec[index]);
		_allSelectIndexVec[index] = clientNum; // change choose to our side
		//_allSelectBoxVec[index] = getOrCreateSelectionBox(index, clientNum);
		//return;

	}
	else if (_allSelectIndexVec[index] == clientNum) //is chosed by our side
	{
		isChosedByMySide = true;
		_allSelectIndexVec[index] = -1; // clear choose
		_root->removeChild(_allSelectBoxVec[index]);
		_allSelectBoxVec[index] = NULL;
		return false;
	}
	_allSelectIndexVec[index] = clientNum;

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

	mat *= lastModel->getMatrix();

	ref_ptr<MatrixTransform> tbox = getOrCreateSelectionBox(index, clientNum);
	tbox->setNodeMask(0x1);
	tbox->setMatrix(mat);

	return true;
}

void PickModelHandler::clearAllchoose(int clientNum)
{
	cout << "clear all choose" << endl;
	for (int i = 0; i < _allSelectIndexVec.size(); i++) {
		if (_allSelectIndexVec[i] == clientNum)
		{
			_allSelectIndexVec[i] = -1;
			_allSelectBoxVec[i]->setNodeMask(0);
		}
	}

}

int  PickModelHandler::handlePickEvent(float clickX, float clickY, int clientNum)//const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
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
					bool foundFlag = false;
					int idx = 0;
					for (idx = 0; idx < _allModelVec.size(); idx++)
					{
						
						if (lastModel == _allModelVec[idx])
						{
							pushToHistory();
							
							chooseOneMatrixTransform(idx, clientNum);
							//pushToHistory();
							//popFromHistory();
							foundFlag = true;
							break;
						}
					}
					
					if (!foundFlag) cout << "can't find corresponding model in handlePickEvent" << endl;

					//chooseOneMatrixTransform(lastModel);
					return idx;
					
				}
			}
		}
		else
		{
			cout << "choose nothing" << endl;
			pushToHistory();
			chooseOneMatrixTransform(-1, clientNum);
			//clearAllchoose();
			//_selectNum = 0;
			//for (int i = 0; i < _selectionBoxVec.size(); i++) {
			//	_selectionBoxVec[i]->setNodeMask(0); //hide all the box
			//}
			//_lastModelVec.clear();
			//_selectCollisionObjVec.clear();
			return -1;
		}
	}
	return -1;
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
		bool isPopHistory = false;

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
		case 'f':
		case 'F':
			isPopHistory = true;
			break;
		case 'z':
		case 'Z':
			addBackground("wall.jpg");

			break;
		default:
			break;
		}

		int client1 = 0, client2 = 1;

		if (isPopHistory) {
			popFromHistoryAPI();
			
		}
		else if (isTranslate) {
			translateAPI(transVec, client1);
		}
		else if (isRotate) {
			rotateAPI(rotMatrix, client1);
		}
		else if (isRotateMultiple) {
			rotateAllAPI(rotMatrix, client1);
		}
		else if (isScale) {
			scaleAPI(scaleFactor, client1);
		}

		else if (isDuplicate) {
			duplicateAPI(client1);
		}
		else if (isDelete) {
			deleteAPI(client1);
		}
		else if (isPermutateRound) {
			permutateRoundAPI(client1);
		}
		else if (isPermutateRow) {
			permutateRowAPI(client1);
		}

	}
}

//****************API************************
void PickModelHandler::popFromHistoryAPI()
{
	popFromHistory();
}
void  PickModelHandler::translateAPI(Vec3 transVec, int clientNum)
{
	int tmp = _allModelVec.size();
	int sNum = 0;
	for (int i = 0; i < _allSelectIndexVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			sNum++;
		}
	}

	if (sNum < 1)
	{
		cout << "nothing to translate" << endl;
		return;
	}
	pushToHistory();

	for (int i = 0; i < tmp; i++) 
	{
		if (_allSelectIndexVec[i] == clientNum)
		    translateOneObj(_allModelVec[i], _allSelectBoxVec[i], _allCollisionObjVec[i], transVec);
		//translateOneObj(_lastModelVec[i], _selectionBoxVec[i], _selectCollisionObjVec[i], transVec);
	}
}

void  PickModelHandler::rotateAPI(Matrix rotMatrix, int clientNum)
{
	int tmp = _allModelVec.size();
	int sNum = 0;
	for (int i = 0; i < _allSelectIndexVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			sNum++;
		}
	}

	if (sNum < 1)
	{
		cout << "nothing to rotate" << endl;
		return;
	}

	pushToHistory();
	bool flag = true;
	int i = 0;
	for (i = 0; i < tmp; i++) {
		if (_allSelectIndexVec[i] == clientNum)
		{
			flag = rotateOneObj(_allModelVec[i], _allSelectBoxVec[i], _allCollisionObjVec[i], rotMatrix);
			if (!flag) {
				break;
			}
		}

	}
	//if (tmp >= 2 && !flag)
	//{
	//	scaleGap(true, 0);
	//	for (; i < tmp; i++) {
	//	rotateOneObj(_lastModelVec[i], _selectionBoxVec[i], _selectCollisionObjVec[i], rotMatrix);
	//	}
	//}
}
void  PickModelHandler::rotateAllAPI(Matrix rotMatrix, int clientNum)
{
	int tmp = _allModelVec.size();
	int sNum = 0;
	for (int i = 0; i < _allSelectIndexVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			sNum++;
		}
	}

	if (sNum < 2)
	{
		cout << "too few objs to rotate" << endl;
	}
	else
	{
		pushToHistory();

		rotateMultipleObj(rotMatrix, clientNum);
	}
}
void  PickModelHandler::duplicateAPI(int clientNum)
{
	int tmp = _allModelVec.size();
	int sNum = 0;
	for (int i = 0; i < _allSelectIndexVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			sNum++;
		}
	}

	if (sNum < 1)
	{
		cout << "nothing to duplicate" << endl;
		return;
	}
	
	int tNum = 0;
	pushToHistory();
	for (int i = 0; i < tmp; i++) {
		if (_allSelectIndexVec[i] == clientNum)
		{
			if (_allModelVec[i] == NULL) continue;
			tNum++;
			MatrixTransform* tmatrix = duplicateOneObj(_allModelVec[i]);
			if (tmatrix != NULL) {
				chooseOneMatrixTransform(tmatrix, i, clientNum);
			}
			break;
		}
	}
}
void PickModelHandler::deleteAPI(int clientNum)
{
	int tmp = _allModelVec.size();
	int sNum = 0;
	for (int i = 0; i < _allSelectIndexVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			sNum++;
		}
	}
	
	if (sNum < 1)
	{
		cout << "nothing to delete" << endl;
		return;
	}

	pushToHistory();
	for (unsigned int i = 0; i < tmp; i++) {
		if (_allSelectIndexVec[i] == clientNum)
		{
			deleteOneObj(i);
		}
	}
	clearAllchoose(clientNum);

	cout << "all model vec size:" << _allModelVec.size() << " " << _allCollisionObjVec.size() << " "<< _allSelectIndexVec.size() <<endl;
}
void  PickModelHandler::scaleAPI(Vec3 scaleFactor, int clientNum)
{
	int tmp = _allModelVec.size();
	int sNum = 0;
	for (int i = 0; i < _allSelectIndexVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			sNum++;
		}
	}

	if (sNum < 1)
	{
		cout << "nothing to scale" << endl;
		return;
	}
	bool isIncreaseGap = true;
	if (scaleFactor.x() < 1.0) isIncreaseGap = false;
	if (tmp < 2)
	{
		pushToHistory();
		for (int i = 0; i < tmp; i++) {
			scaleOneObj(_allModelVec[i], _allSelectBoxVec[i], _allCollisionObjVec[i], scaleFactor);
		}
	}
	else
	{
		pushToHistory();
		scaleGap(isIncreaseGap, clientNum);
	}

}
void  PickModelHandler::permutateRoundAPI(int clientNum)
{
	int tmp = _allModelVec.size();
	int sNum = 0;
	for (int i = 0; i < _allSelectIndexVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			sNum++;
		}
	}

	if (sNum < 2)
	{
		cout << "too few objects are selected";
	}
	else
	{
		pushToHistory();
		permutateRound(clientNum);

	}
}
void  PickModelHandler::permutateRowAPI(int clientNum)
{
	int tmp = _allModelVec.size();
	int sNum = 0;
	for (int i = 0; i < _allSelectIndexVec.size(); i++)
	{
		if (_allSelectIndexVec[i] == clientNum)
		{
			sNum++;
		}
	}

	if (sNum < 2)
	{
		cout << "too few objects are selected";
	}
	else
	{
		pushToHistory();
		permutateRow(clientNum);
	}
}

int PickModelHandler::getSelectNumAPI(int clientNum)
{
	int res;
	for (int i = 0; i < _allModelVec.size(); i++) {
		if (_allSelectIndexVec[i] == clientNum)
			res++;
	}
	return res;
}
int PickModelHandler::pickAPI(float clickX, float clickY, int clientNum)
{
	
	return handlePickEvent(clickX, clickY, clientNum);
}
bool PickModelHandler::chooseAPI(int index, int clientNum)
{
	
	return chooseOneMatrixTransformWithPush(index, clientNum);
}
void PickModelHandler::addBackground(string imagePath)
{
	_imagePath = imagePath;
	ref_ptr<Image> tImage = osgDB::readImageFile(_imagePath);
	//createBillboardTree(tImage.get(), 800.0);
	createBackboard(tImage.get(), 1600.0);
}

//void PickModelHandler::addBackground(Camera* _camera)
//{
//	_root->addChild(_camera);
//}

#endif
