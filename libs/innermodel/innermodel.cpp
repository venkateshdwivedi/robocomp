/*
 *    Copyright (C) 2010-2015 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <innermodel/innermodel.h>
#include <innermodel/innermodelreader.h>
#include <iostream>
#include <fstream>

bool InnerModel::support_fcl()
{
	#if FCL_SUPPORT==1
		return true;
	#else
		return false;
	#endif
}

///////////////////////
/// (Con/De)structors
///////////////////////
InnerModel::InnerModel(std::string xmlFilePath)
{
	root = NULL;
	if (not InnerModelReader::load(QString::fromStdString(xmlFilePath), this))
	{
		QString error;
		error.sprintf("InnerModelReader::load error using file %s\n", xmlFilePath.c_str());
		throw error;
	}
}

InnerModel::InnerModel()
{	
	// Set Root node
	InnerModelTransform *root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	root->parent = NULL;
	setRoot(root);
	root->innerModel = this;
	hash.put("root",root);
}

InnerModel::InnerModel(const InnerModel &original)
{
	root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	setRoot(root);
	root->innerModel = this;
	hash.put("root",root);
	
	QList<InnerModelNode *>::iterator i;
	for (i=original.root->children.begin(); i!=original.root->children.end(); i++)
	{
		root->addChild((*i)->copyNode(hash, root));
	}
}

InnerModel::InnerModel(InnerModel &original)
{
	
	root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	setRoot(root);
	root->innerModel = this;
	hash.put("root", root);

	QList<InnerModelNode *>::iterator i;
	for (i=original.root->children.begin(); i!=original.root->children.end(); i++)
	{
		root->addChild((*i)->copyNode(hash, root));
	}
}

InnerModel::InnerModel(InnerModel *original)
{
	
	root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	setRoot(root);
	root->innerModel = this;
	hash.put("root", root);

	QList<InnerModelNode *>::iterator i;
	for (i=original->root->children.begin(); i!=original->root->children.end(); i++)
	{
		root->addChild((*i)->copyNode(hash, root));
	}
}

InnerModel::~InnerModel()
{
	foreach (QString id, getIDKeys())
	{
		InnerModelNode *dd = hash[id];
		delete dd;
	}
	hash.clear();
	localHashRot.clear();
	localHashTr.clear();
}

/////////////////////////////////////////////////////////////77
/// Copy and editing methods
///////////////////////////////////////////////////////////////

InnerModel* InnerModel::copy()
{
	InnerModel *inner = new InnerModel();
	QList<InnerModelNode *>::iterator i;
	for (i=root->children.begin(); i!=root->children.end(); i++)
		inner->root->addChild((*i)->copyNode(inner->hash, inner->root));

	return inner;
}

void InnerModel::removeNode(const QString & id)  ///Que pasa con los hijos y el padre?
{
	InnerModelNode *dd = hash[id];
	delete dd;
	hash.remove(id);
}

bool InnerModel::open(std::string xmlFilePath)
{
	return InnerModelReader::load(QString::fromStdString(xmlFilePath), this);
}

void InnerModel::removeSubTree(InnerModelNode *node, QStringList *l)
{
	QList<InnerModelNode*>::iterator i;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		removeSubTree(*i,l);
	}
	node->parent->children.removeOne(node);
	l->append(node->getId());
	removeNode(node->getId());
}

/**
 * @brief Returns a list of node's ID corresponding to the subtree starting at node
 *
 * @param node starting node
 * @param l pointer to QStringList
 * @return void
 */
void InnerModel::getSubTree(InnerModelNode *node, QStringList *l)
{
	
	QList<InnerModelNode*>::iterator i;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		getSubTree(*i,l);
	}
	l->append(node->getId());
}

void InnerModel::getSubTree(InnerModelNode *node, QList<InnerModelNode *> *l)
{
	
	QList<InnerModelNode*>::iterator i;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		l->append((*i));
		getSubTree(*i,l);
	}
}

/**
 * @brief Returns a list of node's ID corresponding to the subtree starting at node
 *
 * @param node starting node
 * @param l pointer to QStringList
 * @return void
 */
void InnerModel::moveSubTree(InnerModelNode *nodeSrc, InnerModelNode *nodeDst)
{
	nodeSrc->parent->children.removeOne(nodeSrc);
	nodeDst->addChild(nodeSrc);
	nodeSrc->setParent(nodeDst);
	computeLevels(nodeDst);
}

void InnerModel::computeLevels(InnerModelNode *node)
{
	if (node->parent != nullptr )
		node->setLevel(node->parent->getLevel() + 1);
	
	QList<InnerModelNode*>::iterator i;
	for (i=node->children.begin(); i!=node->children.end(); i++)
		computeLevels(*i);
}

bool InnerModel::save(QString path)
{
	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return false;

	QTextStream out(&file);
	root->save(out, 0);
	file.close();
	return true;
}


//////////////////////////////////////////////////////////////////////////
/// Tree update methods
///////////////////////////////////////////////////////////////////////////

void InnerModel::update()
{
	root->update();
	cleanupTables();
}

void InnerModel::cleanupTables()
{
	localHashTr.clear();
	localHashRot.clear();
}

void InnerModel::updateTransformValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId)
{
	Lock lockHash(localHashTr.mutex);
	
	cleanupTables();
	if(transformId == "root") return;   										///CHECK THIS
	InnerModelTransform *aux = getNode<InnerModelTransform>(transformId);
	if(aux == nullptr) return;
	//qDebug() << "up2" << aux->parent->id;
	InnerModelTransform *auxParent = getNode<InnerModelTransform>(aux->parent->getId());
	if(auxParent == nullptr) return;
	
	InnerModelTransform *parent = getNode<InnerModelTransform>(parentId);
	if(parent != nullptr)
		aux->transformValues(getTransformationMatrix(aux->parent->getId(),parentId), tx, ty, tz, rx, ry, rz, parent);
	else
		aux->update(tx,ty,tz,rx,ry,rz);
}

void InnerModel::updateTransformValuesS(std::string transformId, float tx, float ty, float tz, float rx, float ry, float rz, std::string parentId)
{
		updateTransformValues(QString::fromStdString(transformId), tx, ty, tz, rx, ry, rz, QString::fromStdString(parentId));
}

void InnerModel::updateTransformValuesS(std::string transformId, QVec v, std::string parentId)
{
		updateTransformValues(QString::fromStdString(transformId), v(0), v(1), v(2), v(3), v(4), v(5), QString::fromStdString(parentId));
}

void InnerModel::updateTransformValues(QString transformId, QVec v, QString parentId)
{
		updateTransformValues(transformId, v(0), v(1), v(2), v(3), v(4), v(5), parentId);
}

void InnerModel::updatePlaneValues(QString planeId, float nx, float ny, float nz, float px, float py, float pz)
{
	Lock lockHash(localHashTr.mutex);
	
	cleanupTables();
	InnerModelPlane *plane = getNode<InnerModelPlane>(planeId);
	//InnerModelPlane *plane = dynamic_cast<InnerModelPlane *>(hash[planeId]);
	if (plane == nullptr)
		throw InnerModelException("InnerModel::updatePlaneValues Error: node not found " + planeId.toStdString());
	
	plane->update(nx, ny, nz, px, py, pz);
	
}

void InnerModel::updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId)
{
	Lock lockHash(localHashTr.mutex);
	
	cleanupTables();
	if(transformId == "root") return;   												///CHECK THIS
	InnerModelTransform *aux = getNode<InnerModelTransform>(transformId);
	if(aux == nullptr) return;

	InnerModelTransform *auxParent = getNode<InnerModelTransform>(aux->parent->getId());
	if(auxParent == nullptr) return;

	InnerModelTransform *parent = getNode<InnerModelTransform>(parentId);
	if(parent != nullptr)
		aux->translateValues(getTransformationMatrix(aux->parent->getId(),parentId), tx, ty, tz, parent);
	else
		aux->updateT(tx,ty,tz);
} 

void InnerModel::updateRotationValues(QString transformId, float rx, float ry, float rz, QString parentId)
{
	Lock lockHash(localHashTr.mutex);
	
	cleanupTables();
	if(transformId == "root") return;   												///CHECK THIS
	InnerModelTransform *aux = getNode<InnerModelTransform>(transformId);
	if(aux == nullptr) return;

	InnerModelTransform *auxParent = getNode<InnerModelTransform>(aux->parent->getId());
	if(auxParent == nullptr) return;
	
	InnerModelTransform *parent = getNode<InnerModelTransform>(parentId);
	if(parent != nullptr)
		aux->rotateValues(getTransformationMatrix(aux->parent->getId(),parentId), rx, ry, rz, parent);
	else
		aux->updateR(rx,ry,rz);
}

void InnerModel::updateJointValue(QString jointId, float angle, bool force)
{
	Lock lockHash(localHashTr.mutex);
	
	cleanupTables();
	InnerModelJoint *j = getNode<InnerModelJoint>(jointId);
	if(j == nullptr) 
		throw InnerModelException("InnerModel::updateJointValue Error: node not found " + jointId.toStdString());
	
	j->setAngle(angle, force);
 }

void InnerModel::updatePrismaticJointPosition(QString jointId, float pos)
{
	Lock lockHash(localHashTr.mutex);
	
	cleanupTables();
	InnerModelPrismaticJoint *j = getNode<InnerModelPrismaticJoint>(jointId);
	if (j == nullptr)
		throw InnerModelException("InnerModel::updateJointValue Error: node not found " + jointId.toStdString());

	j->setPosition(pos);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Information retrieval methods
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QVec InnerModel::transform(const QString &destId, const QVec &initVec, const QString &origId)
{	
	if (initVec.size()==3)
	{
		return (getTransformationMatrix(destId, origId) * initVec.toHomogeneousCoordinates()).fromHomogeneousCoordinates();
	}
	else if (initVec.size()==6)
	{
		const QMat M = getTransformationMatrix(destId, origId);
		const QVec a = (M * initVec.subVector(0,2).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
		const Rot3D R(initVec(3), initVec(4), initVec(5));

		const QVec b = (M.getSubmatrix(0,2,0,2)*R).extractAnglesR_min();
		QVec ret(6);
		ret(0) = a(0);
		ret(1) = a(1);
		ret(2) = a(2);
		ret(3) = b(0);
		ret(4) = b(1);
		ret(5) = b(2);
		return ret;
	}
	else
		throw InnerModelException("InnerModel::transform was called with an unsupported vector size: " +std::to_string(initVec.size()));
}

QVec InnerModel::transformS(const std::string & destId, const QVec &origVec, const std::string & origId)
{
		return transform(QString::fromStdString(destId), origVec, QString::fromStdString(origId));
}

QVec InnerModel::rotationAngles(const QString & destId, const QString & origId)
{
	return getTransformationMatrix(destId, origId).extractAnglesR();
}

//////////////////////////////////////////////////////////////////////////////////////
/// Thread safe Matrix transformation retrieval methods
/////////////////////////////////////////////////////////////////////////////////////
RTMat InnerModel::getTransformationMatrix(const QString &to, const QString &from)
{	
	std::pair<bool, RTMat> r = localHashTr.checkandget(QPair<QString, QString>(to, from));
	if(r.first) return r.second;
	
	RTMat ret;	
	std::pair<QList<InnerModelNode *>, QList<InnerModelNode *>> list = setLocalLists(from, to);
	QList<InnerModelNode *> &listA = list.first;
	QList<InnerModelNode *> &listB = list.second;
	
	foreach (InnerModelNode *i, listA)
		ret = (*i).matrixmultTS(ret);
	foreach (InnerModelNode *i, listB)
	{
// 		QMat inv = (*i).invertTS();
// 		ret = inv * ret;
 		ret = (*i).invertTS() * ret;
	}
	localHashTr.put(QPair<QString, QString>(to, from),ret);
	return ret;
}

RTMat InnerModel::getTransformationMatrixS(const std::string &destId, const std::string &origId)
{
	return getTransformationMatrix(QString::fromStdString(destId), QString::fromStdString(origId));
}

QMat InnerModel::getRotationMatrixTo(const QString &to, const QString &from)
{
	std::pair<bool, QMat> r = localHashRot.checkandget(QPair<QString, QString>(to, from));
	if(r.first) return r.second;
	
	QMat rret = QMat::identity(3);
	// Get locked list of InnerModelNode pointers. Unlock after using them!
	std::pair<QList<InnerModelNode *>, QList<InnerModelNode *>> list = setLocalLists(from, to);
	QList<InnerModelNode *> &listA = list.first;
	QList<InnerModelNode *> &listB = list.second;
	
	InnerModelTransform *tf = nullptr;
	foreach (InnerModelNode *i, listA)
		if ((tf=dynamic_cast<InnerModelTransform *>(i)) != nullptr)
		{
			rret = tf->getR() * rret;
			//tf->unlock();
		}
		
	foreach (InnerModelNode *i, listB)
		if ((tf=dynamic_cast<InnerModelTransform *>(i)) != nullptr)
		{
			rret = tf->getR().transpose() * rret;
			//tf->unlock();
		}
	
	localHashRot[QPair<QString, QString>(to, from)] = rret;
	return rret;
}

QVec InnerModel::getTranslationVectorTo(const QString &to, const QString &from)
{
	QMat m = getTransformationMatrix(to, from);
	return m.getCol(3);
}

std::pair<QList<InnerModelNode *>, QList<InnerModelNode *>> InnerModel::setLocalLists(const QString & origId, const QString & destId)
{
	InnerModelNode *a = hash.checkandget(origId).second;
	InnerModelNode *b = hash.checkandget(destId).second;
	
	if (a == nullptr)
		throw InnerModelException("InnerModel::setLocalLists: Cannot find node: \""+ origId.toStdString()+"\"");
	if (b == nullptr)
		throw InnerModelException("InnerModel::setLocalLists: Cannot find node: "+ destId.toStdString()+"\"");

	int minLevel = a->getLevel() < b->getLevel() ? a->getLevel() : b->getLevel();
	QList<InnerModelNode *> listA;
	while (a->getLevel() >= minLevel)
	{
		listA.push_back(a);
		if(a->parent == nullptr)
			break;
		a=a->parent;
	}
	QList<InnerModelNode *> listB;
	while (b->getLevel() >= minLevel)
	{
		listB.push_front(b);
		if(b->parent == nullptr)
			break;
		b=b->parent;
	}
	while (b!=a)
	{
		listA.push_back(a);
		listB.push_front(b);
		a = a->parent;
		b = b->parent;
	}
	return std::make_pair(listA, listB);
}

/////////////////////////////////////////////////////////////////
/// Model construction methods
/////////////////////////////////////////////////////////////////

void InnerModel::setRoot(InnerModelNode *node)
{
	root = node;
	hash.put("root", root);
	root->parent = nullptr;
}

InnerModelJoint *InnerModel::newJoint(QString id, InnerModelTransform *parent,float lx, float ly, float lz,float hx, float hy, float hz, float tx, float ty, float tz, float rx, float ry, float rz, float min, float max, uint32_t port,std::string axis, float home)
{
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newJoint: Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");
	
	InnerModelJoint *newnode = new InnerModelJoint(id,lx,ly,lz,hx,hy,hz, tx, ty, tz, rx, ry, rz, min, max, port, axis, home, parent);
	hash.put(id,newnode); 
	return newnode;
}

InnerModelTouchSensor *InnerModel::newTouchSensor(QString id, InnerModelTransform *parent, QString stype, float nx, float ny, float nz, float min, float max, uint32_t port)
{
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newTouchSensor: Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");
	
	InnerModelTouchSensor *newnode = new InnerModelTouchSensor(id, stype, nx, ny, nz, min, max, port, parent);
	hash.put(id,newnode);
	return newnode;
}

InnerModelPrismaticJoint *InnerModel::newPrismaticJoint(QString id, InnerModelTransform *parent, float min, float max, float value, float offset, uint32_t port,std::string axis, float home)
{	
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newPrismaticJoint: Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");

	InnerModelPrismaticJoint *newnode = new InnerModelPrismaticJoint(id, min, max, value, offset, port, axis, home, parent);
	hash.put(id,newnode);
	return newnode;
}

InnerModelDifferentialRobot *InnerModel::newDifferentialRobot(QString id, InnerModelTransform *parent, float tx, float ty, float tz, float rx, float ry, float rz, uint32_t port, float noise, bool collide)
{
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newDifferentialRobot: Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");
	
	InnerModelDifferentialRobot *newnode = new InnerModelDifferentialRobot(id, tx, ty, tz, rx, ry, rz, port, noise, collide, parent);
	hash.put(id,newnode);
	return newnode;
}

InnerModelOmniRobot *InnerModel::newOmniRobot(QString id, InnerModelTransform *parent, float tx, float ty, float tz, float rx, float ry, float rz, uint32_t port, float noise, bool collide)
{
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newOmniRobot: Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");	
		
	InnerModelOmniRobot *newnode = new InnerModelOmniRobot(id, tx, ty, tz, rx, ry, rz, port, noise, collide, parent);
	hash.put(id,newnode);
	return newnode;
}

InnerModelCamera *InnerModel::newCamera(QString id, InnerModelNode *parent, float width, float height, float focal)
{
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newCamera: Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");	
		
	InnerModelCamera *newnode = new InnerModelCamera(id, width, height, focal, this, parent);
	hash.put(id,newnode);
	return newnode;
}

InnerModelRGBD *InnerModel::newRGBD(QString id, InnerModelNode *parent, float width, float height, float focal, float noise, uint32_t port, QString ifconfig)
{
	if (noise < 0)
		throw InnerModelException("InnerModel::newRGBD: Error: noise can't have negative values " + std::to_string(noise) + "\n");	

	if (hash.contains(id))
		throw InnerModelException("InnerModel::newRGBD: Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");	
		
	InnerModelRGBD *newnode = new InnerModelRGBD(id, width, height, focal, noise, port, ifconfig, this, parent);
	hash.put(id,newnode);
	return newnode;
}

InnerModelIMU *InnerModel::newIMU(QString id, InnerModelNode *parent, uint32_t port)
{
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newIMU: Error Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");	
		
	InnerModelIMU *newnode = new InnerModelIMU(id, port, parent);
	hash.put(id,newnode);
	return newnode;
}

InnerModelLaser *InnerModel::newLaser(QString id, InnerModelNode *parent, uint32_t port, uint32_t min, uint32_t max, float angle, uint32_t measures, QString ifconfig)
{
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newLaser Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");
		
	InnerModelLaser *newnode = new InnerModelLaser(id, port, min, max, angle, measures, ifconfig, this, parent);
	hash.put(id,newnode);
	return newnode;
}

InnerModelPlane *InnerModel::newPlane(QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx, float ny, float nz, float px, float py, float pz, bool collidable)
{
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newPlane Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");
		
	InnerModelPlane *newnode = new InnerModelPlane(id, texture, width, height, depth, repeat, nx, ny, nz, px, py, pz, collidable, parent);
	hash.put(id,newnode);
	return newnode;
}

InnerModelMesh *InnerModel::newMesh(QString id, InnerModelNode *parent, QString path, float scalex, float scaley, float scalez, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable)
{
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newMesh Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");
		
	InnerModelMesh *newnode = new InnerModelMesh(id, path, scalex, scaley, scalez, (InnerModelMesh::RenderingModes)render, tx, ty, tz, rx, ry, rz, collidable, parent);
	hash.put(id,newnode);
	return newnode;
}

InnerModelMesh *InnerModel::newMesh(QString id, InnerModelNode *parent, QString path, float scale, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable)
{
	return newMesh(id,parent,path,scale,scale,scale,render,tx,ty,tz,rx,ry,rz, collidable);
}

InnerModelPointCloud *InnerModel::newPointCloud(QString id, InnerModelNode *parent)
{
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newPointCloud Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");
		
	InnerModelPointCloud *newnode = new InnerModelPointCloud(id, parent);
	hash.put(id,newnode);
	return newnode;
}

InnerModelTransform *InnerModel::newTransform(QString id, QString engine, InnerModelNode *parent, float tx, float ty, float tz, float rx, float ry, float rz, float mass)
{
	if (hash.contains(id))
		throw InnerModelException("InnerModel::newTransform: Error: Trying to insert a node with an already-existing key: " + id.toStdString() + "\n");
		
	InnerModelTransform *newnode = new InnerModelTransform(id, engine, tx, ty, tz, rx, ry, rz, mass, parent);
	hash.put(id,newnode);
	return newnode;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// Thread safe Auxiliary
/////////////////////////////////////////////////////////////////////////////////////////////
QString InnerModel::getParentIdentifier(QString id)
{
	InnerModelNode *n = getNode<InnerModelNode>(id);
	if(n != nullptr)
		return n->getId();
	else
		throw InnerModelException("InnerModel::getParentIdentifier Error: non existing node " + id.toStdString() + "\n");
}

std::string InnerModel::getParentIdentifierS(const std::string &id)
{
	return getParentIdentifier(QString::fromStdString(id)).toStdString();
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// Collision detection
/////////////////////////////////////////////////////////////////////////////////////////////

bool InnerModel::collidable(const QString &a)
{
	InnerModelNode *n = getNode<InnerModelNode>(a);
	if( n != nullptr)
		return n->getCollidable();
	else
		throw InnerModelException("InnerModel::collidable Error: Trying to insert a node with an already-existing key: " + a.toStdString() + "\n");		
}

bool InnerModel::collide(const QString &a, const QString &b)
{
#if FCL_SUPPORT==1
	InnerModelNode *n1 = getNode<InnerModelNode>(a);
	QMat r1q = getRotationMatrixTo("root", a);
	fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
	QVec t1v = getTranslationVectorTo("root", a);
	fcl::Vec3f T1( t1v(0), t1v(1), t1v(2) );
	n1->getCollisionObject()->setTransform(R1, T1);

	InnerModelNode *n2 = getNode<InnerModelNode>(b);
	QMat r2q = getRotationMatrixTo("root", b);
	fcl::Matrix3f R2( r2q(0,0), r2q(0,1), r2q(0,2), r2q(1,0), r2q(1,1), r2q(1,2), r2q(2,0), r2q(2,1), r2q(2,2) );
	QVec t2v = getTranslationVectorTo("root", b);
	fcl::Vec3f T2( t2v(0), t2v(1), t2v(2) );
	n2->getCollisionObject()->setTransform(R2, T2);

	fcl::CollisionRequest request;
	fcl::CollisionResult result;

	n1->getCollisionObject()->computeAABB();
	n2->getCollisionObject()->computeAABB();

	// NOTE: Un poco de documentacion nunca esta mal, sabeis --> http://gamma.cs.unc.edu/FCL/fcl_docs/webpage/generated/namespacefcl.html
	// std::size_t 	collide (const CollisionObject *o1, const CollisionObject *o2, const CollisionRequest &request, CollisionResult &result)
	fcl::collide(n1->getCollisionObject(), n2->getCollisionObject(), request, result);
	// return binary collision result --> http://gamma.cs.unc.edu/FCL/fcl_docs/webpage/generated/structfcl_1_1CollisionResult.html#ed599cb31600ec6d0585d9adb4cde946
	// True if There are collisions, and false if there arent collisions.
	n1->unlock(); n2->unlock();
	return result.isCollision();
#else
	throw InnerModelException("InnerModel was not compiled with collision support");
#endif
}


/**
 * @brief ...
 *
 * @param a ...
 * @param obj ...
 * @return bool
 */
#if FCL_SUPPORT==1
bool InnerModel::collide(const QString &a, const fcl::CollisionObject *obj)
{
	
	InnerModelNode *n1 = getNode<InnerModelNode>(a);
	QMat r1q = getRotationMatrixTo("root", a);
	fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
	QVec t1v = getTranslationVectorTo("root", a);
	fcl::Vec3f T1( t1v(0), t1v(1), t1v(2) );
	n1->getCollisionObject()->setTransform(R1, T1);

	fcl::CollisionRequest request;
	fcl::CollisionResult result;

	fcl::collide(n1->getCollisionObject(), obj, request, result);

	n1->unlock();
	return result.isCollision();
}
#endif

float InnerModel::distance(const QString &a, const QString &b)
{
#if FCL_SUPPORT==1
	InnerModelNode *n1 = getNode<InnerModelNode>(a);
	QMat r1q = getRotationMatrixTo("root", a);
	fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
	QVec t1v = getTranslationVectorTo("root", a);
	fcl::Vec3f T1( t1v(0), t1v(1), t1v(2) );
	n1->getCollisionObject()->setTransform(R1, T1);

	InnerModelNode *n2 = getNode<InnerModelNode>(b);
	QMat r2q = getRotationMatrixTo("root", b);
	fcl::Matrix3f R2( r2q(0,0), r2q(0,1), r2q(0,2), r2q(1,0), r2q(1,1), r2q(1,2), r2q(2,0), r2q(2,1), r2q(2,2) );
	QVec t2v = getTranslationVectorTo("root", b);
	fcl::Vec3f T2( t2v(0), t2v(1), t2v(2) );
	n2->getCollisionObject()->setTransform(R2, T2);

	fcl::DistanceResult result;
	fcl::DistanceRequest request;

	n1->getCollisionObject()->computeAABB();
	n2->getCollisionObject()->computeAABB();

	fcl::distance(n1->getCollisionObject(), n2->getCollisionObject(), request, result);
	
	n1->unlock(); n2->unlock();
	return result.min_distance;
#else
	throw InnerModelException("InnerModel was not compiled with collision support");
#endif
}


/////////////////////////////////////////////////////////////////////////////////////////////
/// Jacobian
/////////////////////////////////////////////////////////////////////////////////////////////

QMat InnerModel::jacobian(QStringList &listaJoints, const QVec &motores, const QString &endEffector)
{
	// La lista de motores define una secuencia contigua de joints, desde la base hasta el extremo.
	// Inicializamos las filas del Jacobiano al tamaño del punto objetivo que tiene 6 ELEMENTOS [tx, ty, tz, rx, ry, rz]
	// y las columnas al número de motores (Joints): 6 filas por n columnas. También inicializamos un vector de ceros

	QMat jacob(6, listaJoints.size(), 0.f);  //6 output variables
	QVec zero = QVec::zeros(3);
	int j=0; //índice de columnas de la matriz: MOTORES

	foreach(QString linkName, listaJoints)
	{
		if(motores[j] == 0)
		{
			QString frameBase = listaJoints.last();

			// TRASLACIONES: con respecto al último NO traslada
			InnerModelJoint *n = getNode<InnerModelJoint>(linkName); 
			QVec axisTip = n->unitaryAxis(); //vector de ejes unitarios
			n->unlock();
			
			axisTip = transform(frameBase, axisTip, linkName);
			QVec axisBase = transform(frameBase, zero, linkName);
			QVec axis = axisBase - axisTip;
			QVec toEffector = (axisBase - transform(frameBase, zero, endEffector) );
			QVec res = toEffector.crossProduct(axis);

			jacob(0,j) = res.x();
			jacob(1,j) = res.y();
			jacob(2,j) = res.z();

			// ROTACIONES
			InnerModelJoint *n2 = getNode<InnerModelJoint>(linkName); 
			QVec axisTip2 = n2->unitaryAxis(); //vector de ejes unitarios
			n2->unlock();
			axisTip2 = transform(frameBase, axisTip2, linkName); 	//vector de giro pasado al hombro.
			QVec axisBase2 = transform(frameBase, zero, linkName); 	//motor al hombro
			QVec axis2 = axisBase2 - axisTip2; 						//vector desde el eje de giro en el sist. hombro, hasta la punta del eje de giro en el sist. hombro.

			jacob(3,j) = axis2.x();
			jacob(4,j) = axis2.y();
			jacob(5,j) = axis2.z();
		}
		j++;
	}
	return jacob;
}


/*
void InnerModel::updateTransformValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId)
{
	cleanupTables();
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform *>(hash[transformId]);
	if (aux != NULL)
	{
		if (parentId != "")
		{
			InnerModelTransform *auxParent = dynamic_cast<InnerModelTransform *>(hash[parentId]);
			if (auxParent!=NULL)
			{
				RTMat Tbi;
				Tbi.setTr(tx,ty,tz);
				Tbi.setR (rx,ry,rz);

				///Tbp Inverse = Tpb. This gets Tpb directly. It's the same
				RTMat Tpb = getTransformationMatrix ( getNode ( transformId)->parent->id,parentId );
				///New Tpi
				RTMat Tpi = Tpb*Tbi;

				QVec angles = Tpi.extractAnglesR();
				QVec tr = Tpi.getTr();

				rx = angles.x();
				ry = angles.y();
				rz = angles.z();
				tx = tr.x();
				ty = tr.y();
				tz = tr.z();
			}
			else if (hash[parentId] == NULL)
			{
				qDebug() << __FUNCTION__ << "There is no such" << parentId << "node";
			}
		}
		//always update
		aux->update(tx,ty,tz,rx,ry,rz);
	}
	else if (hash[transformId] == NULL)
	{
		qDebug() << __FUNCTION__ << "There is no such" << transformId << "node";
	}
}*/
