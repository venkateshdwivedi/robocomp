#ifndef INNERMODEL_H
#define INNERMODEL_H

// System includes
#include <stdexcept>
#include <stdint.h>
#include <typeinfo>
#include <type_traits>

#include <innermodel/safe_ptr.h>

// Qt includes
#include <QHash>
#include <mutex>

// RoboComp includes
#include <qmat/QMatAll>

//Derived and auxiliary classes
#include <innermodel/innermodelconfig.h>
#include <innermodel/innermodelexception.h>
#include <innermodel/innermodeltransform.h>
#include <innermodel/innermodelnode.h>
#include <innermodel/innermodeljoint.h>
#include <innermodel/innermodeltouchsensor.h>
#include <innermodel/innermodeldifferentialrobot.h>
#include <innermodel/innermodelomnirobot.h>
#include <innermodel/innermodelprismaticjoint.h>
#include <innermodel/innermodelplane.h>
#include <innermodel/innermodelcamera.h>
#include <innermodel/innermodelrgbd.h>
#include <innermodel/innermodellaser.h>
#include <innermodel/innermodelmesh.h>
#include <innermodel/innermodelimu.h>
#include <innermodel/innermodelpointcloud.h>
#include <innermodel/innermodeltouchsensor.h>
//#include <innermodel/threadsafehash.h>

#if FCL_SUPPORT==1
#include <boost/shared_ptr.hpp>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/ccd/motion.h>
#include <fcl/BV/BV.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/traversal/traversal_node_bvh_shape.h>
#include <fcl/traversal/traversal_node_bvhs.h>
typedef fcl::BVHModel<fcl::OBBRSS> FCLModel;
typedef boost::shared_ptr<FCLModel> FCLModelPtr;
#endif

#ifdef PYTHON_BINDINGS_SUPPORT
#include <boost/python/stl_iterator.hpp>
#endif

typedef std::lock_guard<std::recursive_mutex> Lock;
//typedef sf::safe_ptr< std::map<std::string, InnerModelNode *> > THash;

using namespace RMat;

class InnerModelReader;

class InnerModel
{
	friend InnerModelReader;

	public:
		using NodePtr = std::shared_ptr<InnerModelNode>;
		using TransformPtr = std::shared_ptr<InnerModelTransform>;
		using JointPtr = std::shared_ptr<InnerModelJoint>;
		using TouchSensorPtr = std::shared_ptr<InnerModelTouchSensor>;
		using PrismaticJointPtr = std::shared_ptr<InnerModelPrismaticJoint>;
		using DifferentialRobotPtr = std::shared_ptr<InnerModelDifferentialRobot>;
		using OmniRobotPtr = std::shared_ptr<InnerModelOmniRobot>;
		using CameraPtr = std::shared_ptr<InnerModelCamera>;
		using RGBDPtr = std::shared_ptr<InnerModelRGBD>;
		using IMUPtr = std::shared_ptr<InnerModelIMU>;
		using LaserPtr = std::shared_ptr<InnerModelLaser>;
		using PlanePtr = std::shared_ptr<InnerModelPlane>;
		using MeshPtr = std::shared_ptr<InnerModelMesh>;
		using PointCloudPtr = std::shared_ptr<InnerModelPointCloud>;
		
		static bool support_fcl();
		/////////////////////////
		/// (Con/De)structors
		/////////////////////////
		InnerModel();
		InnerModel(std::string xmlFilePath);
		InnerModel(const InnerModel &original);
		InnerModel(InnerModel &original);
		InnerModel(InnerModel *original);
		~InnerModel();

		/////////////////////////////////
		//// Auxiliary methods
		/////////////////////////////////
		bool open(std::string xmlFilePath);
		bool save(QString path);
		InnerModel* copy();
		void print(QString s="")							 { treePrint(s, true); }
		void treePrint(QString s="", bool verbose=false)	 { root->treePrint(QString(s), verbose); }
		//Thread safe
		QString getParentIdentifier(QString id);
		std::string getParentIdentifierS(const std::string &id);

		///////////////////////
		/// Tree update methods
		///////////////////////
		void setRoot(TransformPtr node);
		void update();
		void cleanupTables();
		void updateTransformValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId="");
		void updateTransformValues(QString transformId, QVec v, QString parentId="");
		void updateTransformValuesS(std::string transformId, float tx, float ty, float tz, float rx, float ry, float rz, std::string parentId="");
		void updateTransformValuesS(std::string transformId, QVec v, std::string parentId="");
		void updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId="");
		void updateRotationValues(QString transformId, float rx, float ry, float rz,QString parentId="");
		void updateJointValue(QString jointId, float angle, bool force=false);
		void updatePrismaticJointPosition(QString jointId, float position);
		void updatePlaneValues(QString planeId, float nx, float ny, float nz, float px, float py, float pz);

		////////////////////////////////
		/// Factory constructors
		///////////////////////////////

		template<typename T, typename... Ts>
		std::shared_ptr<T> newNode(Ts&&... params)
		{
			std::shared_ptr<InnerModelNode> node(nullptr);
			if(std::is_same<T, InnerModelTransform>::value)
			{	node.reset(new InnerModelTransform(std::forward<Ts>(params)...)); }
			if(std::is_same<T, InnerModelJoint>::value)
			{ 	node.reset(new InnerModelJoint(std::forward<Ts>(params)...)); }
			if(std::is_same<T, InnerModelTouchSensor>::value)
			{ 	node.reset(new InnerModelTouchSensor(std::forward<Ts>(params)...)); }
			if(std::is_same<T, InnerModelPrismaticJoint>::value)
			{ 	node.reset(new InnerModelPrismaticJoint(std::forward<Ts>(params)...)); }
			if(std::is_same<T, InnerModelDifferentialRobot>::value)
			{ 	node.reset(new InnerModelDifferentialRobot(std::forward<Ts>(params)...));} 
			if(std::is_same<T, InnerModelOmniRobot>::value)
			{ 	node.reset(new InnerModelOmniRobot(std::forward<Ts>(params)...)); }
			if(std::is_same<T, InnerModelCamera>::value)
			{ 	node.reset(new InnerModelCamera(std::forward<Ts>(params)...));} 
			if(std::is_same<T, InnerModelIMU>::value)
			{ 	node.reset(new InnerModelIMU(std::forward<Ts>(params)...)); }
			if(std::is_same<T, InnerModelRGBD>::value)
			{ 	node.reset(new InnerModelRGBD(std::forward<Ts>(params)...)); }
			if(std::is_same<T, InnerModelLaser>::value)
			{ 	node.reset(new InnerModelLaser(std::forward<Ts>(params)...)); }
			if(std::is_same<T, InnerModelPlane>::value)
			{ 	node.reset(new InnerModelPlane(std::forward<Ts>(params)...)); }
			if(std::is_same<T, InnerModelMesh>::value)
			{ 	node.reset(new InnerModelMesh(std::forward<Ts>(params)...)); }
			if(std::is_same<T, InnerModelPointCloud>::value)
			{ 	node.reset(new InnerModelPointCloud(std::forward<Ts>(params)...)); }
			return node;
		}
		
// 		InnerModelTransform* newTransform(QString id, QString engine, InnerModelNode *parent, float tx=0, float ty=0, float tz=0, float rx=0, float ry=0, float rz=0, float mass=0);
// 		InnerModelJoint* newJoint(QString id, InnerModelTransform* parent, float lx = 0, float ly = 0, float lz = 0, float hx = 0, float hy = 0, float hz = 0,  float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, float min=-INFINITY, float max=INFINITY, uint32_t port = 0, std::string axis = "z", float home=0);
// 		InnerModelTouchSensor* newTouchSensor(QString id, InnerModelTransform* parent, QString type, float nx = 0, float ny = 0, float nz = 0, float min=0, float max=INFINITY, uint32_t port=0);
// 		InnerModelPrismaticJoint* newPrismaticJoint(QString id, InnerModelTransform* parent, float min=-INFINITY, float max=INFINITY, float value=0, float offset=0, uint32_t port = 0, std::string axis = "z", float home=0);
// 		InnerModelDifferentialRobot* newDifferentialRobot(QString id, InnerModelTransform* parent, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, uint32_t port = 0, float noise=0., bool collide=false);
// 		InnerModelOmniRobot* newOmniRobot(QString id, InnerModelTransform* parent, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, uint32_t port = 0, float noise=0., bool collide=false);
// 		InnerModelCamera* newCamera(QString id, InnerModelNode *parent, float width, float height, float focal);
// 		InnerModelRGBD* newRGBD(QString id, InnerModelNode *parent, float width, float height, float focal, float noise, uint32_t port = 0, QString ifconfig="");
// 		InnerModelIMU* newIMU(QString id, InnerModelNode *parent, uint32_t port = 0);
// 		InnerModelLaser* newLaser(QString id, InnerModelNode *parent, uint32_t port = 0, uint32_t min=0, uint32_t max=30000, float angle = M_PIl, uint32_t measures = 360, QString ifconfig="");
// 		InnerModelPlane* newPlane(QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx=0, float ny=0, float nz=0, float px=0, float py=0, float pz=0, bool collidable=0);
// 		InnerModelMesh* newMesh(QString id, InnerModelNode *parent, QString path, float scale, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable=0);
// 		InnerModelMesh* newMesh(QString id, InnerModelNode *parent, QString path, float scalex, float scaley, float scalez, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable=0);
// 		InnerModelPointCloud* newPointCloud(QString id, InnerModelNode *parent);

		////////////////////////////////
		/// NOT thread safe Accessors. Use template class  getNodeSafeAndLock() below
		///////////////////////////////
// 		TransformPtr getTransform(const QString &id)                 { return getNode<TransformPtr>(id); }
// 		JointPtr getJoint(const QString &id)                         { return getNode<JointPtr>(id); }
// 		JointPtr getJoint(const std::string &id)                     { return getNode<JointPtr>(QString::fromStdString(id)); }
// 		JointPtr getJointS(const std::string &id)                    { return getNode<JointPtr>(QString::fromStdString(id)); }
// 		//OJO
// 		//JointPtr &getJointRef(const std::string &id)               { return *getNode<InnerModelJoint>(QString::fromStdString(id)); }
// 		TouchSensorPtr getTouchSensor(const QString &id)             { return getNode<TouchSensorPtr>(id); }
// 		PrismaticJointPtr getPrismaticJoint(const QString &id)       { return getNode<PrismaticJointPtr>(id); }
// 		DifferentialRobotPtr getDifferentialRobot(const QString &id) { return getNode<DifferentialRobotPtr>(id); }
// 		OmniRobotPtr getOmniRobot(const QString &id)                 { return getNode<OmniRobotPtr>(id); }
// 		CameraPtr getCamera(QString id)                              { return getNode<CameraPtr>(id); }
// 		RGBDPtr getRGBD(QString id)                                  { return getNode<RGBDPtr>(id); }
// 		IMUPtr getIMU(QString id)                                    { return getNode<IMUPtr>(id); }
// 		LaserPtr getLaser(QString id)                                { return getNode<LaserPtr>(id); }
// 		PlanePtr getPlane(const QString &id)                         { return getNode<PlanePtr>(id); }
// 		MeshPtr getMesh(const QString &id)                           { return getNode<MeshPtr>(id); }
// 		//PointCloud *getPointCloud(const QString &id)               { return getNode<InnerModelPointCloud>(id); }

		QList<QString> getIDKeys() 									 { return hash->keys(); }
		//NodePtr getNode(const QString & id) 	 					 { return hash->value(id); }
		
		template <typename N> 
		std::shared_ptr<N> getNode(const QString &id) 
		{
			N* r = dynamic_cast<N*>(hash->value(id).get());
			return std::shared_ptr<N>(r);
		}

		// Thread safe node getter. It might be null
// 		template <class N> N* getNodeSafe(const QString &id) 
// 		{
// 			return dynamic_cast<N *>(hash->value(id));
// 		}
// 		// Thread safe node getter that returns a locked node. 
// 		template <class N> N* getNodeSafeAndLock(const QString &id) 
// 		{
// 			N* r = dynamic_cast<N *>(hash->value(id));
// // 			if (r == nullptr)
// // 				throw InnerModelException("InnerModel::getNodeSafeAndLock() Error getting non existing node: " + id.toStdString());
// 			return r;	
// 		}
		
		////////////////////////////////////////////////////////////////////////
		/// Thread safe kinematic transformation methods
		/// Transformation is guaranteed to occur with existing, stable nodes but the final result might correspond to nodes no longer existing in the tree
		////////////////////////////////////////////////////////////////////////
		QVec transform(  const QString & destId, const QVec &origVec, const QString & origId);
		QVec transform(  const QString &destId, const QString & origId) 								{ return transform(destId, QVec::vec3(0,0,0), origId); };
		QVec transformS( const std::string &destId, const QVec &origVec, const std::string & origId);
		QVec transformS( const std::string &destId, const std::string &origId)							{ return transform(QString::fromStdString(destId), QVec::vec3(0,0,0), QString::fromStdString(origId)); }
		QVec transform6D(const QString &destId, const QVec &origVec, const QString & origId)			{ return transform(destId, origVec, origId); }
		QVec transform6D(const QString &destId, const QString & origId) 								{ return transform(destId, QVec::vec6(0,0,0,0,0,0), origId); }
		QVec transformS6D(const std::string &destId, const std::string & origId) 						{ return transform(QString::fromStdString(destId), QVec::vec6(0,0,0,0,0,0), QString::fromStdString(origId)); }
		QVec transformS6D(const std::string &destId, const QVec &origVec, const std::string & origId)	{ return transform(QString::fromStdString(destId), origVec, QString::fromStdString(origId)); }
		
		////////////////////////////////////////////
		/// Thread safe transformation matrix retrieval methods
		/// Transformation is guaranteed to occur with existing, stable nodes but the final result might correspond to nodes no longer existing in the tree
		///////////////////////////////////////////
		RTMat getTransformationMatrix(const QString &destId, const QString &origId);
		RTMat getTransformationMatrixS(const std::string &destId, const std::string &origId);
		QMat getRotationMatrixTo(const QString &to, const QString &from);
		QVec getTranslationVectorTo(const QString &to, const QString &from);
		QVec rotationAngles(const QString & destId, const QString & origId);

		/////////////////////////////////////////////
		/// Graoh editing methods
		/////////////////////////////////////////////
		void removeSubTree(NodePtr item, QStringList *l);
		void markSubTreeForRemoval(NodePtr node);
		void moveSubTree(NodePtr nodeSrc, NodePtr nodeDst);
		void getSubTree(NodePtr node, QStringList *l);
		void getSubTree(NodePtr node, QList<NodePtr> *l);
		void computeLevels(NodePtr node);
		NodePtr getRoot() { return root; }
	
		/////////////////////
		/// Set debug level
		/////////////////////
		int debugLevel(int level=-1) { static int debug_level=0; if (level>-1) debug_level=level; return debug_level; }

		////////////////////////////
		// Thread safe FCL related
		////////////////////////////
		bool collidable(const QString &a);
		bool collide(const QString &a, const QString &b);
		float distance(const QString &a, const QString &b);

	#if FCL_SUPPORT==1
		bool collide(const QString &a, const fcl::CollisionObject *obj);
	#endif

		////////////////////////////
		// Jacobian computation
		////////////////////////////
		/**
			* @brief Computes the jacobian of a list of joints at a given configuration point given by motores
			*
			* @param listaJoints list of names of joints in InnerModel that conform the open kinematic chain
			* @param motores value of motro joints where the jacobian will be evaluated
			* @param endEffector name of end effector of the kin. chain. It can be an element further away than the last in listaJoint.
			* @return RMat::QMat Jacobian as MxN matrix of evaluated partial derivatives. M=joints, N=6 (pose cartesian coordinates of the endEffector) (CHECK ORDER)
			*/
		QMat jacobian(QStringList &listaJoints, const QVec &motores, const QString &endEffector);
		QMat jacobianS(std::vector<std::string> &listaJoints, const QVec &motores, const std::string &endEffector)
		{
			QStringList listaJointQ/* = QStringList::fromStdList(listaJoints)*/;
			for (auto e : listaJoints)
			{
				listaJointQ.push_back(QString::fromStdString(e));
			}
			return jacobian(listaJointQ, motores, QString::fromStdString(endEffector));
		}

		#ifdef PYTHON_BINDINGS_SUPPORT
			QMat jacobianSPython(const  boost::python::list &listaJointsP, const QVec &motores, const std::string &endEffector)
			{
				std::vector<std::string> listaJoint = std::vector<std::string>(boost::python::stl_input_iterator<std::string>(listaJointsP), boost::python::stl_input_iterator<std::string>( ) );
				return jacobianS(listaJoint, motores, endEffector);
			}
		#endif

		//QMutex *mutex;
		mutable std::recursive_mutex mutex;
	
		//Thread safe hash
		using THash = sf::safe_ptr< QHash<QString, std::shared_ptr<InnerModelNode>>>;
		THash hash; 
		
	protected:
		NodePtr root;
		//ThreadSafeHash<QPair<QString, QString>, RTMat> localHashTr;
		//ThreadSafeHash<QPair<QString, QString>, QMat> localHashRot;
		sf::safe_ptr< QHash<QPair<QString, QString>, QMat> > localHashRot;
		sf::safe_ptr< QHash<QPair<QString, QString>, RTMat> > localHashTr;
		std::pair<QList<InnerModelNode *>, QList<InnerModelNode *>> setLocalLists(const QString & origId, const QString & destId);		
		
		void removeNode(const QString & id);
};

#endif
