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
		void setRoot(const TransformPtr &node);
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
			auto t = std::make_tuple(params...);
			QString id = QString(std::get<0>(t));
			if(hash->contains(id))
				throw InnerModelException("InnerModel::newNode Error: Cannot insert new node with already existing name" + id.toStdString());
			
			std::shared_ptr<T> node(new T(std::forward<Ts>(params)...)); 
			hash->insert(id, std::static_pointer_cast<InnerModelNode>(node));
			return node;
		}
		
		QList<QString> getIDKeys() 	{ return hash->keys(); }
		
		template <typename N> 
		std::shared_ptr<N> getNode(const QString &id) 
		{
			return std::dynamic_pointer_cast<N>(hash->value(id));
		}
		

		template <typename N>
		std::shared_ptr<N> getNodeProxy(const QString &id)
		{
			if( n = hash->value(id))
			{
				new N(n);
				
			}
			
		}
		
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
		sf::safe_ptr< QHash<QPair<QString, QString>, QMat> > localHashRot;
		sf::safe_ptr< QHash<QPair<QString, QString>, RTMat> > localHashTr;
		std::pair<QList<NodePtr>, QList<NodePtr>> setLocalLists(const QString & origId, const QString & destId);		
		
		void removeNode(const QString & id);
};

#endif
