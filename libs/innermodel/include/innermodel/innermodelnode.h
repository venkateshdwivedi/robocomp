/*
 * Copyright 2016 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef INNERMODELNODE_H
#define INNERMODELNODE_H

// RoboComp includes
#include <qmat/QMatAll>
#include <innermodel/innermodelconfig.h>
//#include <innermodel/threadsafehash.h>
#include <mutex>
#include <innermodel/safe_ptr.h>

#if FCL_SUPPORT==1
	#include <boost/shared_ptr.hpp>
	#include <fcl/collision.h>
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

class InnerModel;
	
typedef std::lock_guard<std::recursive_mutex> Lock;

class InnerModelNode : public RTMat
{
	public:
		using THash = sf::safe_ptr< QHash<QString, std::shared_ptr<InnerModelNode>>>;
		using NodePtr = std::shared_ptr<InnerModelNode>;
	
		InnerModelNode(QString id_, NodePtr parent_= nullptr);
		virtual ~InnerModelNode();
	
		struct AttributeType
		{
			QString type;
			QString value;
		};
		
		void treePrint(QString s, bool verbose=false);
		virtual void print(bool verbose) = 0;
		virtual NodePtr copyNode(THash hash, NodePtr parent) = 0;
		virtual void save(QTextStream &out, int tabs) = 0;
		void setParent(NodePtr parent_);
		void addChild(NodePtr child);

		/////////////////////////////////////////
		/// Thread safe API for InnerModel nodes
		/////////////////////////////////////////
		mutable std::recursive_mutex mutex;
		
		QString getId() const
		{
			Lock lock(mutex);
			return id;
		};
		void setId(QString id_)
		{
			Lock lock(mutex);
			id = id_;
		};
		void lock()
		{
			mutex.lock();
		}
		void unlock()
		{
			mutex.unlock();
		}
		QMat getRTS()
		{
			Lock lock(mutex);
			return InnerModelNode::getR();
		}
		QVec getTrTS()
		{
			Lock lock(mutex);
			return InnerModelNode::getTr();
		}
		RTMat matrixmultTS(const RTMat &rt)
		{
			Lock lock(mutex);
			return this->operator*(rt);
		}
		RTMat invertTS()
		{
			Lock lock(mutex);
			return InnerModelNode::invert();
		}
		QMat transposeTS()
		{
			Lock lock(mutex);
			return InnerModelNode::transpose();
		}
		int getLevel() const
		{
			Lock lock(mutex);
			return level;
		}
		void setLevel(int l)
		{
			Lock lock(mutex);
			level = l;
		}
		bool isFixed() const
		{
			Lock lock(mutex);
			return fixed;
		}
		void setFixed(bool v)
		{
			Lock lock(mutex);
			fixed = v;
		}
		QHash<QString, AttributeType> getAttributes() const
		{
			Lock lock(mutex);
			return attributes;
		}
		void setAttributes(const QHash<QString, AttributeType> &att)
		{
			Lock lock(mutex);
			attributes = att;
		}
		bool getCollidable() const
		{
			Lock lock(mutex);
			return collidable;
		}
		fcl::CollisionObject* getCollisionObject()
		{
			Lock lock(mutex);
			return collisionObject;
		}
		void markForDelete()
		{
			markedForDelete = true;
		}
		
		sf::safe_ptr<QList<NodePtr>> children;
		NodePtr parent;
		InnerModel *innerModel;
			
		protected:
			QString id;
			int level;
			bool fixed;
			QHash<QString, AttributeType> attributes;
			std::atomic<bool> markedForDelete;
			
			//////////////////////
			// FCLModel
			//////////////////////
			bool collidable;
			#if FCL_SUPPORT==1
				FCLModelPtr fclMesh;
				fcl::CollisionObject *collisionObject;
			#endif
};

#endif // INNERMODELNODE_H
