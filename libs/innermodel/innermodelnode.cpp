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

#include "innermodelnode.h"

InnerModelNode::InnerModelNode(QString id_, NodePtr parent_) : RTMat()
{
	collidable = false;
	#if FCL_SUPPORT==1
		collisionObject = NULL; 
	#endif

	fixed = true;
	parent = parent_;
	if (parent)
		level = parent->level+1;
	else
		level = 0;
	id = id_;
	attributes.clear();
	markedForDelete = false;
}

InnerModelNode::~InnerModelNode()
{
	#if FCL_SUPPORT==1
		if (collisionObject!=NULL)
		{
			delete collisionObject;
		}
		fclMesh.reset();
	#endif
}

void InnerModelNode::treePrint(QString s, bool verbose)
{
	printf("%s%s l(%d) [%d]\n", qPrintable(s), qPrintable(id), level, children->size());
 	QList<NodePtr>::iterator i;
 	for (i=children->begin(); i!=children->end(); i++)
 	{
 		if (verbose)
 			(*i)->print(verbose);
 		(*i)->treePrint(s+QString("  "), verbose); 
 	}
}

void InnerModelNode::setParent(NodePtr parent_)
{
	parent = parent_;
	level = parent->level + 1;
}

void InnerModelNode::addChild(NodePtr child)
{
	child->innerModel = innerModel;
	
	if (not children->contains(child))
	{
		children->append(child);
	}
	//child->parent.reset(this);  //MIRAR
}



