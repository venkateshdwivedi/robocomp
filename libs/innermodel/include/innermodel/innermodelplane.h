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

#ifndef INNERMODELPLANE_H
#define INNERMODELPLANE_H

#include <innermodel/innermodelnode.h>
#include <osg/TriangleFunctor>
#include <osg/io_utils>
#include <osg/Geode>
#include <osg/MatrixTransform>

class InnerModelPlane : public InnerModelNode
{
	public:
		InnerModelPlane(QString id_, QString texture_, float width_, float height_,float depth_, int repeat_, float nx_, float ny_, float nz_, float px_, float py_, float pz_, bool collidable, InnerModelNode *parent_=NULL);
		void print(bool verbose);
		void save(QTextStream &out, int tabs);
		//void setUpdatePointers(float *nx_, float *ny_, float *nz_, float *px_, float *py_, float *pz_);
		void update();
		void update(float nx_, float ny_, float nz_, float px_, float py_, float pz_);
		virtual InnerModelNode *copyNode(ThreadSafeHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);
		QVec getNormal() 					{ Lock lock(mutex); return normal;};
		QVec getPoint() 					{ Lock lock(mutex); return point;};
		float getWidth() 					{ Lock lock(mutex); return width;};
		float getHeight() 					{ Lock lock(mutex); return height;};
		float getDepth() 					{ Lock lock(mutex); return depth;};
		int getRepeat() 					{ Lock lock(mutex); return repeat;};
		QString getTexture()				{ Lock lock(mutex); return texture;};
		void setTexture( const QString &t)  { Lock lock(mutex); texture = t;};
		
	private:
		QVec normal, point;
		QString texture;
		float width, height, depth;
		int repeat;
		//float *nx, *ny, *nz;
		//float *px, *py, *pz;
};

#endif // INNERMODELPLANE_H
