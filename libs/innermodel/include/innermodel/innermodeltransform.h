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

#ifndef INNERMODELTRANSFORM_H
#define INNERMODELTRANSFORM_H

#include "innermodelnode.h"

		
class InnerModelTransform : public InnerModelNode
{
	public:
		InnerModelTransform(QString id_, QString engine_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float mass_, InnerModelNode *parent_=NULL);
		virtual ~InnerModelTransform();

		void print(bool verbose);
		void save(QTextStream &out, int tabs);
		//void update();
		void update(float tx_, float ty_, float tz_, float rx_, float ry_, float rz_);
		void updateT(float tx_, float ty_, float tz_);
		void updateR(float rx_, float ry_, float rz_);
		virtual InnerModelNode *copyNode(ThreadSafeHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);
		void transformValues(const RTMat &Tpb, float tx, float ty, float tz, float rx, float ry, float rz, const InnerModelNode *parentNode);
		void translateValues(const RTMat &Tpb, float tx, float ty, float tz, const InnerModelNode *parentNode);
		void rotateValues(const RTMat &Tpb, float rx, float ry, float rz, const InnerModelNode *parentNode);
		void setGuiTranslation(bool v)		{ Lock lock(mutex); gui_translation = v; };
		void setGuiRotation(bool v)			{ Lock lock(mutex); gui_rotation = v; };
		float getMass() const				{ Lock lock(mutex); return mass;}
		QString getEngine() const			{ Lock lock(mutex); return engine;}
		float getBacktX() const				{ Lock lock(mutex); return backtX;}
		float getBacktY() const				{ Lock lock(mutex); return backtY;}
		float getBacktZ() const				{ Lock lock(mutex); return backtZ;}
		float getBackrX() const				{ Lock lock(mutex); return backrX;}
		float getBackrY() const				{ Lock lock(mutex); return backrY;}
		float getBackrZ() const				{ Lock lock(mutex); return backrZ;}
		
	protected:
//  		float *tx, *ty, *tz;
//  		float *rx, *ry, *rz;
		float mass;
		float backtX, backtY, backtZ;
		float backrX, backrY, backrZ;
		bool gui_translation, gui_rotation;
		QString engine;
};
	

#endif // INNERMODELTRANSFORM_H
