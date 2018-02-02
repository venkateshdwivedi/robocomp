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

#ifndef INNERMODELLASER_H
#define INNERMODELLASER_H

#include <innermodel/innermodelnode.h>

class InnerModel;

class InnerModelLaser : public InnerModelNode
{
	public:
		InnerModelLaser(QString id_, uint32_t _port, uint32_t _min, uint32_t _max, float _angle, uint32_t _measures, QString _ifconfig, InnerModel *innermodel_, NodePtr parent_ = nullptr);
		void save(QTextStream &out, int tabs);
		void print(bool verbose);
		void update();
		virtual NodePtr copyNode(THash hash, NodePtr parent);
		
		/**
		* \brief Local laser measure of range r and angle alfa is converted to Any RS
		* @param r range measure
		* @param alfa angle measure
		* @return 3-vector of x,y,z coordinates un WRS
		*/
		QVec laserTo(const QString& dest, float r, float alpha);
		uint32_t getPort() const		{ Lock lock(mutex); return port; }
		uint32_t getMeasures() const	{ Lock lock(mutex); return measures; }
		float getAngle() const			{ Lock lock(mutex); return angle; }
		uint32_t getMin() const			{ Lock lock(mutex); return min; }
		uint32_t getMax() const			{ Lock lock(mutex); return max; }
		QString getIfconfig() const		{ Lock lock(mutex); return ifconfig; }
		
	private:
		uint32_t port;
		uint32_t min, max;
		float angle;
		uint32_t measures;
		QString ifconfig;
		
		//mutable QMutex mutex;
	
private:
		InnerModel *innermodel;
};


#endif // INNERMODELLASER_H
