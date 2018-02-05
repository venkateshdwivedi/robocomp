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

#ifndef INNERMODELTOUCHSENSOR_H
#define INNERMODELTOUCHSENSOR_H

#include <innermodel/innermodelnode.h>

class InnerModelTouchSensor :public InnerModelNode
{
	public:
		InnerModelTouchSensor(QString id_, QString stype, float nx_, float ny_, float nz_, float min_=-INFINITY, float max_=INFINITY, uint32_t port_=0, NodePtr parent_ = nullptr);
		void print(bool verbose) {}
		void save(QTextStream &out, int tabs) {}
		QVec getMeasure() { return value; }
		virtual NodePtr copyNode(THash hash, NodePtr parent);
		uint32_t getPort() const			{ Lock lock(mutex); return port;}
		float getNx()						{ Lock lock(mutex); return nx;}
		float getNy()						{ Lock lock(mutex); return ny;}
		float getNz()						{ Lock lock(mutex); return nz;}
		float getMin()						{ Lock lock(mutex); return min;}
		float getMax()						{ Lock lock(mutex); return max;}
		float getValue()					{ Lock lock(mutex); return value;}
		QString getStype()					{ Lock lock(mutex); return stype;}

	private:
		float nx, ny, nz;
		float min, max;
		float value;
		QString stype;
		uint32_t port;
};
#endif // INNERMODELTOUCHSENSOR_H
