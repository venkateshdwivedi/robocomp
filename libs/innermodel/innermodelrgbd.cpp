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

#include "innermodelrgbd.h"
#include <innermodel/innermodel.h>

InnerModelRGBD::InnerModelRGBD(QString id_, float width, float height, float focal, float _noise, uint32_t _port, QString _ifconfig, InnerModel *innermodel_, InnerModelNode *parent_) 
: InnerModelCamera(id_, width, height, focal, innermodel_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	noise = _noise;
	port = _port;
	ifconfig = _ifconfig;
}



void InnerModelRGBD::save(QTextStream &out, int tabs)
{
	Lock lock(mutex);
	
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<rgbd id=\"" << id << "\" width=\"" <<QString::number( width, 'g', 10) << "\" height=\"" <<QString::number(  height, 'g', 10)  << "\" focal=\"" << QString::number(camera.getFocal(), 'g', 10)
	<<"\" port=\""<<port<<"\" ifconfig=\""<<ifconfig<<"\" noise=\""<<QString::number(noise, 'g', 10)<< "\" />\n";
}

InnerModelNode * InnerModelRGBD::copyNode(THash hash, InnerModelNode *parent)
{
	Lock lock(mutex);
	
	InnerModelRGBD *ret = new InnerModelRGBD(id, width, height, focal, noise, port, ifconfig, innermodel, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children->clear();
	ret->attributes.clear();
// 	hash->insert(id,ret);
// 
// 	for (QList<InnerModelNode*>::iterator i=children->begin(); i!=children->end(); i++)
// 	{
// 		ret->addChild((*i)->copyNode(hash, ret));
// 	}
	return ret;
}

