/*
 * Copyright 2018 <copyright holder> <email>
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

#ifndef THREADSAFEHASH_H
#define THREADSAFEHASH_H

#include <mutex>

class InnerModelNode;
typedef std::lock_guard<std::recursive_mutex> Lock;

template <typename Val>
class ThreadSafeList : QList<Val>
{
	public:
		ThreadSafeList()
		{}
 		ThreadSafeList(const ThreadSafeList<Val> &list_) : QList<Val>(list_)
 		{
 		};
		void clear()
		{
			Lock lock(mutex);
			this->clear();
		};
		void push_back(const Val &v)
		{
			Lock lock(mutex);
			this->push_back(v);
		};
		void push_front(const Val &v)
		{
			Lock lock(mutex);
			this->push_front(v);
		};
		Val operator[](int i)
		{
			Lock lock(mutex);
			return this->operator[](i);
		};
		bool size()
		{
			Lock lock(mutex);
			return this->size();
		};
	private:
		std::recursive_mutex mutex;
		//QList<Val> list;
};

template <typename Key, typename Val>
class ThreadSafeHash
{
	public:
		ThreadSafeHash(){};
		std::recursive_mutex mutex;
		void put(const Key &key, Val v)
		{
			Lock lock(mutex);
			hash[key] = v;
		}
		Val get(const Key &key)
		{
			Lock lock(mutex);
			return hash[key];
		}
		void clear()
		{
			Lock lock(mutex);
			hash.clear();
		}
		QList<Key> keys()
		{
			Lock lock(mutex);
			return hash.keys();
		}
		bool contains(const Key &key) 
		{
			Lock lock(mutex);
			return hash.contains(key);
		};
		std::pair<bool, Val> checkandget(const Key &key)
		{
			Lock lock(mutex);
			if( hash.contains(key) )
				return std::make_pair(true, hash[key]);
			else 
				return std::make_pair(false, Val());
		};
		Val checkandgetandlock(const Key &key)
		{
			Lock lock(mutex);
			if(hash.contains(key))
			{
				Val v = hash[key];
				if(dynamic_cast<InnerModelNode *>(v) != nullptr)
					v->lock();
				return v;
			}
			else 
				return nullptr;			
		}
		Val operator[](const Key &key)
		{
			Lock lock(mutex);
			return hash[key];
		}
		void remove(const Key &key)
		{
			Lock lock(mutex);
			hash.remove(key);
		}
		void lock()
		{
			mutex.lock();
		}
		void unlock()
		{
			mutex.unlock();
		}
		
	private:
		
		QHash<Key, Val> hash;
};

#endif // THREADSAFEHASH_H
