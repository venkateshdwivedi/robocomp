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

typedef std::lock_guard<std::recursive_mutex> Lock;

template <typename Val>
class ThreadSafeList
{
	public:
		ThreadSafeList(){};
		void clear()
		{
			Lock lock(mutex);
			list.clear();
		};
		void push_back(const Val &v)
		{
			Lock lock(mutex);
			list.push_back(v);
		};
		void push_front(const Val &v)
		{
			Lock lock(mutex);
			list.push_front(v);
		};
		Val operator[](int i)
		{
			Lock lock(mutex);
			return list[i];
		};
		bool size()
		{
			Lock lock(mutex);
			return list.size();
		};
	private:
		std::recursive_mutex mutex;
		QList<Val> list;
};

template <typename Key, typename Val>
class ThreadSafeHash
{
	public:
		ThreadSafeHash(){};
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
	private:
		std::recursive_mutex mutex;
		QHash<Key, Val> hash;
};

#endif // THREADSAFEHASH_H
