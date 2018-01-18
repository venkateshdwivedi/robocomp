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
class ThreadSafeHash
{
	public:
		ThreadSafeHash(){};
		void put(const QString &key, Val v)
		{
			Lock lock(mutex);
			hash[key] = v;
		}
		Val get(const QString &key)
		{
			Lock lock(mutex);
			return hash[key];
		}
		void clear()
		{
			Lock lock(mutex);
			hash.clear();
		}
		QList<QString> keys()
		{
			Lock lock(mutex);
			return hash.keys();
		}
		bool contains(const QString &key) 
		{
			Lock lock(mutex);
			return hash.contains(key);
		}
		Val operator[](const QString &key)
		{
			Lock lock(mutex);
			return hash[key];
		}
	private:
		std::recursive_mutex mutex;
		QHash<QString, Val> hash;
};

#endif // THREADSAFEHASH_H
