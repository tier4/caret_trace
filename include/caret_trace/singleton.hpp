// Copyright 2021 Research Institute of Systems Planning, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CARET_TRACE__SINGLETON_HPP_

#include <cassert>
#include <mutex>

class SingletonFinalizer
{
public:
  using FinalizerFunc = void (*)();
  static void addFinalizer(FinalizerFunc func);
  static void finalize();
};

template<typename T>
class Singleton final
{
public:
  static T & get_instance()
  {
    std::call_once(initFlag, create);
    assert(instance);
    return *instance;
  }

private:
  static void create()
  {
    instance = new T;
    SingletonFinalizer::addFinalizer(&Singleton<T>::destroy);
  }

  static void destroy()
  {
    delete instance;
    instance = nullptr;
  }

  static std::once_flag initFlag;
  static T * instance;
};

template<typename T>
std::once_flag Singleton<T>::initFlag;

template<typename T>
T * Singleton<T>::instance = nullptr;

#endif  // CARET_TRACE__SINGLETON_HPP_
#define CARET_TRACE__SINGLETON_HPP_
