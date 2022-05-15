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

#ifndef CARET_TRACE__VIRTUAL_MEMBER_VARIABLE_HPP_

#include <unordered_map>


template<typename KeyT, typename VarT>
class VirtualMemberVariable
{
public:
  bool has(const KeyT & key) const
  {
    return _var.count(key) > 0;
  }

  VarT & get(const KeyT & key)
  {
    return _var[key];
  }
  void set(const KeyT & key, VarT var)
  {
    _var[key] = var;
  }
  size_t size() const
  {
    return _var.size();
  }

private:
  std::unordered_map<KeyT, VarT> _var;
};

template<typename KeyT, typename VarT>
class VirtualMemberVariables
{
  using VarMapT = VirtualMemberVariable<KeyT, VarT>;

public:
  VarMapT & get_vars(const void * obj_addr)
  {
    if (_var.count(obj_addr) == 0) {
      _var[obj_addr] = VarMapT();
    }
    return _var[obj_addr];
  }

private:
  std::unordered_map<const void *, VarMapT> _var;
};

#endif  // CARET_TRACE__VIRTUAL_MEMBER_VARIABLE_HPP_
#define CARET_TRACE__VIRTUAL_MEMBER_VARIABLE_HPP_
