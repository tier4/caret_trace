#include <unordered_map>

template<typename KeyT, typename VarT>
class VirtualMemberVariable
{
public:
  bool has(const KeyT &key) const
  {
    return _var.count(key) > 0;
  }

  VarT & get(const KeyT &key)
  {
    return _var[key];
  }
  void set(const KeyT &key, VarT var)
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
