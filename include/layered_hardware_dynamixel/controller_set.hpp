#ifndef LAYERED_HARDWARE_DYNAMIXEL_CONTROLLER_SET_HPP
#define LAYERED_HARDWARE_DYNAMIXEL_CONTROLLER_SET_HPP

#include <set>
#include <string>

#include <hardware_interface/controller_info.h>
#include <layered_hardware_dynamixel/common_namespaces.hpp>

namespace layered_hardware_dynamixel {

class ControllerSet : public std::set<std::string> {
private:
  typedef std::set<std::string> Base;

public:
  std::pair<iterator, bool> insert(const hi::ControllerInfo &info) {
    return Base::insert(info.name);
  }

  std::size_t erase(const hi::ControllerInfo &info) { return Base::erase(info.name); }

  template <class Container> void update(const Container &start_list, const Container &stop_list) {
    for (const auto /* std::string or hi::ControllerInfo */ &info : start_list) {
      insert(info);
    }
    for (const auto &info : stop_list) {
      erase(info);
    }
  }

  template <class Container>
  ControllerSet updated(const Container &start_list, const Container &stop_list) const {
    ControllerSet res(*this);
    res.update(start_list, stop_list);
    return res;
  }

  bool contains(const std::string &name) const { return find(name) != end(); }

  bool contains(const hi::ControllerInfo &info) const { return contains(info.name); }

  template <class Container> bool contains(const Container &infos) const {
    for (const auto /* std::string or hi::ControllerInfo */ &info : infos) {
      if (!contains(info)) {
        return false;
      }
    }
    return true;
  }
};
} // namespace layered_hardware_dynamixel

#endif