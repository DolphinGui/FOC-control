#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <typeindex>
#include <vector>

struct Broadcaster
{
  struct Msg;
  using MsgPtr = std::shared_ptr<Msg>;
  using Listener = std::move_only_function<void(MsgPtr)>;
  using WeakListener = std::weak_ptr<Listener>;
  using SharedListener = std::shared_ptr<Listener>;

  void subscribe(SharedListener f)
  {
    listeners.emplace_back(f);
  }

  template<typename T>
  void emit(T&& item)
  {
    auto shared = std::make_shared<Msg>(std::move(item));
    for (auto it = listeners.begin(); it < listeners.end(); ++it) {
      auto funct = it->lock();
      if (!funct) {
        it = listeners.erase(it);
        continue;
      }
      std::invoke(*funct, shared);
    }
  }

private:
  std::vector<WeakListener> listeners;
};

struct Broadcaster::Msg
{
  using Dtor = void (*)(void*);
  template<typename T>
  static void destroy(void* ptr)
  {
    std::destroy_at<T>(static_cast<T*>(ptr));
  }
  template<typename T>
  Msg(T&& data)
    : type(std::type_index(typeid(T)))
    , destructor(&destroy<T>)
  {
    if constexpr (alignof(T) <= 8 && sizeof(T) < sizeof(buffer)) {
      new (buffer) T(std::move(data));
      is_inplace = true;
    } else {
      auto ptr = new T(std::move(data));
      new (buffer) void*(ptr);
      is_inplace = false;
    }
  }
  Msg(Msg const&) = delete;
  Msg(Msg&&) = delete;

  // may return nullptr if type does not match
  template<typename T>
  T* get_data()
  {
    if (type == typeid(T)) {
      return reinterpret_cast<T*>(buffer);
    }
    return nullptr;
  }

  template<typename T>
  T const* get_data() const
  {
    if (type == typeid(T)) {
      return reinterpret_cast<T*>(buffer);
    }
    return nullptr;
  }

  ~Msg()
  {
    if (is_inplace)
      destructor(buffer);
    else {
      auto ptr = *((void**)buffer);
      destructor(ptr);
    }
  }

private:
  alignas(8) uint8_t buffer[104];
  std::type_index type;
  Dtor destructor;
  bool is_inplace;
};
