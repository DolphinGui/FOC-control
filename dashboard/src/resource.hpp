#pragma once

#include <concepts>
#include <type_traits>
#include <utility>

template<typename T, typename D>
  requires(std::is_trivially_destructible_v<T> &&
           noexcept(std::declval<D>()(std::declval<T>())))
struct Resource
{
  T resource;
  D deleter;

  Resource(T t, D d)
    : resource(t)
    , deleter(d)
  {
  }

  explicit Resource()
    requires(std::default_initializable<T> && std::default_initializable<D>)
    : resource{}
    , deleter{}
  {
  }

  Resource(Resource const&) = delete;

  Resource(Resource&& other)
    requires(std::move_constructible<T> && !std::is_pointer_v<T>)
  {
    resource = std::move(other.resource);
  }
  Resource(Resource&& other)
    requires(std::is_pointer_v<T>)
  {
    resource = other.resource;
    other.resource = nullptr;
  }

  Resource& operator=(Resource other)
  {
    swap(*this, other);
    return *this;
  }

  friend void swap(Resource& a, Resource& b)
    requires(std::swappable<T>)
  {
    std::swap(a.resource, b.resource);
  }

  T& operator*()
  {
    return resource;
  }
  T const& operator*() const
  {
    return resource;
  }

  T* operator->()
  {
    return &resource;
  }
  T const* operator->() const
  {
    return &resource;
  }

  ~Resource()
  {
    deleter(resource);
  }
};
