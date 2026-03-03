#pragma once

#include <chrono>
#include <concepts>
#include <coroutine>
#include <cstddef>
#include <exception>
#include <utility>

namespace co {
using Microseconds = std::chrono::duration<uint32_t, std::micro>;

template<typename Return>
struct Task;

template<typename T>
concept Pollable = requires(T t) {
  { t.poll() } -> std::convertible_to<Microseconds>;
};

template<typename T>
concept Evaluable = requires(T t) {
  { t.value() };
};

template<typename T>
concept Runnable = Pollable<T> && Evaluable<T>;

template<>
struct Task<void>
{
  virtual Microseconds poll() = 0;
  virtual ~Task() = default;
};

template<typename Return>
struct Task : Task<void>
{
  virtual Return value() = 0;
};

template<Pollable R>
struct TaskWrapper;
template<Runnable R>
struct TaskWrapper<R> : Task<decltype(std::declval<R>().value())>
{
  TaskWrapper(R r)
    : runnable(r)
  {
  }
  Microseconds poll() override
  {
    return runnable.poll();
  }
  decltype(std::declval<R>().value()) value() override
  {
    return runnable.value();
  }

private:
  R runnable;
};
template<Pollable R>
struct TaskWrapper : Task<void>
{
  TaskWrapper(R r)
    : runnable(std::move(r))
  {
  }
  TaskWrapper(TaskWrapper&&) = delete;
  TaskWrapper(TaskWrapper const&) = delete;
  Microseconds poll() override
  {
    return runnable.poll();
  }

private:
  R runnable;
};

template<typename Return>
struct Coroutine;
template<>
struct Coroutine<void> : public Task<void>
{
  struct promise_type;
  using handle_type = std::coroutine_handle<promise_type>;

  Coroutine(handle_type t)
    : handle(t)
  {
  }
  Coroutine(Coroutine const&) = delete;
  Coroutine(Coroutine&& c)
    : handle(c.handle)
  {
    c.handle = nullptr;
  }

  Microseconds poll() override;

  ~Coroutine() override
  {
    if (handle)
      handle.destroy();
  }

private:
  handle_type handle;
};

template<typename Return>
struct Coroutine : public Task<Return>
{
  struct promise_type;
  using handle_type = std::coroutine_handle<promise_type>;

  Coroutine(handle_type t)
    : handle(t)
  {
  }
  Coroutine(Coroutine const&) = delete;
  Coroutine(Coroutine&& c)
    : handle(c.handle)
  {
    c.handle = nullptr;
  }

  Microseconds poll() override;
  Return value() override;

  ~Coroutine() override
  {
    if (handle)
      handle.destroy();
  }

private:
  handle_type handle;
};

struct promise_base;
template<Pollable Type>
struct PollAwaiter
{
  bool await_ready()
  {
    return false;
  }
  void await_suspend(auto)
  {
  }
  auto await_resume();
  PollAwaiter(Type&&, promise_base*);

private:
  Type* task;
  promise_base* awaiter;
};

struct promise_base
{
  Microseconds delay;
  std::exception_ptr except;
  Task<void>* continuation = nullptr;
  std::suspend_always initial_suspend() const noexcept
  {
    return {};
  }
  std::suspend_always final_suspend() noexcept
  {
    using namespace std::chrono_literals;
    delay = 0us;
    return {};
  }
  void unhandled_exception()
  {
    except = std::current_exception();
  }
  std::suspend_always yield_value(Microseconds time)
  {
    using namespace std::chrono_literals;
    if (time == 0us) {
      delay = 1us;
    } else {
      delay = time;
    }
    return {};
  }

  template<typename R>
  auto await_transform(R&& p)
    requires(std::convertible_to<R*, Task<void>*>)
  {
    return PollAwaiter{ std::forward<R&&>(p), this };
  }
};

struct Coroutine<void>::promise_type : promise_base
{
  Coroutine get_return_object()
  {
    return { std::coroutine_handle<promise_type>::from_promise(*this) };
  }
  void return_void()
  {
  }
};

template<typename Ret>
struct Coroutine<Ret>::promise_type : promise_base
{
  Coroutine get_return_object()
  {
    return { std::coroutine_handle<promise_type>::from_promise(*this) };
  }
  Ret result;
  void return_value(Ret r)
  {
    result = std::move(r);
  }
};

inline Microseconds Coroutine<void>::poll()
{
  using namespace std::chrono_literals;
  if (handle.done())
    return 0us;
  auto& p = handle.promise();
  do {
    if (p.continuation) {
      Microseconds delay = p.continuation->poll();
      if (delay != 0us)
        return delay;
    }
    handle.resume();
  } while (p.continuation != nullptr);
  if (p.except) {
    std::rethrow_exception(p.except);
  }
  return p.delay;
}

template<typename Return>
Microseconds Coroutine<Return>::poll()
{
  using namespace std::chrono_literals;
  if (handle.done())
    return 0us;
  auto& p = handle.promise();
  do {
    if (p.continuation) {
      Microseconds delay = p.continuation->poll();
      if (delay != 0us)
        return delay;
    }
    handle.resume();
  } while (p.continuation != nullptr);
  if (p.except) {
    std::rethrow_exception(p.except);
  }
  return p.delay;
}

template<typename Return>
Return Coroutine<Return>::value()
{
  if constexpr (!std::same_as<Return, void>) {
    auto& p = handle.promise();
    return p.result;
  }
}

template<Pollable Type>
PollAwaiter<Type>::PollAwaiter(Type&& t, promise_base* p)
  : task(&t)
  , awaiter(p)
{
  awaiter->continuation = &t;
}

template<Pollable Type>
auto PollAwaiter<Type>::await_resume()
{
  if constexpr (Evaluable<Type>) {
    auto r = this->task->value();
    this->awaiter->continuation = nullptr;
    return r;
  }
}
}  // namespace co
