#include <thread>
#include <cstdint>
#include <time.h>
#include <type_traits>
#include <stdexcept>

#include <pthread.h>

#ifdef __APPLE__
#ifdef __MACH__
#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <mach/thread_act.h>

void pinThread(int core) {
    thread_affinity_policy_data_t policy = { core };
    mach_port_t mach_thread = pthread_mach_thread_np(pthread_self());

    thread_policy_set(mach_thread,
                      THREAD_AFFINITY_POLICY,
                      (thread_policy_t)&policy,
                      THREAD_AFFINITY_POLICY_COUNT);

}
#endif
#endif
#ifdef __unix__
void pinThread(uint32_t core) {
  ::cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core, &cpuset);
  if (::pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) == -1) {
    throw std::runtime_error("error while pinning thread");
  }
}
#endif

#include "benchmark/benchmark.h"
#include "Fifo4a.hpp"
#include "boost/lockfree/spsc_queue.hpp"
#include "rigtorp/SPSCQueue.h"

#define RING_BUFFER_SIZE (16*1024)

uint64_t getTime() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

template<typename T>
struct isRigtorpQueue : std::false_type {};

template<typename V>
struct isRigtorpQueue<rigtorp::SPSCQueue<V>> : std::true_type {};


template<template<typename> typename QueueType>
void BM_SPSCQueue(benchmark::State& state) {
  using Queue = QueueType<std::int_fast32_t>;
  using QueueValueType = std::int_fast32_t;
  Queue q(RING_BUFFER_SIZE);
  
  double numIts = 0; 
  for (auto _ : state) {
    numIts++;
    auto t1 = std::thread([&](){
      pinThread(8);
      for (int i=0; i<state.range(0); ++i) {
        if constexpr(isRigtorpQueue<Queue>::value) {
          while(auto x = !q.try_push(i)) { benchmark::DoNotOptimize(x);}
        } else {
          while(auto x = !q.push(i)) { benchmark::DoNotOptimize(x);}
        } 
      }
    });

    uint32_t chk = 0;
    QueueValueType val;
    pinThread(9);
    for (int i=0; i<state.range(0); ++i) {
      if constexpr(isRigtorpQueue<Queue>::value) {
        for(;;)  {
          auto p = q.front();
          if (!p) continue;
          chk ^= *p;
          benchmark::DoNotOptimize(p);
          break;
        }
        q.pop();
      } else {
        while(!q.pop(val)); 
        benchmark::DoNotOptimize(val);
        chk ^= val;
      }
    }  
    t1.join();
    benchmark::DoNotOptimize(chk);
  }
  
  state.counters["op/s"] = benchmark::Counter(double(state.range(0)*numIts), benchmark::Counter::kIsRate);
};

// Weird hack to make it work with Google benchmark
#define redef(old, new) \
  template<typename T> \
  using new = old<T>

namespace {
  redef(rigtorp::SPSCQueue, rigtorp_);
  BENCHMARK_TEMPLATE(BM_SPSCQueue, rigtorp_)
    ->Arg(1'000'000)
    ->Arg(10'000'000)
    ->Arg(500'000'000)
    ->Unit(benchmark::kMillisecond);

  redef(boost::lockfree::spsc_queue, boost_spsc_);
  BENCHMARK_TEMPLATE(BM_SPSCQueue, boost_spsc_)
    ->Arg(1'000'000)
    ->Arg(500'000'000)
    ->Unit(benchmark::kMillisecond);
  redef(Fifo4a, fifo4a_);
  BENCHMARK_TEMPLATE(BM_SPSCQueue, fifo4a_)
    ->Arg(1'000'000)
    ->Arg(500'000'000)
    ->Unit(benchmark::kMillisecond);
}
BENCHMARK_MAIN();
