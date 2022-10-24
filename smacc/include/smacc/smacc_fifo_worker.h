#pragma once
#include <boost/statechart/fifo_worker.hpp>

typedef std::allocator<void> SmaccAllocator;
typedef boost::statechart::fifo_worker<SmaccAllocator> SmaccFifoWorker;
