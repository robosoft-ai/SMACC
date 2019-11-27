#include <boost/statechart/fifo_scheduler.hpp>
#include <smacc/smacc_worker.h>

typedef boost::statechart::fifo_scheduler<SmaccWorker, SmaccAllocator> SmaccScheduler;