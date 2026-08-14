#pragma once

#include <iostream>
#include <queue>
#include <functional>
#include <vector>
#include <cstdint>

using Time_us = uint64_t;
using EventCallback = std::function<void(Time_us)>;

struct Event {
    EventCallback callback;
    uint64_t timestamp;

    Event(EventCallback cb, uint64_t ts) : callback(std::move(cb)), timestamp(ts) {}

    // For priority_queue ordering
    bool operator>(const Event& other) const {
    return timestamp > other.timestamp;
    }
};

class Scheduler {
public:
    Scheduler() = default;
    void schedule(Time_us delay_us, EventCallback callback) {
        event_queue.emplace(callback, current_time + delay_us);
    }
    void run_until(Time_us end_time_us) {
        while (!event_queue.empty() && event_queue.top().timestamp <= end_time_us) {
            // 1. Grab the next scheduled event
            Event next_event = event_queue.top();
            event_queue.pop();

            // 2. Advance the global simulation time
            current_time = next_event.timestamp;

            // 3. Execute the callback (this might schedule future events)
            next_event.callback(current_time);
        }
    }
private:
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> event_queue;
    Time_us current_time = 0;
    // Run the simulation loop until a specific end time
    

    Time_us get_current_time() const { return current_time; }
};