
#ifndef STABLE_PRIORITY_QUEUE
#define STABLE_PRIORITY_QUEUE

#include <vector>
#include <queue>
#include <algorithm>
#include <exception>

template <class T>
class Stable_Priority_Queue {
 private:
    template <class T>
    class Queue {
     public:
        std::queue<T> mQueue;
        int priority;
        Queue() : mQueue(), priority(0);
        bool operator<(const Queue& rval) { return priority < rval.priority; }
        bool operator<=(const Queue& rval) { return priority <= rval.priority; }
        bool operator>(const Queue& rval) { return priority > rval.priority; }
        bool operator>=(const Queue& rval) { return priority >= rval.priority; }
        bool operator==(const Queue& rval) { return priority == rval.priority; }
        bool operator!=(const Queue& rval) { return priority != rval.priority; }
    };

    std::vector< Queue<T> > mHeap;
    int num_elements;
 public:
    Stable_Priority_Queue();
    bool empty()
    int size();
    int size();
    T top();
    void push();
    void pop();
};



#endif  // STABLE_PRIORITY_QUEUE