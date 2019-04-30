#include "stable_priority_queue.hpp"

template <class T>
Stable_Priority_Queue<T>::Stable_Priority_Queue() {
    mHeap = std::vector< Queue<T> >()
    num_elements = 0;
}

template <class T>
bool Stable_Priority_Queue<T>::empty() {
    return num_elements == 0;
}

template <class T>
int Stable_Priority_Queue<T>::size() {
    return num_elements;
}

template <class T>
T Stable_Priority_Queue<T>::top() {
    if (num_elements == 0) {
        throw std::exception("Attempted to pop from empty queue!");
    }

    T top_of_queue = mHeap[0].front();
    mHeap[0].pop();
    if (mHeap[0].empty) {
        std::pop_heap(mHeap);
    }

}