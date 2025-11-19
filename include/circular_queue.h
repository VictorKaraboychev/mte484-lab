#ifndef __CIRCULAR_QUEUE_H__
#define __CIRCULAR_QUEUE_H__

#include <Arduino.h>

class CircularQueue {
private:
  float* buffer;
  int capacity;
  int head;  // Points to the most recent element
  int size;  // Current number of elements in the queue

public:
  // Constructor: creates a circular queue with given capacity
  CircularQueue(int cap) : capacity(cap), head(0), size(0) {
    buffer = new float[capacity];
    // Initialize all values to 0
    for (int i = 0; i < capacity; i++) {
      buffer[i] = 0.0f;
    }
  }

  // Destructor: free allocated memory
  ~CircularQueue() {
    delete[] buffer;
  }

  // Push a new value to the front (most recent)
  // This overwrites the oldest value if the queue is full
  void push(float value) {
    // Move head backward (circularly)
    head = (head - 1 + capacity) % capacity;
    buffer[head] = value;
    
    // Update size (don't exceed capacity)
    if (size < capacity) {
      size++;
    }
  }

  // Get value at index (0 = most recent, 1 = second most recent, etc.)
  float get(int index) const {
    if (index < 0 || index >= capacity) {
      return 0.0f;  // Out of bounds, return 0
    }
    int pos = (head + index) % capacity;
    return buffer[pos];
  }

  // Get the capacity of the queue
  int getCapacity() const {
    return capacity;
  }

  // Get the current size of the queue
  int getSize() const {
    return size;
  }

  // Reset all values to 0
  void reset() {
    for (int i = 0; i < capacity; i++) {
      buffer[i] = 0.0f;
    }
    head = 0;
    size = 0;
  }
};

#endif // __CIRCULAR_QUEUE_H__

