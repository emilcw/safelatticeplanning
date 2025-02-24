/* Author: Mattias Tiger (2018)
*/
#pragma once
#ifndef __MEMORY_POOL_SIMPLE_H__
#define __MEMORY_POOL_SIMPLE_H__

#include <list>
#include <vector>
#include <iostream>

template<class T>
class MemoryPoolSimple
{
public:
  void init(int chunk_size);
  T * allocate();
  T * allocate(T & other);
  void reset();

private:
  std::vector<T> pool;
  int counter;
};

template<class T>
void MemoryPoolSimple<T>::init(int chunk_size) {
  pool.resize(chunk_size);
  counter = 0;
}

template<class T>
T * MemoryPoolSimple<T>::allocate() {
  if(counter == pool.size()) {
    std::cerr << "ERROR: MAX ALLOCATIONS REACHED (" << counter << ")!\n";
    return 0;
  }
  T * data = &pool[counter];
  counter++;
  return data;
}

template<class T>
T * MemoryPoolSimple<T>::allocate(T & other) {
  T * data = pool[counter];
  counter++;
  *data = T(other);
  return data;
}
/*
template<class T>
void MemoryPoolSimple<T>::free(T * data) {
  delete data;
}*/

template<class T>
void MemoryPoolSimple<T>::reset() {
  counter = 0;
}

#endif // __MEMORY_POOL_SIMPLE_H__
