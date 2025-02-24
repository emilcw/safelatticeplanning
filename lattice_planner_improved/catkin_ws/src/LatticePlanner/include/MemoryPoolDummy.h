#pragma once
#ifndef __MEMORY_POOL_DUMMY_H__
#define __MEMORY_POOL_DUMMY_H__

#include <list>
#include <vector>

template<class T>
class MemoryPoolDummy
{
public:
  void init(int chunk_size);
  T * allocate();
  T * allocate(T & other);
  void free(T * data);
  void reset();

private:
  int chunk_size_;
};

template<class T>
void MemoryPoolDummy<T>::init(int chunk_size) {
  chunk_size_ = chunk_size;
}

template<class T>
T * MemoryPoolDummy<T>::allocate() {
  T * data = new T();
  return data;
}

template<class T>
T * MemoryPoolDummy<T>::allocate(T & other) {
  T * data = new T(other);
  return data;
}

template<class T>
void MemoryPoolDummy<T>::free(T * data) {
  delete data;
}

template<class T>
void MemoryPoolSimple<T>::reset() {

}

#endif // __MEMORY_POOL_DUMMY_H__
