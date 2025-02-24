#pragma once
#ifndef __MEMORY_POOL_H__
#define __MEMORY_POOL_H__

#include <list>
#include <vector>
#include "State.h"

class MemoryPool
{
public:
  void init(int chunk_size);
  State * allocate();
  State * allocate(State & other);
  void free(State * state);
  void reset();

private:
  int chunk_size_;
  std::list<std::vector<State> > data_;
  std::list<std::vector<State*> > free_list_;
};


#endif // __MEMORY_POOL_H__
