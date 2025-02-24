#include "MemoryPool.h"

void MemoryPool::init(int chunk_size) {
  chunk_size_ = chunk_size;

  data_.clear();
  //data_.reserve(chunk_size);  // Force the list to not de-allocate the first chunk_size number of vectors when cleared
  data_.resize(1);  // Allocate one std::vector
  data_.front().reserve(chunk_size_);

  free_list_.clear();
  //free_list_.reserve(chunk_size);  // Force the list to not de-allocate the first chunk_size number of vectors when cleared
  free_list_.resize(1); // Allocate one std::vector
  free_list_.front().reserve(chunk_size_);
}

State * MemoryPool::allocate() {
  State * state;

  // If there is an available state in the free list, us it and update the free list
  if(!free_list_.empty()) {
    state = free_list_.back().back();
    free_list_.back().pop_back();
    if(free_list_.back().size() == 0) {
      free_list_.pop_back();
    }
    return state;
  }

  // If end of chunk is reached, create a new chunk
  if(data_.back().size() - 1 == chunk_size_) {
    data_.push_back(std::vector<State>());
    data_.back().reserve(chunk_size_);
  }

  // Allocate a new state from the data-container (data_)
  data_.back().push_back(State());
  state = &data_.back().back();
  return state;
}

State * MemoryPool::allocate(State & other) {
  State * state;

  // If there is an available state in the free list, us it and update the free list
  if(!free_list_.empty()) {
    state = free_list_.back().back();
    free_list_.back().pop_back();
    if(free_list_.back().size() == 0) {
      free_list_.pop_back();
    }
    *state = State(other);
    return state;
  }

  // If end of chunk is reached, create a new chunk
  if(data_.back().size() - 1 == chunk_size_) {
    data_.push_back(std::vector<State>());
    data_.back().reserve(chunk_size_);
  }

  // Allocate a new state from the data-container (data_)
  data_.back().push_back(State(other));
  state = &data_.back().back();
  *state = State(other);
  return state;
}

void MemoryPool::free(State * state) {
  // If end of chunk is reached, create a new chunk
  if(free_list_.back().size() - 1 == chunk_size_) {
    free_list_.push_back(std::vector<State*>());
    free_list_.back().reserve(chunk_size_);
  }
  // Add state to free list
  free_list_.back().push_back(state);
}

void MemoryPool::reset() {
  free_list_.clear();
  data_.clear();
}
