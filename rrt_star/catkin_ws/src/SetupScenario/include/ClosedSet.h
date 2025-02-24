#pragma once
#ifndef __CLOSED_SET_H__
#define __CLOSED_SET_H__

#include "ros/ros.h"
#include <list>
#include <set>
#include <string>
#include <sstream>


//TODO: Remove?
//#include "State.h"
#include "UAV/State.h"
#include "UAV/MotionPrimitive.h"
#include "Search/Search.h"
using State = search::SearchState<UAV::SearchState>;

class ClosedSet {

public:
  virtual ~ClosedSet() {};
  virtual void set_visited(State * state) = 0;
  virtual bool is_visited(State * state) = 0;
  virtual void clear() = 0;
  virtual int size() = 0;
  virtual std::string to_string() = 0;
  virtual std::vector<State*> get_states() { return std::vector<State*>(); }
};

class ClosedSetNone : public ClosedSet {
  public:
    ClosedSetNone() {}
    ~ClosedSetNone() {}

    void set_visited(State * state) override {
      N++;
    } 

    bool is_visited(State * state) override {
      return false;
    } 

    void clear() override {
      N = 0;
    } 

    int size() override {
      return N;
    }

    std::string to_string() {
      return "None";
    }


  private:
    int N;
};


// Base line (list)
class ClosedSetList : public ClosedSet {
  public:
    ClosedSetList() {}
    ~ClosedSetList() {}

    void set_visited(State * state) override {
      closed_set.push_back(state);
    } 

    bool is_visited(State * state) override {
      for(std::list<State*>::iterator n = this->closed_set.begin(); n != this->closed_set.end(); n++)
      {
        if(state->state == (*n)->state)
          return true;
      }
      return false;
    } 

    void clear() override {
      closed_set.clear();
    } 

    int size() override {
      return closed_set.size();
    }

    std::string to_string() {
      std::stringstream out;
      for(std::list<State*>::iterator state = closed_set.begin(); state != closed_set.end(); state++) {
        out << "(" << *state << "): " << *(*state) << std::endl; 
      }
      return out.str();
    }

  std::vector<State*> get_states() override { 
    return std::vector<State*>(closed_set.begin(), closed_set.end()); 
  }


  private:
    std::list<State*> closed_set;
};

// Set variant
class ClosedSetSet : public ClosedSet {
  public:
    struct LessThan { // Used by Closed set when using Set (instead of a list)
      bool operator()(State * s1, State * s2) {
        return s1->state.position.x() < s2->state.position.x() ||
               (s1->state.position.x() == s2->state.position.x() &&
                (s1->state.position.y() < s2->state.position.y() ||
                 (s1->state.position.y() == s2->state.position.y() &&
                  (s1->state.position.z() < s2->state.position.z() ||
                   (s1->state.position.z() == s2->state.position.z() &&
                    (s1->state.velocity.x() < s2->state.velocity.x()||
                     (s1->state.velocity.x() == s2->state.velocity.x() &&
                      (s1->state.velocity.y() < s2->state.velocity.y()||
                       (s1->state.velocity.y() == s2->state.velocity.y() &&
                        s1->state.velocity.z() < s2->state.velocity.z()
                       )
                      )
                     )
                    )
                   )
                  )
                 )
                )
               );
      }
    };
  public:
    ClosedSetSet() {}
    ~ClosedSetSet() {}

    void set_visited(State * state) override {
      closed_set.insert(state);
    } 

    bool is_visited(State * state) override {
      return closed_set.find(state) != closed_set.end();
    } 

    void clear() override {
      closed_set.clear();
    }  

    int size() override {
      return closed_set.size();
    }

    std::string to_string() {
      std::stringstream out;
      for(std::set<State*,LessThan>::iterator state = closed_set.begin(); state != closed_set.end(); state++) {
        out << "(" << *state << "): " << *(*state) << std::endl; 
      }
      return out.str();
   }

  std::vector<State*> get_states() override { 
    return std::vector<State*>(closed_set.begin(), closed_set.end()); 
  }

  private:
    std::set<State*,LessThan> closed_set;
};

// Set variant with time
class ClosedSetSetTime : public ClosedSet {
  public:
    struct LessThan { // Used by Closed set when using Set (instead of a list)
      ClosedSetSet::LessThan tmp;
      bool operator()(State * s1, State * s2) {
        return s1->time < s2->time || 
               (s1->time == s2->time && tmp(s1,s2));
      }
    };
  public:
    ClosedSetSetTime() {}
    ~ClosedSetSetTime() {}

    void set_visited(State * state) override {
      closed_set.insert(state);
    } 

    bool is_visited(State * state) override {
      return closed_set.find(state) != closed_set.end();
    } 

    void clear() override {
      closed_set.clear();
    }  

    int size() override {
      return closed_set.size();
    }

    std::string to_string() {
      std::stringstream out;
      for(std::set<State*,LessThan>::iterator state = closed_set.begin(); state != closed_set.end(); state++) {
        out << "(" << *state << "): " << *(*state) << std::endl; 
      }
      return out.str();
  }

  std::vector<State*> get_states() override { 
    return std::vector<State*>(closed_set.begin(), closed_set.end()); 
  }

  private:
    std::set<State*,LessThan> closed_set;
};

// Set variant with logical time (wait time)
class ClosedSetSetWaitTime : public ClosedSet {
  public:
    struct LessThan { // Used by Closed set when using Set (instead of a list)
      ClosedSetSet::LessThan tmp;
      bool operator()(State * s1, State * s2) {
        return s1->wait_time < s2->wait_time || 
               (s1->wait_time == s2->wait_time && tmp(s1,s2));
      }
    };
  public:
    ClosedSetSetWaitTime() {}
    ~ClosedSetSetWaitTime() {}

    void set_visited(State * state) override {
      closed_set.insert(state);
    } 

    bool is_visited(State * state) override {
      return closed_set.find(state) != closed_set.end();
    } 

    void clear() override {
      closed_set.clear();
    }  

    int size() override {
      return closed_set.size();
    }

    std::string to_string() {
      std::stringstream out;
      for(std::set<State*,LessThan>::iterator state = closed_set.begin(); state != closed_set.end(); state++) {
        out << "(" << *state << "): " << *(*state) << std::endl; 
      }
      return out.str();
  }

  std::vector<State*> get_states() override { 
    return std::vector<State*>(closed_set.begin(), closed_set.end()); 
  }

  private:
    std::set<State*,LessThan> closed_set;
};

// Set variant with discretized time
class ClosedSetSetTimeTruncated : public ClosedSet {
  public:
    struct LessThan { // Used by Closed set when using Set (instead of a list)
      ClosedSetSet::LessThan tmp;
      LessThan(double factor = 1.0) : factor(factor) {}
      bool operator()(State * s1, State * s2) {
        int t1 = int(s1->time*factor);
        int t2 = int(s2->time*factor);
        return t1 < t2 || 
               (t1 == t2 && tmp(s1,s2));
      }
      double factor;
    };
  public:
    ClosedSetSetTimeTruncated(double factor = 1.0) : factor(factor) {
      closed_set = std::set<State*,LessThan>(LessThan(factor));
    }
    ~ClosedSetSetTimeTruncated() {}

    void set_visited(State * state) override {
      closed_set.insert(state);
    } 

    bool is_visited(State * state) override {
      return closed_set.find(state) != closed_set.end();
    } 

    void clear() override {
      closed_set.clear();
    }  

    int size() override {
      return closed_set.size();
    }

    std::string to_string() {
      std::stringstream out;
      for(std::set<State*,LessThan>::iterator state = closed_set.begin(); state != closed_set.end(); state++) {
        out << "(" << *state << "): " << *(*state) << std::endl; 
      }
      return out.str();
  }

  std::vector<State*> get_states() override { 
    return std::vector<State*>(closed_set.begin(), closed_set.end()); 
  }

  private:
    std::set<State*,LessThan> closed_set;
    double factor;
};

// Set variant with discretized time
class ClosedSetSetTimeRounded : public ClosedSet {
  public:
    struct LessThan { // Used by Closed set when using Set (instead of a list)
      ClosedSetSet::LessThan tmp;
      bool operator()(State * s1, State * s2) {
        double t1 = std::round(s1->time);
        double t2 = std::round(s2->time);
        return t1 < t2 || 
               (t1 == t2 && tmp(s1,s2));
      }
    };
  public:
    ClosedSetSetTimeRounded() {}
    ~ClosedSetSetTimeRounded() {}

    void set_visited(State * state) override {
      closed_set.insert(state);
    } 

    bool is_visited(State * state) override {
      return closed_set.find(state) != closed_set.end();
    } 

    void clear() override {
      closed_set.clear();
    }  

    int size() override {
      return closed_set.size();
    }

    std::string to_string() {
      std::stringstream out;
      for(std::set<State*,LessThan>::iterator state = closed_set.begin(); state != closed_set.end(); state++) {
        out << "(" << *state << "): " << *(*state) << std::endl; 
      }
      return out.str();
  }

  std::vector<State*> get_states() override { 
    return std::vector<State*>(closed_set.begin(), closed_set.end()); 
  }

  private:
    std::set<State*,LessThan> closed_set;
};


#endif // __CLOSED_SET_H__
