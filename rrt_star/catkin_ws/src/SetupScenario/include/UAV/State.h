#pragma once
#ifndef __UAV_STATE_H__
#define __UAV_STATE_H__

#include <Eigen/Eigen>
#include <ostream>

namespace UAV {

  /* Floating point type */
  using FT = float;
    
  /* Utility */
  inline void clean_floating_number(FT & number) {
    if(std::abs(number) < 1e-5)
      number = 0.0;
  }
  
  /* SearchState: State used in search 
    ====================================*/
  class SearchState {
    public:
    Eigen::Vector3f position{};
    Eigen::Vector3f velocity{};
      
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
    public:
      SearchState() {}
      SearchState(FT x, FT y, FT z, FT vx = 0, FT vy = 0, FT vz = 0) {
        position << x, y, z;
        velocity << vx, vy, vz;
      }
      SearchState(Eigen::Vector3f position, Eigen::Vector3f velocity = Eigen::Vector3f(0,0,0)) {
        this->position = position;
        this->velocity = velocity;
      }
      /*
      bool operator==(const SearchState & o) {
        return position == o.position && 
               velocity == o.velocity;
      }*/
      bool operator==(const SearchState & o) {
        return position == o.position && 
               (velocity - o.velocity).norm() < 0.01;
      }
      bool operator!=(const SearchState & o) {
        return !(*this == o);
      }
      void clean_up_numbers() {
        clean_floating_number(position.x());
        clean_floating_number(position.y());
        clean_floating_number(position.z());
        clean_floating_number(velocity.x());
        clean_floating_number(velocity.y());
        clean_floating_number(velocity.z());
      }
      friend std::ostream& operator<<(std::ostream& os, const SearchState & o) {  
        os << "(x,y,z): (" << o.position.x() << "," << o.position.y() << "," << o.position.z() << ")";
        os << ", (vx,vy,vz): (" << o.velocity.x() << "," << o.velocity.y() << "," << o.velocity.z() << ")";  
        return os;
      }

  };

  inline SearchState operator+(SearchState lhs, const SearchState& rhs) {
    SearchState ret = lhs;
    ret.position = lhs.position + rhs.position;
    return ret;
  }

  /* TrajectoryState: State used in motion primitives
    ====================================*/
  class TrajectoryState : public SearchState {
    public:
      FT roll, pitch, yaw; // Attitude
      FT thrust;
      FT time;             // Time on the trajectory
      
    public:
      TrajectoryState() {}
      TrajectoryState(FT x, FT y, FT z, FT vx, FT vy, FT vz, FT pitch, FT roll, FT yaw, FT thrust, FT time = 0.0) 
        : roll(roll), pitch(pitch), yaw(yaw), thrust(thrust), time(time) {
        position << x, y, z;
        velocity << vx, vy, vz;
      }
      void clean_up_numbers() {
        SearchState::clean_up_numbers();
        clean_floating_number(roll);
        clean_floating_number(pitch);
        clean_floating_number(yaw);
      }
      friend std::ostream& operator<<(std::ostream& os, TrajectoryState & o) {  
        os << "(x,y,z): (" << o.position.x() << "," << o.position.y() << "," << o.position.z() << ")";
        os << ", (vx,vy,vz): (" << o.velocity.x() << "," << o.velocity.y() << "," << o.velocity.z() << ")";  
        os << ", (r,p,y): (" << o.roll << "," << o.pitch << "," << o.yaw << ")";
        os << ", (thurst,t): (" << o.thrust << "," << o.time << ")";  
        return os;
      }  
  };
}
/*
inline bool operator<(const UAV::SearchState & s1, const UAV::SearchState & s2) {
  return s1.position.x() < s2.position.x() ||
         (s1.position.x() == s2.position.x() &&
          (s1.position.y() < s2.position.y() ||
           (s1.position.y() == s2.position.y() &&
            (s1.position.z() < s2.position.z() ||
             (s1.position.z() == s2.position.z() &&
              (s1.velocity.x() < s2.velocity.x()||
               (s1.velocity.x() == s2.velocity.x() &&
                (s1.velocity.y() < s2.velocity.y()||
                 (s1.velocity.y() == s2.velocity.y() &&
                  s1.velocity.z() < s2.velocity.z()
                 )
                )
               )
              )
             )
            )
           )
          )
         );
}*/
inline bool equal(const float & a, const float & b) {
  return std::abs(a-b) < 0.01;
}
inline bool less(const float & a, const float & b) {
  return a-b < -0.01;
}
inline bool operator<(const UAV::SearchState & s1, const UAV::SearchState & s2) {
  return less(s1.position.x(), s2.position.x()) ||
         (equal(s1.position.x(), s2.position.x()) &&
          (less(s1.position.y(), s2.position.y()) ||
           (equal(s1.position.y(), s2.position.y()) &&
            (less(s1.position.z(), s2.position.z()) ||
             (equal(s1.position.z(), s2.position.z()) &&
              (less(s1.velocity.x(), s2.velocity.x())||
               (equal(s1.velocity.x(), s2.velocity.x()) &&
                (less(s1.velocity.y(), s2.velocity.y())||
                 (equal(s1.velocity.y(), s2.velocity.y()) &&
                  less(s1.velocity.z(), s2.velocity.z())
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


#endif
