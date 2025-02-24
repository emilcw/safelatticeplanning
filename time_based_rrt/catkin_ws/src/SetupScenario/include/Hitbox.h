#pragma once
#ifndef __HITBOX_H__
#define __HITBOX_H__

#include <iostream>
#include <math.h>
#include <string>
#include <Eigen/Eigen>

inline double squared(double x) {
  return x*x;
}

class AABB {
public:
  AABB() 
    : position(0,0,0), radius(0,0,0) {
    update_minmax();
  }

  AABB(Eigen::Vector3f position, Eigen::Vector3f radius)
    : position(position), radius(radius) {
    update_minmax();
  }
  AABB(double x, double y, double z, double rx, double ry, double rz)
    : position(x,y,z), radius(rx,ry,rz) {
    update_minmax();
  }
    
  void set_center(double x, double y, double z) {
    position = Eigen::Vector3f(x,y,z);
    update_minmax();
  }

  void set_radius(double rx, double ry, double rz) {
    radius = Eigen::Vector3f(rx,ry,rz);
    update_minmax();
  }

  void update_minmax() {
    position_min = position - radius;
    position_max = position + radius;  
  }

  double getX() const { return position.x(); }
  double getY() const { return position.y(); }
  double getZ() const { return position.z(); }
  double getRX() const { return radius.x(); }
  double getRY() const { return radius.y(); }
  double getRZ() const { return radius.z(); }
  double getMinX() const { return position_min.x(); }
  double getMinY() const { return position_min.y(); }
  double getMinZ() const { return position_min.z(); }
  double getMaxX() const { return position_max.x(); }
  double getMaxY() const { return position_max.y(); }
  double getMaxZ() const { return position_max.z(); }

  bool is_inside(double x, double y, double z) {
    return position_min.x() <= x && x <= position_max.x() &&
           position_min.y() <= y && y <= position_max.y() &&
           position_min.z() <= z && z <= position_max.z();
           
  }
    
  friend std::ostream& operator<<(std::ostream& os,const AABB & o) {  
    os << "Pose: (x,y,z): (" << o.position.x() << "," << o.position.y() << "," << o.position.z() << ")";
    os << ", Radius: (rx,ry,rz): " << o.radius.x() << "," << o.radius.y() << "," << o.radius.z() << ")";  
    os << ", min: (xm,ym,zm): " << o.position_min.x() << "," << o.position_min.y() << "," << o.position_min.z() << ")";  
    os << ", max: (xm,ym,zm): " << o.position_max.x() << "," << o.position_max.y() << "," << o.position_max.z() << ")";  
    return os;  
  }

public:
  Eigen::Vector3f position;
  Eigen::Vector3f radius;

  // Used to speed up collision checking.
  Eigen::Vector3f position_min;
  Eigen::Vector3f position_max;
};


class Hitbox;
inline bool sphere2sphere_collision(Hitbox & a, Hitbox & b);
inline bool sphere2AABB_collision(Hitbox & a, AABB & b);
inline bool AABB2AABB_collision(AABB & a, AABB & b);

#include "ros/ros.h"
class Hitbox {
public:
  Hitbox()
    : position(0,0,0), r(1.0), has_AABB(false) {}
  
  Hitbox(double x, double y, double z, double r)
    : position(x,y,z), r(r), has_AABB(false) {}
    
  void set_position(double x, double y, double z) {
    position = Eigen::Vector3f(x,y,z);
    aabb.set_center(x,y,z);
  }
  
  void set_radius(double r) {
   this->r = r;
  }

  void add_AABB(double rx, double ry, double rz) {
    aabb = AABB(position, Eigen::Vector3f(rx,ry,rz));
    has_AABB = true;
    r = std::sqrt(rx*rx+ry*ry+rz*rz);
  }

  double getX() const { return position.x(); }
  double getY() const { return position.y(); }
  double getZ() const { return position.z(); }
  double getR() const { return r; }
  Eigen::Vector3f getPosition() const { return position; }
  
  bool intersecting(Hitbox other) {
    bool collision = sphere2sphere_collision(*this, other);
    if(!collision) return false;

    if(has_AABB) {
      if(other.has_AABB) {
        collision = AABB2AABB_collision(aabb, other.aabb);
      }
      else {
        collision = sphere2AABB_collision(other, aabb);
      }
    }
    else if(other.has_AABB) {
      collision = sphere2AABB_collision(*this, other.aabb);
    }
 
    return collision;
  }

  double distance_spheres(const Hitbox& other) {
    return (this->position-other.position).norm() - (this->r + other.r);
    //return sqrt(distance2(this->x, this->y, this->z, other.x, other.y, other.z)) - (this->r + other.r);
  }
  
  friend std::ostream& operator<<(std::ostream& os,const Hitbox & o) {  
    os << "Pose: (x,y,z): (" << o.position.x() << "," << o.position.y() << "," << o.position.z() << ")";
    os << ", Radius: " << o.r;  
    return os;  
  }
    
public:

  Eigen::Vector3f position;
  double r;

  bool has_AABB;
  AABB aabb;

};


bool sphere2sphere_collision(Hitbox & h1, Hitbox & h2) {
  double dist2 = (h1.position - h2.position).squaredNorm(); 
  return dist2 < (h1.r + h2.r)*(h1.r + h2.r);
  //distance2(h1.x, h1.y, h1.z, h2.x, h2.y, h2.z);
  //return dist2 < (h1.r + h2.r)*(h1.r + h2.r);
}

bool AABB2AABB_collision(AABB & a, AABB & b) {
  if ( std::fabs(a.getX() - b.getX()) > (a.getRX() + b.getRX()) ) return false;
  if ( std::fabs(a.getY() - b.getY()) > (a.getRY() + b.getRY()) ) return false;
  if ( std::fabs(a.getZ() - b.getZ()) > (a.getRZ() + b.getRZ()) ) return false;

  return true;
}

bool sphere2AABB_collision(Hitbox & a, AABB & b) {
  double sphere_r2 = a.r*a.r;
  double diff_r2 = 0.0;
  if(a.getX() < b.getMinX())
    diff_r2 += squared(a.getX() - b.getMinX());
  else 
  if (a.getX() > b.getMaxX())
    diff_r2 += squared(a.getX() - b.getMaxX());

  if(a.getY() < b.getMinY())
    diff_r2 += squared(a.getY() - b.getMinY());
  else 
  if (a.getY() > b.getMaxY())
    diff_r2 += squared(a.getY() - b.getMaxY());

  if(a.getZ() < b.getMinZ())
    diff_r2 += squared(a.getZ() - b.getMinZ());
  else 
  if (a.getZ() > b.getMaxZ())
    diff_r2 += squared(a.getZ() - b.getMaxZ());

  return diff_r2 <= sphere_r2;
}

inline AABB create_AABB(double minX, double maxX,
                         double minY, double maxY,
                         double minZ, double maxZ) {
  double rX = (maxX - minX)/2;
  double rY = (maxY - minY)/2;
  double rZ = (maxZ - minZ)/2;
  double cX =  minX + rX;
  double cY =  minY + rY;
  double cZ =  minZ + rZ;
      
  return AABB(cX, cY, cZ, rX, rY, rZ);
}

#endif
