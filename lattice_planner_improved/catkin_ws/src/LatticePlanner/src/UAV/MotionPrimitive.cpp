#include "UAV/MotionPrimitive.h"

namespace UAV {

  bool MotionPrimitive::load_from_file(std::string primitive_file_path, int startID) {

    std::ifstream file;
    std::string line;
    int line_number = 0;
    std::string label;

    this->wait_time = 0;

    DLOG("Entering load_from_file. File-path: " << primitive_file_path << "\n");

    file.open(primitive_file_path);
    if(file.is_open()) {
      while(getline(file, line)) {
        std::istringstream line_stream(line);
        DLOG("Line " << line_number << ": " << line << "\n");
        if(has_prefix(line, "ID:")) {
          DLOG(" [ID] ");
          line_stream >> label >> this->ID;
          ROS_INFO_STREAM("Loading primitives with parameters " << "\n  ID=: " << this->ID << " startID=:" << startID);
          this->ID += startID;
          DLOG(this->ID << "\n");

        }
        else if(has_prefix(line, "Name:")) {
          DLOG(" [Name] ");
          line_stream >> label >> this->name;
          DLOG(this->name << "\n");
        }
        else if(has_prefix(line, "Cost:")) {
          DLOG(" [Cost] ");
          line_stream >> label >> this->cost;
          DLOG(this->cost << "\n");
        }
        else if(has_prefix(line, "Group:")) {
          DLOG(" [Group] ");
          line_stream >> label >> this->group;
          ROS_INFO_STREAM("Loading primitives with parameters " << "\n  group=: " << this->group);
          DLOG(this->group << "\n");
        }
        else if(has_prefix(line, "finalTime:")) {
          DLOG(" [finalTime] ");
          line_stream >> label >> this->duration;
          DLOG(this->duration << "\n");
        }
        else if(has_prefix(line, "ToFromLabel:")) {
          DLOG(" [ToFromLabel] ");
          std::string l1, l2, l3, l4, l5, l6;
          line_stream >> label >> l1 >> l2 >> l3 >> l4 >> l5 >> l6;
          this->map_label.push_back(l1); 
          this->map_label.push_back(l2);
          this->map_label.push_back(l3);
          this->map_label.push_back(l4);
          this->map_label.push_back(l5);
          this->map_label.push_back(l6);
          DLOG(" " << l1 << " " << l2 << " " << l3 << " " << l4);
          for(int n = 0; n < this->map_label.size(); n++) {
            DLOG(this->map_label[n] << " ");
          }
          DLOG("\n");
        }
        else if(has_prefix(line, "From:")) {
          DLOG(" [From] ");
          double x,y,z,vx,vy,vz;
          line_stream >> label >> x >> y >> z >> vx >> vy >> vz;
          this->from = UAV::SearchState(x,y,z,vx,vy,vz);
          DLOG(x << " " << y << " " << z << " " 
               << vx << " " << vy << " " << vz << "\n");
        }
        else if(has_prefix(line, "To:")) {
          DLOG(" [To] ");
          double x,y,z,vx,vy,vz;
          line_stream >> label >> x >> y >> z >> vx >> vy >> vz;
          this->to = UAV::SearchState(x,y,z,vx,vy,vz);
          DLOG(x << " " << y << " " << z << " " 
               << vx << " " << vy << " " << vz << "\n");
        }
        else if(has_prefix(line, "Trajectory:")) {
          DLOG(" [Trajectory]\n");
          if(!getline(file, line)) {
            return false;
          }
          line_number++;
          DLOG("  > Line " << line_number << ": " << line << "\n");
          std::istringstream line_stream(line);

          std::string l1, l2, l3, l4, l5, l6, l7, l8, l9, l10;
          line_stream >> l1 >> l2 >> l3 >> l4 >> l5 >> l6;
          this->trajectory_state_label.push_back(l1); 
          this->trajectory_state_label.push_back(l2);
          this->trajectory_state_label.push_back(l3);
          this->trajectory_state_label.push_back(l4);
          this->trajectory_state_label.push_back(l5);
          this->trajectory_state_label.push_back(l6);
          this->trajectory_state_label.push_back(l6);
          this->trajectory_state_label.push_back(l7);
          this->trajectory_state_label.push_back(l8);
          this->trajectory_state_label.push_back(l9);
          this->trajectory_state_label.push_back(l10);
          DLOG("     Trajectory state labels: ");
          for(int n = 0; n < this->trajectory_state_label.size(); n++) {
            DLOG(this->trajectory_state_label[n] << " ");
          }
          DLOG("\n");

          while(getline(file, line)) {
            std::istringstream line_stream(line);
            DLOG("  > Line " << line_number << ": " << line << "\n");
            
            double t,x,y,z,vx,vy,vz,roll,pitch,yaw,thrust;
            line_stream >> t >> x >> y >> z >> vx >> vy >> vz >> roll >> pitch >> yaw >> thrust;
            this->trajectory.push_back(TrajectoryState(x,y,z,vx,vy,vz,roll,pitch,yaw,thrust,t));

            DLOG("     " << this->trajectory.size()-1 << "] " << this->trajectory.back() << "\n");
            
            line_number++;
          }
        }  
        line_number++;
      }
      clean_up_numbers();
      return true;
    }
    else {
      return false;
    }
  }
  
  Hitbox MotionPrimitive::calculate_hitbox(Hitbox vehicle) {
    // Calculate centroid (mean position)
    Eigen::Vector3f centroid = Eigen::Vector3f(0,0,0);
    for(int n = 0; n < trajectory.size(); n++) {
      centroid += trajectory[n].position;
    }
    centroid /= trajectory.size();
    
    // Calculate the maximum radius
    float max_radius = 0;
    for(int n = 0; n < trajectory.size();  n++) {
      float radius = (centroid - trajectory[n].position).norm();
      max_radius = std::max(max_radius, radius);
    }
    
    // Add vehicle hitbox radius
    max_radius = max_radius + vehicle.getR();
    
    Hitbox motion_primitive_hitbox(0,0,0,max_radius);
    return motion_primitive_hitbox;
  }
  
  std::vector<TrajectoryState> MotionPrimitive::apply(const UAV::SearchState & state, double time)
  {
    std::vector<TrajectoryState> translatedTrajectory;
    auto delta = state.position - from.position;
    
    TrajectoryState tempState;
    for(int n = 0; n < trajectory.size();  n++) 
    {      
      tempState = trajectory[n];
      tempState.position += delta;
      tempState.time += time;      
      translatedTrajectory.push_back(tempState);
    }
    
    return translatedTrajectory;
  }
  
  Hitbox MotionPrimitive::get_hitbox(UAV::SearchState & state) {
    Hitbox transformed_hitbox = hitbox;
    transformed_hitbox.position += state.position;
    return transformed_hitbox;
  }
  
  void MotionPrimitive::transform_state(UAV::SearchState & state) {
    state.position += (to.position - from.position);
    state.velocity = to.velocity;
  }
  
  void MotionPrimitive::clean_up_numbers() {
    from.clean_up_numbers();
    to.clean_up_numbers();
    for(int n = 0; n < trajectory.size();  n++) {
      trajectory[n].clean_up_numbers();
    }
  }
  
  bool has_prefix(std::string string, std::string prefix) {
    return string.compare(0, prefix.length(), prefix) == 0;
  }
}
