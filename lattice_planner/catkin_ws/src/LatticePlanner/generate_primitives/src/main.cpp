#include "ACADODjiMatrice.h"
#include "MotionPrimitiveGeneration.h"
#include <iostream>
#include <string>
#include "GridPoint.h"

using namespace std;

int main (int argc, char *argv[])
{
    
    int number_of_groups = 14;
    double sampling_time = 0.1; //50 Hz
    
    for(int group = 1 ; group < number_of_groups + 1 ; group++){
    
      //Set true to print a lot of info
      bool verbose = true;

      string latticeFile = string("../config/lattice_group") + std::to_string(group) + string(".txt");

      FILE * inFile;
      inFile = fopen(latticeFile.c_str(), "r");

      int numberOfPrimitives;
      fscanf(inFile, "%d", &numberOfPrimitives);
  
      char buffer[100];
      std::cout << "Group: " << group << " out of " << number_of_groups << " groups has " << numberOfPrimitives <<  " number of primitives." << std::endl;
      int solved = 0;
      for (int i = 0; i < numberOfPrimitives; i++)
      {
          State initialState = State();
          State finalState = State();

          int id;
          fscanf(inFile, "\n%d",  &id);
          
          fscanf(inFile, "%s",  buffer);
          string name(buffer);
          
          double finalTime;
          fscanf(inFile, "%lf", &finalTime);

          // Read xinit yinit zinit vxinit vyinit vzinit
          fscanf(inFile, "%lf %lf %lf %lf %lf %lf",
                 &initialState.x, &initialState.y, &initialState.z,
                 &initialState.vx, &initialState.vy, &initialState.vz);

          // Read xfinal yfinal zfinal vxfinal vyfinal vzfinal
          fscanf(inFile, "%lf %lf %lf %lf %lf %lf",
                 &finalState.x, &finalState.y, &finalState.z,
                 &finalState.vx, &finalState.vy, &finalState.vz);

          initialState.time = 0;
          finalState.time = finalTime;

          GridPoint gridPoint(initialState,finalState,name,id,group);

          double opt_cost = -1;
          if ( GenerateMotionPrimitive(*gridPoint.getRef2Traj(), gridPoint.getInitialState() , gridPoint.getFinalState(),sampling_time, finalTime, opt_cost) )
          {
              solved++;
              gridPoint.setCost(opt_cost);
              std::cout << "Solved: " << solved << " of " << numberOfPrimitives << std::endl;
              string SolutionPath =  string("../primitives/primitive") + string("_group_") + std::to_string(group) + 
                string("_id_") + std::to_string(id) + string(".txt");

              gridPoint.printSolution(SolutionPath.c_str());
          }
          else
          {
              std::cout << "Primitive with id "<< id << " in group " << group << " failed " << std::endl;
          }
      }
      fclose(inFile);
    }
    
    return 0;
}
