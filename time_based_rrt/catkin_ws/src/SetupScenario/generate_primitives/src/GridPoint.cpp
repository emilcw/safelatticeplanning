#include "GridPoint.h"

void GridPoint::setInitialState(double vx, double vy, double vz, double x, double y, double z)
{
    this->initialState.vx = vx;
    this->initialState.vy = vy;
    this->initialState.vx = vz;
    this->initialState.x  = x;
    this->initialState.y  = y;
    this->initialState.z  = z;
}


void GridPoint::setFinalState(double vx, double vy, double vz, double x, double y, double z)
{
    this->finalState.vx = vx;
    this->finalState.vy = vy;
    this->finalState.vz = vz;
    this->finalState.x  = x;
    this->finalState.y  = y;
    this->finalState.z  = z;
}

void GridPoint::printSolution(const char* fileName)
{
    FILE * outFile;
    outFile = fopen(fileName, "w+");

    if(outFile == NULL)
    {
        std::cerr << "GridPoint::printSolution: Error opening file: " << fileName << endl;
        exit(-1);
    }

    fprintf(outFile, "Generated trajectory:\n");
    fprintf(outFile, "Group: %d \n",this->group);
    fprintf(outFile, "ID: %d \n", this->ID);
    fprintf(outFile, "Name: %s \n", this->name.c_str());
    fprintf(outFile, "finalTime: %lf  \n", this->trajectory.stateList.back().time);
    fprintf(outFile, "Cost: %lf  \n", this->cost);
    fprintf(outFile, "ToFromLabel: x y z vx vy vz \n");
    fprintf(outFile, "From: %lf %lf %lf %lf %lf %lf \n",
            this->trajectory.stateList.begin()->x,  this->trajectory.stateList.begin()->y,  this->trajectory.stateList.begin()->z,
            this->trajectory.stateList.begin()->vx, this->trajectory.stateList.begin()->vy, this->trajectory.stateList.begin()->vz);
    fprintf(outFile, "To: %lf %lf %lf %lf %lf %lf \n",  
            this->trajectory.stateList.back().x,  this->trajectory.stateList.back().y,  this->trajectory.stateList.back().z,
            this->trajectory.stateList.back().vx, this->trajectory.stateList.back().vy, this->trajectory.stateList.back().vz);
    fprintf(outFile , "Trajectory: \n");
    fprintf(outFile, "t        x        y         z        vx       vy       vz       roll     pitch     yaw     thrust \n");
    fclose(outFile);

    trajectory.print(fileName);
}


void GridPoint::setCost(double opt_cost){
    double timeDuration = this->trajectory.stateList.back().time;
    this->cost = opt_cost;
    double energy_cost = opt_cost - timeDuration;
    std::cout << "Cost for primtive " << this->ID << " in group " << this->group << " is " << this->cost << std::endl;
    std::cout << "Physical energy is: " << energy_cost  << " and timeCost " <<  opt_cost - energy_cost << std::endl;
    std::cout << "Primitive duration compared to guess: " << static_cast<int>(100 * (timeDuration / this->finalState.time)-1) << "%" << std::endl;
}






