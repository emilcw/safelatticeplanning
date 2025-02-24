#include "Trajectory.h"

void Trajectory::print(const char* fileName)
{
	FILE * outFile;
    outFile = fopen(fileName, "a");

    if(outFile == NULL)
    {
        std::cerr << "Error opening file: " << fileName << endl;
        exit(-1);
    }

    for (State s : stateList)
        fprintf(outFile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
        s.time,
        s.x,
        s.y,
        s.z,
        s.vx,
        s.vy,
        s.vz,
        s.roll,
        s.pitch,
        s.yaw,
        s.thrust);

	  fclose(outFile);
}

double Trajectory::getSquaredLength()
{
  double length = this->getLength();
  return length*length;
}

double Trajectory::getLength()
{
	// find lenght of input trajecory
  double length = 0;
  double lastX = this->stateList.begin()->x;
	double lastY = this->stateList.begin()->y;
  double lastZ = this->stateList.begin()->z;
	for (list<State>::iterator it = this->stateList.begin(); it != this->stateList.end(); ++it)
	{
        double curX = it->x;
        double curY = it->y;
        double curZ = it->z;
        length += sqrt((lastX - curX)*(lastX - curX) + (lastY - curY)*(lastY - curY) + (lastZ - curZ)*(lastZ - curZ));
            lastX = curX;
            lastY = curY;
        lastZ = curZ;
	}

    return length*length;
}
