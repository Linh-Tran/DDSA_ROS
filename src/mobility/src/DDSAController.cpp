#include "DDSAController.h"
#include <cmath>
#include <algorithm>

using namespace std;

DDSAController::DDSAController()
{
  step_length = 0.5;
  x = 0;
  y = 0;
}

DDSAController::DDSAController( int num_circuits, int num_robots, int robot_index )
{
  step_length = 0.25; // .5 meters
  x = 0;
  y = 0;
  generatePattern(num_circuits, num_robots, robot_index);
}

void DDSAController::setX(float x)
{
  this->x = x;
}

void DDSAController::setY(float y)
{
  this->y = y;
}

GoalState DDSAController::calcNextGoalState()
{
  GoalState gs;

  if (pattern.size() > 0) 
    {
        char curDir = pattern [pattern.size()-1];
	pattern.pop_back();
        switch(curDir)
        {
            case 'N':
                return getTargetN();
                break;
            case 'S':
                return getTargetS();
                break;
            case 'E':
                return getTargetE();
                break;
            case 'W':
                return getTargetW();
                break;
        }
    }
  else
    {
      gs.dir = '0';
      gs.x = '0';
      gs.y = '0';
      gs.yaw = '0';
    }

  return gs;
}

GoalState DDSAController::getTargetN()
{
  GoalState gs;
  gs.yaw = M_PI/2.0f;
  gs.x = x + (step_length*cos(gs.yaw));
  gs.y = y + (step_length*sin(gs.yaw));
  gs.dir = 'N';
  return gs;
}


GoalState DDSAController::getTargetE()
{
  GoalState gs;
  gs.yaw = 0.0;
  gs.x = x + (step_length*cos(gs.yaw));
  gs.y = y + (step_length*sin(gs.yaw));
  gs.dir = 'E';
  return gs;
}

GoalState DDSAController::getTargetS()
{
  GoalState gs;
  gs.yaw = -M_PI/2.0f;
  gs.x = x + (step_length*cos(gs.yaw));
  gs.y = y + (step_length*sin(gs.yaw));
  gs.dir = 'S';
  return gs;
}

GoalState DDSAController::getTargetW()
{
  GoalState gs;
  gs.yaw = M_PI;
  gs.x = x + (step_length*cos(gs.yaw));
  gs.y = y + (step_length*sin(gs.yaw));
  gs.dir = 'W';
  return gs;
}

void DDSAController::generatePattern(int N_circuits, int N_robots, int robot_index)
{  
    vector<string> paths;
    string ith_path;
   
    for (int i_robot = 1; i_robot <= N_robots; i_robot++)
    {
        for (int i_circuit = 0; i_circuit < N_circuits; i_circuit++)
        {
            int n_steps_north = calcDistanceTravel(i_robot, i_circuit, N_robots,'N');
            for (int j = 0; j < n_steps_north; j++)
            {
                ith_path += 'N';
            }

            int n_steps_east = calcDistanceTravel(i_robot, i_circuit, N_robots,'E');
            for (int j = 0; j < n_steps_east; j++)
            {
                ith_path += 'E';
            }

            int n_steps_south = calcDistanceTravel(i_robot, i_circuit, N_robots,'S');
            for (int j = 0; j < n_steps_south; j++)
            {
                ith_path += 'S';
            }

            int n_steps_west = calcDistanceTravel(i_robot, i_circuit, N_robots,'W');
            for (int j = 0; j < n_steps_west; j++)
            {
                ith_path += 'W';
            }
        }

        paths.push_back(ith_path);
        ith_path.clear();
    }	

    reversePattern(paths[robot_index]);
}

 // Calculate the edge length for this part of the spiral. The return value is the number of times to move north, south, etc.
int DDSAController::calcDistanceTravel (int i_robot, int i_circuit, int N_robots, char dir)
{
    int i = i_robot;
    int j = i_circuit;
    int N = N_robots;
    int n_steps = 0;

    if (dir == 'N' || dir == 'E')
    {
        if (j == 0)
        {
            n_steps = i;
            return n_steps;
        }
        else if (j == 1)
        {
            n_steps = calcDistanceTravel(i, j-1, N, dir) + i + N;
            return n_steps;
        }
        else 
        {
            n_steps = calcDistanceTravel(i, j-1, N, dir) + 2*N;
            return n_steps;
        }
    }

    else if (dir == 'S' || dir == 'W')
    {
        if (j == 0)
        {
            n_steps = calcDistanceTravel(i, j , N, 'N') + i;
            return n_steps;
        }

        else if (j > 0)
        {
            n_steps = calcDistanceTravel(i, j, N, 'N') + N;
            return n_steps;
        }
    }
    return 0;
}

// Reverses the pattern because push_back was used
void DDSAController::reversePattern (string ith_pattern)
{
    copy(ith_pattern.begin(), ith_pattern.end(), back_inserter(pattern));
    reverse(pattern.begin(), pattern.end());
}

string DDSAController::getPath()
{
  string path = "";
  for (int i = 0; i<pattern.size(); i++)
    {
      path += pattern[i];
    }

  return path;
}

DDSAController::~DDSAController()
{
}
