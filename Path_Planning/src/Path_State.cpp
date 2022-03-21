#pragma once
#include <cstdio>
#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <functional>
#include "Temp_Map.cpp"
#include "Directions.cpp"
#include "Position.cpp"
#include "Print_Map.cpp"
#include "Map_Reading.cpp"
#include "Degree_State.cpp"

using namespace std;

// delete this comment, git was just being annoying and not letting me commit bc it said i made no changes

class Path_State {
  public:
  Position position;
  int VehicleHeading;
  Path_State* Parent;
  Path_State* Children;
  float cost;
  float HeuristicValue;
  Direction DirectionFromParent;
  RealMap* map;
  Degree_State* degState;
  int PMoveDirection;
  

  Path_State() {}

  void setValues(Position pos, int heading, RealMap* amap) {
    Parent = NULL;
    PMoveDirection = 0;
    position = pos;
    VehicleHeading = heading;
    cost = 1;
    map = amap;
    degState = new Degree_State(position.x, position.y, heading);
  }

  //Constructor
  Path_State(Path_State* parent, int pMoveDirection, Position pos, double pHeading, RealMap* amap)
  {
    Parent = parent;
    Degree_State ParentDegState = *Parent->degState;
    pHeading = ParentDegState.aDeg;

    if ((337.5 < pHeading&& pHeading< 360) || (0 <= pHeading&& pHeading< 22.5)) {
        DirectionFromParent = Right;
    }
    else if (22.5 < pHeading&& pHeading< 67.5) {
        DirectionFromParent = UpRight;
    }
    else if (67.5 < pHeading&& pHeading< 112.5) {
        DirectionFromParent = Up;
    }
    else if (112.5 < pHeading&& pHeading< 157.5) {
        DirectionFromParent = UpLeft;
    }
    else if (157.5 < pHeading&& pHeading< 202.5) {
        DirectionFromParent = Left;
    }
    else if (202.5 < pHeading&& pHeading< 247.5) {
        DirectionFromParent = DownLeft;
    }
    else if (247.5 < pHeading&& pHeading< 292.5) {
        DirectionFromParent = Down;
    }
    else if (292.5 < pHeading&& pHeading< 337.5) {
        DirectionFromParent = UpLeft;
    }
    else {
      cout << "for some reason pHeading = " << pHeading << endl;
    }
    PMoveDirection = pMoveDirection;
    degState = ParentDegState.move(PMoveDirection);
    position = pos;
    VehicleHeading = degState->aDeg;
    cost = parent->cost + 1;
    HeuristicValue = 0;
    map = amap;
  }

  //This constructor is used for MiniGoals
  Path_State(Position pos, int heading, RealMap* amap) {
    Parent = NULL;
    position = pos;
    VehicleHeading = heading;
    cost = 1;
    map = amap;
    degState = new Degree_State(position.x, position.y, VehicleHeading, 550, 550);
  }

  void PrintPosition() {
    cout << "X: "<< position.x << " Y: " << position.y << " head: " << VehicleHeading << " x1: " << degState->x1 << " y1: " << degState->y1 << endl;
  }

  bool isMovePossible(int moveDirection) {
    Degree_State* possibleState = degState->move(moveDirection);
    int px = possibleState->mx;
    int py = possibleState->my;
    double pHeading = possibleState->aDeg;
    // cout << "checking " << moveDirection << endl;
    // cout << "turned " << moveDirection << ": px: " << px << ", py: " << py << ", pH: " << pHeading << ", px1: " << possibleState->x1 << ", py1: " << possibleState->y1 << endl;
    vector<Degree_State*> bStates = degState->GetBetweenStates(moveDirection);
    vector<Degree_State*> pCorners = possibleState->CornerPoints();
    vector<Degree_State*> pPerimeter = possibleState->PerimeterStates();

    if (px < 0 || px >= map->cols || py < 0 || py >= map->rows) {   // checks if possible pos is outside map boundaries
      return false;
    }
    if (map->matrix[py][px] == 1) {   // checks if possible pos is a wall
      return false;
    }

    for (int i = 0; i < bStates.size(); i++ ) {  // checks through each state between current state and move option
      int bx = bStates[i]->mx;
      int by = bStates[i]->my;     
      if (map->matrix[by][bx] == 1) {
        return false;
      }
    }

    // checks corner points of vehicle
    // for (int i = 0; i < pCorners.size(); i++) {   
    //   int cx = pCorners[i]->mx;
    //   int cy = pCorners[i]->my;
    //   if (cx < 0 || cx >= map->cols || cy < 0 || cy >= map->rows) {   // checks if corner is outside map boundaries
    //     return false;
    //   }
    //   if (map->matrix[cy][cx] == 1) { // checks if corner is in a wall
    //     return false;
    //   }
    // }

    // checks perimeter of rectangle encompassing vehicle
    // for (int i = 0; i < pPerimeter.size(); i++) {   
    //   int ppx = pPerimeter[i]->mx;
    //   int ppy = pPerimeter[i]->my;
    //   // TODO: REMOVE TESTING
    //   // if (position.x == 49 && position.y == 318) {
    //   //   cout << "checking pPerimeter " << i << "/" << pPerimeter.size()-1 << endl;
    //   // }
    //   if (map->matrix[ppy][ppx] == 1) {
    //     return false;
    //   }
    // }

    return true;  // if none of the above situations are true, the move is possible
  }

  // Gets possible moves and outputs them in a vector
  vector<int> GetPossibleMovements() { 
    vector<int> moveDirections;
    // actual car turning radius allows for max m value of 6, had to increase as it does not work with the current map
    for (int m = -12; m <= 12; m += 6) {
      // cout << "possible m = " << m << endl;
      if (isMovePossible(m)) {
        // cout << "m is possible" << endl;
        moveDirections.push_back(m);
      }
    }
    // sorts moveDirections favouring smaller and no turn
    sort(moveDirections.begin(), moveDirections.end(), [](int d1, int d2){ return abs(0-d1) < abs(0-d2); });
    // cout << "moveDirections size = " << moveDirections.size() << endl;
    return moveDirections;
  }

    //This function needs to create a new Path_State and return it based off the input direction. 
  Path_State* Move(int moveDirection) {
    Position newpos;
    double newheading;
    Degree_State* newDegState = degState->move(moveDirection);

    newpos.x = newDegState->mx;
    newpos.y = newDegState->my;
    newheading = newDegState->aDeg;
    //cout << newpos.x << " "<< newpos.y << endl;
    Path_State *path = new Path_State(this, moveDirection, newpos, newheading, map);
    //WALL CHECKER NOT USED
    //path->WallChecker();
    return path;
  }

  // Takes possible movements creates states for them and outputs them
  vector<Path_State*> Explore() {
    vector<int> moveDirections = GetPossibleMovements();
    vector<Path_State*> possiblemoves;

    if (moveDirections.size() < 1) {
      cout << "no possible moves" << endl;
    } 
    //BUG HERE when path states are added to the list they dont seem to be whats actually added
    for (int i = 0; i < moveDirections.size();i++)
    {
        possiblemoves.push_back(Move(moveDirections.at(i)));

    }
    return possiblemoves;
  }

  // Gets the path all the way back to the starting point 
  deque<Path_State*> GetPathToState() {

    deque<Path_State*> directions;

    if (Parent == NULL) {
      directions.push_back(this);
      return directions;
    }
    else {
      //cout << Parent << endl;
      deque<Path_State*> pathToParent = Parent->GetPathToState();
      
      for (int i=0; i < pathToParent.size(); i++) {
        directions.push_back(pathToParent.at(i));
      }
      directions.push_back(this);
      return directions;
    }
  }

  //Calculates the distance from input node and changes heuristic value
  void HeuristicCalc(Path_State* end) {
    float disttotal = DistanceFromPoint(end);
    HeuristicValue += disttotal;
    // cout << "Heuristic Value = " << HeuristicValue << endl;
  }

  //Calculates the distance current node is from the input point
  //returns the distance
  float DistanceFromPoint(Path_State* end) {
    float nodex = position.x + (fmod(degState->x1, 100)/100);
    float nodey = position.y + 1-(fmod(degState->y1, 100)/100);
    float distx = end->position.x + 0.5 - nodex;
    float disty = end->position.y + 0.5 - nodey;
    float disttotal = hypot(distx, disty);  // uses pythagoras to calculate distance
    float mindist;

    return disttotal;
  }

  // This checks how far a wall is and if it is further from walls the i value is higher which is good.
  // NOT CURRENTLY USED BUT MAY BE USEFUL IN FUTURE
  void WallChecker() {
    int tiles = 0;
    bool wall_found = false;
    int i = 1;
    int py = position.y;
    int px = position.x;
    while(!wall_found) {
    //UP
    if (map->matrix[py - i][px] == 1) {wall_found = true;}
    //DOWN
    if (map->matrix[py + i][px] == 1) {wall_found = true;}
    //LEFT
    if (map->matrix[py][px - i] == 1) {wall_found = true;}
    //RIGHT
    if (map->matrix[py][px + i] == 1) {wall_found = true;}
    //UP RIGHT
    if (map->matrix[py - i][px + i] == 1) {wall_found = true;}
    // UP LEFT
    if (map->matrix[py - i][px - i] == 1) {wall_found = true;}
    // DOWN RIGHT
    if (map->matrix[py + i][px + i] == 1) {wall_found = true;}
    // DOWN LEFT
    if (map->matrix[py + i][px - i] == 1) {wall_found = true;}

    i++;
    }
    HeuristicValue += i * -0.5;
  }

  vector<int> WallDistances() {
    bool upwall = false, leftwall = false, rightwall = false, downwall = false, 
      uprightwall = false, upleftwall = false, downrightwall = false, downleftwall = false;
    int upvalue = 0, leftvalue = 0, rightvalue = 0, downvalue = 0;
    int uprightvalue = 0, upleftvalue = 0, downrightvalue = 0, downleftvalue = 0;
    int i = 1;
    int py = position.y;
    int px = position.x;

    while(upwall == false || leftwall == false || rightwall == false || downwall == false || uprightwall == false ||
    upleftwall == false || downrightwall == false || downleftwall == false) {
      //UP
      if (upwall == false) {
        if (map->matrix[py - i][px] == 1) {upwall = true; upvalue = i;}
      }
      //DOWN
      if (downwall == false) {
        if (map->matrix[py + i][px] == 1) {downwall = true; downvalue = i;}
      }
      //LEFT
      if (leftwall == false) {
        if (map->matrix[py][px - i] == 1) {leftwall = true; leftvalue = i;}
      }
      //RIGHT
      if (rightwall == false) {
        if (map->matrix[py][px + i] == 1) {rightwall = true; rightvalue = i;}
      }
      //UP RIGHT
      if (uprightwall == false) {
        if (map->matrix[py - i][px + i] == 1) {uprightwall = true; uprightvalue = i;}
      }
      // UP LEFT
      if (upleftwall == false) {
        if (map->matrix[py - i][px - i] == 1) {upleftwall = true; upleftvalue = i;}
      }
      // DOWN RIGHT
      if (downrightwall == false) {
        if (map->matrix[py + i][px + i] == 1) {downrightwall = true; downrightvalue = i;}
      }
      // DOWN LEFT
      if (downleftwall == false) {
        if (map->matrix[py + i][px - i] == 1) {downleftwall = true; downleftvalue = i;}
      }
    //cout << i << ", ";
    //cout << endl;
    i++;
    }
    vector<int> wallDistances = {upvalue, uprightvalue, rightvalue, downrightvalue, downvalue, downleftvalue, leftvalue, upleftvalue};
    // cout << "UpValue: " << upvalue << endl;
    // cout << "downvalue: " << downvalue << endl;
    // cout << "leftvalue: " << leftvalue << endl;
    // cout << "rightvalue: " << rightvalue << endl; 
    // cout << "upright: " << uprightvalue << endl;
    // cout << "upleft: " << upleftvalue << endl;
    // cout << "downright: " << downrightvalue << endl;
    // cout << "downleft: " << downleftvalue << endl; 
    return wallDistances;
  }

  vector<Path_State*> GetBetweenPathStates() {
    vector<Path_State*> Between_States;
    vector<Degree_State*> BDegStates;
    if (Parent == NULL) {
      Between_States.push_back(this);
      return Between_States;
    }
    else {
      Degree_State* ParentDegState = Parent->degState;
      BDegStates = ParentDegState->GetBetweenStates(PMoveDirection);

      for (int i = 0; i < BDegStates.size(); i++) {
        Position newpos;
        newpos.x = BDegStates.at(i)->mx;
        newpos.y = BDegStates.at(i)->my;
        int newHeading = BDegStates.at(i)->aDeg;

        Path_State* ParentBetweenState = new Path_State(newpos, newHeading, map);
        Between_States.push_back(ParentBetweenState);
      }
      Between_States.push_back(this);
      return Between_States;
    }
  }

  // checks if a path state 'is equal' to another state, used to check if state is goal state while accounting for a range of angles
  bool Matches(Path_State* aPath_State) {
    vector<Path_State*> Between_States = GetBetweenPathStates();
    for (int i=0; i < Between_States.size(); i++) {
      Path_State* currentState = Between_States.at(i);
      if (currentState->position.x == aPath_State->position.x && currentState->position.y == aPath_State->position.y) {
        int cH = currentState->VehicleHeading;    // current heading value
        int eH = aPath_State->VehicleHeading;             // end heading value
        if (min(fmod(cH-eH+360, 360), fmod(eH-cH+360, 360)) < 45) { // have to add 360 because C++ mod doesn't work properly with negative numbers
          return true;
        }
      }
    }
    return false;
  }

  // used to check if two path states are exactly the same, used to check for already searched states in frontier
  bool Equals(Path_State* aPath_State) {
    if (position.x == aPath_State->position.x && position.y == aPath_State->position.y) {
        if (degState->x1 == aPath_State->degState->x1 && degState->y1 == aPath_State->degState->y1) {
          if (VehicleHeading == aPath_State->VehicleHeading) {
            return true;
          }
        }
      }
    return false;
  }

  private:

};