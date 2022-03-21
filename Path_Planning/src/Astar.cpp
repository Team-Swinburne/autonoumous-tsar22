#pragma once
#include <cstdio>
#include <iostream>
#include <vector>
#include <cmath>
#include "Path_State.cpp"
#include "Temp_Map.cpp"
#include "Directions.cpp"
#include "Position.cpp"
#include "Print_Map.cpp"

class Astar {
  public:
  int minigoalint, wallwidth, minigoals_to_calc = 0, minigoaltofollow = 0, loopcounter = 0;
  int mdistx = 0, mdisty = 0, mtotaldist = 0;
  vector<Path_State*> minigoallist;
  bool minigoal_reached = true, minigoalatend = false;



  // holds the main search loop
  Path_State* Search(Path_State *start, Path_State* end) {
    Path_State* StartState = start;
    Path_State* EndState = end;
    vector<Path_State*> newstates;

    cout << "Start Value:" << StartState << endl;

    // Calculates mini goals for the path finding to follow which is meant
    // to make it a lot more efficient
    if (minigoals_to_calc == 0) {
      cout << "first minigoal" << endl;
      Direction direction = TurnChecker(start);
      Path_State *minigoal = new Path_State();
      minigoal = MinigoalCalc(minigoalint, start, direction, 25);
      minigoallist.push_back(minigoal);
      minigoals_to_calc++;
    }
    // Mini goals 2-3 as they may have vision on goal state due to bad line drawing
    while (minigoals_to_calc < 3) {
      cout << "minigoal: " << minigoals_to_calc << endl;
      Direction direction = TurnChecker(minigoallist.at(minigoallist.size() - 1));
      Path_State *minigoal = new Path_State();
      minigoal = MinigoalCalc(minigoalint, minigoallist.at(minigoallist.size() - 1), direction, 25);
      minigoallist.push_back(minigoal);
      minigoals_to_calc++;
    }
    // all mini goals after the third
    while(minigoalatend != true) {
        cout << "minigoal: " << minigoals_to_calc << endl;
        Direction direction = TurnChecker(minigoallist.at(minigoallist.size() - 1));
        Path_State *minigoal = new Path_State();
        minigoal = MinigoalCalc(minigoalint, minigoallist.at(minigoallist.size() - 1), direction, 25);
        minigoalatend = IsGoalInSight(minigoal, end);
        minigoallist.push_back(minigoal);
        minigoals_to_calc++;
    }
    //Adds goal state to mini goal list
    if (minigoalatend == true) {
      minigoallist.push_back(end);
      end->map->ChangeMapWithPos(end->position);
    }
    
    cout << "BEFORE ADJUSTMENT" << endl;
    for (int i = 0; i < minigoallist.size();i++) {
      cout << minigoallist.at(i)->position.x << ", " << minigoallist.at(i)->position.y << ", "<< minigoallist.at(i)->VehicleHeading << endl;
    }


    // Prints mini goals positions
    cout << "AFTER ADJUSTMENT" << endl;
    minigoallist = MinigoalAdjustment(0, minigoallist);
    for (int i = 0; i < minigoallist.size();i++) {
      minigoallist[i]->map->ChangeMapWithPos(minigoallist[i]->position);
      cout << minigoallist.at(i)->position.x << ", " << minigoallist.at(i)->position.y << ", "<< minigoallist.at(i)->VehicleHeading << endl;
    }
    cout << endl;
    // start->map->print_map();
    // exit(EXIT_SUCCESS);

    int stateCheck = 0;
    AddToFrontier(StartState);
    Path_State* CurrentNode;
    vector<Path_State*> ReachedGoalNodes;
    vector<Path_State*> oldFrontier = Frontier; // frontier before clearing when goal reached
    vector<Path_State*> olderFrontier = Frontier; // frontier before clearing when goal reached
    int mindist = 90;
    Path_State* closestState;
    int failedgoal = 0;
    int furthestminigoal = 0;
    while(Frontier.size() > 0)
    {
      cout << endl;
      cout << "state check number " << stateCheck << endl;
      stateCheck++;
      CurrentNode = PopFrontier();
      CurrentNode->PrintPosition();
      
      //cout << "CurrentNode:" << CurrentNode << endl;
      cout << "CURRENT MINI GOAL: " << minigoaltofollow << " = (" 
      << minigoallist.at(minigoaltofollow)->position.x << ", " << minigoallist.at(minigoaltofollow)->position.y << ")" << endl;

      if (CurrentNode->Matches(EndState)) {
          cout << "Found solution";
          return CurrentNode;
      }
      else 
      {
        //This checks if current mini goal has been reached, if so next mini goal is followed
        mtotaldist = CurrentNode->DistanceFromPoint(minigoallist.at(minigoaltofollow));
        cout << "DISTANCE TO MINI GOAL: " << mtotaldist << endl;

        if (mtotaldist < 25) {
          if (minigoaltofollow < minigoallist.size() - 1) {
            minigoaltofollow++;
            olderFrontier = oldFrontier;
            oldFrontier = Frontier;
            Frontier.clear();
            cout << "NEXT MINI GOAL: " << minigoaltofollow << endl;
          }
        }
        newstates = CurrentNode->Explore();
        //cout << "\nNew States";
        //Printfrontier(newstates);
        for (int i = 0; i < newstates.size();i++) {
          if (!(newstates[i]->Matches(EndState))) { // if node is not the goal
              // Path_State* cmg = minigoallist.at(minigoaltofollow);
              // Path_State* nmg = minigoallist.at(minigoaltofollow+1);
              // /* if distance from current node to current minigoal is smaller than from next node to cmg, it is moving TOWARDS the minigoal
              //    if it's moving away from the goal, but also away from the next minigoaltofollow, it might just be readjusting itself
              //    if it's moving away from goal but is getting closer to nmg, it is moving PAST the minigoal, so do not add it to frontier
              //    UNUSED!!!
              // */
              // if (CurrentNode->DistanceFromPoint(cmg) >= newstates[i]->DistanceFromPoint(cmg)
              // || CurrentNode->DistanceFromPoint(nmg) <= newstates[i]->DistanceFromPoint(nmg)) {
            // cout << "Possible move: "; 
            // newstates.at(i)->PrintPosition();
            newstates.at(i)->HeuristicCalc(minigoallist.at(minigoaltofollow));
            AddToFrontier(newstates.at(i));
              // }

          }
          else {
            cout << "Found solution";
            return newstates.at(i);
          }
        }    
        SortFrontier();      
      }

      // attempted to implement backtracking for when the path gets stuck from a bad minigoal entry
      // if (Frontier.size() == 0) {
      //   cout << "\nNo Frontier" << endl;
      //   failedgoal++;
      //   minigoaltofollow -= 1;
      //   // minigoallist.erase(minigoallist.begin()+minigoaltofollow);  // removes previous minigoal and restarts from minigoal before that one
      //   Frontier = oldFrontier; // brings back frontier from before minigoal was reached
      // }
    }
    // Returns the node that reached the last goal
    cout << "\nNo Frontier" << endl;
    return ReachedGoalNodes[ReachedGoalNodes.size()-1];
  }

  private:
  vector<Path_State*> Frontier;
  vector<Path_State*> Searched;

  // This prints the Frontier or any vector input
  void Printfrontier(vector<Path_State*> frontier) {
    cout << "\n" << endl;
    for (int i = 0;i < frontier.size();i++) {
        cout << i <<" X: "<< frontier.at(i)->position.x << " Y: " << frontier.at(i)->position.y << " Heading: " << frontier.at(i)->VehicleHeading << " Heuristic: " << frontier.at(i)->HeuristicValue << endl;
    }
  }

  //This is used to create a mini goal and return it when given current node, direction and distance
  Path_State* MinigoalCalc(int posfinder, Path_State* currentNode, Direction dir, int distfromwall) {
    int x, y;
    Position pos;
    int posfind = posfinder - distfromwall;
    switch(dir) {
      case Up:
        x = currentNode->position.x;
        y = currentNode->position.y - posfind;
        pos.x = x;
        pos.y = y;
        break;
      case UpRight:
        x = currentNode->position.x  + posfind;
        y = currentNode->position.y - posfind;
        pos.x = x;
        pos.y = y;
        break;
      case Right:
        x = currentNode->position.x  + posfind;
        y = currentNode->position.y;
        pos.x = x;
        pos.y = y;
        break;
      case DownRight:
        x = currentNode->position.x  + posfind;
        y = currentNode->position.y + posfind;
        pos.x = x;
        pos.y = y;
        break;
      case Down:
        x = currentNode->position.x;
        y = currentNode->position.y + posfind;
        pos.x = x;
        pos.y = y;
        break;
      case DownLeft:
        x = currentNode->position.x  - posfind;
        y = currentNode->position.y + posfind;
        pos.x = x;
        pos.y = y;
        break;
      case Left:
        x = currentNode->position.x  - posfind;
        y = currentNode->position.y;
        pos.x = x;
        pos.y = y;
        break;
      case UpLeft:
        x = currentNode->position.x  - posfind;
        y = currentNode->position.y - posfind;
        pos.x = x;
        pos.y = y;
        break;
    
      default:
        break;
    }
    cout << "MINI GOAL" << currentNode->map->matrix[pos.y][pos.x] << endl;
    // currentNode->map->ChangeMapWithPos(pos);
    int heading = directionToAngle(dir);
    Path_State *minigoal = new Path_State(pos, heading, currentNode->map);
    return minigoal;
  }

  /* adjusts minigoals to be on opposite wall to current location - works fine but some minigoals shouldn't be adjusted and idk how to 
  get it to differentiate between the two, so for now it is only being used to remove close minigoals */

  vector<Path_State*> MinigoalAdjustment(int start, vector<Path_State*> aMinigoallist) {
    vector<Path_State*> adjustedlist = minigoallist;
  // vector<Path_State*> adjustedlist;
  //   for (int i = start; i < aMinigoallist.size()-1; i++) {
  //     // cout << "minigoal " << i;
  //     int cgx = aMinigoallist[i]->position.x;
  //     int cgy = aMinigoallist[i]->position.y;
  //     int agx, agy;
  //     // cout << ": (" << cgx << ", " << cgy << ")" << endl;

  //     vector<int> wallDistances = aMinigoallist[i]->WallDistances();
  //     double vertical = wallDistances[0] + wallDistances[4];   // distance between top and bottom walls
  //     double diagonalURDL = sqrt(2*pow(wallDistances[1], 2)) + sqrt(2*pow(wallDistances[5], 2)); // distance between up/right and down/left walls
  //     double horizontal = wallDistances[2] + wallDistances[6]; // distance between left and right walls
  //     double diagonalDRUL = sqrt(2*pow(wallDistances[3], 2)) + sqrt(2*pow(wallDistances[7], 2)); // distance between down/right and up/left walls

  //     // cout << "vertical: " << vertical << endl;
  //     // cout << "diagonalURDL: " << diagonalURDL << endl;
  //     // cout << "horizontal: " << horizontal << endl; 
  //     // cout << "diagonalDRUL: " << diagonalDRUL << endl;
  //     vector<double> widths = {vertical, diagonalURDL, horizontal, diagonalDRUL};
  //     double pathWidth = *min_element(widths.begin(), widths.end());
  //     // cout << "MIN = " << pathWidth << endl;
      
  //     if (pathWidth == vertical) {
  //       // cout << "vertical width" << endl;
  //       if (wallDistances[0] < wallDistances[4]) {  // if current goal is closer to top than bottom
  //         agx = cgx;
  //         agy = cgy + wallDistances[4] - 15;
  //       }
  //       else {  // current goal is closer to bottom than top
  //         agx = cgx;
  //         agy = cgy - wallDistances[0] + 15;
  //       }
  //     }
  //     else if (pathWidth == diagonalURDL) {
  //       // cout << "diagonal URDL / width" << endl;
  //       if (wallDistances[1] < wallDistances[5]) {  // if current goal is closer to top right than bottom left
  //         agx = cgx - wallDistances[5] + 15;
  //         agy = cgy + wallDistances[5] - 15;
  //       }
  //       else {  // current goal is closer to bottom left than top right
  //         agx = cgx + wallDistances[1] - 15;
  //         agy = cgy - wallDistances[1] + 15;
  //       }
  //     }
  //     else if (pathWidth == horizontal) { 
  //       // cout << "horizontal width" << endl;
  //       if (wallDistances[2] < wallDistances[6]) {  // if current goal is closer to right than left
  //         agx = cgx - wallDistances[6] + 15;
  //         agy = cgy;
  //       }
  //       else {  // current goal is closer to left than right
  //         agx = cgx + wallDistances[2] - 15;
  //         agy = cgy;
  //       }
  //     }
  //     else if (pathWidth == diagonalDRUL) {
  //       // cout << "diagonal DRUL \\ width" << endl;
  //       if (wallDistances[3] < wallDistances[7]) {  // if current goal is closer to bottom right than top left
  //         agx = cgx - wallDistances[7] + 15;
  //         agy = cgy - wallDistances[7] + 15;
  //       }
  //       else {  // current goal is closer to top left than bottom right
  //         agx = cgx + wallDistances[3] - 15;
  //         agy = cgy + wallDistances[3] - 15;
  //       }
  //     }
  //     else {
  //       cout << "ERROR: NO PATH WIDTH" << endl;
  //       break;
  //     }
  //     // cout << "agx = " << agx << ", agy = " << agy << endl;
  //     Position pos;
  //     pos.x = agx;
  //     pos.y = agy;
  //     Path_State* adjustedgoal = new Path_State(pos, minigoallist[i]->VehicleHeading, minigoallist[i]->map);
  //     adjustedlist.push_back(adjustedgoal);

  //     // minigoallist[i]->map->ChangeMap(cgx, cgy, 0);
  //     // adjustedgoal->map->ChangeMapWithPos(adjustedgoal->position);
  //   }
    // adjustedlist.push_back(minigoallist[minigoallist.size()-1]);
    // g1 and g2 stuff just for testing
    int g1 = 0;
    int g2 = 1;
    // checks distance between current and next minigoal
    for (int i = 0; i < adjustedlist.size()-1; i++) {  
      cout << "goal " << g1 << " to goal " << g2 << " distance = " << adjustedlist[i]->DistanceFromPoint(adjustedlist[i+1]) << endl;
      // if distance less than 50, removes next minigoal unless it is end goal
      if (adjustedlist[i]->DistanceFromPoint(adjustedlist[i+1]) < 50) {
        if (i < adjustedlist.size()-2) { adjustedlist.erase(adjustedlist.begin()+i+1); }
        else { adjustedlist.erase(adjustedlist.begin()+i); }    // if end goal is too close, remove current minigoal
        i--;
      }
      else {
        g1 = g2;
        adjustedlist[i]->map->ChangeMapWithPos(adjustedlist[i]->position);
      }
      g2++;
    }
    return adjustedlist;
  }

  //Checks if the goal is in sight, any walls in between
  // This may need to draw straighter lines its not working that well atm
  bool IsGoalInSight(Path_State* currentNode, Path_State* goalNode) {
    int xdist, ydist;
    Position travelling;
    travelling.x = currentNode->position.x;
    travelling.y = currentNode->position.y;
    xdist = goalNode->position.x - currentNode->position.x;
    ydist = goalNode->position.y - currentNode->position.y;

    //cout << "Mini Goal Pos: " << currentNode->position.x << ", " << currentNode->position.y << endl;

    //cout << "Mini Goal Pos: " << goalNode->position.x << ", " << goalNode->position.y << endl;

    // cout << "Diff: " << xdist << ", " << ydist << endl; 

    while (travelling.x != goalNode->position.x || travelling.y != goalNode->position.y) {
      // X Check
      if (travelling.x != goalNode->position.x) {
        if (xdist < 0) {
          travelling.x -= 1;
        } else if (xdist > 0) {
          travelling.x += 1;
        }
        //cout << "Point: " << travelling.x << ", " << travelling.y << endl;
        // cout << "Goal: " << goalNode->map->matrix[travelling.y][travelling.x] << endl;
        if (goalNode->map->matrix[travelling.y][travelling.x] == 1) {
          return false;
        }
      }
      // Y check
      if (travelling.y != goalNode->position.y) {
        if (ydist < 0) {
          travelling.y -= 1;
        } else if (ydist > 0) {
          travelling.y += 1;
        }
        // cout << "Point: " << travelling.x << ", " << travelling.y << endl;
        // cout << "Goal: " << goalNode->map->matrix[travelling.y][travelling.x] << endl;
          if (goalNode->map->matrix[travelling.y][travelling.x] == 1) {
          return false;
        }
      }
    }
    // cout << "GOAL IN SIGHT: " << travelling.x << ", " << travelling.y << endl;
    return true;
  }

  // Transfers directions to angles, is used to create mini goals
  int directionToAngle(Direction dir) {
    switch(dir) {
      case Up:
        return 90;
        break;
      case UpRight:
        return 45;
        break;
      case Right:
        return 0;
        break;
      case DownRight:
        return 315;
        break;
      case Down:
        return 270;
        break;
      case DownLeft:
        return 225;
        break;
      case Left:
        return 180;
        break;
      case UpLeft:
        return 135;
        break;
    }
  }

  // Sorts the Frontier and puts the lowest heuristics at the front
  void SortFrontier() {
    //  cout << "Unsorted: " << endl;
    //  Printfrontier(Frontier);

    sort(Frontier.begin(), Frontier.end(), [](Path_State* a, Path_State* b){ return a->HeuristicValue < b->HeuristicValue; });

    //  cout << "sorted: " << endl;
    //  Printfrontier(Frontier);

  }

  //takes out of frontier to search next element, add it to searched as well
  Path_State* PopFrontier() {
    Path_State* state = Frontier.at(0);
    //cout << "\nPop Frontier Start";
    //Printfrontier(Frontier);

    state = Frontier.at(0);
    Frontier.erase(Frontier.begin());

    //cout << "Pop Frontier End\n";
    //Printfrontier(Frontier);
    Searched.push_back(state);

    return state;
  }

  //Checks search list and adds to frontier
  void AddToFrontier(Path_State* state) {
    for (int i = 0; i < Searched.size();i++) {
      // if ((state->position.x == 59 && state->position.y == 309) || (state->position.x == 26 && state->position.y == 297)) {
      //   cout << i << ": (" << Searched[i]->position.x << ", " << Searched[i]->position.y << "), H = " << Searched[i]->HeuristicValue << endl;
      // }
      if (Searched.at(i)->Equals(state)) {
        cout << "Already searched: (" << Searched.at(i)->position.x << ", " << Searched.at(i)->position.y << ")" << endl;
        return;
      }
    }
    Frontier.push_back(state);
  }

    // This is used to calculate the mini goals
    // It pretty much checks for the furthest wall and returns the direction its in and how far.
  Direction TurnChecker(Path_State* currentnode) {
    bool upwall = false;
    bool leftwall = false;
    bool rightwall = false;
    bool downwall = false;
    bool uprightwall = false;
    bool upleftwall = false;
    bool downrightwall = false;
    bool downleftwall = false;
    int upvalue = 0, leftvalue = 0, rightvalue = 0, downvalue = 0;
    int uprightvalue = 0, upleftvalue = 0, downrightvalue = 0, downleftvalue = 0;
    int i = 1;
    int py = currentnode->position.y;
    int px = currentnode->position.x;

    switch (currentnode->VehicleHeading) {
      case 90:
        downwall = true;
        downleftwall = true;
        downrightwall = true;
        break;
      case 45:
        downleftwall = true;
        downwall = true;
        leftwall = true;
        break;
      case 0:
        leftwall = true;
        upleftwall = true;
        downleftwall = true;
        break;
      case 315:
        upleftwall = true;
        upwall = true;
        leftwall = true;
        break;
      case 270:
        upwall = true;
        upleftwall = true;
        uprightwall = true;
        break;
      case 225:
        uprightwall = true;
        upwall = true;
        rightwall = true;
        break;
      case 180:
        rightwall = true;
        uprightwall = true;
        downrightwall = true;
        break;
      case 135:
        downrightwall = true;
        rightwall = true;
        downwall = true;
        break;
      
      default:
        break;
    }

    while(upwall == false || leftwall == false || rightwall == false || downwall == false || uprightwall == false ||
    upleftwall == false || downrightwall == false || downleftwall == false) {
      //UP
      if (upwall == false) {
        if (currentnode->map->matrix[py - i][px] == 1) {upwall = true; upvalue = i;}
      }
      //DOWN
      if (downwall == false) {
        if (currentnode->map->matrix[py + i][px] == 1) {downwall = true; downvalue = i;}
      }
      //LEFT
      if (leftwall == false) {
        if (currentnode->map->matrix[py][px - i] == 1) {leftwall = true; leftvalue = i;}
      }
      //RIGHT
      if (rightwall == false) {
        if (currentnode->map->matrix[py][px + i] == 1) {rightwall = true; rightvalue = i;}
      }
      //UP RIGHT
      if (uprightwall == false) {
        if (currentnode->map->matrix[py - i][px + i] == 1) {uprightwall = true; uprightvalue = i;}
      }
      // UP LEFT
      if (upleftwall == false) {
        if (currentnode->map->matrix[py - i][px - i] == 1) {upleftwall = true; upleftvalue = i;}
      }
      // DOWN RIGHT
      if (downrightwall == false) {
        if (currentnode->map->matrix[py + i][px + i] == 1) {downrightwall = true; downrightvalue = i;}
      }
      // DOWN LEFT
      if (downleftwall == false) {
        if (currentnode->map->matrix[py + i][px - i] == 1) {downleftwall = true; downleftvalue = i;}
      }

    //cout << i << ", ";
    //cout << endl;
    i++;
    }
    cout << "UpValue: " << upvalue << endl;
    cout << "downvalue: " << downvalue << endl;
    cout << "leftvalue: " << leftvalue << endl;
    cout << "rightvalue: " << rightvalue << endl; 
    cout << "upright: " << uprightvalue << endl;
    cout << "upleft: " << upleftvalue << endl;
    cout << "downright: " << downrightvalue << endl;
    cout << "downleft: " << downleftvalue << endl; 
    minigoalint = i;

    vector<double> dOptions = {(double)upvalue, (double)downvalue, (double)leftvalue, (double)rightvalue, 
      sqrt(2*pow(uprightvalue, 2)), sqrt(2*pow(upleftvalue, 2)), sqrt(2*pow(downrightvalue, 2)), sqrt(2*pow(downleftvalue, 2))} ;

    double maxDistance = *max_element(dOptions.begin(), dOptions.end());
    if (maxDistance == upvalue) { cout << "Favouring UP" << endl; return Up; }
    if (maxDistance == downvalue) { cout << "Favouring DOWN" << endl; return Down; }
    if (maxDistance == rightvalue) { cout << "Favouring RIGHT" << endl; return Right; }
    if (maxDistance == leftvalue) { cout << "Favouring LEFT" << endl; return Left; }
    if (maxDistance == sqrt(2*pow(upleftvalue, 2))) { cout << "Favouring UP LEFT" << endl; return UpLeft; }
    if (maxDistance == sqrt(2*pow(uprightvalue, 2))) { cout << "Favouring UP RIGHT" << endl; return UpRight; }
    if (maxDistance == sqrt(2*pow(downleftvalue, 2))) { cout << "Favouring DOWN LEFT" << endl; return DownLeft; }
    if (maxDistance == sqrt(2*pow(downrightvalue, 2))) { cout << "Favouring DOWN RIGHT" << endl; return DownRight; }
    else { cout << "DEFAULT" << endl; return Right; }
  }
};