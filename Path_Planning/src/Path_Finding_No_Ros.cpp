#include <chrono>
#include <memory>
#include "Path_State.cpp"
#include "Temp_Map.cpp"
#include "Directions.cpp"
#include "Position.cpp"
#include "Astar.cpp"
#include "Print_Map.cpp"
#include "Map_Reading.cpp"
#include "Degree_State.cpp"
#include <deque>

using namespace std::chrono_literals;

// delete this comment, git was just being annoying and not letting me commit bc it said i made no changes

// Stores the map the starting position and heading
class Map {
  public:
  Position startingPos;
  Position endingPos;
  int starting_heading;
  //HardcodedMap map;
  RealMap* map = new RealMap("5_laps_test.yaml");

  //Constructor
  Map() {
    //Later on this should not be hardcoded and the map reading class will pass these values through
    startingPos.x = 100;
    startingPos.y = 32;
    endingPos.x = 90;
    endingPos.y = 32;
    //test end pos 40 , 70
    starting_heading = 0;
    
    map->setpos(startingPos);
  }
  private:

};

//Starting off this will be the class that creates the nodes and calls the algorithm that does the planning
class PathPlanning {
  public:
  Map map;
  Path_State Start_State;
  Path_State End_State;
  Astar aStar;

  //Constructor that creates start and end states
  PathPlanning() {
    Start_State.setValues(map.startingPos, map.starting_heading, map.map);
    End_State = Path_State(map.endingPos, map.starting_heading, map.map);  // end state is set differently for distance calculations
  }

  //This will call an algorithm class pass it through the starting node and the ending node
  // The algorithm class should be another file and it should return the path.
  void PlanPath() {
  //map.map->print_map();
  //cout << "Map: " << map.map->matrix[110][30];
  
    Path_State* path = aStar.Search(&Start_State, &End_State);
    path->PrintPosition();
    Path_State* parent = path->Parent;
    parent->PrintPosition();
    deque<Path_State*> directions_ = path->GetPathToState();
    PrintPathToState(directions_);

    map.map->print_map();
  }

  // Print the path back to the starting point 
  void PrintPathToState(deque<Path_State*> directions) {
    cout << endl;
    string direction;
    int x;
    int y;
    for (int i=0; i < directions.size(); i++) {

      directions[i]->PrintPosition();

      // prints '+' for perimeter around car
      // vector<Degree_State*> perimeter = directions[i]->degState->PerimeterStates();
      // for (int j=0; j < perimeter.size(); j++) {
      //   x = perimeter[j]->mx0;
      //   y = perimeter[j]->my0;
      //   // cout << "CORNER: (" << x << ", " << y << ")" << endl;
      //   map.map->ChangeMap(x, y, 5);
      // }

      vector<Path_State*> betweenStates = directions.at(i)->GetBetweenPathStates();
      for (int j=0; j < betweenStates.size(); j++) {
        x = betweenStates[j]->position.x;
        y = betweenStates[j]->position.y;
        // betweenStates[j]->PrintPosition(); // use this instead of directions[i]->PrintPosition() to print ALL states
        map.map->ChangeMap(x, y, 4);  // uses 'o' for between states so the calculated states are distinguishable
        if (j == betweenStates.size()-1) {
          map.map->ChangeMap(x, y, 2);  // prints '@' on map for state
        }
      }
    }
  }

  //Outputs a visual of the map at the start an on completion.
  // Dont know if it will work for larger maps
  void outputMap() {
  }
  private:
};


int main(int argc, char ** argv){

  // Everything in this function is for test purposes only!

    PathPlanning Planning;
    vector<Direction> directions;

  Planning.Start_State.PrintPosition();

  //cout << Planning.map.map->matrix[310][90] << endl;

  Planning.PlanPath();

  return 0;}