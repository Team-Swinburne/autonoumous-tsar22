#pragma once
#include <cstdio>
#include <iostream>
#include <vector>
#include "Position.cpp"

using namespace std;

// delete this comment, git was just being annoying and not letting me commit bc it said i made no changes

//Hardcoded map
class HardcodedMap
{
  public:
  Position pos;

  HardcodedMap(){
    pos.x = 5;
    pos.y = 8;
  }
  // 1 = traffic cone
  // 2 = starting point
  // 3 = end point
  // 0 = empty
  int map[10][10] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
    {1, 0, 0, 1, 1, 1, 1, 0, 0, 1}, 
    {1, 0, 0, 1, 1, 1, 1, 0, 0, 1}, 
    {1, 0, 0, 1, 1, 1, 1, 0, 0, 1}, 
    {1, 0, 0, 1, 1, 1, 1, 0, 0, 1}, 
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1}, 
    {1, 0, 0, 3, 0, 2, 0, 0, 0, 1}, 
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 
  };

  int GetMapValue(int x, int y) {
      return map[x][y];
  } 

  void ChangeMap(int change){
    int position[2];
    position[0] = pos.x;
    position[1] = pos.y;

    printf("X: %d, Y: %d\n", position[1], position[0]);
    switch(change) { 
        case 1:
          //printf("Up - GetPos\n");
          map[position[1]][position[0]] = 0;
          map[position[1]-1][position[0]] = 2;
          pos.y = pos.y - 1;
          break;
        case 2:
          //printf("Diagonal Up/Right - GetPos\n");
          map[position[1]][position[0]] = 0;
          map[position[1]-1][position[0]+1] = 2;
          pos.y = pos.y - 1;
          pos.x = pos.x + 1;
          break;
        case 3:
          //printf("Right - GetPos\n");
          map[position[1]][position[0]] = 0;
          map[position[1]][position[0]+1] = 2;
          pos.x = pos.x + 1;
          break;
        case 4:
          //printf("Diagonal Down/Right - GetPos\n");
          map[position[1]][position[0]] = 0;
          map[position[1]+1][position[0]+1] = 2;
          pos.y = pos.y + 1;
          pos.x = pos.x + 1;
          break;
        case 5:
          //printf("Down - GetPos\n");
          map[position[1]][position[0]] = 0;
          map[position[1]+1][position[0]] = 2;
          pos.y = pos.y + 1;
          break;
        case 6:
          //printf("Diagonal Down/Left - GetPos\n");
          map[position[1]][position[0]] = 0;
          map[position[1]+1][position[0]-1] = 2;
          pos.y = pos.y + 1;
          pos.x = pos.x - 1;
          break;
        case 7:
          //printf("Left - GetPos\n");
          map[position[1]][position[0]] = 0;
          map[position[1]][position[0]-1] = 2;
          pos.x = pos.x - 1;
          break;
        case 8:
          //printf("Diagonal Up/Left - GetPos\n");
          map[position[1]][position[0]] = 0;
          map[position[1]-1][position[0]-1] = 2;
          pos.y = pos.y - 1;
          pos.x = pos.x - 1;
          break;
      }
  }

  /*void GetPosition(int *position){

    for(int i = 0; i < 10; i++){
      for(int j = 0; j < 10; j++){
        if(map[i][j] == 2){
          position[0] = i;
          position[1] = j;
        }
      }
    }
  }*/

};