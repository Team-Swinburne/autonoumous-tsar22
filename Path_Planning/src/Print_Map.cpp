#pragma once
#include <cstdio>
#include <iostream>
#include <vector>
#include <deque>
#include "Temp_Map.cpp"

class Print_Map{
public:
HardcodedMap* map;

    Print_Map() {}

    Print_Map(HardcodedMap* pmap) {
        map = pmap;
    };

    void printmap(){
        for(int i = 0; i < 10; i++){
            for(int j = 0; j < 10; j++){
                printf("%d\t",map->map[i][j]);
            }
            printf("\n");
        }
        printf("\n");
    }
};