

#ifndef MANAGER_HH_
#define MANAGER_HH_

#include <iostream>
#include <vector>
#include "request.hh"
#include "instruction.hh"

class manager{
    public: 
        manager(int id);
        int id;
        void printid();
    static void registerCar(int carId, int turning, int direction);
        static bool stopSignHandleRequest(int carId, int turning, int direction);
        static void stopSignCarLeft(int carId);
    private:
        static int carAmt;
        static std::vector<int> carList;
        static int currentDir; //0 north, 1 east, 2 south, 3 west
        static int currentTurn; // 0 straight, 1 left, 2 right

    
};
#endif