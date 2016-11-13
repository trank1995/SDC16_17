

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
        static void registerCar(int carId);
    private:
        static int carAmt;
        static std::vector<int> carList;
    
};
#endif