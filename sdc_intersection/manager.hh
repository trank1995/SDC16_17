

#ifndef MANAGER_HH_
#define MANAGER_HH_

#include <iostream>

class manager{
    public: 
        manager(int id);
        int id;
        void printid();
        //static void registerCar(int carId);
    private:
        static int carAmt;
    
};
#endif