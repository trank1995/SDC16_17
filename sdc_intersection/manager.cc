#include "manager.hh"

int manager::carAmt = 0;
std::vector<int> manager::carList = std::vector<int>();
manager::manager(int id){
    printf("created manager");
    printf("%d", id);
}

void manager:: printid(){
    printf("id is: ");
    printf("%d", id);
}

void manager::registerCar(int carId) {
    carAmt++;
    printf("registered, and now carAmt is: %i\n", carAmt);
    fflush(stdout);
    carList.push_back(carId);
    printf("registered car Id is: %i\n", carId);
    fflush(stdout);
    
}

bool manager::stopSignHandleRequest(int carId, double x, double y, int direction) {
    
    if (carAmt == 0) { //no car at intersection
        registerCar(carId);
        return true;
    }else if(carAmt > 0) { //one car
        return false;
    }
    return false;
}

void manager::stopSignCarLeft(int carId) {
    carList.erase(std::remove(carList.begin(), carList.end(), carId), carList.end());
    carAmt--;
}