#include "manager.hh"

int manager::carAmt = 0;
int manager::currentDir = 0;
int manager::currentTurn = 0;
std::vector<int> manager::carList = std::vector<int>();
manager::manager(int id){
    printf("created manager");
    printf("%d", id);
}

void manager:: printid(){
    printf("id is: ");
    printf("%d", id);
}

void manager::registerCar(int carId, int turning, int direction) {
    carAmt++;
    currentDir = direction;
    currentTurn = turning;
    printf("registered, and now carAmt is: %i\n", carAmt);
    fflush(stdout);
    carList.push_back(carId);
    printf("registered car Id is: %i\n", carId);
    fflush(stdout);
    
}

bool manager::stopSignHandleRequest(int carId, int turning, int direction) {
//    printf("carAmt: %i\n", carAmt);
//    printf("turning: %i\n", turning);
    fflush(stdout);
    if (carAmt == 0) { //no car at intersection
        registerCar(carId, turning, direction);
        return true;
    }else if(carAmt == 1) { //one car already in intersection
        switch(currentTurn){
            //car in intersection is going straight
            case 0 :
                if (turning == 0 && (direction + 2)%4 == currentDir) {
                    registerCar(carId, turning, direction);
                    return true;
                }
                else if (turning == 2 && ((direction + 2)%4 == currentDir || (direction + 3)%4 == currentDir)) {
                    printf("other car turning right\n");
                    registerCar(carId, turning, direction);
                    return true;
                }
                else{
                    return false;
                }
                
            
            //car in intersection is going left
            case 1:
                return false;
                break;
            //car in intersection is going right
            case 2:
                if (turning == 1) {
                    return false;
                }else if (turning == 2) { //can turn if prev car turning right
                    registerCar(carId, turning, direction);
                    return true;
                }else{
                    if (!(direction == currentDir)) {
                        registerCar(carId, turning, direction);
                        return true;
                    }
                    return false;
                }
            case 3:
                return false;
        
        }
    }
    
    else { //more than 1 car
        return false;
    }
    return false;
}
/*
 turn right case:
 destdirection = 0
 if(!nextcar turn left && ! nextcar straight north):
 give reservation
 
 straight case:
 destdirection = 0
 if(nextcar straight 2 || nextcar right east || nextcar right south
 */

void manager::stopSignCarLeft(int carId) {
    carList.erase(std::remove(carList.begin(), carList.end(), carId), carList.end());
    carAmt--;
}