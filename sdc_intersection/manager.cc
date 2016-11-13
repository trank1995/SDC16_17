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