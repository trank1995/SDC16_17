#include "manager.hh"

int manager::carAmt = 0;
manager::manager(int id){
    printf("created manager");
    printf("%d", id);
}

void manager:: printid(){
    printf("id is: ");
    printf("%d", id);
}

/*void manager::registerCar(int carId) {
    carAmt++;
    printf("registered, and now carAmt is: %i\n", carAmt);
    fflush(stdout);
}*/