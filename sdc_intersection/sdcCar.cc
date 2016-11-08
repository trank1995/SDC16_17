/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


/*
 * Based on UtilityCart.cc written by Nate Koenig, sdcCar provides both
 * interaction with Gazebo's simulation environment as well as logic to
 * make it behave intelligently in a variety of situations. This is the main
 * class used in the Self-Driving Comps project.
 *
 * Physics parameters and Gazebo interfacing are based on UtilityCart.
 */

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "sdcCar.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(sdcCar)

// SDC-defined constants
const double PI = 3.14159265359;

const double DIRECTION_MARGIN_OF_ERROR = 0.00855;
const double STEERING_MARGIN_OF_ERROR = 0.05;
const int LIDAR_DETECTION_MARGIN_OF_ERROR = 2;

// How fast the car turns each update
const double STEERING_ADJUSTMENT_RATE = 0.02;

// How much we can turn the "steering wheel"
const double STEERING_RANGE = 5 * PI;

const double CAR_WIDTH = 0.8;
const double CAR_LENGTH = 2.0;

// The width of the channel in front of the car for which we count objects as
// being directly in front of the car
const double FRONT_OBJECT_COLLISION_WIDTH = CAR_WIDTH + 0.5;

const sdcAngle NORTH = sdcAngle(PI/2);
const sdcAngle SOUTH = sdcAngle(3*PI/2);
const sdcAngle EAST = sdcAngle(0);
const sdcAngle WEST = sdcAngle(PI);

//dijkstra's stuff
std::vector<int> unvisited;
std::vector<sdcIntersection> intersections;
const int size = 5;
const std::pair<double,double> destination = {48,10};

std::vector<sdcWaypoint> WAYPOINT_VEC;


////////////////////////////////
////////////////////////////////
// BEGIN THE BRAIN OF THE CAR //
////////////////////////////////
////////////////////////////////

/*
 * Handles all logic for driving, is called every time the car receives an update
 * request from Gazebo
 */
void sdcCar::Drive()
{
    

//     If not in avoidance, check if we should start following the thing
//     in front of us. If following is done, kick out to default state
//    if(this->currentState != intersection && this->currentState != avoidance){
//        // If there's a stop sign, assume we're at an intersection
//        if(this->ignoreStopSignsCounter == 0 && sdcSensorData::stopSignFrameCount > 5){
//            this->currentState = intersection;
//        }
//
//        // If something is ahead of us, default to trying to follow it
//        if (this->ObjectDirectlyAhead()){
//            this->currentState = follow;
//        }else if(this->currentState == follow && !this->isTrackingObject){
//            this->currentState = DEFAULT_STATE;;
//        }
//
//        // Look for objects in danger of colliding with us, react appropriately
//        if (this->ObjectOnCollisionCourse()){
//            this->currentState = avoidance;
//        }
//    }

    this->ignoreStopSignsCounter = fmax(this->ignoreStopSignsCounter - 1, 0);


    // Possible states: stop, waypoint, intersection, follow, avoidance
    
    switch(this->currentState)
    {
        // Final state, car is finished driving
          
        case stop:
            //printf("in stop\n");
            this->Stop();
        break;

        // Default state; drive straight to target location
        case waypoint:
            
        // Handle lane driving

//          this->Accelerate();
        //  this->Stop();

            this->WaypointDriving(WAYPOINT_VEC);
        break;

        // At a stop sign, performing a turn
        case intersection:
           
            if(this->stoppedAtSign && this->stationaryCount > 2000){
                this->currentState = DEFAULT_STATE;
                this->ignoreStopSignsCounter = 3000;
            }else if(this->stoppedAtSign && this->GetSpeed() < 0.5){
                this->stationaryCount++;
            }else if(!this->stoppedAtSign && this->sensorData.sizeOfStopSign > 6000){
                this->Stop();
                this->stoppedAtSign = true;
                this->stationaryCount = 0;
        }

        break;

        // Follows object that is going in same direction/towards same target
        case follow:
            //this->Follow();
        // Handle lane driving
        break;

        // Smarter way to avoid objects; stopping, swerving, etc.
        case avoidance:
        // Cases: stop, swerve, go around
            //this->Avoidance();
        break;

        // Parks the car
        case parking:
            //this->PerpendicularPark();
            // this->ParallelPark();
        break;
    }

    // Attempts to turn towards the target direction
    this->MatchTargetDirection();
    // Attempts to match the target speed
    this->MatchTargetSpeed();
}

/*
 * Handles turning based on the value of targetDirection. Calculates both which direction
 * to turn and by how much, as well as turning the actual wheel
 */
void sdcCar::MatchTargetDirection(){
    sdcAngle directionAngleChange = this->GetDirection() - this->targetDirection;
    // If the car needs to turn, set the target steering amount
    if (!directionAngleChange.WithinMargin(DIRECTION_MARGIN_OF_ERROR)) {
        // The steering amount scales based on how far we have to turn, with upper and lower limits
        double proposedSteeringAmount = fmax(fmin(-this->turningLimit*tan(directionAngleChange.angle/-2), this->turningLimit), -this->turningLimit);

        // When reversing, steering directions are inverted
        if(!this->reversing){
            this->SetTargetSteeringAmount(proposedSteeringAmount);
        }else{
            this->SetTargetSteeringAmount(-proposedSteeringAmount);
        }
    }

    // Check if the car needs to steer, and apply a small turn in the corresponding direction
    if (!(std::abs(this->targetSteeringAmount - this->steeringAmount) < STEERING_MARGIN_OF_ERROR)) {
        if (this->steeringAmount < this->targetSteeringAmount) {
            this->steeringAmount = this->steeringAmount + STEERING_ADJUSTMENT_RATE;
        }else{
            this->steeringAmount = this->steeringAmount - STEERING_ADJUSTMENT_RATE;
        }
    }
}

/*
 * Attempts to match the current target speed
 */
void sdcCar::MatchTargetSpeed(){
    // Invert all the values if the car should be moving backwards
    int dirConst = this->reversing ? -1 : 1;

    // If the car is moving the wrong direction or slower than the target speed, press on the gas
    if((this->reversing && this->IsMovingForwards()) || (!this->reversing && !this->IsMovingForwards()) || (this->GetSpeed() < this->targetSpeed)){
        this->gas = 1.0 * dirConst;
        this->brake = 0.0;
    } else if(this->GetSpeed() > this->targetSpeed){
        // If the car is moving faster than the target speed, brake to slow down
        this->gas = 0.0;
        if(this->reversing != this->IsMovingForwards()){
            this->brake = -2.0 * dirConst;
        } else {
            // If the car is drifting in the opposite direction it should be, don't brake
            // as this has the side effect of accelerating the car in the opposite direction
            this->brake = 0.0;
        }
    }
}

/*
 * Drive from point to point in the given list
 */
void sdcCar::WaypointDriving(std::vector<sdcWaypoint> WAYPOINT_VEC) {
    int progress = this->waypointProgress;
    if(progress < WAYPOINT_VEC.size()){
        // Pull the next waypoint and set the car to drive towards it

        //printf("waypointvec.size: %i", WAYPOINT_VEC.size());
        this->Accelerate();

        // Check if the car is close enough to the target to move on
        double distance = sqrt(pow(WAYPOINT_VEC[progress].pos.first - this->x,2) + pow(WAYPOINT_VEC[progress].pos.second - this->y,2));
        //printf("distance %f", distance);
        
        // CODE FROM LAST GROUP THAT ASSUMES THAT THE CAR WILL TURN ONCE WE HAVE REACHED AN INTERSECTION
        if (distance < 15) {
            this->turning = true;
        }
        if(this->turning == true){
            this->SetTurningLimit(20);
            GridTurning(WAYPOINT_VEC[progress].waypointType);
        } else {
            math::Vector2d nextTarget = {WAYPOINT_VEC[progress].pos.first,WAYPOINT_VEC[progress].pos.second};
            //printf("first: %f second: %f", WAYPOINT_VEC[progress].pos.first, WAYPOINT_VEC[progress].pos.second);
            sdcAngle targetAngle = AngleToTarget(nextTarget);
            this->SetTargetDirection(targetAngle);
            // this->LanedDriving();
        }
    } else if(this->isFixingParking){
        this->isFixingParking = false;
        
        this->currentState = parking;
        this->currentPerpendicularState = straightPark;
    } else {
   
        this->currentState = stop;
    }
}

/*
 * Uses camera data to detect lanes and sets targetDirection to stay as close
 * as possible to the midpoint.
 */
void sdcCar::LanedDriving() {
    int lanePos = this->sensorData.LanePosition();
    this->SetTurningLimit(this->sensorData.GetNewSteeringMagnitude());
    if (!(lanePos > 320 || lanePos < -320)) {
        // It's beautiful don't question it
        sdcAngle laneWeight = sdcAngle(tan(lanePos/(PI*66.19))/10);
        this->SetTargetDirection(this->GetDirection() + laneWeight);
    }
}





/*
 * Executes a turn at an intersection
 */
void sdcCar::GridTurning(int turn){
    int progress = this->waypointProgress;
    if(turn == 3){
        this->waypointProgress++;

        this->currentState = stop;
        return;
    } else if (turn == 0){
        this->waypointProgress++;
        this->turning = false;
        return;
    }
    math::Vector2d nextTarget = {WAYPOINT_VEC[progress+1].pos.first,WAYPOINT_VEC[progress+1].pos.second};
    sdcAngle targetAngle = AngleToTarget(nextTarget);
    this->SetTargetDirection(targetAngle);
    sdcAngle margin = this->GetOrientation().FindMargin(targetAngle);
    if(margin < .1 && margin > -.1){
        this->turning = false;
        this->waypointProgress++;
    }
}





//////////////////////
// DIJKSTRA METHODS //
//////////////////////

//Generates a series of waypoints to get to the desired destination
void sdcCar::GenerateWaypoints(){
    printf("in generateWaypoints\n");
    GetNSEW();
    initializeGraph();
    const int start = getFirstIntersection();
    printf("got first intersection\n");
    int dest;
    for(int i = 0; i < intersections.size(); ++i){
        if(intersections[i].waypoint.pos.first == destination.first && intersections[i].waypoint.pos.second == destination.second)
            dest = i;
    }
    printf("set dest\n");
    std::vector<int> path;
    removeStartingEdge(start);
    printf("remove starting edge\n");
    fflush(stdout);
    path = dijkstras(start, dest);
    printf("did dijtkstras\n");
    printf ("path.size: %lu", path.size());
    insertWaypointTypes(path, this->currentDir);
    printf("insertedwaypointtypes\n");
    for (int i = path.size()-1; i >=0; --i){
        WAYPOINT_VEC.push_back(intersections[path[i]].waypoint);
        printf("waypoint: %f", intersections[path[i]].waypoint.pos.second);
        printf("waypoint: %f", intersections[path[i]].waypoint.pos.first);
        fflush(stdout);
    }
    printf("end of genwaypoints\n");
}

void sdcCar::removeStartingEdge(int start){
    printf("removing first edge\n");
    Direction dir = east;
    switch (dir) {
        case north:
            for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
                if (intersections[start].neighbors_pairs[n].first == start - 1)
                    intersections[start].neighbors_pairs[n].second =
                    std::numeric_limits<double>::infinity();
            }
            break;
        case south:
            for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
                if (intersections[start].neighbors_pairs[n].first == start + 1)
                    intersections[start].neighbors_pairs[n].second =
                    std::numeric_limits<double>::infinity();
            }
            break;
        case east:
            printf("in the default case. start: %i",start);
            fflush(stdout);
            for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
                printf("outside if. n: %i", n);
                if (intersections[start].neighbors_pairs[n].first == start - size){
                    intersections[start].neighbors_pairs[n].second =
                    std::numeric_limits<double>::infinity();
                    printf("inside if. n: %i", n);
                    fflush(stdout);
                }
            }
            printf("out of for\n");
            fflush(stdout);
            break;
        case west:
            for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
                if (intersections[start].neighbors_pairs[n].first == start + size)
                    intersections[start].neighbors_pairs[n].second =
                    std::numeric_limits<double>::infinity();
            }
            break;
    }
}

std::vector<int> sdcCar::dijkstras(int start, int dest) {
    std::vector<int> path;
    int current;
    intersections[start].dist = 0;
    intersections[start].previous = -1;
    double distance;

    // initializes the unvisited list by placing all of start's neighbors in it
    for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
    // push back each neighbor of the start into unvisited
    unvisited.push_back(intersections[start].neighbors_pairs[n].first);
    // set the distance of each neighbor to the distance of the edge
    // from start to neighbor and make neighbor previous = start
    intersections[intersections[start].neighbors_pairs[n].first].dist =
        intersections[start].neighbors_pairs[n].second;
    intersections[intersections[start].neighbors_pairs[n].first].previous =
        intersections[start].place;
    }
    
    printf("before while");

    // BFS using the unvisted FI FO vector, if unvisited is 0 then we have
    // visited all intersections
    while (unvisited.size() != 0) {
        current = unvisited[0];
        for (int n = 0; n < intersections[current].neighbors_pairs.size(); ++n) {
      // distance to the neighbor from current intersection
            distance = intersections[current].neighbors_pairs[n].second;
      // if the distance of the current intersection + the distance from
      // the current intersection to neighbor is smaller than the distance
      // to neighbor, update distance and previous
            if (intersections[intersections[current].neighbors_pairs[n].first].dist >
                intersections[current].dist + distance) {
        // update distance
                intersections[intersections[current].neighbors_pairs[n].first].dist =
                intersections[current].dist + distance;
        // update previous
                intersections[intersections[current].neighbors_pairs[n].first]
                .previous = intersections[current].place;
            }
      // if the neighbor has not been visited then push back into unvisited
            if (intersections[intersections[current].neighbors_pairs[n].first]
                .visited == 0) {
        // push back neighbor into unvisited
                unvisited.push_back(intersections[current].neighbors_pairs[n].first);
            }
      // mark the current intersection as visited
            intersections[current].visited = 1;
        }
        //pop front
        unvisited.erase(unvisited.begin());
    }
    printf("after while");
    //crawl backwards from dest to start to get the path
//    for (int i = intersections[dest].place; i != -1;) {
//    path.push_back(i);
//    i = intersections[i].previous;
//    }
    path.push_back(intersections[0].place);
    return path;
}
void sdcCar::initializeGraph() {
    //make the sdcIntersections
    printf("making graph\n");
    sdcIntersection aa;
    aa.place = 0;

    aa.waypoint = sdcWaypoint(0,std::pair<double,double>(48, 10));
    aa.wpType = WaypointType_Stop;


    //make the distance to all intersections infinity
    intersections = {aa};
    for (int i = 0; i < intersections.size(); ++i) {
        intersections[i].dist = std::numeric_limits<double>::infinity();
        intersections[i].place = i;
    }
    printf("end of graph\n");
}

int sdcCar::getFirstIntersection(){
//    std::pair<double,double> firstIntr;
//    int firstIntersection;
//
//    switch(this->currentDir){
//
//        case west:
//            firstIntr = {-1000,0};
//            for(int i = 0; i < intersections.size();++i){
//                if(this->y < intersections[i].waypoint.pos.second+5 && this->y > intersections[i].waypoint.pos.second-5 && intersections[i].waypoint.pos.first < this->x - 10 && intersections[i].waypoint.pos.first > firstIntr.first)
//                    firstIntr = intersections[i].waypoint.pos;
//            }
//            break;
//
//        case east:
//            firstIntr = {1000,0};
//            for(int i = 0; i < intersections.size();++i){
//                if(this->y < intersections[i].waypoint.pos.second+5 && this->y > intersections[i].waypoint.pos.second-5 && intersections[i].waypoint.pos.first > this->x + 10 && intersections[i].waypoint.pos.first < firstIntr.first){
//                    firstIntr = intersections[i].waypoint.pos;
//                }
//            }
//            break;
//
//        case north:
//            firstIntr = {0,1000};
//            for(int i = 0; i < intersections.size();++i){
//                if(this->x < intersections[i].waypoint.pos.first+5 && this->x > intersections[i].waypoint.pos.first-5 && intersections[i].waypoint.pos.second > this->y + 10 && intersections[i].waypoint.pos.second < firstIntr.second)
//                    firstIntr = intersections[i].waypoint.pos;
//            }
//            break;
//
//        case south:
//            firstIntr = {0,-1000};
//            for(int i = 0; i < intersections.size();++i){
//                if(this->x < intersections[i].waypoint.pos.first+5 && this->x > intersections[i].waypoint.pos.first-5 && intersections[i].waypoint.pos.second < this->y - 10 && intersections[i].waypoint.pos.second > firstIntr.second)
//                    firstIntr = intersections[i].waypoint.pos;
//            }
//            break;
//    }
//    printf("size of intrs: %i",intersections.size());
//    fflush(stdout);
//    for(int i = 0; i < intersections.size();i++){
//        printf("i outside: %i", i);
//        fflush(stdout);
//        if(firstIntr.first == intersections[i].waypoint.pos.first && firstIntr.second == intersections[i].waypoint.pos.second){
//            firstIntersection = i;
//            printf("i inside: %i", i);
//            fflush(stdout);
//            break;
//        }
//    }
    return 0;
}

void sdcCar::insertWaypointTypes(std::vector<int> path, Direction startDir) {
  Direction curDir = startDir;
  Direction nextDir;
  int current;
  int next;
  // get the direction the car heads in from the current intersection to
  // the next one
  for (int i = path.size() - 1; i > 0; i--) {
    current = path[i];
    next = path[i - 1];
    if (next - current == size) {
      nextDir = east;
    } else if (current - next == size) {
      nextDir = west;
    } else if (next - current == 1) {
      nextDir = north;
    } else if (current - next == 1) {
      nextDir = south;
    }
    switch (curDir) {
      case north:
        switch (nextDir) {
          case north:
            intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
            break;
          case east:
            intersections[current].waypoint.waypointType = WaypointType_TurnRight;
            break;
          case west:
            intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
          case south:
            break;
        }
        break;
      case south:
        switch (nextDir) {
          case south:
            intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
            break;
          case east:
            intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
            break;
          case west:
            intersections[current].waypoint.waypointType = WaypointType_TurnRight;
          case north:
            break;
        }
        break;
      case east:
        switch (nextDir) {
          case north:
            intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
            break;
          case south:
            intersections[current].waypoint.waypointType = WaypointType_TurnRight;
            break;
          case east:
            intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
          case west:
            break;
        }
        break;
      case west:
        switch (nextDir) {
          case north:
            intersections[current].waypoint.waypointType = WaypointType_TurnRight;
            break;
          case south:
            intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
            break;
          case west:
            intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
          case east:
            break;
        }
        break;
    }
    curDir = nextDir;
  }
  intersections[path[0]].waypoint.waypointType = WaypointType_Stop;
}


////////////////////
// HELPER METHODS //
////////////////////

/*
 * Updates the list of objects in front of the car with the given list of new objects
 */
void sdcCar::UpdateFrontObjects(std::vector<sdcVisibleObject> newObjects){
    if(this->frontObjects.size() == 0){
        // The car wasn't tracking any objects, so just set the list equal to the new list
        this->frontObjects = newObjects;
        return;
    }

    std::vector<bool> isOldObjectMissing;
    std::vector<bool> isBrandNewObject;
    for(int i = 0; i < newObjects.size(); i++){
        isBrandNewObject.push_back(true);
    }

    // Compare each old object to the new objects, and determine
    // which of them are getting updated, which are missing, as well
    // as if any of the passed in objects are brand new
    for (int i = 0; i < this->frontObjects.size(); i++) {
        sdcVisibleObject oldObj = this->frontObjects[i];
        isOldObjectMissing.push_back(true);

        for (int j = 0; j < newObjects.size(); j++) {
            // Only match each new object to one old object
            if(!isBrandNewObject[j]) continue;
            sdcVisibleObject newObj = newObjects[j];

            if(oldObj.IsSameObject(newObj)){
                oldObj.Update(newObj);
                this->frontObjects[i] = oldObj;
                isOldObjectMissing[i] = false;
                isBrandNewObject[j] = false;
                break;
            }
        }
    }

    // Delete objects that are missing
    for(int i = isOldObjectMissing.size() - 1; i >= 0; i--){
        if(isOldObjectMissing[i]){
            this->frontObjects.erase(this->frontObjects.begin() + i);
        }
    }

    // Add brand new objects
    for(int i = 0; i < newObjects.size(); i++){
        if(isBrandNewObject[i]){
            this->frontObjects.push_back(newObjects[i]);
        }
    }
}

/*
 * Returns true if the current velocity angle matches the direction the car
 * is facing
 */
bool sdcCar::IsMovingForwards(){
    sdcAngle velAngle = GetDirection();
    sdcAngle carAngle = this->GetOrientation();
    return (carAngle - velAngle).IsFrontFacing();
}

/*
 * Gets the speed of the car
 */
double sdcCar::GetSpeed(){
    return sqrt(pow(this->velocity.x,2) + pow(this->velocity.y,2));
}

/*
 * Gets the current direction the car is travelling
 */
sdcAngle sdcCar::GetDirection(){
    math::Vector3 velocity = this->velocity;
    return sdcAngle(atan2(velocity.y, velocity.x));
}

/*
 * Gets the current direction the car is travelling in NSEW
 */
void sdcCar::GetNSEW(){
    if((this->yaw - WEST).WithinMargin(PI/4)){
        this->currentDir = west;
    } else if((this->yaw - SOUTH).WithinMargin(PI/4)){
        this->currentDir = south;
    } else if((this->yaw - EAST).WithinMargin(PI/4)){
        this->currentDir = east;
    } else {
        this->currentDir = north;
    }
}

/*
 * Gets the direction the car is facing
 */
sdcAngle sdcCar::GetOrientation(){
    return this->yaw;
}

/*
 * Returns the angle from the car's current position to a target position
 */
sdcAngle sdcCar::AngleToTarget(math::Vector2d target) {
    //math::Vector2d position = sdcSensorData::GetPosition();
    math::Vector2d targetVector = math::Vector2d(target.x - this->x, target.y - this->y);
   // printf("x vec: %f y vec: %f", target.x- this->x, target.y- this->y);
    //fflush(stdout);
    return sdcAngle(atan2(targetVector.y, targetVector.x));
}

/*
 * Returns true if there is an object ahead of the car that might collide with us if we
 * continue driving straight ahead
 */
bool sdcCar::ObjectDirectlyAhead() {
    if(this->frontObjects.size() == 0) return false;

    for (int i = 0; i < this->frontObjects.size(); i++) {
        if(this->IsObjectDirectlyAhead(this->frontObjects[i])){
            return true;
        }
    }
    return false;
}

/*
 * Returns true if the given object is directly ahead of us, else false
 */
bool sdcCar::IsObjectDirectlyAhead(sdcVisibleObject obj){
    double leftDist = obj.left.GetLateralDist();
    double rightDist = obj.right.GetLateralDist();
    if(leftDist < 0 && rightDist > 0) return true;
    return fmin(fabs(leftDist), fabs(rightDist)) < FRONT_OBJECT_COLLISION_WIDTH / 2.;
}

/*
 * Returns true if there is an object on a potential collision course with our car
 */
bool sdcCar::ObjectOnCollisionCourse(){
    if(this->frontObjects.size() == 0) return false;

    for (int i = 0; i < this->frontObjects.size(); i++) {
        if(this->IsObjectOnCollisionCourse(this->frontObjects[i])){
            return true;
        }
    }
    return false;
}

/*
 * Returns true if the given object is on a potential collision course with our car
 */
bool sdcCar::IsObjectOnCollisionCourse(sdcVisibleObject obj){
    bool isTooFast = this->IsObjectTooFast(obj);
    bool isTooFurious = this->IsObjectTooFurious(obj);
    return isTooFast || isTooFurious;
}

/*
 * Returns true if the given object is projected to run into the car within a short time period from now
 */
bool sdcCar::IsObjectTooFast(sdcVisibleObject obj){
    math::Vector2d centerpoint = obj.GetCenterPoint();
    bool inLineToCollide = (fabs(obj.lineIntercept) < 1.5 || (fabs(centerpoint.x) < 1.5 && fabs(obj.GetEstimatedXSpeed()) < fabs(0.1 * obj.GetEstimatedYSpeed())));
    bool willHitSoon = obj.dist / obj.GetEstimatedSpeed() < 20;
    return inLineToCollide && willHitSoon;
}

/*
 * Returns true if the given object is very close to the car
 */
bool sdcCar::IsObjectTooFurious(sdcVisibleObject obj){
    math::Vector2d centerpoint = obj.GetCenterPoint();
    return (fabs(centerpoint.x) < FRONT_OBJECT_COLLISION_WIDTH / 2. && fabs(centerpoint.y) < 1.5);
}

///////////////////////////
// BEGIN CONTROL METHODS //
///////////////////////////

/*
 * Speeds up the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcCar::Accelerate(double amt, double rate){
    this->SetTargetSpeed(this->GetSpeed() + amt);
    this->SetAccelRate(rate);
}

/*
 * Slows down the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcCar::Brake(double amt, double rate){
    this->SetTargetSpeed(this->GetSpeed() - amt);
    this->SetBrakeRate(rate);
}

/*
 * Sets the target speed to 0 m/s
 */
void sdcCar::Stop(){
    this->SetTargetSpeed(0);
}

/*
 * Move the car in reverse. Target speed will now be matched with the car going
 * backwards and target direction should be the direction of velocity desired, NOT
 * the direction the front of the car is facing
 */
void sdcCar::Reverse(){
    this->reversing = true;
}

/*
 * Stop reversing the car.
 */
void sdcCar::StopReverse(){
    this->reversing = false;
}

/*
 * Sets the rate of acceleration for the car. The rate is a scalar for the
 * force applies to accelerate the car
 *
 * Default rate: 1.0, can't be negative
 */
void sdcCar::SetAccelRate(double rate){
    this->accelRate = fmax(rate, 0.0);
}

/*
 * Sets the rate of braking for the car. The rate is a scalar for the
 * force applied to brake the car
 *
 * Default rate: 1.0, can't be negative
 */
void sdcCar::SetBrakeRate(double rate){
    this->brakeRate = fmax(rate, 0.0);
}

/*
 * Sets a target direction for the car
 */
void sdcCar::SetTargetDirection(sdcAngle direction){
    this->targetDirection = direction;
}

/*
 * Sets a target steering amount for the steering wheel
 */
void sdcCar::SetTargetSteeringAmount(double a){
    this->targetSteeringAmount = a;
}

/*
 * Sets the target speed for the car, as well as resetting the brake
 * and accel rates to default. Methods wishing to change those parameters
 * should make sure to do so AFTER a call to this method
 */
void sdcCar::SetTargetSpeed(double s){
    this->targetSpeed = fmax(fmin(s, this->maxCarSpeed), 0);
    this->stopping = (this->targetSpeed == 0);
    this->SetAccelRate();
    this->SetBrakeRate();
}

/*
 * Sets the amount by which the car turns. A larger number makes the car turn
 * harder.
 */
void sdcCar::SetTurningLimit(double limit){
    this->turningLimit = limit;
}

//////////////////////////////////////////////////////////////
// GAZEBO METHODS - GAZEBO CALLS THESE AT APPROPRIATE TIMES //
//////////////////////////////////////////////////////////////

/*
 * Called when initially loading the car model from the sdf. Links the car
 * to the OnUpdate methods so we can receive updates
 */
void sdcCar::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the model and chassis of the car for later access
    this->model = _model;
    this->chassis = this->model->GetLink(_sdf->Get<std::string>("chassis"));

    // Get all the wheel joints
    this->joints[0] = this->model->GetJoint(_sdf->Get<std::string>("front_left"));
    this->joints[1] = this->model->GetJoint(_sdf->Get<std::string>("front_right"));
    this->joints[2] = this->model->GetJoint(_sdf->Get<std::string>("back_left"));
    this->joints[3] = this->model->GetJoint(_sdf->Get<std::string>("back_right"));

    // Pull some parameters that are defined in the sdf
    this->maxSpeed = _sdf->Get<double>("max_speed");
    this->aeroLoad = _sdf->Get<double>("aero_load");
    this->tireAngleRange = _sdf->Get<double>("tire_angle_range");
    this->frontPower = _sdf->Get<double>("front_power");
    this->rearPower = _sdf->Get<double>("rear_power");
    this->wheelRadius = _sdf->Get<double>("wheel_radius");

    // Tell Gazebo to call OnUpdate whenever the car needs an update
    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(boost::bind(&sdcCar::OnUpdate, this)));
}

/*
 * Called when the car and world are being (re)initialized.
 */
void sdcCar::Init()
{

    
    // Compute the angle ratio between the steering wheel and the tires
    this->steeringRatio = STEERING_RANGE / this->tireAngleRange;

    // During init, sensors aren't available so pull position and rotation information
    // straight from the car
    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = sdcAngle(pose.rot.GetYaw());
    this->x = pose.pos.x;
    this->y = pose.pos.y;
    
    GenerateWaypoints();
}

/*
 * Called whenever Gazebo needs an update for this model
 */
void sdcCar::OnUpdate()
{
//    if(this->stopping){
//        printf("stopping\n");
//        printf("%f",this->velocity.y);
//    }
    
    
    // Get the current velocity of the car
    this->velocity = this->chassis->GetWorldLinearVel();
    // Get the cars current position
    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = sdcAngle(pose.rot.GetYaw());
    this->x = pose.pos.x;
    this->y = pose.pos.y;
   // this->x = pose.x;
   // this->y = pose.y;
    //printf("x, y is : %f %f \n", this->x, this->y);
    // Get the cars current rotation
   // this->yaw = sdcSensorData::GetYaw();

    // Check if the front lidars have been updated, and if they have update
    // the car's list
    if(this->frontLidarLastUpdate != this->sensorData.GetLidarLastUpdate(FRONT)){
        std::vector<sdcVisibleObject> v = this->sensorData.GetObjectsInFront();
        this->UpdateFrontObjects(v);
        this->frontLidarLastUpdate = this->sensorData.GetLidarLastUpdate(FRONT);
    }

    // Call our Drive function, which is the brain for the car
    //printf("going to drive\n");
    this->Drive();


    ////////////////////////////
    // GAZEBO PHYSICS METHODS //
    ////////////////////////////

    // Compute the angle of the front wheels.
    double wheelAngle = this->steeringAmount / this->steeringRatio;

    // Compute the rotational velocity of the wheels
    double jointVel = (this->gas-this->brake * this->maxSpeed) / this->wheelRadius;

    // Set velocity and max force for each wheel
    this->joints[0]->SetVelocityLimit(1, -jointVel);
    this->joints[0]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->frontPower);

    this->joints[1]->SetVelocityLimit(1, -jointVel);
    this->joints[1]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->frontPower);

    this->joints[2]->SetVelocityLimit(1, -jointVel);
    this->joints[2]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->rearPower);

    this->joints[3]->SetVelocityLimit(1, -jointVel);
    this->joints[3]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->rearPower);

    // Set the front-left wheel angle
    this->joints[0]->SetLowStop(0, wheelAngle);
    this->joints[0]->SetHighStop(0, wheelAngle);
    this->joints[0]->SetLowStop(0, wheelAngle);
    this->joints[0]->SetHighStop(0, wheelAngle);

    // Set the front-right wheel angle
    this->joints[1]->SetHighStop(0, wheelAngle);
    this->joints[1]->SetLowStop(0, wheelAngle);
    this->joints[1]->SetHighStop(0, wheelAngle);
    this->joints[1]->SetLowStop(0, wheelAngle);

    //  aerodynamics
    this->chassis->AddForce(math::Vector3(0, 0, this->aeroLoad * this->velocity.GetSquaredLength()));

    // Sway bars
    math::Vector3 bodyPoint;
    math::Vector3 hingePoint;
    math::Vector3 axis;

    // Physics calculations
    for (int ix = 0; ix < 4; ++ix)
    {
        hingePoint = this->joints[ix]->GetAnchor(0);
        bodyPoint = this->joints[ix]->GetAnchor(1);

        axis = this->joints[ix]->GetGlobalAxis(0).Round();
        double displacement = (bodyPoint - hingePoint).Dot(axis);

        float amt = displacement * this->swayForce;
        if (displacement > 0)
        {
            if (amt > 15)
                amt = 15;

            math::Pose p = this->joints[ix]->GetChild()->GetWorldPose();
            this->joints[ix]->GetChild()->AddForce(axis * -amt);
            this->chassis->AddForceAtWorldPosition(axis * amt, p.pos);

            p = this->joints[ix^1]->GetChild()->GetWorldPose();
            this->joints[ix^1]->GetChild()->AddForce(axis * amt);
            this->chassis->AddForceAtWorldPosition(axis * -amt, p.pos);
        }
    }
}

/*
 * Constructor for the car. Sets several parameters to default values, some of
 * which will get overwritten in Load or Init and others that will be updated
 * when the car is updating
 */
sdcCar::sdcCar(){
    this->sensorData = sdcSensorData();
    this->joints.resize(4);

    // Physics variables
    this->aeroLoad = 0.1;
    this->swayForce = 10;

    this->maxSpeed = 10;
    this->frontPower = 50;
    this->rearPower = 50;
    this->wheelRadius = 0.3;
    this->steeringRatio = 1.0;
    this->tireAngleRange = 1.0;

    // Movement parameters
    this->gas = 0.0;
    this->brake = 0.0;
    this->accelRate = 1.0;
    this->brakeRate = 1.0;

    // Limits on the car's speed
    this->maxCarSpeed = 10;
    this->maxCarReverseSpeed = -10;

    // Initialize state enums
    this->DEFAULT_STATE = waypoint;
    this->currentState = DEFAULT_STATE;

    this->currentPerpendicularState = backPark;
    this->currentParallelState = rightBack;
    this->currentAvoidanceState = notAvoiding;

    // Set starting speed parameters
    this->targetSpeed = 6;

    // Set starting turning parameters
    this->steeringAmount = 0.0;
    this->targetSteeringAmount = 0.0;
    this->targetDirection = sdcAngle(0);
    this->turningLimit = 20.0;

    // Booleans for the car's actions
    this->turning = false;
    this->reversing = false;
    this->stopping = false;

    // Variables for parking
    this->targetParkingAngle = sdcAngle(0.0);
    this->parkingAngleSet = false;
    this->isFixingParking = false;
    this->parkingSpotSet = false;

    // Variables for waypoint driving
    this->waypointProgress = 0;

    // Variables for intersections
    this->stoppedAtSign = false;
    this->ignoreStopSignsCounter = 0;
    this->atIntersection = 0;

    // Variables for following
    this->isTrackingObject = false;
    this->stationaryCount = 0;

    // Variables for avoidance
    this->trackingNavWaypoint = false;
}
