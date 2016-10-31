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
const std::pair<double,double> destination = {0,0};

std::vector<sdcWaypoint> WAYPOINT_VEC;

void sdcCar::Drive()
{
    
    
    switch(this->currentState)
    {
        // Final state, car is finished driving
          
        case stop:
            //printf("in stop\n");
            this->Stop();
            break;
        case waypoint:
            this->WaypointDriving(WAYPOINT_VEC);
            break;

        // At a stop sign, performing a turn
        case intersection:
            printf("intersection\n");
            /**if(this->stoppedAtSign && this->stationaryCount > 2000){
                this->currentState = DEFAULT_STATE;
                this->ignoreStopSignsCounter = 3000;
            }else if(this->stoppedAtSign && this->GetSpeed() < 0.5){
                this->stationaryCount++;
            }else if(!this->stoppedAtSign && sdcSensorData::sizeOfStopSign > 6000){
                this->Stop();
                this->stoppedAtSign = true;
                this->stationaryCount = 0;
        }**/

            break;

        // Follows object that is going in same direction/towards same target
        case follow:
            break;

        // Smarter way to avoid objects; stopping, swerving, etc.
        case avoidance:
            break;

        // Parks the car
        case parking:
            break;
    }

    // Attempts to turn towards the target direction
    //this->MatchTargetDirection();
    // Attempts to match the target speed
    //this->MatchTargetSpeed();
}


void sdcCar::WaypointDriving(std::vector<sdcWaypoint> WAYPOINT_VEC) {
    int progress = this->waypointProgress;
    if(progress < WAYPOINT_VEC.size()){
        // Pull the next waypoint and set the car to drive towards it


        this->Accelerate();

        // Check if the car is close enough to the target to move on
        double distance = sqrt(pow(WAYPOINT_VEC[progress].pos.first - this->x,2) + pow(WAYPOINT_VEC[progress].pos.second - this->y,2));
        if (distance < 7) {
            this->turning = true;
        }
        if(this->turning == true){
            this->SetTurningLimit(20);
            GridTurning(WAYPOINT_VEC[progress].waypointType);
        } else {
            math::Vector2d nextTarget = {WAYPOINT_VEC[progress].pos.first,WAYPOINT_VEC[progress].pos.second};
            sdcAngle targetAngle = AngleToTarget(nextTarget);
            this->SetTargetDirection(targetAngle);
            // this->LanedDriving();
        }
    } else {
   
        this->currentState = stop;
    }
}


/*
 * Executes a turn at an intersection
 * 
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
    path = dijkstras(start, dest);
    printf("did dijtkstras\n");
    insertWaypointTypes(path, this->currentDir);
    printf("insertedwaypointtypes\n");
    for (int i = path.size()-1; i >=0; --i){
        WAYPOINT_VEC.push_back(intersections[path[i]].waypoint);
    }
    printf("end of genwaypoints\n");
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
    path.push_back(intersections[0].place);
    return path;
}

int sdcCar::getFirstIntersection(){
    std::pair<double,double> firstIntr;
    int firstIntersection;

    switch(this->currentDir){

        case west:
            firstIntr = {-1000,0};
            for(int i = 0; i < intersections.size();++i){
                if(this->y < intersections[i].waypoint.pos.second+5 && this->y > intersections[i].waypoint.pos.second-5 && intersections[i].waypoint.pos.first < this->x - 10 && intersections[i].waypoint.pos.first > firstIntr.first)
                    firstIntr = intersections[i].waypoint.pos;
            }
            break;

        case east:
            firstIntr = {1000,0};
            for(int i = 0; i < intersections.size();++i){
                if(this->y < intersections[i].waypoint.pos.second+5 && this->y > intersections[i].waypoint.pos.second-5 && intersections[i].waypoint.pos.first > this->x + 10 && intersections[i].waypoint.pos.first < firstIntr.first){
                    firstIntr = intersections[i].waypoint.pos;
                }
            }
            break;

        case north:
            firstIntr = {0,1000};
            for(int i = 0; i < intersections.size();++i){
                if(this->x < intersections[i].waypoint.pos.first+5 && this->x > intersections[i].waypoint.pos.first-5 && intersections[i].waypoint.pos.second > this->y + 10 && intersections[i].waypoint.pos.second < firstIntr.second)
                    firstIntr = intersections[i].waypoint.pos;
            }
            break;

        case south:
            firstIntr = {0,-1000};
            for(int i = 0; i < intersections.size();++i){
                if(this->x < intersections[i].waypoint.pos.first+5 && this->x > intersections[i].waypoint.pos.first-5 && intersections[i].waypoint.pos.second < this->y - 10 && intersections[i].waypoint.pos.second > firstIntr.second)
                    firstIntr = intersections[i].waypoint.pos;
            }
            break;
    }
    for(int i = 0; i < intersections.size();i++){
        if(firstIntr.first == intersections[i].waypoint.pos.first && firstIntr.second == intersections[i].waypoint.pos.second){
            firstIntersection = i;
            break;
        }
    }
    return firstIntersection;
}

void sdcCar::initializeGraph() {
    //make the sdcIntersections
    printf("making graph\n");
    sdcIntersection aa;
    aa.place = 0;
    aa.waypoint = sdcWaypoint(0,std::pair<double,double>(48,50));
    aa.wpType = WaypointType_Stop;
    intersections = {aa};
    for (int i = 0; i < intersections.size(); ++i) {
        intersections[i].dist = std::numeric_limits<double>::infinity();
        intersections[i].place = i;
    }
    printf("end of graph\n");
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

void sdcCar::removeStartingEdge(int start){
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
        for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
          if (intersections[start].neighbors_pairs[n].first == start - size)
            intersections[start].neighbors_pairs[n].second =
                std::numeric_limits<double>::infinity();
        }
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
    math::Vector2d position = sdcSensorData::GetPosition();
    math::Vector2d targetVector = math::Vector2d(target.x - position.x, target.y - position.y);
    return sdcAngle(atan2(targetVector.y, targetVector.x));
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



///////////////////////////////////////////////////////////////////
//////////////THE STARTING THREE FUNCS ////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
/*
 * Called when initially loading the car model from the sdf. Links the car
 * to the OnUpdate methods so we can receive updates
 */
void sdcCar::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
     printf("load");
}

/*
 * Gets the speed of the car
 */


/*
 * Called when the car and world are being (re)initialized.
 */
void sdcCar::Init()
{
    manager manager(3);
    printf("manager id of this car is");
    printf("init\n");
    // Compute the angle ratio between the steering wheel and the tires
    this->steeringRatio = STEERING_RANGE / this->tireAngleRange;

    // During init, sensors aren't available so pull position and rotation information
    // straight from the car
    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = sdcAngle(pose.rot.GetYaw());
    this->x = pose.pos.x;
    this->y = pose.pos.y;
    printf("going to gen waypoints\n");
    GenerateWaypoints();
}


void sdcCar::OnUpdate()
{
    printf("start updating \n");
    // Get the current velocity of the car
    this->velocity = this->chassis->GetWorldLinearVel();
    // Get the cars current position
    math::Vector2d pose = sdcSensorData::GetPosition();
    this->x = pose.x;
    this->y = pose.y;
    //printf("x, y is : %f %f \n", this->x, this->y);
    // Get the cars current rotation
    this->yaw = sdcSensorData::GetYaw();

    

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
sdcCar::sdcCar(){
    
    printf("initializing car\n");
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

    // Variables for waypoint driving
    this->waypointProgress = 0;

    // Variables for intersections
    this->stoppedAtSign = false;
    this->ignoreStopSignsCounter = 0;
    this->atIntersection = 0;
    
}


