/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       sahilchhina                                               */
/*    Created:      Mon Nov 25 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// lf                   motor         1               
// lb                   motor         2               
// rf                   motor         3               
// rb                   motor         4               
// pushOut              motor         5               
// intakeLeft           motor         6               
// intakeRight          motor         7               
// lift                 motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"
#include "algorithm"

using namespace vex;

//A minimum velocity is necessary. This could also be done per command instead
//of globally.
double minimum_velocity = 65.0;

//This function provides an increasing speed as the robot moves away from start
double mincreasing_speed (double starting_point, double current_position) {
    static const double acceleration_constant = 45.0;
    return acceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

//This function provides a decreasing speed as the robot approaches the end
double mdecreasing_speed (double ending_point, double current_position) {
    static const double deceleration_constant = 45.0;
    return deceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}

double tincreasing_speed (double starting_point, double current_position) 
{
    static const double tacceleration_constant = 100.0;
    return tacceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

double tdecreasing_speed (double ending_point, double current_position) 
{
    static const double tdeceleration_constant = 50.0;
    return tdeceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}

double pincreasing_speed (double starting_point, double current_position) 
{
    static const double acceleration_constant = 100.0;
    return acceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

double pdecreasing_speed (double ending_point, double current_position) 
{
    static const double deceleration_constant = 50.0;
    return deceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}

//This function takes a distance, a maximum velocity, and tries to send the
//robot in a straight line for that distance using a trapezoidal motion profile
//controlled by increasing_speed, decreasing_speed, and maxVelocity
void move (double distanceIn, double maxVelocity) {
    //record nominal wheel circumference
    static const double circumference = 3.14159 * 4;
    
    //if we've got a joker on our hands, punch out
    if (distanceIn == 0) return;
    
    //figure out which direction we're supposed to be going
    double direction = distanceIn > 0 ? 1.0 : -1.0;
    
    //using circumference and commanded inches, convert to revolutions
    double wheelRevs = distanceIn / circumference;
    
    //set the motors to do a position move with 0 velocity
     rf.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
     lf.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
     rb.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
     lb.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
    
    //record starting positions and ending positions
    double leftStartPoint = lf.rotation(rotationUnits::rev);
    double leftEndPoint = leftStartPoint + wheelRevs;
    double rightStartPoint = rf.rotation(rotationUnits::rev);
    double rightEndPoint = rightStartPoint + wheelRevs;
    
    //Back Motors
    double leftBStartPoint = lb.rotation(rotationUnits::rev);
    double leftBEndPoint = leftBStartPoint + wheelRevs;
    double rightBStartPoint = rb.rotation(rotationUnits::rev);
    double rightBEndPoint = rightBStartPoint + wheelRevs;
    
    //execute motion profile
    while (
        (direction * (rf.rotation(rotationUnits::rev) - rightStartPoint) < direction * wheelRevs) ||
        (direction * (lf.rotation(rotationUnits::rev) - leftStartPoint) < direction * wheelRevs)  ||
        (direction * (lb.rotation(rotationUnits::rev) - leftBStartPoint) < direction * wheelRevs)  ||
        (direction * (rb.rotation(rotationUnits::rev) - rightBStartPoint) < direction * wheelRevs)  
    ) {
        
        //set right motor speed to minimum of increasing function, decreasing
        //function, and max velocity, based on current position
        if (direction * (rf.rotation(rotationUnits::rev) - rightStartPoint) < direction * wheelRevs) {
            rf.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(rightStartPoint,rf.rotation(rotationUnits::rev)),
                        mdecreasing_speed(rightEndPoint,rf.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            rf.stop(brakeType::brake);
        }
        
        //do the same for the left motor
        if (direction * (lf.rotation(rotationUnits::rev) - leftStartPoint) < direction * wheelRevs) {
            lf.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(leftStartPoint,lf.rotation(rotationUnits::rev)),
                        mdecreasing_speed(leftEndPoint,lf.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            lf.stop(brakeType::brake);
        }
        
        if (direction * (lb.rotation(rotationUnits::rev) - leftBStartPoint) < direction * wheelRevs) {
            lb.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(leftBStartPoint,lb.rotation(rotationUnits::rev)),
                        mdecreasing_speed(leftBEndPoint,lb.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            lb.stop(brakeType::brake);
        }
        
        if (direction * (rb.rotation(rotationUnits::rev) - rightBStartPoint) < direction * wheelRevs) {
            rb.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(rightBStartPoint,rb.rotation(rotationUnits::rev)),
                        mdecreasing_speed(rightBEndPoint,rb.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            rb.stop(brakeType::brake);
        }
    }
}

void move (double distanceIn) {
    //no max velocity specified, call the version that uses it with max velocity
    //of 100%
    move(distanceIn, 100.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pointturn (double distanceIn, double maxVelocity, double sideval) 
{
    static const double circumference = 360;
    double direction4 = distanceIn > 0 ? 1.0 : -1.0;
    double wheelRevs4 = distanceIn / circumference;
    
    if (sideval == 1)
    {
        lf.stop(hold);
        lb.stop(hold);
        rf.spin(forward,direction4 * minimum_velocity,pct);
        rb.spin(forward,direction4 * minimum_velocity,pct);

        double rightStartPoint = rf.rotation(rev);
        double rightEndPoint = rightStartPoint + wheelRevs4;
        double rightBStartPoint = rb.rotation(rev);
        double rightBEndPoint = rightBStartPoint + wheelRevs4;

        while (
                (direction4 * (rf.rotation(rev) - rightStartPoint) < direction4 * wheelRevs4) ||
                (direction4 * (rb.rotation(rev) - rightBStartPoint) < direction4 * wheelRevs4)  
              ) 
        {
            if (direction4 * (rf.rotation(rev) - rightStartPoint) < direction4 * wheelRevs4) 
            {
                rf.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(rightStartPoint,rf.rotation(rev)),
                            pdecreasing_speed(rightEndPoint,rf.rotation(rev))
                        )
                    ),
                    pct
                );
            } 
            else 
            {
                rf.stop(brake);
            }

            if (direction4 * (rb.rotation(rev) - rightBStartPoint) < direction4 * wheelRevs4) {
                rb.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(rightBStartPoint,rb.rotation(rev)),
                            pdecreasing_speed(rightBEndPoint,rb.rotation(rev))
                        )
                    ),
                    pct
                );
            } 
            else 
            {
                rb.stop(brake);
            }     
        }
        rf.stop(brake);
        rb.stop(brake);
    }
    
    if (sideval == -1)
    {
        rf.stop(hold);
        rb.stop(hold);
        lf.spin(forward,direction4 * minimum_velocity,pct);
        lb.spin(forward,direction4 * minimum_velocity,pct);

        double leftStartPoint = lf.rotation(rev);
        double leftEndPoint = leftStartPoint + wheelRevs4;

        double leftBStartPoint = lb.rotation(rev);
        double leftBEndPoint = leftBStartPoint + wheelRevs4;

        while (
                (direction4 * (lf.rotation(rev) - leftStartPoint) < direction4 * wheelRevs4)  ||
                (direction4 * (lb.rotation(rev) - leftBStartPoint) < direction4 * wheelRevs4)
              ) 
        {
            if (direction4 * (lf.rotation(rev) - leftStartPoint) < direction4 * wheelRevs4) 
            {
                lf.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(leftStartPoint,lf.rotation(rev)),
                            pdecreasing_speed(leftEndPoint,lf.rotation(rev))
                        )
                    ),
                    pct
                );
            } 
            else 
            {
                lf.stop(brake);
            }

            if (direction4 * (lb.rotation(rev) - leftBStartPoint) < direction4 * wheelRevs4) {
                lb.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(leftBStartPoint,lb.rotation(rev)),
                            pdecreasing_speed(leftBEndPoint,lb.rotation(rev))
                        )
                    ),
                    pct
                );
            } 
            else 
            {
                lb.stop(brake);
            }

        }
        lf.stop(brake);
        lb.stop(brake);
    }
    lf.stop(brake);
    lf.stop(brake);
    rf.stop(brake);
    rb.stop(brake);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turn (double distanceIn, double maxVelocity) 
{
    static const double circumference = 360;
    double direction2 = distanceIn > 0 ? -1 : 1;
    double direction3 = distanceIn > 0 ? 1 : -1;
    double wheelRevs2 = (direction2*std::abs(distanceIn)) / circumference;
    double wheelRevs3 = (direction3*std::abs(distanceIn)) / circumference;
    
    rf.spin(forward,direction2 * minimum_velocity,pct);
    lf.spin(forward,direction3 * minimum_velocity,pct);
    rb.spin(forward,direction2* minimum_velocity,pct);
    lb.spin(forward,direction3* minimum_velocity,pct);
    
    double leftStartPoint = lf.rotation(rev);
    double leftEndPoint = leftStartPoint + wheelRevs3;
    double rightStartPoint = rf.rotation(rev);
    double rightEndPoint = rightStartPoint + wheelRevs2;
    
    double leftBStartPoint = lb.rotation(rev);
    double leftBEndPoint = leftBStartPoint + wheelRevs3;
    double rightBStartPoint = rb.rotation(rev);
    double rightBEndPoint = rightBStartPoint + wheelRevs2;
    
    while (
            (direction2* (rf.rotation(rev) - rightStartPoint) < direction2* wheelRevs2) ||
            (direction3* (lf.rotation(rev) - leftStartPoint) < direction3 * wheelRevs3)  ||
            (direction3* (lb.rotation(rev) - leftBStartPoint) < direction3 * wheelRevs3)  ||
            (direction2* (rb.rotation(rev) - rightBStartPoint) < direction2 * wheelRevs2)  
          ) 
    {
        if (direction2 * (rf.rotation(rev) - rightStartPoint) < direction2 * wheelRevs2) 
        {
            rf.setVelocity(
                direction2 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(rightStartPoint,rf.rotation(rev)),
                        tdecreasing_speed(rightEndPoint,rf.rotation(rev))
                    )
                ),
                pct
            );
        } 
        else 
        {
            rf.stop(brake);
        }
        
        if (direction3 * (lf.rotation(rev) - leftStartPoint) < direction3 * wheelRevs3) 
        {
            lf.setVelocity(
                direction3 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(leftStartPoint,lf.rotation(rev)),
                        tdecreasing_speed(leftEndPoint,lf.rotation(rev))
                    )
                ),
                pct
            );
        } 
        else 
        {
            lf.stop(brake);
        }
        
        if (direction3 * (lb.rotation(rev) - leftBStartPoint) < direction3 * wheelRevs3) {
            lb.setVelocity(
                direction3 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(leftBStartPoint,lb.rotation(rev)),
                        tdecreasing_speed(leftBEndPoint,lb.rotation(rev))
                    )
                ),
                pct
            );
        } 
        else 
        {
            lb.stop(brake);
        }
        
        if (direction2 * (rb.rotation(rev) - rightBStartPoint) < direction2 * wheelRevs2) {
            rb.setVelocity(
                direction2 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(rightBStartPoint,rb.rotation(rev)),
                        tdecreasing_speed(rightBEndPoint,rb.rotation(rev))
                    )
                ),
                pct
            );
        } 
        else 
        {
            rb.stop(brake);
        }
    }
    lf.stop(brake);
    lb.stop(brake);
    rf.stop(brake);
    rb.stop(brake);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void poFoward(int speed) {
  pushOut.rotateTo(2, rev, speed, rpm);
}

void poReturn(int speed) {
  pushOut.rotateTo(0, rev, speed, rpm);
}

void intake() {
  intakeLeft.spin(fwd, 200, rpm);
  intakeRight.spin(fwd, 200, rpm);
}

void outake() {
  intakeLeft.spin(reverse, 200, rpm);
  intakeRight.spin(reverse, 200, rpm);
}

int main() {

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  intake();

  move(30, 80);
  wait(200, msec);
  turn(45, 80);
  move(-35, 75);
  turn(-45, 80);
  wait(200, msec);
  move(15,80);
  turn(270, 80);
  move(35, 80);
  poFoward(80);
  move(-5, 80); 
  

}
