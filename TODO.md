# Robot TODO List

Please ensure that you add anything left half-finished to the list, and check off anything you finished.

## General
- [ ] Cleanup EVERYTHING
    - [ ] Especially DriveSubsystem.java
- [ ] Discuss creating/choosing style guide

## Shooter
- [ ] Fix Constants.java
- [ ] Tune low/mid/high velocities
- [ ] Fix trapezoid profile constraints
- [ ] Tune feedforward as needed. kV should theoretically already be pretty close, kS definitely needs tuning
- [ ] Add low/mid/migh to RobotContainer.java
    - [ ] EX: (button) -> .onTrue -> shooter.setVelocity(lowVel);
- [ ] Figure out inversion

## Stationary Aimbot Command
- [X] Flesh out skeleton
    - [X] Need to finish spindexer and kicker code first
- [X] Add plus/minus 6in
- [ ] Change to get x and y relative from shooter instead of robot
- [ ] Add some logging
- [ ] Check shot is possible before attempting it
- [X] Add lock on hub
- [X] Add data file
    - [ ] Turn into a supplier
- [ ] Make moving version


## Drive
- [ ] Figure out gyro inversion
- [X] Auto lock on
    - [ ] Tune PID controller

## Intake
- [ ] Fix CAN ID in Constants.java
- [ ] Figure out inversion
- [ ] Test different motor power levels w/ build team

## Vision
- [ ] Fix camera offsets to robot
- [ ] Add utility other than pose estimation
- [ ] Check if we need to call to periodic

## Auto / PathPlanner
- [ ] Fix robot configs in PathPlanner GUI
    - [ ] Robot mass
    - [ ] Robot MOI
    - [ ] Bumpers
    - [ ] Wheel COF (has to be calculated?)
    - [ ] Robot max speed (empirally measure)
    - [ ] Drive current limit
- [ ] Tune PID (defaults are 5, reduced to 1 for safety until testing)
- [ ] Actually make paths/autos in gui