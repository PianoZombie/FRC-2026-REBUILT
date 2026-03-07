# Robot TODO List

Please ensure that you add anything left half-finished to the list, and check off anything you finished.

## General
- [X] Cleanup EVERYTHING
    - [X] Especially DriveSubsystem.java
- [ ] Discuss creating/choosing style guide
- [X] Add some logging
    - [X] Shooter velocity
    - [X] Swerve module state
    - [X] Position of robot

## Shooter
- [ ] Fix Constants.java
- [ ] Fix trapezoid profile constraints
- [X] Add low/mid/high to RobotContainer.java
    - [X] EX: (button) -> .onTrue -> shooter.setVelocity(lowVel);

## Stationary Aimbot Command
- [X] Flesh out skeleton
    - [X] Need to finish spindexer and kicker code first
- [X] Add plus/minus 6in
- [X] Change to get x and y relative from shooter instead of robot
- [X] Check shot is possible before attempting it
- [X] Add lock on hub
- [X] Add data file
    - [ ] Turn into a supplier
- [ ] Make moving version


## Drive
- [ ] Figure out gyro inversion
- [X] Auto lock on

## Intake
- [X] Fix CAN ID in Constants.java
- [ ] Figure out inversion
- [ ] Test different motor power levels w/ build team

## Vision
- [ ] Fix camera offsets to robot
- [X] Add utility other than pose estimation
- [X] Check if we need to call to periodic

## Auto / PathPlanner
- [ ] Fix robot configs in PathPlanner GUI
    - [ ] Robot mass
    - [ ] Robot MOI
    - [ ] Bumpers
    - [ ] Wheel COF (has to be calculated?)
    - [ ] Robot max speed (empirally measure)
    - [ ] Drive current limit
- [ ] Actually make paths/autos in gui

## Tuning
- [ ] Tune Drive
    - [ ] mFeedbackController
    - [ ] Translation PID
    - [ ] Rotation PID
- [ ] Tune Shooter
    - [ ] lowVel
    - [ ] midVel
    - [ ] highVel
    - [ ] kS
    - [ ] kV
    - [ ] kMaxVelocity
    - [ ] kMaxAcceleration
    - [X] Inversion
- [ ] Tune Spindexer
    - [ ] Inversion
- [ ] Configs
    - [ ] drivingConfig.closedLoop
    - [ ] turningConfig.closedLoop
- [ ] Constants
    - [ ] kGyroPort
    - [ ] kGyroReversed
- [ ] Tune auto
    - [ ] Tune PID
- [ ] Tune Vision
    - [ ] Change offsets
        - [ ] Camera one
        - [ ] Camera two
    - [ ] Calibrate camera two