# Robot TODO List

Please ensure that you add anything left half-finished to the list, and check off anything you finished.

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
- [ ] Add plus/minus 6in
- [ ] Change to get x and y relative from shooter instead of robot
- [ ] Add some logging
- [ ] Check shot is possible before attempting it
- [ ] Add lock on hub
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