// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aimbot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StationaryAimbotCommand extends Command {
  /** Creates a new StationaryAimbotCommand. */

  private final DriveSubsystem drive;
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final SpindexerSubsystem spindexer;

  public StationaryAimbotCommand(DriveSubsystem drive, ShooterSubsystem shooter, KickerSubsystem kicker,
      SpindexerSubsystem spindexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.shooter = shooter;
    this.kicker = kicker;
    this.spindexer = spindexer;
    addRequirements(drive, shooter, kicker, spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset PID for lock in on hub
    drive.mFeedbackController.reset();

    // Activate spindexer
    spindexer.startSpindexer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get hub pos
    Pose3d hubPose = StationaryAimbotCommandData.getHubPose();

    // Get robot pos
    Pose2d robotPose = drive.getPose();
    Pose2d relativePose = hubPose.toPose2d().relativeTo(robotPose);

    // Fancy projectile trajectory formula for required rpm
    double theta = ShooterConstants.theta; // theta of shooter wheel from horizon, radians
    double rS = ShooterConstants.radius; // radius of shooter wheel
    double g = ShooterConstants.g; // acceleration of gravity
    double x = Math.hypot(relativePose.getX(), relativePose.getY()) + StationaryAimbotCommandData.getOffsetMeters(); // horizontal distance from robot to hub
    double y = hubPose.getZ(); // vertical distance from robot to hub
    double k = 1; // efficiency, "fudge factor"

    // Don't divide by zero
    if (Math.tan(theta) * x <= y) {
        return;
    }

    double vB = (x / Math.cos(theta)) * Math.sqrt(g / (2 * (Math.tan(theta) * x - y))); // ball velocity
    double vS = vB / (k * rS); // shooter angular velocity, rad/sec

    shooter.setVelocity(vS);
    drive.lockRotationOnPoint(hubPose.toPose2d());

    // Check if shooter is within rpm tolerance
    // if yes, shoot ball with kicker
    if (shooter.shooterWithinTolerance(vS)) {
      kicker.startKicker();
    } else {
      kicker.stopKicker();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop spinning shooter, indexer, and kicker
    spindexer.stopSpindexer();
    kicker.stopKicker();
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // don't change since we have no way to know if the hopper is empty
    return false;
  }
}
