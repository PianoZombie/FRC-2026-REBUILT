// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  Pose3d hubPose;

  public StationaryAimbotCommand(DriveSubsystem drive, ShooterSubsystem shooter, KickerSubsystem kicker,
      SpindexerSubsystem spindexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
    this.shooter = shooter;
    addRequirements(shooter);
    this.kicker = kicker;
    addRequirements(kicker);
    this.spindexer = spindexer;
    addRequirements(spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Activate lock in on hub

    // Activate spindexer
    spindexer.startSpindexer();

    // Get basket pos
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    hubPose = new Pose3d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32), Units.inchesToMeters(72),
        new Rotation3d());
    if (alliance == Alliance.Red) {
      hubPose = new Pose3d(Units.inchesToMeters(181.56), Units.inchesToMeters(445.32), Units.inchesToMeters(72),
          new Rotation3d());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get robot pos
    Pose2d robotPose = drive.getPose();
    Pose2d relativePose = hubPose.toPose2d().relativeTo(robotPose);

    // Fancy projectile trajectory formula for required rpm
    double theta = ShooterConstants.theta; // theta of shooter wheel from horizon, radians
    double rS = ShooterConstants.radius; // radius of shooter wheel
    double g = ShooterConstants.g; // acceleration of gravity
    double x = Math.hypot(relativePose.getX(), relativePose.getY()); // horizontal distance from robot to hub
    double y = hubPose.getZ(); // vertical distance from robot to hub
    double k = 1; // efficiency, "fudge factor"

    double vB = (x / Math.cos(theta)) * Math.sqrt(g / (2 * (Math.tan(theta) * x - y))); // velocity we need to shoot ball at
    double vS = vB / (k * rS); // angular velocity to spin shooter at, rad/sec

    shooter.setVelocity(vS);

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
    // Disable lock in on hub

    // Stop spinning shooter, indexer, and kicker
    spindexer.stopSpindexer();
    kicker.stopKicker();
    shooter.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // don't change since we have no way to know if the hopper is empty
    return false;
  }
}
