// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax oneStageMotor;
  private final SparkMax twoStageMotor;
  private final SparkClosedLoopController oneStageController;
  private final SparkClosedLoopController twoStageController;
  private final SparkMaxConfig motorConfig;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    oneStageMotor = new SparkMax(ClimberConstants.oneStageCanID, MotorType.kBrushless);
    twoStageMotor = new SparkMax(ClimberConstants.twoStageCanID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motorConfig.inverted(false);
    motorConfig.smartCurrentLimit(40);
    /** Configure PID for position control */
    motorConfig.closedLoop
      .p(ClimberConstants.kP) // please tune
      .i(ClimberConstants.kI)
      .d(ClimberConstants.kD)
      .outputRange(-1, 1);

      oneStageMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      twoStageMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      oneStageController = oneStageMotor.getClosedLoopController();
      twoStageController = twoStageMotor.getClosedLoopController();
  }
  /**Drive motor to first stage in rotations */
  public void setOneStage(double position) {
    oneStageController.setSetpoint(position, ControlType.kPosition);
  }
  /**Drive motor to second stage in rotations */
  public void setTwoStage(double position) {
    twoStageController.setSetpoint(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
