// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class KickerSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;

  /** Creates a new KickerSubsytem. */
  public KickerSubsystem() {
    motor = new SparkMax(KickerConstants.kickerCanID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    motorConfig.inverted(false);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //** Spin the kicker wheel. */
  public void startKicker() {
    motor.set(0.3);
  }

  /** Stop spinning the kicker wheel. */
  public void stopKicker() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    // Called once per scheduler run. Add telemetry or periodic checks here.
  }
}
