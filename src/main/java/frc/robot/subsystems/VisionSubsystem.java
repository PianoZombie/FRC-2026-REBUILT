// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionSubsystem extends SubsystemBase {
  // April tags
  public static final AprilTagFieldLayout kTagLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Cameras
  private final PhotonCamera cameraOne = new PhotonCamera("cameraOne");
  private final PhotonCamera cameraTwo = new PhotonCamera("cameraTwo");
  
  // Camera offsets from robot center
  public static final Transform3d kRobotToCamOne =
    new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0, 0, 0));
  public static final Transform3d kRobotToCamTwo =
    new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0, 0, 0));

  // Pose estimator
  PhotonPoseEstimator poseEstimatorOne = new PhotonPoseEstimator(kTagLayout, kRobotToCamOne);
  PhotonPoseEstimator poseEstimatorTwo = new PhotonPoseEstimator(kTagLayout, kRobotToCamTwo);

  // Current estimates
  private Optional<EstimatedRobotPose> latestEstimate = Optional.empty();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  public Optional<EstimatedRobotPose> getLatestPoseEstimate() {
    return latestEstimate;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var resultOne = cameraOne.getLatestResult();
    var resultTwo = cameraTwo.getLatestResult();

    Optional<EstimatedRobotPose> estimationOne = poseEstimatorOne.estimateCoprocMultiTagPose(resultOne);
    Optional<EstimatedRobotPose> estimationTwo = poseEstimatorTwo.estimateCoprocMultiTagPose(resultTwo);
    if (estimationOne.isEmpty()) {
      estimationOne = poseEstimatorOne.estimateLowestAmbiguityPose(resultOne);
    }
    if (estimationTwo.isEmpty()) {
      estimationTwo = poseEstimatorTwo.estimateLowestAmbiguityPose(resultTwo);
    }

    if (estimationOne.isPresent()) {
      latestEstimate = estimationOne;
    }
    if (estimationTwo.isPresent()) {
      latestEstimate = estimationTwo;
    }
  }
}
