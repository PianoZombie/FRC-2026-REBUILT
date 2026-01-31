// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aimbot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class StationaryAimbotCommandData {
    public static double offset = 0; // Fudge factor to add to model if needed mid-match

    public static void addSixInches() {
        offset += Units.inchesToMeters(6);
    }

    public static void minusSixInches() {
        offset -= Units.inchesToMeters(6);
    }

    public static void resetOffset() {
        offset = 0;
    }

    public static double getOffsetMeters() {
        return offset;
    }

    public static Pose3d getHubPose() {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Pose3d hubPose = new Pose3d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32), Units.inchesToMeters(72),
                new Rotation3d());
        if (alliance == Alliance.Red) {
            hubPose = new Pose3d(Units.inchesToMeters(181.56), Units.inchesToMeters(445.32), Units.inchesToMeters(72),
                    new Rotation3d());
        }

        return hubPose;
    }
}
