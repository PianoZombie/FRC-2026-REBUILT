// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aimbot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class StationaryAimbotCommandData {
    public static double offset = 0; // Fudge factor to add to model if needed mid-match

    /** Add six inches to the aimbot fudge factor. */
    public static void addSixInches() {
        offset += Units.inchesToMeters(6);
    }

    /** Subtract six inches from the aimbot fudge factor. */
    public static void minusSixInches() {
        offset -= Units.inchesToMeters(6);
    }

    /** Reset aimbot fudge factor to zero. */
    public static void resetOffset() {
        offset = 0;
    }

    /** Return aimbot fudge factor. */
    public static double getOffsetMeters() {
        return offset;
    }
}
