// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveTrainPorts { //constants are used to identify the index of the motors in the chasis for driving
        public static final int LeftDriveTalonPort = 1;
        public static final int RightDriveTalonPort = 2;
    }

    public static final class TurretConstants{
        public static final int talonPort = 4; //constants are used to identify the index of the motors in the chasis for turning
        public static final int centerTalonPort = 5; //constants are used to identify the index of the motors in the chasis for turning
        public static final double threshold = 1; //The acceptable degree of error in the turning
    } //
    
    public static final class PIDConstants {
        public static final double kP = 0.027;
        public static final double kI = 0;
        public static final double kD = 0;
    }
}