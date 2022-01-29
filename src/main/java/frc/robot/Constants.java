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
    
    public static final int gamepadPort = 0;

    public static final int lStickY = 1;
    public static final int rStickY = 5;

    public static final double speedFactor = 2.0/3.0;

    public static final int leftForWheelsID = 21;
    public static final int rightForWheelsID = 18;
    public static final int leftBackWheelsID = 17;
    public static final int rightBackWheelsID = 20;
    public static final int intakeID = 28;
    public static final int frontConveyorMotorID = 29;
    public static final int backConveyorMotorID = 22;
    public static final int rightShooterMotorID = 24;
    public static final int leftShooterMotorID = 25;
    public static final int kickerMotorID = 26;
    public static final int winchMotorID = 27;
}
