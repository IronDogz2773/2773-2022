// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Holds constants for bots. ConstantsForTestRobot can replaced with
 * ConstantsForMainRobot
 */
public final class Constants extends ConstantsForTestRobot {

    // USB
    public static final int gamepadPortPilot = 0;
    public static final int gamepadPortCopilot = 1;

    // Gamepad buttons
    // Axis
    public static final int lStickY = 1; // Left drivetrain speed
    public static final int rStickY = 5; // Right drivetrain speed
    public static final int LT = 2; // Activate intake
    public static final int RT = 3;

    // Buttons
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int Select = 7; //this is aurora guessing
    public static final int Start = 8;
    public static final int leftStickPushed = 9;
    public static final int rightStickPushed = 10;


    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double distancePerPulse = .16 * Math.PI / 2048;

    public static final double maxShooterSpeed = 750;

    public static final double turnCmdTimeOut = 2;
}
