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
    
    //USB
    public static final int gamepadPort = 0;
    public static final int joystickPort = 1;

    public static final int lStickY = 1;
    public static final int rStickY = 5;

    public static final double speedFactor = 2.0/3.0;

    //Motors
    //CANID - CAN SPARK MAX Motors
    //PWMID - PWN Motors
    public static final int leftForWheelsCANID = 21;
    public static final int rightForWheelsCANID = 18;
    public static final int leftBackWheelsCANID = 17;
    public static final int rightBackWheelsCANID = 20;
    public static final int intakePWMID = 0;
    public static final int frontConveyorMotorPWMID = 1;
    public static final int backConveyorMotorPWMID = 2;
    public static final int rightShooterMotorPWMID = 3;
    public static final int leftShooterMotorPWMID = 4;
    public static final int kickerMotorPWMID = 5;
    public static final int winchMotorPWMID = 6;

    //Joystick buttons
    public static final int activateIntake = 1;
}
