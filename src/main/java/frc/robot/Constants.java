// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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

    public static final double speedFactor = 2.0/3.0;

    //Pneumatics
    public static final int intakePneumaticsController = 0;
    public static final int leftSolenoidPCM = 1;
    public static final int rightSolenoidPCM = 0;

    //Motors
    //CANID - CAN SPARK MAX Motors
    //PWMID - PWN Motors
    public static final int leftForWheelsCANID = 17;
    public static final int rightForWheelsCANID = 21;
    public static final int leftBackWheelsCANID = 20;
    public static final int rightBackWheelsCANID = 18;
    public static final int intakeCANID = 15; //Placeholder value
    public static final int frontConveyorMotorPWMID = 1;
    public static final int backConveyorMotorPWMID = 2;
    public static final int rightShooterMotorPWMID = 3;
    public static final int leftShooterMotorPWMID = 4;
    public static final int kickerMotorPWMID = 5;
    public static final int winchMotorPWMID = 6;


    //Encoder ports
    public static final int leftEncoderPortA = 0;
    public static final int leftEncoderPortB = 1;
    public static final int rightEncoderPortA = 3;
    public static final int rightEncoderPortB = 4;


    //Gamepad buttons
    //Axis
    public static final int lStickY = 1; //Left drivetrain speed
    public static final int rStickY = 5; //Right drivetrain speed
    public static final int lTrigger = 2; //Activate intake
    public static final int rTrigger = 3;

    //Buttons
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5; //Deploy intake
    public static final int RB = 6; //Retract intake
    public static final int Start = 8;

    //PID values
    //ALL CURRENT VALUES ARE PLASEHOLDERS
    //DO NOT USE THESE CURRENT VALUES AND EXPECT IT TO WORK
    public static final double ksVolts = 0.62975;
    public static final double kvVoltSecondsPerMeter = 2.9196;
    public static final double kaVoltSecondsSquaredPerMeter = 1.732;

    public static final double kPDriveVel = 2.5036;

    public static final double kTrackwidthMeters = 0.53;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = .6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kP = 0.04;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double maxMotorVolts = 12.0;

    public static final double distancePerPulse = .16 * Math.PI / 2048;
}
