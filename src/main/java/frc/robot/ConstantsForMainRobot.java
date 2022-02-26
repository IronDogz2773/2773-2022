// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/** Constants for MainRobot */
public class ConstantsForMainRobot {

    // Presence of subsystems
    public static final boolean hopperPresent = false;
    public static final boolean intakePresent = true;

    // Pneumatics
    public static final int intakePneumaticsController = 0;
    public static final int deploySolenoidPCM = 0;
    public static final int retractSolenoidPCM = 1;

    // Motors
    // CANID - CAN SPARK MAX Motors
    // PWMID - PWN Motors
    public static final int leftForWheelsCANID = 17;
    public static final int rightForWheelsCANID = 21;
    public static final int leftBackWheelsCANID = 20;
    public static final int rightBackWheelsCANID = 18;
    public static final int intakeCANID = 15; // Placeholder value
    public static final int leftHopperCANID = 1;
    public static final int rightHopperCANID = 2;
    public static final int shooterMotorPWMID = 0;
    public static final int kickerMotorPWMID = 1;
    public static final int winchMotorPWMID = 6;

    public static final MotorType motorType = MotorType.kBrushless;

    // Encoder ports
    public static final int leftEncoderPortA = 0;
    public static final int leftEncoderPortB = 1;
    public static final int rightEncoderPortA = 3;
    public static final int rightEncoderPortB = 4;
    public static final int rightShooterEncoderPort = 6; // placeholder
    public static final int leftShooterEncoderPort = 7; // placeholder

    // PID values
    // ALL CURRENT VALUES ARE PLASEHOLDERS
    // DO NOT USE THESE CURRENT VALUES AND EXPECT IT TO WORK
    public static final double ksVolts = 0.62975;
    public static final double kvVoltSecondsPerMeter = 2.9196;
    public static final double kaVoltSecondsSquaredPerMeter = 1.732;

    public static final double kPDriveVel = 2.5036;

    public static final double kTrackwidthMeters = 0.53;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = .6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.3;

    public static final double turnControllerP = 0.04;
    public static final double turnControllerI = 0;
    public static final double turnControllerD = 0;

    public static final double shooterControllerP = 0.00006; // placeholder
    public static final double shooterControllerI = 0; // placeholder
    public static final double shooterControllerD = 0.00002; // placeholder

    public static final double shooterEncoderResolution = 8192;

    public static final double maxMotorVolts = 12.0;
}
