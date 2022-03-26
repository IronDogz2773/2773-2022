// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/** Constants for MainRobot */
public class ConstantsForMainRobot {
    // Presence of subsystems
    public static final boolean climberPresent = false;
    public static final boolean indexerPresent = false;
    public static final boolean hopperPresent = false;
    public static final boolean intakePresent = true;
    public static final boolean shooterPresent = false;
    public static final boolean irPresent = true;

    // using turnvision or proximity sensor
    public static final boolean turnVision = false;
    public static final boolean proximity = false;

    //speeds for drivetrain
    public static final double speedFactorSlow = 1.0 / 3.0;
    public static final double speedFactorFast = .85;
    

    // autonomous shooting checks
    public static final boolean encoder = true; 

    // 2 motor shooter
    public static final boolean dosShooter = true;

    public static final int distanceSensorPin = 5;
    public static final int irSensorPin = 8;

    public static final double indexTime = 1; //TODO change this value for main robot
    public static final double reverseIndexTime = .1;

    // Pneumatics
    public static final int intakePneumaticsController = 0;
    public static final int deploySolenoidPCM = 0;
    public static final int retractSolenoidPCM = 1;

    // Motors
    // CANID - CAN SPARK MAX Motors
    // PWMID - PWN Motors
    public static final int leftForWheelsCANID = 10;
    public static final int rightForWheelsCANID = 23;
    public static final int leftBackWheelsCANID = 19;
    public static final int rightBackWheelsCANID = 22;

    public static final boolean leftWheelsInverted = false;
    public static final boolean rightWheelsInverted = true;

    public static final int frontShooterCANID = 16; // PLACEHOLDER
    public static final int backShooterCANID = 15; // PLACEHOLDER
    public static final int indexerCANID = 12; // PLACEHOLDER

    public static final int intakeCANID = 14;
    public static final int leftHopperCANID = 13;
    public static final int rightHopperCANID = 11;
    public static final int shooterMotorPWMID = 0;
    public static final int kickerMotorPWMID = 1;
    public static final int winchMotorPWMID = 6;

    public static final int leftTelescopeCANID = 20; 
    public static final int rightTelescopeCANID = 17; 

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

    public static final double turnControllerP = 0.05;
    public static final double turnControllerI = 0;
    public static final double turnControllerD = 0.0001;

    public static final double shooterControllerP = 0.00001;
    public static final double shooterControllerI = 0;
    public static final double shooterControllerD = 0.000002;

    public static final double shooterEncoderResolution = 8192;

    public static final double maxMotorVolts = 12.0;
    public static final double maxRotationVolts = 4;// placeholder

    public static final double cameraMountAngle = Math.toRadians(40); //angle in radians
    public static final double cameraMountHeight = 1.9166; //height in feet (1 foot 11 inches)

    public static final double autoSpeed = .3; 
    public static final double autoTime = 1;
}
