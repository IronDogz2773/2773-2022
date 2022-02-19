// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class NavigationSubsystem extends SubsystemBase {
  /** Creates a new NavigationSubsystem. */
  private final ADXRS450_Gyro gyroscope = new ADXRS450_Gyro();

  private final Encoder leftEncoder = new Encoder(Constants.leftEncoderPortA, Constants.leftEncoderPortB);
  private final Encoder rightEncoder = new Encoder(Constants.rightEncoderPortA, Constants.rightEncoderPortB);
  private DifferentialDriveOdometry odometry;
  public NavigationSubsystem() {
    leftEncoder.setDistancePerPulse(-Constants.distancePerPulse);
    rightEncoder.setDistancePerPulse(Constants.distancePerPulse);
      if(gyroscope.isConnected()){
       gyroscope.reset();
       gyroscope.calibrate();
    }
    odometry = new DifferentialDriveOdometry(gyroscope.getRotation2d());
  }

  @Override
  public void periodic() {
    odometry.update(gyroscope.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  
  public void resetGyroAngle(){
      gyroscope.reset();
  }

  public double getGyroAngle(){
      return gyroscope.getAngle();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d initialPose) {
    odometry.resetPosition(initialPose, gyroscope.getRotation2d());
  }
}
