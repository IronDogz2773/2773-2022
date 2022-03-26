// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class NavigationSubsystem extends SubsystemBase implements DistanceSystem {

  // gyroscope
  private final ADXRS450_Gyro gyroscope = new ADXRS450_Gyro();
  // distance sensor
  private final DigitalInput distanceSensor = new DigitalInput(Constants.distanceSensorPin);
  // encoders
  private final Encoder leftEncoder = new Encoder(Constants.leftEncoderPortA, Constants.leftEncoderPortB);
  private final Encoder rightEncoder = new Encoder(Constants.rightEncoderPortA, Constants.rightEncoderPortB);
  // blank odometry object
  private DifferentialDriveOdometry odometry;

  public NavigationSubsystem() {
    leftEncoder.setDistancePerPulse(Constants.distancePerPulse);
    rightEncoder.setDistancePerPulse(-Constants.distancePerPulse);
    // sets odometry rotation to gyro, otherwise set to 0
    if (gyroscope.isConnected()) {
      odometry = new DifferentialDriveOdometry(gyroscope.getRotation2d());
      gyroscope.reset();
      gyroscope.calibrate();
    } else {
      odometry = new DifferentialDriveOdometry(new Rotation2d(0));
    }
  }

  @Override
  public void periodic() {
    // at start of each cycle, update rotation and encoders
    odometry.update(gyroscope.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    System.out.println("angle: " + gyroscope.getAngle());

    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var angle = table.getEntry("pose_a");
    var x = table.getEntry("pose_x");
    var y = table.getEntry("pose_y");
    var p = getPose();
    angle.setNumber(p.getRotation().getDegrees());
    x.setNumber(p.getX());
    y.setNumber(p.getY());
  }

  public void resetEncoder() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void resetGyroAngle() {
    gyroscope.reset();
  }

  public double getGyroAngle() {
    return gyroscope.getAngle();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public boolean tooCloseToWall() {
    return !distanceSensor.get();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d initialPose) {
    odometry.resetPosition(initialPose, gyroscope.getRotation2d());
  }
}
