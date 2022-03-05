// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class ShooterTestSubsystem extends ShooterBaseSubsystem{
  private final Spark motor = new Spark(Constants.shooterMotorPWMID);
  private final Encoder encoder = new Encoder(Constants.rightShooterEncoderPort, Constants.leftShooterEncoderPort);
  private final PIDController pid = new PIDController(Constants.shooterControllerP, Constants.shooterControllerI,
      Constants.shooterControllerD);

  private double rpm = 0.0;
  private double speed = 0.0;
  private boolean viaPid = false;

  NetworkTableEntry frontEntry;
  NetworkTableEntry backEntry;

  /** Creates a new ShooterSubsystem. */
  public ShooterTestSubsystem() {
    encoder.setDistancePerPulse(-60.0 / Constants.shooterEncoderResolution); // 8192
    pid.setSetpoint(rpm);
    pid.setTolerance(30);

  }

  @Override
  public void periodic() {
    if (viaPid) {
      var delta = pid.calculate(encoder.getRate());
      speed = MathUtil.clamp(speed + delta, 0, 1);
    }
    motor.set(speed);
  }

  @Override
  public void setRpm(double rpmFront, double rpmBack) {
    System.out.println("rpm front: " + rpmFront + " rpm back: " + rpmBack);
    this.rpm = rpmFront;
    this.viaPid = true;
    pid.setSetpoint(rpm);
  }

  @Override
  public void setSpeed(double speed, double speedBack) {
    this.speed = speed;
    this.viaPid = false;
  }

  @Override
  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  @Override
  public void stop() {
    viaPid = false;
    this.speed = 0;
    this.rpm = 0;
    motor.set(0);
  }
}
