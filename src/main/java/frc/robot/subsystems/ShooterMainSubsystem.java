// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class ShooterMainSubsystem extends ShooterBaseSubsystem {
  private final CANSparkMax backMotor = new CANSparkMax(Constants.frontShooterCANID, MotorType.kBrushless);
  private final CANSparkMax frontMotor = new CANSparkMax(Constants.backShooterCANID, MotorType.kBrushless);
  private final RelativeEncoder backEncoder = backMotor.getEncoder();
  private final RelativeEncoder frontEncoder = frontMotor.getEncoder();

  private final PIDController pidFront = new PIDController(Constants.shooterControllerP, Constants.shooterControllerI,
      Constants.shooterControllerD);
  private final PIDController pidBack = new PIDController(Constants.shooterControllerP, Constants.shooterControllerI,
      Constants.shooterControllerD);

  private double speedFront = 0.0;
  private double speedBack = 0.0;

  private double rpmFront = 0.0;
  private double rpmBack = 0.0;

  private boolean viaPid = false;

  NetworkTableEntry frontEntry;
  NetworkTableEntry backEntry;

  /** Creates a new ShooterSubsystem. */
  public ShooterMainSubsystem() {
    pidFront.setSetpoint(rpmFront);
    pidFront.setTolerance(10);

    pidBack.setSetpoint(rpmBack);
    pidBack.setTolerance(10);

    frontMotor.setInverted(false);
    backMotor.setInverted(false);

  }

  @Override
  public void periodic() {
    if (viaPid) {
      var deltaFront = pidFront.calculate(frontEncoder.getVelocity());
      speedFront = MathUtil.clamp(speedFront + deltaFront, 0, 1);
      var deltaBack = pidBack.calculate(backEncoder.getVelocity());
      speedBack = MathUtil.clamp(speedBack + deltaBack, 0, 1);
    }
    if(frontEncoder.getVelocity() != 0){
      System.out.println("front: " + frontEncoder.getVelocity() + " back: " + backEncoder.getVelocity());
    }
    frontMotor.set(speedFront);
    backMotor.set(speedBack);
  }

  @Override
  public void setRpm(double rpmFront, double rpmBack) {
    viaPid = true;
    this.rpmBack = rpmBack;
    this.rpmFront = rpmFront;

    pidFront.setSetpoint(rpmFront);
    pidBack.setSetpoint(rpmBack);
  }

  @Override
  public void setSpeed(double speedFront, double speedBack) {
    viaPid = false;
    this.speedFront = speedFront;
    this.speedBack = speedBack;
  }

  @Override
  public boolean atSetpoint() {
    if (!viaPid) {
      return true;
    }
    return pidFront.atSetpoint() && pidBack.atSetpoint();
  }

  @Override
  public void stop() {
    viaPid = false;
    this.speedBack = 0;
    this.speedFront = 0;
    backMotor.set(speedBack);
    frontMotor.set(speedFront);
  }
}
