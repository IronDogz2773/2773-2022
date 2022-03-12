// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class IndexerMainSubsystem extends IndexerBaseSubsystem {
  private final CANSparkMax indexMotor = new CANSparkMax(Constants.indexerCANID, MotorType.kBrushless);

  private double speed = 0;

  /** Creates a new IndexSubsystem. */
  public IndexerMainSubsystem() {}

  @Override
  public void periodic() {
    indexMotor.set(speed);
  }

  @Override
  public void motorOn() {
    speed = -1;
    }

  @Override
  public void motorOff() {
     speed = 0;
  }

  public void setSpeed(double speed){
    this.speed = speed;
  }

  @Override
  public void reverseMotor() {
     speed = .5;
  }
}

