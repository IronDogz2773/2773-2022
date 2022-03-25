// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IndexerMainSubsystem extends IndexerBaseSubsystem {
  private final CANSparkMax indexMotor = new CANSparkMax(Constants.indexerCANID, MotorType.kBrushless);
  private final DigitalInput irSensor = new DigitalInput(Constants.irSensorPin);


  private double speed = 0;

  /** Creates a new IndexSubsystem. */
  public IndexerMainSubsystem() {}

  @Override
  public void periodic() {
    indexMotor.set(speed);
    System.out.println(irSensor.get());
  }

  @Override
  public void motorOn() {
    speed = -.75;
    }

  @Override
  public void motorOff() {
     speed = 0;
  }

  public void setSpeed(double speed){
    this.speed = speed;
  }

  @Override
  public boolean ballTooCloseToShooter(){
    return irSensor.get();
  }

  @Override
  public void reverseMotor() {
     speed = .5;
  }
}

