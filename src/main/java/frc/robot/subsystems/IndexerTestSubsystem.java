// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class IndexerTestSubsystem extends IndexerBaseSubsystem {
  private final Spark kicker = new Spark(Constants.kickerMotorPWMID);
  private final DigitalInput irSensor = null;

  private double speed = 0;

  /** Creates a new IndexSubsystem. */
  public IndexerTestSubsystem() {}

  @Override
  public void periodic() {
    kicker.set(speed);
  }

  @Override
  public void motorOn() {
    speed = -.5;
  }

  @Override
  public void motorOff() {
     speed = 0;
  }

  @Override
  public boolean ballTooCloseToShooter(){
    return false;
  }

  @Override
  public void reverseMotor() {
     speed = .2;
  }
}

