// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkMax indexMotor = new CANSparkMax(Constants.frontShooterCANID, MotorType.kBrushless);

  private double speed = 0;

  /** Creates a new IndexSubsystem. */
  public IndexerSubsystem() {}

  @Override
  public void periodic() {
    indexMotor.set(speed);
  }

  public void motorOn() {
    speed = .5;
  }

  public void motorOff() {
     speed = 0;
  }

  public void reverseMotor() {
     speed = -.5;
  }
}

