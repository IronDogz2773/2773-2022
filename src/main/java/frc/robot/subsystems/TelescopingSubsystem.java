// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopingSubsystem extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(13, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  private double distance;
  private boolean atMax;

  NetworkTableEntry positionEntry;
  
  /** Creates a new TestMotorSubsystem. */
  public TelescopingSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("climber");
    positionEntry = table.getEntry("position");
    distance = 15;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    positionEntry.setDouble(encoder.getPosition());
  }

  public void resetDistance(){
    encoder.setPosition(0);
  }

  public void motorOn(){
    motor.set(0.1);
  }

  public void motorBack(){
    motor.set(-0.1);
  }

  public void motorOff(){
    motor.stopMotor();
  }

  public boolean atDistance(){
    return encoder.getPosition() >= distance;
  }

  public boolean atStart(){
    return encoder.getPosition() <= 0;
  }
}
