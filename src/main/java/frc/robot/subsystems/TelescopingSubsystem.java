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
  private final CANSparkMax leftMotor = new CANSparkMax(Constants.leftTelescopeCANID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(Constants.rightTelescopeCANID, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private double leftMaxDistance = 168;
  private double rightMaxDistance = 165;

  public double speed = .5;

  NetworkTableEntry positionLeftEntry;
  NetworkTableEntry positionRightEntry;

  /** Creates a new TestMotorSubsystem. */
  public TelescopingSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("climber");
    positionLeftEntry = table.getEntry("positionLeft");
    positionRightEntry = table.getEntry("positionRight");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    positionLeftEntry.setDouble(leftEncoder.getPosition());
    positionRightEntry.setDouble(rightEncoder.getPosition());
  }

  public void resetDistance(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void leftMotorOn(){
    leftMotor.set(speed);
  }

  public void leftMotorOff(){
    leftMotor.stopMotor();
  }

  public void rightMotorOn(){
    rightMotor.set(speed);
  }

  public void rightMotorOff(){
    rightMotor.stopMotor();
  }

  public void toggleDirection(){
    speed = -speed;
  }

  public boolean leftAtSetpoint(){
    if(speed > 0){
      return leftEncoder.getPosition() >= leftMaxDistance;
    }
    else{
      return leftEncoder.getPosition() <= 0;
    }
  }

  public boolean rightAtSetpoint(){
    if(speed > 0){
      return rightEncoder.getPosition() >= rightMaxDistance;
    }
    else{
      return rightEncoder.getPosition() <= 0;
    }
  }
}
