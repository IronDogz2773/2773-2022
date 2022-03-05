// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class HopperSubsystem extends SubsystemBase {
  private final CANSparkMax leftHopperCAN = new CANSparkMax(Constants.leftHopperCANID, MotorType.kBrushless);
  private final CANSparkMax rightHopperCAN = new CANSparkMax(Constants.rightHopperCANID, MotorType.kBrushless);
  private final MotorControllerGroup motors = new MotorControllerGroup(leftHopperCAN, rightHopperCAN);

  private double speed = 0;

  /** Creates a new ConveyorSubsystem. */
  public HopperSubsystem() {
    leftHopperCAN.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    motors.set(speed);
  }

  public void motorOn() {
    speed = .5;
  }

  public void motorOff() {
    speed = 0;
  }

  public void motorToggle() {
    if (speed != 0) {
      this.motorOff();
    } else {
      this.motorOn();
    }
  }

  public double getSpeed() {
    return speed;
  }

}
