// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
  private final PWMVictorSPX frontMotor = new PWMVictorSPX(Constants.frontConveyorMotorPort);
  private final PWMVictorSPX backMotor = new PWMVictorSPX(Constants.backConveyorMotorPort);

  private final MotorControllerGroup motors = new MotorControllerGroup(frontMotor, backMotor);

  private double speed = 0;

  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    motors.set(speed);
  }

  public void motorOn() {
    speed = 1;
  }

  public void motorOff() {
    speed = 0;
  }

}
