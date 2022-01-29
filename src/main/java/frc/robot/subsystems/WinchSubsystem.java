// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WinchSubsystem extends SubsystemBase {
  /** Creates a new WinchSubsystem. */
  private final PWMVictorSPX winchMotor = new PWMVictorSPX(Constants.winchMotorID);

  public WinchSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
