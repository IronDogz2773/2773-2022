// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  private final PWMVictorSPX leftForMotor = new PWMVictorSPX(Constants.leftForWheelsPort);
  private final PWMVictorSPX rightForMotor = new PWMVictorSPX(Constants.rightForWheelsPort);
  private final PWMVictorSPX leftBackMotor = new PWMVictorSPX(Constants.leftBackWheelsPort);
  private final PWMVictorSPX rightBackMotor = new PWMVictorSPX(Constants.rightBackWheelsPort);
  

  // Speed controller groups
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftForMotor, leftBackMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightForMotor, rightBackMotor);

  // Differential drive
  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);


  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void rawDrive(final double leftSpeed, final double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
}
}

