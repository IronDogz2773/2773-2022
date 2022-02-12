// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  //Initializes the intake motor
  private final PWMVictorSPX intakeMotor = new PWMVictorSPX(Constants.intakePWMID);
  private final Compressor pcmCompressor = new Compressor(Constants.activateIntake, PneumaticsModuleType.CTREPCM);

  private double speed = 0;

  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeMotor.set(speed);
  }

  public void motorOn() {
    speed = 1;
  }

  public void motorOff() {
    speed = 0;
  }

  public double getSpeed() {
    return speed;
  }

  public void disablePneumatics() {
    pcmCompressor.enableDigital();
    pcmCompressor.disable();
  }

  public void deployIntake() {
  }

  public void retractIntake() {

  }
}
