// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  //Initializes the intake motor
  private final CANSparkMax intakeCAN = new CANSparkMax(Constants.intakeCANID, MotorType.kBrushless);
  private final Solenoid leftSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.leftSolenoidPCM);
  private final Solenoid rightSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.rightSolenoidPCM);

  public double speed = 0;

  public IntakeSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeCAN.set(speed);
  }

  public void motorOn() {
    speed = 1;
  }

  public void motorOff() {
    speed = 0;
  }

  public void motorToggle() {
    if(speed != 0) {
      this.motorOff();
    } else {
      this.motorOn();
    }
  }

  public double getSpeed() {
    return speed;
  }

  public void deployIntake() {
    leftSolenoidPCM.set(true);
    rightSolenoidPCM.set(false);
  }

  public void retractIntake() {
    leftSolenoidPCM.set(false);
    rightSolenoidPCM.set(true);
  }

}
