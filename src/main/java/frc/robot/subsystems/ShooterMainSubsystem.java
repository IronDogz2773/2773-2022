// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterMainSubsystem extends ShooterBaseSubsystem{
  private final CANSparkMax backMotor = new CANSparkMax(Constants.frontShooterCANID, MotorType.kBrushless);
  private final CANSparkMax frontMotor = new CANSparkMax(Constants.backShooterCANID, MotorType.kBrushless);
  private final CANSparkMax indexerMotor = new CANSparkMax(Constants.indexerCANID, MotorType.kBrushless);
  private final RelativeEncoder backEncoder = backMotor.getEncoder();
  private final RelativeEncoder frontEncoder = frontMotor.getEncoder();

  private boolean extended = false;
  private final Timer timer = new Timer();

  /** Creates a new ShooterSubsystem. */
  public ShooterMainSubsystem() {

  }

  @Override
  public void periodic() {
    if (extended && timer.hasElapsed(1)){
      retractIndex();
    }
  }

  @Override
  public void setRpm(double rpm) {
    
  }

  @Override
  public void setSpeed(double speedFront, double speedBack) {
    backMotor.set(speedFront);
    frontMotor.set(speedBack);
  }

  @Override
  public boolean atSetpoint() {
    return false;
  }

  @Override
  public void extendIndex() {
    timer.reset();
    timer.start();
    extended = true;
    indexerMotor.set(.1);
  }

  @Override
  public void retractIndex() {
    timer.stop();
    indexerMotor.set(0);
    extended = false;
  }

  @Override
  public void stop() {
    
  }
}
