// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class ShooterBaseSubsystem extends SubsystemBase {

  public abstract void setRpm(double rpmFront, double rpmBack);

  public abstract void setSpeed(double speedFront, double speedBack);

  public void setSpeed(double speed){
    setSpeed(speed, speed);
  }

  public abstract boolean atSetpoint();

  public abstract void stop();
}
