// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final Spark motor = new Spark(0);
  private final Spark indexer = new Spark(1);
  private final Encoder encoder = new Encoder(6, 7);
  private final PIDController pid = new PIDController(0.00006, 0, 0.00002);

  private double rpm = 0.0;
  private double speed = 0.0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    encoder.setDistancePerPulse(-60.0 / 8192);
    pid.setSetpoint(rpm);
  }

  @Override
  public void periodic() {
    var delta = pid.calculate(encoder.getRate());
    speed = MathUtil.clamp(speed + delta, 0, 1);
    motor.set(speed);
  }

  public void setRpm(double rpm){
      this.rpm = rpm;
      pid.setSetpoint(rpm);
  }

  public void extendIndex(){
    indexer.set(0.2);
  }

  public void retractIndex(){
    indexer.set(0);
  }
}
