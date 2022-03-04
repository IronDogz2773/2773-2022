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
import edu.wpi.first.util.sendable.SendableBuilder.BackendKind;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDUtil;

public class ShooterMainSubsystem extends ShooterBaseSubsystem{
  private final CANSparkMax backMotor = new CANSparkMax(Constants.frontShooterCANID, MotorType.kBrushless);
  private final CANSparkMax frontMotor = new CANSparkMax(Constants.backShooterCANID, MotorType.kBrushless);
  private final CANSparkMax indexerMotor = new CANSparkMax(Constants.indexerCANID, MotorType.kBrushless);
  private final RelativeEncoder backEncoder = backMotor.getEncoder();
  private final RelativeEncoder frontEncoder = frontMotor.getEncoder();

  private final PIDController pidFront = new PIDController(Constants.shooterControllerP, Constants.shooterControllerI,
      Constants.shooterControllerD);
  private final PIDController pidBack = new PIDController(Constants.shooterControllerP, Constants.shooterControllerI,
      Constants.shooterControllerD);

      private PIDUtil pidu = new PIDUtil(pidFront, "tune_f");

  private double speedFront = 0.0;
  private double speedBack = 0.0;

  private double rpmFront = 0.0;
  private double rpmBack = 0.0;

  private boolean extended = false;
  private boolean viaPid = false;

  private final Timer timer = new Timer();

  /** Creates a new ShooterSubsystem. */
  public ShooterMainSubsystem() {
    pidu.setSetpoint(rpmFront);
    pidFront.setTolerance(20);

    pidBack.setSetpoint(rpmBack);
    pidBack.setTolerance(20);

    frontMotor.setInverted(true);
    backMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    if (extended && timer.hasElapsed(1)){
      retractIndex();
    }
    if (viaPid) {
      pidu.update();
      var deltaFront = pidu.calculate(frontEncoder.getVelocity());
      speedFront = MathUtil.clamp(speedFront + deltaFront, 0, 1);
      var deltaBack = pidBack.calculate(backEncoder.getVelocity());
      speedBack = MathUtil.clamp(speedBack + deltaBack, 0, 1);
      // if(rpmBack == 0){
      //   speedBack = 0;
      // }
      // if(rpmFront == 0){
      //   speedFront = 0;
      // }
    }
    frontMotor.set(speedFront);
    backMotor.set(speedBack);
    DriverStation.reportWarning("front " + frontEncoder.getVelocity() + " back " + backEncoder.getVelocity(), false);
  }

  @Override
  public void setRpm(double rpmFront, double rpmBack) {
    viaPid = true;
    this.rpmBack = rpmBack;
    this.rpmFront = rpmFront;

    pidu.setSetpoint(rpmFront);
    pidBack.setSetpoint(rpmBack);
  }

  @Override
  public void setSpeed(double speedFront, double speedBack) {
    viaPid = false;
    this.speedFront = speedFront;
    this.speedBack = speedBack;
  }

  @Override
  public boolean atSetpoint() {
    if (!viaPid) {
      return true;
    }
    return pidFront.atSetpoint() && pidBack.atSetpoint();
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
    viaPid = false;
    this.speedBack = 0;
    this.speedFront = 0;
    backMotor.set(speedBack);
    frontMotor.set(speedFront);
  }
}
