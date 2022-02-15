// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class NavigationSubsystem extends SubsystemBase {
  /** Creates a new NavigationSubsystem. */
  private final ADXRS450_Gyro gyroscope = new ADXRS450_Gyro();

  private final Encoder leftEncoder = new Encoder(Constants.leftEncoderPortA, Constants.leftEncoderPortB);
  private final Encoder rightEncoder = new Encoder(Constants.rightEncoderPortA, Constants.rightEncoderPortB);
  public NavigationSubsystem() {
      if(gyroscope.isConnected()){
       gyroscope.reset();
       gyroscope.calibrate();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getGyroAngle(){
      return gyroscope.getAngle();
  }
}
