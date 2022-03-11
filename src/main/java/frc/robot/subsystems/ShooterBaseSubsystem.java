// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ConstantsForMainRobot;

public abstract class ShooterBaseSubsystem extends SubsystemBase {

  private boolean vision;

  public ShooterBaseSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("shooter");
    NetworkTableEntry piDistanceEntry = table.getEntry("piDistance");
    NetworkTableEntry manualDistanceEntry = table.getEntry("manualDistance");
    piDistanceEntry.setDouble(0);
    manualDistanceEntry.setDouble(0);

    SmartDashboard.putBoolean("IsPiManual", vision);
  }

  public abstract void setRpm(double rpmFront, double rpmBack);

  public abstract void setSpeed(double speedFront, double speedBack);

  public void setSpeed(double speed) {
    setSpeed(speed, speed);
  }

  // In this array: first number is frontRpm, second is backRpm.
  private static final double[][] LutOfRpms = new double[][] {
      new double[] { 3800, 1200 },
      new double[] { 3800, 1350 },
      new double[] { 3800, 1600 },
      new double[] { 3800, 1700 },
      new double[] { 3800, 2100 },
      new double[] { 3800, 2400 },
      new double[] { 3800, 2600 },
      new double[] { 3800, 2675 },
      new double[] { 3750, 2725 },
      new double[] { 3700, 2800 }
  };
  private static int LUTOffset = 6;

  public void setNetworkRpm() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable shooterTable = inst.getTable("shooter");
    NetworkTable pivisionTable = inst.getTable("pivision");
    double distance;
    double frontRpm;
    double backRpm;

    NetworkTable coPilotTable = inst.getTable("coPilot");
    vision = coPilotTable.getEntry("distanceVision").getBoolean(true);
    if (false) {
      distance = pivisionTable.getEntry("retro_distance").getDouble(0);
    } else {
      if(coPilotTable.getEntry("high_goal").getBoolean(false)){
        distance = coPilotTable.getEntry("manualDistance").getDouble(0);
      }
      else{
        distance = 3;
      }
    }
    System.out.println(distance);

    if (distance < LUTOffset) {
      frontRpm = 700; // TODO check if it reaches low basket
      backRpm = 700;
    } else {
      if (distance > LUTOffset + LutOfRpms.length - 1) {
        distance = LUTOffset + LutOfRpms.length - 1;
      }
      int index = (int) Math.floor(distance - LUTOffset);
      if (index >= LutOfRpms.length - 1) {
        index = LutOfRpms.length - 2;
      }
      double fraction = distance - index - LUTOffset;
      frontRpm = LutOfRpms[index][0] * (1 - fraction) + LutOfRpms[index + 1][0] * fraction;
      backRpm = LutOfRpms[index][1] * (1 - fraction) + LutOfRpms[index + 1][1] * fraction;
    }

    setRpm(frontRpm, backRpm);
  }

  public double getDistance(double angle){
    return((Constants.topGoalHeight - Constants.cameraMountHeight) / (Math.tan(Math.toRadians(angle) + Constants.cameraMountAngle)));
  }

  public abstract boolean atSetpoint();

  public abstract void stop();
}
