// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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
    manualDistanceEntry.setDouble(6); // default start distance

    SmartDashboard.putBoolean("IsPiManual", vision);
  }

  public abstract void setRpm(double rpmFront, double rpmBack);

  public abstract void setSpeed(double speedFront, double speedBack);

  public void setSpeed(double speed) {
    setSpeed(speed, speed);
  }

  // In this array: first number is frontRpm, second is backRpm.
  private static final double[][] LutOfRpms = new double[][] {
      new double[] { 2300, 2100 }, // 6
      new double[] { 2400, 2000 }, // 7
      new double[] { 2600, 1900 }, // 8
      new double[] { 2700, 1800 }, // 9
      new double[] { 4500,  100 }, // 10
      new double[] { 5000,  100 }, // 11
  };
  private static int LUTOffset = 6;

  public void setNetworkRpm() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable shooterTable = inst.getTable("shooter");
    double frontRpm = shooterTable.getEntry("front").getDouble(0);
    double backRpm = shooterTable.getEntry("back").getDouble(0);
    setRpm(frontRpm, backRpm);
  }

  public void setDetectedRpm() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable piVisionTable = inst.getTable("pivision");

    double distance;
    NetworkTable coPilotTable = inst.getTable("coPilot");
    vision = coPilotTable.getEntry("distanceVision").getBoolean(true);

    if (vision) {
      // TODO change this to match preston's angle name
      if(piVisionTable.getEntry("retro_present").getBoolean(false)){
        distance = piVisionTable.getEntry("retro_distance").getDouble(0);

      }
      else{
        distance = 0;
      }
    } else {
      NetworkTable table = inst.getTable("shooter");
      NetworkTableEntry manualDistanceEntry = table.getEntry("manualDistance");
      distance = manualDistanceEntry.getDouble(6);
      if (distance == 0) {
        // Distance set to 0, fallback to nt values.
        setNetworkRpm();
        return;
      }
    }

    double frontRpm, backRpm;
    if (distance < LUTOffset) {
      frontRpm = 1000; // TODO change to 2150?
      backRpm = 2000;
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

  public double getDistance(double angle) {
    return ((Constants.topGoalHeight - Constants.cameraMountHeight)
        / (Math.tan(Math.toRadians(angle) + Constants.cameraMountAngle)));
  }

  public abstract boolean atSetpoint();

  public abstract void stop();
}
