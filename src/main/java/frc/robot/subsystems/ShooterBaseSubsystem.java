// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ShooterBaseSubsystem extends SubsystemBase {

  private boolean manual = true;

  public ShooterBaseSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("shooter");
    NetworkTableEntry piDistanceEntry = table.getEntry("piDistance");
    NetworkTableEntry manualDistanceEntry = table.getEntry("manualDistance");
    piDistanceEntry.setDouble(0);
    manualDistanceEntry.setDouble(0);
  }

  public abstract void setRpm(double rpmFront, double rpmBack);

  public abstract void setSpeed(double speedFront, double speedBack);

  public void setSpeed(double speed) {
    setSpeed(speed, speed);
  }

  public void setNetworkRpm() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("shooter");
    double distance;
    double frontRpm;
    double backRpm;
    if(!manual){
      distance = table.getEntry("piDistance").getDouble(0); //TODO change this to match preston's angle name
    }
    else{
      distance = table.getEntry("manualDistance").getDouble(0);
    }

    switch ((int) Math.round(distance)){
      case 6:
        frontRpm = 3800;
        backRpm = 1200;
        break;
      case 7:
        frontRpm = 3800;
        backRpm = 1350;
        break;
      case 8:
        frontRpm = 3800;
        backRpm = 1600;
        break;
      case 9:
        frontRpm = 3800;
        backRpm = 1700;
        break;
      case 10:
        frontRpm = 3800;
        backRpm = 2100;
        break;
      case 11:
        frontRpm = 3800;
        backRpm = 2400;
        break;
      case 12:
        frontRpm = 3800;
        backRpm = 2600;
        break;
      case 13:
        frontRpm = 3800;
        backRpm = 2675;
      case 14:
        frontRpm = 3750;
        backRpm = 1725;
        break;
      case 15:
        frontRpm = 3700;
        backRpm = 2800;
        break;
      default:
        frontRpm = 0;
        backRpm = 0;
        break;
    }

    setRpm(frontRpm, backRpm);
  }

  public abstract boolean atSetpoint();

  public abstract void stop();
}
