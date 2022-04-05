// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConstantsForMainRobot;

public class LEDSubsytem extends SubsystemBase {
  private final DigitalOutput pin1 = new DigitalOutput(ConstantsForMainRobot.led1Port);
  private final DigitalOutput pin2 = new DigitalOutput(ConstantsForMainRobot.led2Port);

  private boolean rainbow = false;

  public LEDSubsytem() {
  }

  @Override
  public void periodic() {
    if (rainbow) {
      setRainbowRoad();
    } else {
      setTeamColor();
    }
  }

  public void runTheRainbow(boolean enable) {
    rainbow = enable;
  }
  

  private void setBlue() {
    pin1.set(true);
    pin2.set(false);
  }

  private void setRed() {
    pin1.set(false);
    pin2.set(true);
  }

  private void setRainbowRoad() {
    pin1.set(false);
    pin2.set(false);
  }
  private void setTeamColor() { 
    NetworkTable fmsInfoTable = NetworkTableInstance.getDefault().getTable("FMSInfo");
    boolean isRedTeam = fmsInfoTable.getEntry("IsRedAlliance").getBoolean(false);
    if (isRedTeam) {
      setRed();
    }
    else {
      setBlue();
    }


  }
}
