// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.PIDUtil;

public class TurnDegreesCommand extends CommandBase {
  /** Creates a new NavigationCommand. */
  private final NavigationSubsystem nav;
  private final DriveSubsystem drive;

  PIDController pidController;
  PIDUtil pidu;

  private double rotation;
  private double target;

  private double timeOut;
  private double lastAngleFromNT;

  private final Timer timer = new Timer();

  public TurnDegreesCommand(NavigationSubsystem nav, DriveSubsystem drive, double timeOut) {
    this.nav = nav;
    this.drive = drive;
    this.timeOut = timeOut;
    // Use addRequirements() here to declare subsystem dependencies.
    // navigation can be used by multiple commands at once without a problem so
    // it is not a dependency
    addRequirements(drive);

    double kP = Constants.turnControllerP;
    double kI = Constants.turnControllerI;
    double kD = Constants.turnControllerD;

    pidController = new PIDController(kP, kI, kD);
    pidu = new PIDUtil(pidController, "turn");

    lastAngleFromNT = 0;
    // resetPid(getNetworkTableAngle());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    resetPid(getNetworkTableAngle());
  }

  void resetPid(double angle) {
    nav.resetGyroAngle();

    target = nav.getGyroAngle() + angle;
    pidController.setTolerance(.2);
    pidu.setSetpoint(target);
    pidController.reset();
  }

  // returns 0 if angle did not change in network tables
  private double getNetworkTableAngle() {
    // creates instance of table with pivision
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("pivision");
    NetworkTableEntry presentEntry = table.getEntry("red_1_present");
    boolean present = presentEntry.getBoolean(false);
    // if there is no angle on screen, no angle/0 angle
    if (!present) {
      lastAngleFromNT = 0;
      return 0;
    }

    NetworkTableEntry angleEntry = table.getEntry("red_1_x");
    double angle = angleEntry.getDouble(0);

    if (lastAngleFromNT == angle) {
      // angle did not change
      return 0;
    }
    lastAngleFromNT = angle;
    return angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidu.update();
    if (!pidController.atSetpoint()) {
      DriverStation.reportWarning("angle = " + pidController.getSetpoint() + "; " + nav.getGyroAngle(), false);
      // calls from pid to give values to rotate
      rotation = pidu.calculate(nav.getGyroAngle());
      drive.tankDriveVolts(rotation, -rotation);
    } else {
      drive.stop();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.rawDrive(0, 0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeOut);
  }
}