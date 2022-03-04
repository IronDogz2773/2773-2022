// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

public class TurnTrajectoryCommand extends CommandBase {
  private final DriveSubsystem drive;
  private final NavigationSubsystem nav;
  private Command turnCommand;
  private double angle = 0.0;

  /** Creates a new TurnTrajectoryCommand. */
  public TurnTrajectoryCommand(DriveSubsystem drive, NavigationSubsystem nav) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.nav = nav;

    addRequirements(drive);

  }

  private double getNetworkTableAngle() {
    // creates instance of table with pivision
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("pivision");

    NetworkTableEntry angleEntry = table.getEntry("red_1_x");
    double angle = angleEntry.getDouble(0);

    return angle;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = getNetworkTableAngle();
    Command turnCommand = PathCommandBuilder.fromAngle(drive, nav, angle).build();
    this.turnCommand = turnCommand;
    turnCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turnCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnCommand.isFinished();
  }
}
