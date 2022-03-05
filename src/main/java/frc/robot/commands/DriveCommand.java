// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DistanceSystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class DriveCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem drive;
  private final DistanceSystem distance;
  private final Joystick gamepad;

  /**
   * Creates a new DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem drive, DistanceSystem distance, Joystick gamepad) {
    this.drive = drive;
    this.distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.gamepad = gamepad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled
  // Repeated constantly
  @Override
  public void execute() {

    // negation here might have broken code idk we couldnt test
    double leftSpeed = -gamepad.getRawAxis(Constants.lStickY) * Constants.speedFactor;
    double rightSpeed = -gamepad.getRawAxis(Constants.rStickY) * Constants.speedFactor;

    if ((leftSpeed > 0 || rightSpeed > 0) && distance.tooCloseToWall()) {
      drive.stop();
      return;
    }
    drive.rawDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
