// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;


public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem drive;
  private final Joystick gamepad;

  private double prevLeftSpeed;
  private double prevRightSpeed;

  private double alpha;

  /**
   * Creates a new DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem subsystem, Joystick gamepad) {
    drive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.gamepad = gamepad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevLeftSpeed = 0.0;
    prevRightSpeed = 0.0;

    alpha = 0.85;

  }

  // Called every time the scheduler runs while the command is scheduled
  // Repeated constantly
  @Override
  public void execute() {

    /*
    OLD CODE
    double leftSpeed = gamepad.getRawAxis(Constants.lStickY) * Constants.speedFactor;
    double rightSpeed = -gamepad.getRawAxis(Constants.rStickY) * Constants.speedFactor;
    */

    double leftRawSpeed = gamepad.getRawAxis(Constants.lStickY) * Constants.speedFactor;
    double rightRawSpeed = -gamepad.getRawAxis(Constants.rStickY )* Constants.speedFactor;

    // prev% + current%
    double leftSpeed = prevLeftSpeed * (1- alpha) + leftRawSpeed*alpha;
    double rightSpeed = prevRightSpeed * (1- alpha) + rightRawSpeed*alpha;

    drive.rawDrive(leftSpeed, rightSpeed);

    prevLeftSpeed = leftSpeed;
    prevRightSpeed = rightSpeed;
  }




  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
