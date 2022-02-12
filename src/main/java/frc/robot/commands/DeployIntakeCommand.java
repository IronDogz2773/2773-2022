// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCommand extends CommandBase {
  private final IntakeSubsystem intake;
  private final Joystick gamepad;
  /** Creates a new DeployIntakeCommand. */
  public DeployIntakeCommand(IntakeSubsystem subsystem, Joystick gamepad) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    intake = subsystem;
    this.gamepad = gamepad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.disableCompressor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(gamepad.getRawButton(Constants.LB)) {
      intake.deployIntake();
    } else if (gamepad.getRawButton(Constants.RB)) {
      intake.retractIntake();
    }
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
