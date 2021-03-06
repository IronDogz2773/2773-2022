
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCommand extends CommandBase {
  private final IntakeSubsystem intake;

  /** Creates a new DeployIntakeCommand. */
  public DeployIntakeCommand(IntakeSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    intake = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // toggle. if intake is deployed, retract. else, deploy
    if (intake.isDeployed() == false) {
      intake.deployIntake();
    } else {
      intake.retractIntake();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ends immediatly
    return true;
  }
}