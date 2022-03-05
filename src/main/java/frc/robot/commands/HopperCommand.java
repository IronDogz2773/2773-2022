// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.jsontype.impl.SubTypeValidator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

/** Funnels the ball from the intake to the indexer */
public class HopperCommand extends CommandBase {
  private final HopperSubsystem hopper;

  /** Creates a new HopperCommand. */
  public HopperCommand(HopperSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    hopper = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.motorToggle();
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
    return true;
  }
}
