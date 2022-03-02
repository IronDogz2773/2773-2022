// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterBaseSubsystem;

public class ShotCommand extends CommandBase {
  private final ShooterBaseSubsystem subsystem;
  private final Joystick gamepad;

  /** Creates a new ShotCommand. */
  public ShotCommand(ShooterBaseSubsystem subsystem, Joystick gamepad) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
    this.gamepad = gamepad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //subsystem.setSpeed(gamepad.getRawAxis(Constants.lTrigger), gamepad.getRawAxis(Constants.rTrigger));
    int p = 3;
    subsystem.setRpm(Math.round(p*gamepad.getRawAxis(Constants.lTrigger))*5000/p,Math.round(p*gamepad.getRawAxis(Constants.rTrigger))*5000/p);
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
