// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

public class HopperForIndexCommand extends CommandBase {
  private final HopperSubsystem hopper;
  private final Joystick gamepad;

  /** Creates a new HopperForIndexCommand. */
  public HopperForIndexCommand(HopperSubsystem hopper, Joystick gamepad) {
    this.hopper = hopper;
    this.gamepad = gamepad;
    addRequirements(hopper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(gamepad.getRawAxis(Constants.RT) > .2) {
      hopper.motorOn();
    }
    else{
      hopper.motorOff();
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
