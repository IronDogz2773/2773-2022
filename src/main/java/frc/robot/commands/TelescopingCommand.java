// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopingSubsystem;

public class TelescopingCommand extends CommandBase {
  private final TelescopingSubsystem telescope;
  private final Joystick gamepad;

  /** Creates a new TelescopingCommand. */
  public TelescopingCommand(TelescopingSubsystem telescope, Joystick gamepad) {
    this.telescope = telescope;
    this.gamepad = gamepad;
    addRequirements(telescope);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var noProtection = gamepad.getPOV() == 90;
    //if left trigger pressed past buffer zone of .2, and either has no protection or isn't at the setpoint: move the left motor
    if(gamepad.getRawAxis(Constants.LT) > .2 && 
       (noProtection || !telescope.leftAtSetpoint())){
      telescope.leftMotorOn();
      if (noProtection) telescope.resetDistance(); //constantly resets encoders for left motor if no protection
    }
    else{
      telescope.leftMotorOff();
    }

    //if right trigger pressed past buffer zone of .2, and either has no protection or isn't at the setpoint: move the right motor
    if(gamepad.getRawAxis(Constants.RT) > .2 &&
     (noProtection || !telescope.leftAtSetpoint())){
      telescope.rightMotorOn();
      if (noProtection) telescope.resetDistance(); //constantly resets encoders for right motor if no protection
    }
    else{
      telescope.rightMotorOff();
    }
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
