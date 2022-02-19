// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class TurnDegreesCommand extends CommandBase {
  /** Creates a new NavigationCommand. */
  private final NavigationSubsystem nav;
  private final DriveSubsystem drive;

  PIDController pidController;

  private double rotation;
  private double angle;
  private double testAngle = 10;
  private double target;

  public TurnDegreesCommand(NavigationSubsystem nav, DriveSubsystem drive) {
    this.nav = nav;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    // navigation can be used by multiple commands at once without a problem so
    // it is not a dependency
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double kP = Constants.kP;
    double kI = Constants.kI;
    double kD = Constants.kD;

    pidController = new PIDController(kP, kI, kD);
    nav.resetGyroAngle();
    angle = testAngle;

    target = nav.getGyroAngle() + angle;
    pidController.setTolerance(.5);
    pidController.setSetpoint(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotation = pidController.calculate(nav.getGyroAngle());
    rotation = MathUtil.clamp(rotation, -1, 1);
    drive.arcadeDrive(0, rotation, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
