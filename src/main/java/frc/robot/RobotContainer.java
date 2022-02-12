// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ActivateIntakeCommand;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
  private final WinchSubsystem winchSubsystem = new WinchSubsystem();

  //Commands
  private final ActivateIntakeCommand activateIntakeCommand = new ActivateIntakeCommand(intakeSubsystem);
  private final DeployIntakeCommand deployIntakeCommand = new DeployIntakeCommand(intakeSubsystem, gamepad);
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem, gamepad);

  private static Joystick joystick = new Joystick(Constants.joystickPort);
  private static Joystick gamepad = new Joystick(Constants.gamepadPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(driveCommand);
    intakeSubsystem.setDefaultCommand(deployIntakeCommand);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*final JoystickButton intakeButton = new JoystickButton(gamepad, Constants.A);
    intakeButton.whenPressed(activateIntakeCommand, false);
    final JoystickButton deployButton = new JoystickButton(gamepad, Constants.LB);
    deployButton.whenPressed(deployIntakeCommand, false);*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An DriveCommand will run in autonomous
    return null;
  }
}
