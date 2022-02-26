// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ActivateIntakeCommand;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PathCommandBuilder;
import frc.robot.commands.ShotRpmCommand;
import frc.robot.commands.TurnDegreesCommand;
import frc.robot.commands.HopperCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = Constants.intakePresent ? new IntakeSubsystem() : null;
  private final NavigationSubsystem navigationSubsystem = new NavigationSubsystem();
  private final HopperSubsystem hopperSubsystem = Constants.hopperPresent ? new HopperSubsystem() : null;

  // Commands
  private final ActivateIntakeCommand activateIntakeCommand = Constants.intakePresent
      ? new ActivateIntakeCommand(intakeSubsystem, gamepad)
      : null;
  private final DeployIntakeCommand deployIntakeCommand = Constants.intakePresent
      ? new DeployIntakeCommand(intakeSubsystem)
      : null;
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem, gamepad);
  private final TurnDegreesCommand turnDegreesCommand = new TurnDegreesCommand(navigationSubsystem, driveSubsystem);
  private final HopperCommand hopperCommand = Constants.hopperPresent ? new HopperCommand(hopperSubsystem) : null;

  private final ShooterSubsystem shooter = new ShooterSubsystem();
  //private final ShotCommand shotCommand = new ShotCommand(shooter, gamepad);

  //private final IndexCommand indexCommand = new IndexCommand(shooter);

  // private static Joystick joystick = new Joystick(Constants.joystickPort);
  private static Joystick gamepad = new Joystick(Constants.gamepadPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(driveCommand);
    if (Constants.intakePresent) {
      intakeSubsystem.setDefaultCommand(activateIntakeCommand);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton turnButton = new JoystickButton(gamepad, Constants.X);
    turnButton.whenHeld(turnDegreesCommand, true);

    final JoystickButton resetPose = new JoystickButton(gamepad, Constants.Start);
    resetPose.whenPressed(() -> {
      // command to reset all gyro and coordanates
      navigationSubsystem.resetGyroAngle();
      navigationSubsystem.resetEncoder();
      navigationSubsystem.resetOdometry(new Pose2d());
    });

    //when RB is pressed, set rpm to a specific value, extend the indexer to touch the ball, wait, then turn off
    final JoystickButton autoShootButton = new JoystickButton(gamepad, Constants.RB);
    final Command autoShootCommand = new ShotRpmCommand(shooter, 500).andThen(() -> {
      shooter.extendIndex();
    }).andThen(new WaitCommand(1)).andThen(() -> {
      shooter.setRpm(0);
      shooter.retractIndex();
    });
    autoShootButton.whenPressed(autoShootCommand, true);

    if (Constants.intakePresent) {
      final JoystickButton deployButton = new JoystickButton(gamepad, Constants.A);
      deployButton.whenPressed(deployIntakeCommand, true);
    }

    if (Constants.hopperPresent) {
      final JoystickButton hopperButton = new JoystickButton(gamepad, Constants.B);
      hopperButton.whenPressed(hopperCommand, true);
    }

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An DriveCommand will run in autonomous
    PathCommandBuilder builder = new PathCommandBuilder(driveSubsystem, navigationSubsystem);
    return builder.build();
  }
}
