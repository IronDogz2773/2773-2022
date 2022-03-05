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
import frc.robot.commands.ReverseIndexCommand;
import frc.robot.commands.ShotCommand;
import frc.robot.commands.TurnDegreesCommand;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IndexTimedCommand;
import frc.robot.commands.MultistepAutoBuilder;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerBaseSubsystem;
import frc.robot.subsystems.IndexerMainSubsystem;
import frc.robot.subsystems.IndexerTestSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterBaseSubsystem;
import frc.robot.subsystems.ShooterMainSubsystem;
import frc.robot.subsystems.ShooterTestSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final ShooterBaseSubsystem shooterSubsystem = Constants.dosShooter ? new ShooterMainSubsystem()
      : new ShooterTestSubsystem();
  private final IntakeSubsystem intakeSubsystem = Constants.intakePresent ? new IntakeSubsystem() : null;
  private final NavigationSubsystem navigationSubsystem = new NavigationSubsystem();
  private final HopperSubsystem hopperSubsystem = Constants.hopperPresent ? new HopperSubsystem() : null;
  private final IndexerBaseSubsystem indexerSubsystem = Constants.indexerPresent ? new IndexerMainSubsystem()
      : new IndexerTestSubsystem();

  // Commands
  private final Command activateIntakeCommand = Constants.intakePresent
      ? new ActivateIntakeCommand(intakeSubsystem, gamepadPilot)
      : doNothing();
  private final Command deployIntakeCommand = Constants.intakePresent
      ? new DeployIntakeCommand(intakeSubsystem)
      : doNothing();
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem, navigationSubsystem, gamepadPilot);
  private final TurnDegreesCommand turnDegreesCommand = new TurnDegreesCommand(navigationSubsystem, driveSubsystem,
      Constants.turnCmdTimeOut);
  private final Command hopperCommand = Constants.hopperPresent ? new HopperCommand(hopperSubsystem) : doNothing();

  private final ShotCommand shotCommand = new ShotCommand(shooterSubsystem, gamepadPilot);
  // private final ShotRpmCommand shotRpmCommand = new
  // ShotRpmCommand(shooterSubsystem, 1000, 1000);

  private final IndexCommand indexCommand = new IndexCommand(shooterSubsystem);

  // private static Joystick joystick = new Joystick(Constants.joystickPort);
  private static Joystick gamepadPilot = new Joystick(Constants.gamepadPortPilot);
  private static Joystick gamepadCopilot = new Joystick(Constants.gamepadPortCopilot);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // default commands
    driveSubsystem.setDefaultCommand(driveCommand);
    shooterSubsystem.setDefaultCommand(shotCommand);
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
    // Pilot buttons
    // A held, run hopper and index
    final JoystickButton indexHopperButton = new JoystickButton(gamepadPilot, Constants.A);
    Command indexHopperCommand = new ParallelCommandGroup(
        indexCommand,
        hopperCommand);
    indexHopperButton.whenHeld(indexHopperCommand);

    if (Constants.indexerPresent) {
      // B pressed, run index for a short amount of time
      final JoystickButton indexTimedButton = new JoystickButton(gamepadPilot, Constants.B);
      Command indexTimedCommand = new IndexTimedCommand(indexerSubsystem);
      indexTimedButton.whenPressed(indexTimedCommand);

      // X held, run index backward
      final JoystickButton reverseIndexButton = new JoystickButton(gamepadPilot, Constants.X);
      Command reverseIndexCommand = new ReverseIndexCommand(indexerSubsystem);
      reverseIndexButton.whenHeld(reverseIndexCommand);
    }

    if (Constants.hopperPresent) {
      // RT held, run intake and hopper
      final JoystickButton intakeHopperButton = new JoystickButton(gamepadPilot, Constants.RT);
      Command intakeHopperCommand = new ParallelCommandGroup(
          activateIntakeCommand, hopperCommand);
      intakeHopperButton.whenHeld(intakeHopperCommand);
    }

    // RB pressed, toggle slowmode
    final JoystickButton slowButton = new JoystickButton(gamepadPilot, Constants.RB);
    final CommandBase toggleSlowModeCommand = new CommandBase() {
      @Override
      public void initialize() {
        driveCommand.toggleSlowMode();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    slowButton.whenPressed(toggleSlowModeCommand);

    // LB pressed, toggle drive direction
    final JoystickButton directionButton = new JoystickButton(gamepadPilot, Constants.LB);
    final Command toggleDirectionCommand = new CommandBase() {
      @Override
      public void initialize() {
        driveCommand.toggleDirection();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    directionButton.whenPressed(toggleDirectionCommand);

    // LT held, firing sequence
    final JoystickButton firingTrigger = new JoystickButton(gamepadPilot, Constants.LT);
    final Command fireCommand = new CommandBase() {
      public void initialize() {
        indexerSubsystem.motorOn();
      }

      @Override
      public void end(boolean interupted) {
        indexerSubsystem.motorOff();
      }
    };
    firingTrigger.whenHeld(fireCommand, true);

    // either VISION AIM, run kicker backwards briefly, flywheel up to speed
    // or kicker backwards then flywheel up to speed if vision is disabled

    // Select pressed, toggle intake pneumatic
    final JoystickButton deployIntakeButton = new JoystickButton(gamepadPilot, Constants.Select);
    deployIntakeButton.whenPressed(deployIntakeCommand);

  }

  private static Command doNothing() {
    return new CommandBase() {
      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An DriveCommand will run in autonomous
    return new MultistepAutoBuilder(driveSubsystem, navigationSubsystem, shooterSubsystem, indexerSubsystem).build();
  }
}
