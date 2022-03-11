// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.node.DoubleNode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ActivateIntakeCommand;
import frc.robot.commands.AutoShootBuilder;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveForTimeCommand;
import frc.robot.commands.ReverseIndexCommand;
import frc.robot.commands.ShotCommand;
import frc.robot.commands.TelescopingCommand;
import frc.robot.commands.TurnDegreesCommand;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.HopperForIndexCommand;
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
import frc.robot.subsystems.TelescopingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
  private final TelescopingSubsystem telescopingSubsystem = Constants.climberPresent ? new TelescopingSubsystem()
      : null;

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
  private final Command hopperForIndexCommand = Constants.hopperPresent
      ? new HopperForIndexCommand(hopperSubsystem, gamepadPilot)
      : doNothing();
  private final Command telescopingCommand = Constants.climberPresent
      ? new TelescopingCommand(telescopingSubsystem, gamepadCopilot)
      : doNothing();
  
  private final ShotCommand shotCommand = new ShotCommand(shooterSubsystem);
  // private final ShotRpmCommand shotRpmCommand = new
  // ShotRpmCommand(shooterSubsystem, 1000, 1000);

  private final IndexCommand indexCommand = new IndexCommand(indexerSubsystem, gamepadPilot);



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
    if (Constants.climberPresent) {
      telescopingSubsystem.setDefaultCommand(telescopingCommand);
    }
    intakeSubsystem.setDefaultCommand(activateIntakeCommand);
    hopperSubsystem.setDefaultCommand(hopperForIndexCommand);
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

    if (Constants.hopperPresent) {
      // RB held, run intake and hopper
      final JoystickButton idexHopperButton = new JoystickButton(gamepadPilot, Constants.RB);
      Command intakeHopperCommand = new ParallelCommandGroup(
          indexCommand, hopperCommand);
      idexHopperButton.whenHeld(intakeHopperCommand);
    }

    // A pressed, toggle slowmode
    final JoystickButton slowButton = new JoystickButton(gamepadPilot, Constants.A);
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

    // X pressed, toggle drive direction
    final JoystickButton directionButton = new JoystickButton(gamepadPilot, Constants.X);
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

    // LB pressed, firing sequence
    final JoystickButton firingTrigger = new JoystickButton(gamepadPilot, Constants.LB);
    final Command fireCommand = new AutoShootBuilder(shooterSubsystem, driveSubsystem, navigationSubsystem,
        indexerSubsystem, Constants.manual).build();
    firingTrigger.whenPressed(fireCommand, true); // TODO change to held

    if (Constants.intakePresent) {
      // Select pressed, toggle intake pneumatic
      final JoystickButton deployIntakeButton = new JoystickButton(gamepadPilot, Constants.Select);
      deployIntakeButton.whenPressed(deployIntakeCommand);
    }

    // CoPilot buttons
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable copilotTable = inst.getTable("coPilot");
    NetworkTableEntry turnVisionEntry = copilotTable.getEntry("turnVision");
    NetworkTableEntry distanceVisionEntry = copilotTable.getEntry("distanceVision");
    NetworkTableEntry proximityEntry = copilotTable.getEntry("proximity");
    NetworkTableEntry manualDistanceEntry = copilotTable.getEntry("manualDistance");

    POVButton manualUpButton = new POVButton(gamepadCopilot, 0);
    Command manualUpButtonCommand = new CommandBase(){
      @Override
      public void initialize(){
        manualDistanceEntry.setDouble(manualDistanceEntry.getDouble(0) + 1);
      }

      @Override
      public boolean isFinished(){
        return true;
      }
    };
    manualUpButton.whenPressed(manualUpButtonCommand);

    POVButton manualDownButton = new POVButton(gamepadCopilot, 180);
    Command manualDownButtonCommand = new CommandBase(){
      @Override
      public void initialize(){
        manualDistanceEntry.setDouble(manualDistanceEntry.getDouble(0) - 1);
      }

      @Override
      public boolean isFinished(){
        return true;
      }
    };
    manualDownButton.whenPressed(manualDownButtonCommand);

    JoystickButton toggleTurnVisionButton = new JoystickButton(gamepadCopilot, Constants.A);
    final Command toggleTurnVisionCommand = new CommandBase() {
      @Override
      public void initialize() {
        if (turnVisionEntry.getBoolean(true)) {
          turnVisionEntry.setBoolean(false);
        } else {
          turnVisionEntry.setBoolean(true);
        }
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    toggleTurnVisionButton.whenPressed(toggleTurnVisionCommand);

    JoystickButton toggleDistanceVisionButton = new JoystickButton(gamepadCopilot, Constants.B);
    final Command toggleDistanceVisionCommand = new CommandBase() {
      @Override
      public void initialize() {
        if (distanceVisionEntry.getBoolean(true)) {
          distanceVisionEntry.setBoolean(false);
        } else {
          distanceVisionEntry.setBoolean(true);
        }
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    toggleDistanceVisionButton.whenPressed(toggleDistanceVisionCommand);

    JoystickButton toggleProximityButton = new JoystickButton(gamepadCopilot, Constants.Y);
    final Command toggleProximityCommand = new CommandBase() {
      @Override
      public void initialize() {
        if (proximityEntry.getBoolean(true)) {
          proximityEntry.setBoolean(false);
        } else {
          proximityEntry.setBoolean(true);
        }
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    toggleProximityButton.whenPressed(toggleProximityCommand);

    JoystickButton toggleClimbDirectionButton = new JoystickButton(gamepadCopilot, Constants.RB);
    final Command toggleClimbDirectionCommand = new CommandBase() {
      @Override
      public void initialize() {
        telescopingSubsystem.toggleDirection();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    toggleClimbDirectionButton.whenPressed(toggleClimbDirectionCommand);

    JoystickButton resetTelescopePositionButton = new JoystickButton(gamepadCopilot, Constants.X);
    final Command resetTelescopePositionCommand = new CommandBase() {
      @Override
      public void initialize() {
        telescopingSubsystem.resetDistance();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    resetTelescopePositionButton.whenPressed(resetTelescopePositionCommand);
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
    //return new MultistepAutoBuilder(driveSubsystem, navigationSubsystem, shooterSubsystem, indexerSubsystem).build();

    //two ball auto
    /*
    Command autoShoot = new AutoShootBuilder(shooterSubsystem, driveSubsystem, navigationSubsystem, indexerSubsystem, Constants.dosShooter).build();
    Command autoShoot2 = new AutoShootBuilder(shooterSubsystem, driveSubsystem, navigationSubsystem, indexerSubsystem, Constants.dosShooter).build();
    Command AutoCommand = autoShoot.andThen(() -> {
      driveSubsystem.rawDrive(Constants.autoSpeed, Constants.autoSpeed);
      intakeSubsystem.motorOn();
    }).andThen(new WaitCommand(Constants.autoTime)).andThen(() -> {
      driveSubsystem.stop();
      intakeSubsystem.motorOff();
    }).andThen(autoShoot2);
    return AutoCommand;
    */

    //one ball auto
    return new AutoShootBuilder(shooterSubsystem, driveSubsystem, navigationSubsystem, indexerSubsystem, Constants.dosShooter).build();

    //one ball auto and taxi 
    /*
    Command autoShoot = new AutoShootBuilder(shooterSubsystem, driveSubsystem, navigationSubsystem, indexerSubsystem, Constants.dosShooter).build()
    return autoShoot.andThen(() -> {
      driveSubsystem.rawDrive(Constants.autoSpeed, Constants.autoSpeed);
    }).andThen(new WaitCommand(Constants.autoTime)).andThen(() -> {
      driveSubsystem.stop();
    });
    */
  }
}
