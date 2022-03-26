// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ActivateIntakeCommand;
import frc.robot.commands.AutoShootBuilder;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveStraightForTimeCommand;
import frc.robot.commands.ShotCommand;
import frc.robot.commands.TelescopingCommand;
import frc.robot.commands.TurnDegreesCommand;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.IndexBackwardsCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerBaseSubsystem;
import frc.robot.subsystems.IndexerMainSubsystem;
import frc.robot.subsystems.IndexerTestSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterBaseSubsystem;
import frc.robot.subsystems.ShooterMainSubsystem;
import frc.robot.subsystems.ShooterTestSubsystem;
import frc.robot.subsystems.TelescopingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  // some subsystems will be null if on the test or main robot
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterBaseSubsystem shooterSubsystem = Constants.shooterPresent ? (Constants.dosShooter ? new ShooterMainSubsystem()
      : new ShooterTestSubsystem()) : new ShooterBaseSubsystem() {

        @Override
        public void setRpm(double rpmFront, double rpmBack){}

        @Override
        public void setSpeed(double speedFront, double speedBack){}

        @Override
        public boolean atSetpoint(){return true;}

        @Override
        public void stop(){}
      };
  private final IntakeSubsystem intakeSubsystem = Constants.intakePresent ? new IntakeSubsystem() : null;
  private final NavigationSubsystem navigationSubsystem = new NavigationSubsystem();
  private final HopperSubsystem hopperSubsystem = Constants.hopperPresent ? new HopperSubsystem() : null;
  private final IndexerBaseSubsystem indexerSubsystem = Constants.indexerPresent ? new IndexerMainSubsystem()
      : new IndexerTestSubsystem();
  private final TelescopingSubsystem telescopingSubsystem = Constants.climberPresent ? new TelescopingSubsystem()
      : null;
  private final LEDSubsytem ledSubsytem = new LEDSubsytem();

  // Commands
  // some commands will do nothing if on test or main robot
  private final Command activateIntakeCommand = Constants.intakePresent
      ? new ActivateIntakeCommand(intakeSubsystem, gamepadPilot)
      : doNothing();
  private final Command deployIntakeCommand = Constants.intakePresent
      ? new DeployIntakeCommand(intakeSubsystem)
      : doNothing();
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem, navigationSubsystem, gamepadPilot);
  private final TurnDegreesCommand turnDegreesCommand = new TurnDegreesCommand(navigationSubsystem, driveSubsystem,
      Constants.turnCmdTimeOut);
  private final Command hopperCommand = Constants.hopperPresent
      ? new HopperCommand(hopperSubsystem, gamepadPilot)
      : doNothing();
  private final Command telescopingCommand = Constants.climberPresent
      ? new TelescopingCommand(telescopingSubsystem, gamepadCopilot)
      : doNothing();
  private final ShotCommand shotCommand = new ShotCommand(shooterSubsystem);
  private final IndexCommand indexCommand = new IndexCommand(indexerSubsystem, gamepadPilot);

  // Joysticks
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
    if(Constants.hopperPresent){
      hopperSubsystem.setDefaultCommand(hopperCommand);
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
    // Pilot buttons

    if (Constants.hopperPresent) {
      // RB held, run index and hopper
      final JoystickButton idexHopperButton = new JoystickButton(gamepadPilot, Constants.RB);
      Command hopperCommand = new CommandBase() {
        {
          addRequirements(hopperSubsystem);
        }

        @Override
        public void initialize() {
          hopperSubsystem.motorOn();
        }

        @Override
        public void end(boolean interrupted) {
          hopperSubsystem.motorOff();
        }

        @Override
        public boolean isFinished() {
          return false;
        }
      };
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

    // B held, turn
    final JoystickButton turnButton = new JoystickButton(gamepadPilot, Constants.B);
    final TurnDegreesCommand turnCommand = new TurnDegreesCommand(navigationSubsystem, driveSubsystem, 5);
    turnButton.whenHeld(turnCommand);

    // LB pressed, firing sequence
    if(Constants.shooterPresent){
      final JoystickButton firingTrigger = new JoystickButton(gamepadPilot, Constants.LB);
      final Command fireCommand = new AutoShootBuilder(shooterSubsystem, driveSubsystem, navigationSubsystem,
          indexerSubsystem, hopperSubsystem, Constants.encoder).build();
      firingTrigger.whenPressed(fireCommand, true); // TODO change to held
    }

    // Select pressed, toggle intake pneumatic
    if (Constants.intakePresent) {
      final JoystickButton deployIntakeButton = new JoystickButton(gamepadPilot, Constants.Select);
      deployIntakeButton.whenPressed(deployIntakeCommand);
    }

    // Y held, run index backwards
    final JoystickButton backwardsIndexButton = new JoystickButton(gamepadPilot, Constants.Y);
    Command indexBackwardsCommand = new CommandBase() {
      {
        addRequirements(indexerSubsystem);
      }

      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        indexerSubsystem.reverseMotor();
      }

      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        indexerSubsystem.motorOff();
      }

      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
    };
    backwardsIndexButton.whenHeld(indexBackwardsCommand);

    // CoPilot buttons

    // network table for copilot values
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable copilotTable = inst.getTable("coPilot");
    NetworkTableEntry turnVisionEntry = copilotTable.getEntry("turnVision");
    NetworkTableEntry distanceVisionEntry = copilotTable.getEntry("distanceVision");
    NetworkTableEntry proximityEntry = copilotTable.getEntry("proximity");



    // if bot using turnvision, and A pressed, toggle turnvision
    if (Constants.turnVision) {
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
    }



    // if B pressed, toggle distance vision
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



    // if Y pressed, toggle using proximity sensor
    if (Constants.proximity) {
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
    }
      


    // LB held, run all systems backwards, ejects ball out of intake
    // TODO TEST THIS PLEASE IDK IF IT WORKS
    final JoystickButton frontEjectButton = new JoystickButton(gamepadCopilot, Constants.LB);
    Command frontEjectCommand = new CommandBase() {
      {
        addRequirements(intakeSubsystem, indexerSubsystem, hopperSubsystem);
      }

      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        intakeSubsystem.reverseMotor();
        indexerSubsystem.reverseMotor();
        hopperSubsystem.reverseMotor();
      }

      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        intakeSubsystem.motorOff();
        indexerSubsystem.motorOff();
        hopperSubsystem.motorOff();
      }

      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
    };
    frontEjectButton.whenHeld(frontEjectCommand);



    // RB held, run all systems forwards, ejects out of intake at slow RPM
    // TODO TEST THIS PLEASE IDK IF IT WORKS
    final JoystickButton shootEjectButton = new JoystickButton(gamepadCopilot, Constants.RB);
    Command shootEjectCommand = new CommandBase() {
      {
        addRequirements(intakeSubsystem, indexerSubsystem, hopperSubsystem, shooterSubsystem);
      }

      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        intakeSubsystem.motorOn();
        indexerSubsystem.motorOn();
        hopperSubsystem.motorOn();
        shooterSubsystem.setRpm(100, 100); // PLACEHOLDER
        ledSubsytem.runTheRainbow(true);
      }

      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        intakeSubsystem.motorOff();
        indexerSubsystem.motorOff();
        hopperSubsystem.motorOff();
        shooterSubsystem.stop();
        ledSubsytem.runTheRainbow(false);
      }

      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
    };
    shootEjectButton.whenHeld(shootEjectCommand);

  }

  // do nothing command for null commands on test and main bot
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
    // one ball auto and taxi no pathweaver

    // retracts the intake and returns immediatly
    Command autoIntakeCommand = new CommandBase() {
      @Override
      public void initialize() {
        intakeSubsystem.retractIntake();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };

    // sets rpm to 2150 and returns when it reaches this value
    Command autoShootCommand0 = new CommandBase() {
      @Override
      public void initialize() {
        shooterSubsystem.setRpm(2150, 2150);
      }

      @Override
      public boolean isFinished() {
        return shooterSubsystem.atSetpoint();
      }
    };

    // runs the indexer and then stop indexer and shooter
    Command autoShootCommand = autoShootCommand0.andThen(() -> {
      indexerSubsystem.motorOn();
    }).andThen(new WaitCommand(Constants.indexTime)).andThen(() -> {
      shooterSubsystem.stop();
      indexerSubsystem.motorOff();
    });

    // drives for an amount of time
    Command driveCommand = new DriveStraightForTimeCommand(driveSubsystem, .3, 1.5);

    // retracts intake, sets RPM, runs indexer, turns off shooter and index, drives
    // an amount of time
    //Command autoCommand = autoIntakeCommand.andThen(autoShootCommand).andThen(driveCommand);

    return driveCommand;
  }
}
