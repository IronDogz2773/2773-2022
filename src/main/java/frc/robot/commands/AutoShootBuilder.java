// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerBaseSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterBaseSubsystem;

public class AutoShootBuilder {
  private final ShooterBaseSubsystem shooter;
  private final DriveSubsystem drive;
  private final NavigationSubsystem nav;
  private final IndexerBaseSubsystem indexer;
  private final HopperSubsystem hopper;
  private final boolean encoder;
  private final boolean twoBalls;
  private boolean vision;

  /** Creates a new AutoShootBuilder. */
  public AutoShootBuilder(ShooterBaseSubsystem shooter, DriveSubsystem drive, NavigationSubsystem nav,
      IndexerBaseSubsystem indexer, HopperSubsystem hopper, boolean encoder, boolean twoBalls) {
    this.shooter = shooter;
    this.drive = drive;
    this.nav = nav;
    this.indexer = indexer;
    this.hopper = hopper;
    this.encoder = encoder;
    this.twoBalls = twoBalls;
  }

  public Command build() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // get copilot table
    NetworkTable table = inst.getTable("coPilot");
    NetworkTableEntry visionEntry = table.getEntry("turnVision");
    vision = false;

    // creates shotcommand using speed if encoder is not present, using shoot
    // command (with rpm) if it is
    Command shootCommand;
    if (!encoder) {
      shootCommand = new CommandBase() {
        @Override
        public void initialize() {
          shooter.setSpeed(.5);
        }

        @Override
        public boolean isFinished() {
          return true;
        }
      }.andThen(new WaitCommand(1));
    } else {
      shootCommand = new ShotCommand(shooter);
    }

    // sets up shot using vision if vision if presence, creates a command that will
    // instantly end if not
    Command visionCommand;
    if (vision) {
      visionCommand = new TurnDegreesCommand(nav, drive, Constants.turnCmdTimeOut);
    } else {
      visionCommand = new CommandBase() {
        @Override
        public boolean isFinished() {
          return true;
        }
      };
    }

    Command autoShootCommand;
    if (twoBalls) {
      Command indexForTime = new IndexForTimeCommand(indexer, .2);
      Command indexForTime2 = new IndexForTimeCommand(indexer, .2);
      Command hopperOnCommand = new CommandBase() {
        {
          addRequirements(hopper);
        }

        @Override
        public void initialize() {
          hopper.motorOn();
        }

        @Override
        public boolean isFinished() {
          return true;
        }
      };
      Command hopperOffCommand = new CommandBase() {
        {
          addRequirements(hopper);
        }

        @Override
        public void initialize() {
          hopper.motorOff();
        }

        @Override
        public boolean isFinished() {
          return true;
        }
      };
      IndexCommand indexCommand = new IndexCommand(indexer);
      IndexCommand indexCommand2 = new IndexCommand(indexer);
      ShotCommand shootCommand2 = new ShotCommand(shooter);
      Command indexAndShooter = new ParallelRaceGroup(indexCommand, shootCommand2);
      Command indexAndShooter2 = new ParallelRaceGroup(new WaitUntilCommand(shooter::atSetpoint), indexCommand2);
      autoShootCommand = visionCommand.andThen(hopperOnCommand).andThen(indexAndShooter).andThen(indexForTime)
          .andThen(new WaitCommand(.2)).andThen(indexAndShooter2).andThen(indexForTime2).andThen(hopperOffCommand)
          .andThen(() -> {
            shooter.stop();
          });
    } else {
      // calls vision command, reverses ball briefly to prevent getting caught in
      // flywheel,
      // shoot command, pulls indexer up to touch ball to
      // flywheel, wait for a second, then turn off indexer and shooter
      autoShootCommand = visionCommand.andThen(shootCommand).andThen(() -> {
        indexer.motorOn();
      }).andThen(new WaitCommand(Constants.indexTime)).andThen(() -> {
        shooter.stop();
        hopper.motorOff();
        indexer.motorOff();
      });
    }

    return autoShootCommand;
  }
}
