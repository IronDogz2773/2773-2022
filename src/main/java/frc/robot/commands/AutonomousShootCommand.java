// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerBaseSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterBaseSubsystem;

public class AutonomousShootCommand {
  private final ShooterBaseSubsystem shooter;
  private final DriveSubsystem drive;
  private final NavigationSubsystem nav;
  private final IndexerBaseSubsystem indexer;
  private final boolean oneWheelShoot;
  private boolean vision;

  /** Creates a new AutoShootBuilder. */
  public AutonomousShootCommand(ShooterBaseSubsystem shooter, DriveSubsystem drive, NavigationSubsystem nav,
      IndexerBaseSubsystem indexer, boolean oneWheelShoot) {
    this.shooter = shooter;
    this.drive = drive;
    this.nav = nav;
    this.indexer = indexer;
    this.oneWheelShoot = oneWheelShoot;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Command build() {
    // creates a shootcommand using speed if we don't have an encoder, or rpm if we
    // do
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("coPilot");
    NetworkTableEntry visionEntry = table.getEntry("turnVision");
    vision = visionEntry.getBoolean(false);
    
    Command shootCommand;
    if (oneWheelShoot) {
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

    // calls vision command, reverses ball briefly to prevent getting caught in flywheel,
    //shoot command, pulls indexer up to touch ball to
    // flywheel, wait for a second, then releases indexer
    Command autoShootCommand = visionCommand.andThen(() -> {
      indexer.reverseMotor();
    }).andThen(new WaitCommand(Constants.reverseIndexTime)).andThen(() -> {
      indexer.motorOff();
    }).andThen(shootCommand).andThen(() -> {
      indexer.motorOn();
    }).andThen(new WaitCommand(Constants.indexTime)).andThen(() -> {
      shooter.stop();
      indexer.motorOff();
    });

    return autoShootCommand;
  }
}
