// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerBaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterBaseSubsystem;

/** Add your docs here. */
public class MultistepAutoBuilder {
    private final DriveSubsystem drive;
    private final NavigationSubsystem nav;
    private final ShooterBaseSubsystem shooter;
    private final IndexerBaseSubsystem indexer;
    private final HopperSubsystem hopper;
    private final IntakeSubsystem intake;

    public MultistepAutoBuilder(DriveSubsystem drive, NavigationSubsystem nav, ShooterBaseSubsystem shooter,
            IndexerBaseSubsystem indexer, HopperSubsystem hopper, IntakeSubsystem intake) {
        this.drive = drive;
        this.nav = nav;
        this.shooter = shooter;
        this.indexer = indexer;
        this.hopper = hopper;
        this.intake = intake;
    }

    public Command build() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("auto");
        NetworkTableEntry gameplanEntry = table.getEntry("gameplan");

        int gameplan = gameplanEntry.getNumber(0).intValue();

        // changes autonomous path based on gameplan
        switch (gameplan) {
            case 1:
                return plan1();
            default:
                return null;
        }
    }

    //one ball auto and taxi
    private Command plan1() {
        Command autoIntakeCommand = new CommandBase() {
            @Override
            public void initialize() {
              intake.retractIntake();
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
              shooter.setRpm(2150, 2150);
            }
      
            @Override
            public boolean isFinished() {
              return shooter.atSetpoint();
            }
          };
      
          // runs the indexer and then stop indexer and shooter
          Command autoShootCommand = autoShootCommand0.andThen(() -> {
            indexer.motorOn();
          }).andThen(new WaitCommand(Constants.indexTime)).andThen(() -> {
            shooter.stop();
            indexer.motorOff();
          });
      
          // drives for an amount of time
          Command driveCommand = new DriveStraightForTimeCommand(drive, .3, 1.5);
      
          // retracts intake, sets RPM, runs indexer, turns off shooter and index, drives
          // an amount of time
          Command autoCommand = autoIntakeCommand.andThen(autoShootCommand).andThen(driveCommand);
      
          return autoCommand;
    }

    //two ball auto
    private Command plan2(){
        Command autoShoot1 = new AutoShootBuilder(shooter, drive, nav, indexer, hopper, Constants.encoder, false).build();
        Command deployIntake = new DeployIntakeCommand(intake);
        Command autoPath = new PathCommandBuilder(drive, nav, "paths/auto.wpilib.json").build(); //TODO path is a placeholder
        Command intakeAndDrive = deployIntake.andThen(() -> {
            intake.motorOn();
        }).andThen(autoPath).andThen(() -> {
            intake.motorOff();
        });
        Command autoShoot2 = new AutoShootBuilder(shooter, drive, nav, indexer, hopper, Constants.encoder, false).build();
        Command fullAuto = autoShoot1.andThen(deployIntake).andThen(intakeAndDrive).andThen(autoShoot2);
        return fullAuto;
    }
}
