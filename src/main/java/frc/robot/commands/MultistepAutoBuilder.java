// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerBaseSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterBaseSubsystem;

/** Add your docs here. */
public class MultistepAutoBuilder {
    private final DriveSubsystem drive;
    private final NavigationSubsystem nav;
    private final ShooterBaseSubsystem shooter;
    private final IndexerBaseSubsystem indexer;
    private final HopperSubsystem hopper;

    public MultistepAutoBuilder(DriveSubsystem drive, NavigationSubsystem nav, ShooterBaseSubsystem shooter,
            IndexerBaseSubsystem indexer, HopperSubsystem hopper) {
        this.drive = drive;
        this.nav = nav;
        this.shooter = shooter;
        this.indexer = indexer;
        this.hopper = hopper;
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

    private Command plan1() {
        Command plan1Command; // creates blank command
        Command autoShootCommand = new AutoShootBuilder(shooter, drive, nav, indexer, hopper, Constants.encoder, false)
                .build(); // creates autoshoot command
        Command pathCommand = new PathCommandBuilder(drive, nav, "paths/Circle.wpilib.json").build(); // creates path to
                                                                                                      // run
        Command turnCommand = new TurnDegreesCommand(nav, drive, 2); // creates command
        plan1Command = new SequentialCommandGroup(autoShootCommand, turnCommand, pathCommand); // creates entire
                                                                                               // command, in order of
                                                                                               // creation
        return null;
    }
}
