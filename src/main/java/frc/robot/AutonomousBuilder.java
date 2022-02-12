// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

public class AutonomousBuilder {
    String trajectoryJSON;
    Trajectory trajectory;
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    DriveSubsystem driveSubsystem;
    NavigationSubsystem navigationSubsystem;
    String trajectoryFilePath;

    public AutonomousBuilder(DriveSubsystem driveSubsystem, NavigationSubsystem navigationSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
    }

    public AutonomousBuilder(DriveSubsystem driveSubsystem, NavigationSubsystem navigationSubsystem, String filePath){
        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        
    }

    public SetVoltageConstraint(){
        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(ks, kv), kinematics, maxVoltage)
    }
}
