// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.RamseteCommandPlus;

public class AutonomousBuilder {
    String trajectoryJSON;
    Trajectory trajectory;
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    DriveSubsystem driveSubsystem;
    NavigationSubsystem navigationSubsystem;
    String filePath;

    public AutonomousBuilder(DriveSubsystem driveSubsystem, NavigationSubsystem navigationSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
    }

    public AutonomousBuilder(DriveSubsystem driveSubsystem, NavigationSubsystem navigationSubsystem, String filePath){
        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        this.filePath = filePath;
    }

    private void SetVoltageConstraint(){
        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter), Constants.kDriveKinematics, 10);
    }

    private void setTestTrajectory() {
        TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint);
        trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);
    }

    private void setTrajectory(String filepath){
        trajectoryJSON = filePath;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }

    public Command build(){
        var ramseteCommand = new RameseteCommandPlus(trajectory, driveSubsystem, navigationSubsystem);
        return ramseteCommand;
    }
}
