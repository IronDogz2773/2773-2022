// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

/** Add your docs here. */
public final class PathCommandBuilder {

        private final DriveSubsystem drive;
        private final NavigationSubsystem nav;
        private final Trajectory trajectory;

        // creates pathcommandbuilder with specified path to trajectory
        public PathCommandBuilder(DriveSubsystem drive, NavigationSubsystem nav, String path) {
                this.drive = drive;
                this.nav = nav;
                Trajectory trajectory = null;
                try {
                        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
                        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
                }
                this.trajectory = trajectory;
        }

        // creates pathcommandbuilder with specified trajectory
        public PathCommandBuilder(DriveSubsystem drive, NavigationSubsystem nav, Trajectory trajectory) {
                this.drive = drive;
                this.nav = nav;
                this.trajectory = trajectory;
        }

        // creates pathcommandbuilder with default trajectory
        public PathCommandBuilder(DriveSubsystem drive, NavigationSubsystem nav) {
                this.drive = drive;
                this.nav = nav;

                // Create a voltage constraint to ensure we don't accelerate too fast
                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(
                                                Constants.ksVolts,
                                                Constants.kvVoltSecondsPerMeter,
                                                Constants.kaVoltSecondsSquaredPerMeter),
                                Constants.kDriveKinematics,
                                10);

                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.kMaxSpeedMetersPerSecond,
                                Constants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(Constants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);

                // An example trajectory to follow. All units in meters.
                this.trajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(3, 0, new Rotation2d(0)),
                                // Pass config
                                config);
        }

        // builds command based on trajectory specified in constructor
        public Command build() {
                RamseteCommand ramseteCommand = new RamseteCommand(
                                trajectory,
                                nav::getPose,
                                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                new SimpleMotorFeedforward(
                                                Constants.ksVolts,
                                                Constants.kvVoltSecondsPerMeter,
                                                Constants.kaVoltSecondsSquaredPerMeter),
                                Constants.kDriveKinematics,
                                nav::getWheelSpeeds,
                                new PIDController(Constants.kPDriveVel, 0, 0),
                                new PIDController(Constants.kPDriveVel, 0, 0),
                                // RamseteCommand passes volts to the callback
                                drive::tankDriveVolts,
                                drive);

                // Reset odometry to the starting pose of the trajectory.
                nav.resetOdometry(trajectory.getInitialPose());

                // Run path following command, then stop at the end.
                return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));

        }
}
