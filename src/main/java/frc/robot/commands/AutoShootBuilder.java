// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterBaseSubsystem;

public class AutoShootBuilder {
  private final ShooterBaseSubsystem shooter;
  private final DriveSubsystem drive;
  private final NavigationSubsystem nav;
  private final boolean vision;
  private final boolean manual;
  
  /** Creates a new AutoShootBuilder. */
  public AutoShootBuilder(ShooterBaseSubsystem shooter, DriveSubsystem drive, NavigationSubsystem nav, boolean manual, boolean vision) {
    this.shooter = shooter;
    this.drive = drive;
    this.nav = nav;
    this.manual = manual;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Command build(){
    //creates a shootcommand using speed if we don't have an encoder, or rpm if we do
    Command shootCommand;
    if(manual){
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
    }
    else{
      shootCommand = new ShotRpmCommand(shooter, Constants.maxShooterSpeed, 0);
    }

    //sets up shot using vision if vision if presence, creates a command that will instantly end if not
    Command visionCommand;
    if(vision){
      visionCommand = new TurnDegreesCommand(nav, drive, Constants.turnCmdTimeOut);
    }
    else {
      visionCommand = new CommandBase() {
        @Override
        public boolean isFinished(){
          return true;
        }
      };
    }

    
    //calls vision command, shoot command, pulls indexer up to touch ball to flywheel, wait for a second, then releases indexer
    Command autoShootCommand = visionCommand.andThen(shootCommand).andThen(() -> {
      //shooter.extendIndex();
    }).andThen(new WaitCommand(1)).andThen(() -> {
      shooter.stop();
      //shooter.retractIndex();
    });

    return autoShootCommand;
  }
}
