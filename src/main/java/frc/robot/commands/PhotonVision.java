// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.auto.MoveXYHeading;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PhotonVision extends Command {
  /** Creates a new PhotonVision. */


  //private final PhotonVisionSubsystem camera = PhotonVisionSubsystem.getYaw();

  @Override
  public void initialize() {
  //addRequirements(camera);

  }

  @Override
  public void execute() {

    //if(getYaw()>0){
    //new MoveXYHeading(0, 0, 20, drivebase);

      
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
