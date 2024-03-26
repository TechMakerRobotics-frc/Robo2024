// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PhotonVisionNote;

public class DriveStraigh extends Command {

  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  private final Timer timer = new Timer();
  private Command defaultCommand;
  private double _timeout;
  private boolean rear;
  
  public DriveStraigh(double timeout, Boolean rear) {
    addRequirements(swerve);
    _timeout = timeout;
    this.rear = rear;
  }
  public DriveStraigh() {
    this(20.0, false);    
  }
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    
  }

  @Override
  public void execute() {
     
    PhotonPipelineResult p = PhotonVisionNote.getLatestPipeline();
    
    if (PhotonVisionNote.hasTarget(p)) {
      double vy = (rear?-2:2);      
      swerve.drive(ChassisSpeeds.fromRobotRelativeSpeeds(vy,0, 0, new Rotation2d()));
    }
  }

  @Override
  public boolean isFinished() {
    return (timer.get() >= _timeout );
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds());
  }
}
