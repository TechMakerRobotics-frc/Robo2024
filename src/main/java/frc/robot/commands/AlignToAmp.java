// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PhotonVisionNote;
import frc.robot.util.PhotonVisionTags;
import frc.robot.Constants.AlignConstants;


public class AlignToAmp extends Command {
  private static PIDController vyAmpController = new PIDController(AlignConstants.kvyAmpP,AlignConstants.kvyAmpI,AlignConstants.kvyAmpD);
  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private final Timer timer = new Timer();
  private double _timeout;
  private Command defaultCommand;

  public AlignToAmp(double timeout) {
    addRequirements(swerve);
    vyAmpController.setSetpoint(AlignConstants.kTargetArea);
    _timeout = timeout;
  }
  public AlignToAmp() {
    this(20);
  }
  
  @Override
  public void initialize() {
    vyAmpController.reset();
    timer.reset();
    timer.start();
    defaultCommand = swerve.getDefaultCommand();
    swerve.removeDefaultCommand();
  }

  @Override
  public void execute() {
    PhotonPipelineResult p = PhotonVisionNote.getLatestPipeline();
    
    if (PhotonVisionTags.hasTarget(p)) {
      PhotonTrackedTarget t = PhotonVisionTags.getBestTarget(p);
      SmartDashboard.putData("PID AMP",vyAmpController);
      double vo = PhotonVisionTags.getPitch(t)/50;
      double vy = vyAmpController.calculate(PhotonVisionTags.getArea(t)); 
      SmartDashboard.putNumber("Angular", vo);
      SmartDashboard.putNumber("X", vy);
      SmartDashboard.putNumber("Distance", PhotonVisionTags.getArea(t));
      swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-vy,0, vo, swerve.getHeading()));
      if(vyAmpController.atSetpoint()){
      }
    }
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= _timeout;
  }

  @Override
  public void end(boolean interrupted) {

    swerve.drive(new ChassisSpeeds());
    swerve.setDefaultCommand(defaultCommand);
  }
}
