// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PhotonVisionNote;

public class AlignToNoteSensor extends Command {

  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private static PIDController vyNoteController = new PIDController(AlignConstants.kvyNoteP,AlignConstants.kvyNoteI,AlignConstants.kvyNoteD);
  private static IntakeSubsystem intake = IntakeSubsystem.getInstance();

  private final Timer timer = new Timer();
  private Command defaultCommand;
  private double _timeout;
  
  public AlignToNoteSensor(double timeout) {
    addRequirements(swerve, intake);
    _timeout = timeout;
  }
  public AlignToNoteSensor() {
    this(20.0);    
  }
  @Override
  public void initialize() {
    vyNoteController.reset();
    timer.reset();
    timer.start();
    defaultCommand = swerve.getDefaultCommand();
    swerve.removeDefaultCommand();
    vyNoteController.setSetpoint(AlignConstants.kMaxPitch);
    
  }

  @Override
  public void execute() {
     if(intake.getSensor()){
        intake.setMotorPower(0);
      }
      else{
        intake.setMotorPower(IntakeConstants.kPower);
      }
    PhotonPipelineResult p = PhotonVisionNote.getLatestPipeline();
    
    if (PhotonVisionNote.hasTarget(p)) {
      PhotonTrackedTarget t = PhotonVisionNote.getBestTarget(p);
      SmartDashboard.putData("PID NOTE",vyNoteController);
      double vo = -PhotonVisionNote.getYaw(t)/ 20;
      double vy = vyNoteController.calculate(PhotonVisionNote.getPitch(t));
      swerve.drive(ChassisSpeeds.fromRobotRelativeSpeeds(vy,0, vo, new Rotation2d()));
     
      

    }
  }

  @Override
  public boolean isFinished() {
    return (timer.get() >= _timeout || intake.getSensor());
  }

  @Override
  public void end(boolean interrupted) {
    intake.setMotorPower(0);
    swerve.drive(new ChassisSpeeds());
    swerve.setDefaultCommand(defaultCommand);
  }
}
