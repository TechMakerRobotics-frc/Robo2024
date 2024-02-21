// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PhotonVisionNote;

public class AlignToNote extends Command {

  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private static PIDController vyNoteController = new PIDController(AlignConstants.kvyNoteP,AlignConstants.kvyNoteI,AlignConstants.kvyNoteD);


  private final Timer timer = new Timer();
  private Command defaultCommand;
  private double _timeout;
  
  public AlignToNote(double timeout) {
    addRequirements(swerve);
    _timeout = timeout;
  }
  public AlignToNote() {
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
    PhotonPipelineResult p = PhotonVisionNote.getLatestPipeline();
    if (PhotonVisionNote.hasTarget(p)) {
      PhotonTrackedTarget t = PhotonVisionNote.getBestTarget(p);
      SmartDashboard.putData("PID NOTE",vyNoteController);
      double vo = -PhotonVisionNote.getYaw(t)/ 20;
      double vy = vyNoteController.calculate(PhotonVisionNote.getPitch(t));
      PhotonVisionNote.printToDashboard();
      swerve.drive(new Translation2d( vy,0),vo,false);

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
