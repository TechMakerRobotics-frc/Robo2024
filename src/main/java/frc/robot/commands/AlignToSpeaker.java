// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.LimelightConstants;

public class AlignToSpeaker extends Command {
  private static PIDController vyStageController = new PIDController(AlignConstants.kvyStageP,AlignConstants.kvyStageI,AlignConstants.kvyStageD);
  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private final Timer timer = new Timer();
  private double _timeout;
  private Command defaultCommand;
  private LimelightSubsystem limelight = LimelightSubsystem.getInstance();

  public AlignToSpeaker(double timeout) {
    addRequirements(swerve);
    vyStageController.setSetpoint(AlignConstants.kDistanceFromSpeakerToShoot);
    _timeout = timeout;
  }
  public AlignToSpeaker() {
    this(20);
  }
  @Override
  public void initialize() {
    limelight.startLimelight() ;
    vyStageController.reset();
    timer.reset();
    timer.start();
    defaultCommand = swerve.getDefaultCommand();
    swerve.removeDefaultCommand();
  }

  @Override
  public void execute() {
    if (limelight.atSpeaker()) {
      SmartDashboard.putData("PID SPEAKER",vyStageController);
      double vo = -limelight.getTx()/50;
      double vy = vyStageController.calculate(limelight.getDistance()); 
      SmartDashboard.putNumber("Angular", vo);
      SmartDashboard.putNumber("X", vy);
      SmartDashboard.putNumber("Distance", limelight.getDistance());
      swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-vy,0, vo, swerve.getHeading()));

    }
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= _timeout;
  }

  @Override
  public void end(boolean interrupted) {

    limelight.setPipeline(LimelightConstants.kPosePipeline);
        swerve.resetOdometry(limelight.getBotpose());
    swerve.zeroGyro();
    swerve.drive(new ChassisSpeeds());
    swerve.setDefaultCommand(defaultCommand);
  }
}
