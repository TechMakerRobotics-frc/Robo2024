// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Limelight;
import frc.robot.Constants.Auto;
import frc.robot.Constants.LimelightConstants;

public class AlignToSpeaker extends Command {
  private static PIDController vxStageController = new PIDController(Auto.VX_STAGE_kP, 0, 0);
  private static PIDController vyStageController = new PIDController(Auto.VY_STAGE_kP, 0, 0);
  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private final Timer timer = new Timer();
  private double _timeout;
  private Command defaultCommand;

  public AlignToSpeaker(double timeout) {
    addRequirements(swerve);
    vxStageController.setTolerance(Auto.MAX_ERROR_DEG_TX_STAGE);
    vyStageController.setTolerance(Auto.MAX_ERROR_DEG_TY_STAGE);
    vxStageController.setSetpoint(0);
    vyStageController.setSetpoint(Auto.VERTICAL_DEG_STAGE);
    _timeout = timeout;
  }
  public AlignToSpeaker() {
    addRequirements(swerve);
    vxStageController.setTolerance(Auto.MAX_ERROR_DEG_TX_STAGE);
    vyStageController.setTolerance(Auto.MAX_ERROR_DEG_TY_STAGE);
    vxStageController.setSetpoint(0);
    vyStageController.setSetpoint(Auto.VERTICAL_DEG_STAGE);
    _timeout = 20;
  }
  @Override
  public void initialize() {
    Limelight.startLimelight() ;
    vxStageController.reset();
    vyStageController.reset();
    timer.reset();
    timer.start();
    defaultCommand = swerve.getDefaultCommand();
    swerve.removeDefaultCommand();
  }

  @Override
  public void execute() {
    if (Limelight.atSpeaker()) {
      double vo = -Limelight.getTx()/50;
      double vy = -(LimelightConstants.SPEAKER_DISTANCE_TO_SHOOT-Limelight.getCentimetersFromTarget())/50;
      SmartDashboard.putNumber("Tx", vo);
      SmartDashboard.putNumber("Distance", Limelight.getCentimetersFromTarget());
      swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vy,0, vo, swerve.getHeading()));

    }
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= _timeout;
  }

  @Override
  public void end(boolean interrupted) {

    Limelight.setPipeline(LimelightConstants.POSE_PIPELINE);
        swerve.resetOdometry(Limelight.getBotPose2d());
    swerve.zeroGyro();
    swerve.drive(new ChassisSpeeds());
    swerve.setDefaultCommand(defaultCommand);
  }
}
