// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.TunableConstant;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Limelight;
import frc.robot.util.PhotonVisionNote;

public class AlignToNote extends Command {

  private static SwerveSubsystem swerve = SwerveSubsystem.getInstance();

/* private TunableConstant targetVelocityCoefficient = new TunableConstant("kTargetCoefficient", VisionConstants.kTargetCoefficient);
  private TunableConstant maxAngularSpeed = new TunableConstant("Max Angular Speed", DriveConstants.kMaxAngularSpeedRadiansPerSecond);
  private TunableConstant maxAngularAcceleration = new TunableConstant("Max Angular Acceleration", DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
  private TunableConstant p = new TunableConstant("kP", SwerveModuleConstants.kTurningPID.p());
  private TunableConstant i = new TunableConstant("kI", SwerveModuleConstants.kTurningPID.i());
  private TunableConstant d = new TunableConstant("kD", SwerveModuleConstants.kTurningPID.d());*/

  private final Timer timer = new Timer();
  private Command defaultCommand;
  private double _timeout;
  public AlignToNote() {
    addRequirements(swerve);
    _timeout = 20;
  }
  public AlignToNote(double timeout) {
    addRequirements(swerve);
    _timeout = timeout;
  }
  @Override
  public void initialize() {

    timer.reset();
    timer.start();
    defaultCommand = swerve.getDefaultCommand();
    swerve.removeDefaultCommand();
  }

  @Override
  public void execute() {
    PhotonPipelineResult p = PhotonVisionNote.getLatestPipeline();
    if (PhotonVisionNote.hasTarget(p)) {
      PhotonTrackedTarget t = PhotonVisionNote.getBestTarget(p);

      double vo = -PhotonVisionNote.getYaw(t)/ 20;
      double vy = -(PhotonVisionNote.getPitch(t)+VisionConstants.MAX_PITCH )/ 20;
      PhotonVisionNote.printToDashboard();
      //swerve.drive(ChassisSpeeds.fromRobotRelativeSpeeds(vy, 0, vo, swerve.getHeading()));
      swerve.drive(new Translation2d( vy,0),vo,false);

    }
  }
 /*private void updateConstants() {
    if (maxAngularSpeed.hasChanged() || maxAngularAcceleration.hasChanged()) {
      thetaController.setConstraints(new TrapezoidProfile.Constraints(maxAngularSpeed.get(), maxAngularAcceleration.get()));
    }

    if (p.hasChanged() || i.hasChanged() || d.hasChanged()) {
      thetaController.setPID(p.get(), i.get(), d.get());
    }
  }*/
  @Override
  public boolean isFinished() {
    return timer.get() >= _timeout;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.resetOdometry(Limelight.getBotPose2d());
    swerve.zeroGyro();
    swerve.drive(new ChassisSpeeds());
    swerve.setDefaultCommand(defaultCommand);
  }
}
