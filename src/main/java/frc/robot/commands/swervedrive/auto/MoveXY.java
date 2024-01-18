// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

public class MoveXY extends Command {
  /** Creates a new MoveStraight. */
  double distanceX, distanceY;
  SwerveSubsystem swerve;
  boolean finish = false;
  private final SwerveController controller;
  public MoveXY(double distanceX, double distanceY, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    SmartDashboard.putNumber("Distance Xi", distanceX);
    SmartDashboard.putNumber("Distance Yi", distanceY);
    this.swerve = swerve;
    this.controller = swerve.getSwerveController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.resetOdometry(new Pose2d());
    SmartDashboard.putString("Ja acabou", "NAO");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Distance X", distanceX);
    SmartDashboard.putNumber("Difference X", swerve.getPose().getX());
    SmartDashboard.putNumber("Distance Y", distanceY);
    SmartDashboard.putNumber("Difference Y", swerve.getPose().getY());
    double speedX = 0;
    double speedY = 0;
    
    finish = true;
    if(Math.abs(swerve.getPose().getX())<Math.abs(distanceX))
    {
      speedX = 0.5;
      Math.copySign(speedX, distanceX);
      finish = false;

    }
    if(Math.abs(swerve.getPose().getY())<Math.abs(distanceY))
    {
      speedY = 0.5;
      Math.copySign(speedY, distanceY);
      finish = false;

    }
    double xVelocity   = Math.pow(speedX, 3);
    double yVelocity   = Math.pow(speedY, 3);
    double angVelocity = Math.pow(0, 3);
  
    // Drive using raw values.
    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity * controller.config.maxAngularVelocity,       true );
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.lock();
    SmartDashboard.putString("Ja acabou", "SIM");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
