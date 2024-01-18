// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MoveXYHeading extends Command {
  /** Creates a new MoveStraight. */
  double distanceX, distanceY, direction;
  SwerveSubsystem swerve;
  boolean finish = false;
  public MoveXYHeading(double distanceX, double distanceY,double heading, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    direction = heading;
    SmartDashboard.putNumber("Distance Xi", distanceX);
    SmartDashboard.putNumber("Distance Yi", distanceY);
    this.swerve = swerve;

    //pidControllerX = new PIDController(0.088, 0.04, 0);
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
    SmartDashboard.putNumber("Direction", direction);
    SmartDashboard.putNumber("Difference Direction", swerve.getHeading().getDegrees());
    double speedX = 0;
    double speedY = 0;
    double heading = 0;
    
    finish = true;
    if(swerve.getPose().getX()<distanceX)
    {
      speedX = 0.5;
      finish = false;

    }
    if(swerve.getPose().getY()<distanceY){
      speedY = 0.5;
      finish = false;
    }
    if(swerve.getHeading().getDegrees()<direction){
      heading = 1;
      finish = false;
    }
    double xVelocity   = Math.pow(speedX, 3);
    double yVelocity   = Math.pow(speedY, 3);
    double angVelocity = Math.pow(heading, 3);
  
    // Drive using raw values.
    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity , true);
    
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
