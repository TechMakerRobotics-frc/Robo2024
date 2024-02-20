
package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutonomousConstants;

public class MoveXYHeading extends Command {
  
  double distanceX, distanceY, heading;
  
  SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  boolean finish = false;
  
  double lastTimestamp;
  
  double lastErrorX = 0;
  double lastErrorY = 0;
  double lastErrorH = 0;
  
  double errorSumX = 0;
  double errorSumY = 0;
  double errorSumH = 0;
  
  public MoveXYHeading(double distanceX, double distanceY, double heading) {
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    this.heading = heading;
    SmartDashboard.putNumber("Distance Xi", distanceX);
    SmartDashboard.putNumber("Distance Yi", distanceY);
    SmartDashboard.putNumber("Giro", heading);
  }

  @Override
  public void initialize() {
    swerve.resetOdometry(new Pose2d());
    swerve.zeroGyro();
    lastTimestamp = Timer.getFPGATimestamp();
    lastErrorX = 0;
    lastErrorY = 0;
    lastErrorH = 0;

  }

  @Override
  public void execute() {
    
    double speedX = 0;
    double speedY = 0;
    double speedH = 0;
    
    
    finish = true;
    if(Math.abs(swerve.getPose().getX())<Math.abs(distanceX))
    {
      finish = false;
    }
    if(Math.abs(swerve.getPose().getY())<Math.abs(distanceY))
    {
      finish = false;
    }
    if(Math.abs(swerve.getPose().getRotation().getDegrees())<Math.abs(heading))
    {
      finish = false;
    }

    // Cálculos -PID-
    double sensorX = swerve.getPose().getX();
    double errorX = distanceX - sensorX;
    speedX = AutonomousConstants.kp*errorX;

    double sensorY = swerve.getPose().getY();
    double errorY = distanceY - sensorY;
    speedY = AutonomousConstants.kp*errorY;

    double sensorH = swerve.getPose().getRotation().getDegrees();
    double errorH = heading - sensorH;
    speedH = AutonomousConstants.kpH*errorH;

  
    
    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorRateX = (errorX - lastErrorX) / dt;
    double errorRateY = (errorY - lastErrorY) / dt;
    double errorRateH = (errorH - lastErrorH) / dt;

    errorSumX += errorX * dt;
    errorSumY += errorY * dt;
    errorSumH += errorH * dt;

    speedX = AutonomousConstants.kp * errorX + AutonomousConstants.ki * errorSumX + AutonomousConstants.kd * errorRateX;
    speedY = AutonomousConstants.kp * errorY + AutonomousConstants.ki * errorSumY + AutonomousConstants.kd * errorRateY;
    speedH = AutonomousConstants.kpH * errorH + AutonomousConstants.kiH * errorSumH + AutonomousConstants.kdH * errorRateH;
    lastTimestamp = Timer.getFPGATimestamp();
    lastErrorX = errorX;
    lastErrorY = errorY;
    lastErrorH = errorH;

    double xVelocity   = Math.pow(speedX, 3);
    double yVelocity   = Math.pow(speedY, 3);
    double angVelocity = Math.pow(speedH, 3);
    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity,yVelocity, angVelocity, swerve.getHeading()));

  }

  @Override
  public void end(boolean interrupted) {
   // swerve.lock();
  }

  @Override
  public boolean isFinished() {
    return finish;
  }
}