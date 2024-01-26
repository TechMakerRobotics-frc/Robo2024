
package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class MoveXYHeading extends CommandBase {
  
  double distanceX, distanceY, distanceH;
  SwerveSubsystem swerve;
  boolean finish = false;
  double lastTimestamp;
  double lastErrorX;
  double lastErrorY;
  double lastErrorH;
  public MoveXYHeading(double distanceX, double distanceY,double heading, SwerveSubsystem swerve) {
    
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    //this.distanceH = distanceH;
    
    distanceH = heading;
    SmartDashboard.putNumber("Distance Xi", distanceX);
    SmartDashboard.putNumber("Distance Yi", distanceY);
    SmartDashboard.putNumber("Distance Hi", distanceH);

    this.swerve = swerve;

  }
  @Override
  public void initialize() {

    swerve.resetOdometry();
    SmartDashboard.putString("O Johnny é calvo?", "NAO");
    lastTimestamp = Timer.getFPGATimestamp();
    lastErrorH = 0;
  }

  @Override
  public void execute() {

    SmartDashboard.putNumber("Distance X", distanceX);
    SmartDashboard.putNumber("Difference X", swerve.getPose().getX());
    SmartDashboard.putNumber("Distance Y", distanceY);
    SmartDashboard.putNumber("Difference Y", swerve.getPose().getY());
    SmartDashboard.putNumber("Distance H", distanceH);
    SmartDashboard.putNumber("Difference H", swerve.getYaw().getDegrees());

    double speedX = 0;
    double speedY = 0;
    double heading = 0;

    
    finish = true;
    if(swerve.getPose().getX()<distanceX)
    {finish = false;}
    if(swerve.getPose().getY()<distanceY)
    {finish = false;}
    if(swerve.getYaw().getDegrees()<distanceH)
    {finish = false;}

    double xVelocity   = Math.pow(speedX, 3);
    double yVelocity   = Math.pow(speedY, 3);
    double angVelocity = Math.pow(heading, 3);
  
    // Drive using raw values.
    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity ,
                 true ,false);


    // Cálculos -PID-
    double sensorX = swerve.getPose().getX();
    double errorX = distanceX - sensorX;
    speedX = Auton.kp*errorX;

    double sensorY = swerve.getPose().getY();
    double errorY = distanceY - sensorY;
    speedY = Auton.kp*errorY;

    double sensorH = swerve.getYaw().getDegrees();
    double errorH = distanceH - sensorH;
    heading = Auton.kpH*errorH;


    double errorSumX = 0;
    double errorSumY = 0;
    double errorSumH = 0;   

    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    double errorRateX = (errorX - lastErrorX) / dt;
    double errorRateY = (errorY - lastErrorY) / dt;
    double errorRateH = (errorH - lastErrorH) / dt;

    errorSumX += errorX * dt;
    errorSumY += errorY * dt;
    errorSumH += errorH * dt;

    speedX = Auton.kp * errorX + Auton.ki * errorSumX + Auton.kd * errorRateX;
    speedY = Auton.kp * errorY + Auton.ki * errorSumY + Auton.kd * errorRateY;
    heading = Auton.kpH * errorH + Auton.kiH * errorSumH + Auton.kdH * errorRateH;

    lastTimestamp = Timer.getFPGATimestamp();

    lastErrorX = errorX;
    lastErrorY = errorY;
    lastErrorH = errorH;
    
  }

  
  @Override
  public void end(boolean interrupted) {
    swerve.lock();
    SmartDashboard.putString("O Johnny é calvo?", "SIM");
  }

  // Retornar finish para terminar.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
