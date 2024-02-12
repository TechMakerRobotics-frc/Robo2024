package frc.robot.subsystems;

import java.io.File;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class SwerveSubsystem extends SubsystemBase {

//swerve drive object
    private final SwerveDrive swerveDrive;
    private static SwerveSubsystem instance;
//Velocidade maxida do robô em metros por segundo, usado para limitar a aceleração.
   
  public double maximumSpeed = 3.0;

  /* The auto builder for PathPlanner, there can only ever be one created so we save it just incase we generate multiple
paths with events.
   */

    public SwerveSubsystem(File directory){
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.MACHINE;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        SmartDashboard.putString("Swerve", "Iniciado");
        
        instance = this;
        
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }
    public static SwerveSubsystem getInstance() {
        return instance;
    }
    @Override
    public void periodic(){}

    @Override
    public void simulationPeriodic(){}

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }
    public void resetOdometry() {
        swerveDrive.resetOdometry(new Pose2d());
    }
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory){
        swerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro(){
        swerveDrive.zeroGyro();
    }

    public void setMotorBrake (boolean brake){
        swerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading(){
        return swerveDrive.getYaw();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians(), maximumSpeed);
    }

    public ChassisSpeeds getFieldVelocity(){
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity(){
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController(){
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration(){
        return swerveDrive.swerveDriveConfiguration;
    }

    public void lock(){
        swerveDrive.lockPose();
    }

    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }
    public Rotation2d getYaw(){
        return swerveDrive.getYaw();
    }
}
