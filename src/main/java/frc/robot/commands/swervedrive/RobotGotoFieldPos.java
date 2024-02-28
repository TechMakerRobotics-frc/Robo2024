package frc.robot.commands.swervedrive;
/*
 * The template code does not have a function for setting the robot to face a certain way, so we're just gonna have to implement that
 * ourselves using the drive function, the getHeading function, and a pid controller that we'll have to tune.
 */

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HeadingConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.MathUtils;

public class RobotGotoFieldPos extends Command {

    private final SwerveSubsystem drive = SwerveSubsystem.getInstance();

    private final PIDController pidControllerX = new PIDController(HeadingConstants.kTranslationP, 
                                                                  HeadingConstants.kTranslationI, 
                                                                  HeadingConstants.kTranslationD);
    private final PIDController pidControllerY = new PIDController(HeadingConstants.kTranslationP, 
                                                                  HeadingConstants.kTranslationI, 
                                                                  HeadingConstants.kTranslationD);
    private final PIDController pidControllerAngle = new PIDController(HeadingConstants.kHeadingP, 
                                                                  HeadingConstants.kHeadingI, 
                                                                  HeadingConstants.kHeadingD);
    private boolean m_complete = false;

    private Pose2d m_desiredRobotoPos = null;

    /** 
     * Uses PID to make the robot go to a certain postion relative to the field.  
     */
    public RobotGotoFieldPos(Pose2d desiredRobotoPos) {


        m_desiredRobotoPos = desiredRobotoPos;

        pidControllerX.setTolerance(HeadingConstants.kTranslationTolerance);
        pidControllerY.setTolerance(HeadingConstants.kTranslationTolerance);
        pidControllerAngle.setTolerance(HeadingConstants.kHeadingTolerance);

        pidControllerAngle.enableContinuousInput(-180, 180);

        addRequirements(drive);
        

    }
  
    public RobotGotoFieldPos(Pose2d desiredRobotoPos, double timeout) {

        this(desiredRobotoPos);
        withTimeout(timeout);
    }
    /** 
     * Uses PID to make the robot go to a certain postion relative to the field.  
     */
    public RobotGotoFieldPos(double xPosition, double yPosition, double angle) {
        this(new Pose2d(xPosition, yPosition, new Rotation2d(Math.toRadians(angle))));
    }
    public RobotGotoFieldPos(double xPosition, double yPosition, double angle,double timeout){
        this(new Pose2d(xPosition, yPosition, new Rotation2d(Math.toRadians(angle))));
        withTimeout(timeout);
    }

    /*
     * This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set
     * m_complete to false so the command doesn't
     * instantly end.
     */
    // When not overridden, this function is blank.
    @Override
    public void initialize() {
        SmartDashboard.putString("POSE COMANDO", m_desiredRobotoPos.toString());
        m_complete = false;
        pidControllerX.reset();
        pidControllerY.reset();
        pidControllerAngle.reset();

       
    }

    /*
     * This function is called repeatedly when the schedueler's "run()" function is
     * called.
     * Once you want the function to end, you should set m_complete to true.
     */
    // When not overridden, this function is blank.
    @Override
    public void execute() {
        Pose2d currentPos = drive.getPose();
        SmartDashboard.putData("X", pidControllerX);
        SmartDashboard.putData("Y", pidControllerY);
        SmartDashboard.putData("Angle", pidControllerAngle);
 pidControllerX.setSetpoint(m_desiredRobotoPos.getX());
        pidControllerY.setSetpoint(m_desiredRobotoPos.getY());
        pidControllerAngle.setSetpoint(MathUtils.angleConstrain(m_desiredRobotoPos.getRotation().getDegrees()));
     
        double xSpeed = pidControllerX.calculate(currentPos.getTranslation().getX());
        double ySpeed = pidControllerY.calculate(currentPos.getTranslation().getY());
        double angleSpeed = pidControllerAngle.calculate(drive.getHeading().getDegrees());

        xSpeed = MathUtil.clamp(xSpeed, -HeadingConstants.kTranslationMaxOutput, HeadingConstants.kTranslationMaxOutput);
        ySpeed = MathUtil.clamp(ySpeed, -HeadingConstants.kTranslationMaxOutput, HeadingConstants.kTranslationMaxOutput);
        angleSpeed = MathUtil.clamp(angleSpeed, -HeadingConstants.kHeadingMaxOutput, HeadingConstants.kHeadingMaxOutput);


        drive.drive(new Translation2d(xSpeed,ySpeed),angleSpeed,true);
        
        if(pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerAngle.atSetpoint()){
            m_complete = true;
        }
       
        
    }

    /*
     * This function is called once when the command ends.
     * A command ends either when you tell it to end with the "isFinished()"
     * function below, or when it is interupted.
     * Whether a command is interrupted or not is determined by
     * "boolean interrupted."
     * Things initialized in "initialize()" should be closed here.
     */
    // When not overridden, this function is blank.
    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds( ));
    }

    /*
     * This fuction is used to tell the robot when the command has ended.
     * This function is called after each time the "execute()" function is ran.
     * Once this function returns true, "end(boolean interrupted)" is ran and the
     * command ends.
     * It is recommended that you don't use this for commands that should run
     * continuously, such as drive commands.
     */
    // When not overridden, this function returns false.
    @Override
    public boolean isFinished() {
        return m_complete;
    }

}
