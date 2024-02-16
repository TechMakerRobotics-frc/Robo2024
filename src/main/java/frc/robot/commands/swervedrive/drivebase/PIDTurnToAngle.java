package frc.robot.commands.swervedrive.drivebase;

import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class PIDTurnToAngle extends Command {    
    private SwerveSubsystem s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    public double rotationVal = 0;

    public double targetAngle = 0;
    public double currentAngle = 0;
    public double acceptableError = 0;

    private final PIDController angleController = new PIDController(0.012, 0, 0); //ki used to be 0

    public PIDTurnToAngle(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, Boolean robotCentricSup, int targetAngle) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.targetAngle = targetAngle;
        angleController.enableContinuousInput(0, 360);
    }

    public void initialize() {
        angleController.setTolerance(5); //used to be 5
    }

    @Override
    public void execute() {
        // elevatorHeight = RobotContainer.elevator.getCurrentPosition();
        // currentAngle = s_Swerve.getGyroYaw().getDegrees() + 180;
        rotationVal = angleController.calculate(currentAngle, targetAngle);
        
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND);
   
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Auto.MAX_SPEED), 
            rotationVal * 4, 
            true
        );
    }

    public boolean isFinished() {
        return false;//angleController.atSetpoint();
    }

    protected void end() {
        
    }

    protected void interrupted(){
        end();
    }
}