package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Limelight;
import frc.robot.Constants.Auto;

public class AlignToStage extends Command {

    private static PIDController vxStageController = new PIDController( Auto.VX_STAGE_kP, 0, 0);
    private static PIDController vyStageController = new PIDController(Auto.VY_STAGE_kP, 0, 0);

    private final Timer timer = new Timer();

    public AlignToStage() {
        addRequirements(SwerveSubsystem.getInstance());
        vxStageController.setTolerance(Auto.MAX_ERROR_DEG_TX_STAGE);
        vyStageController.setTolerance(Auto.MAX_ERROR_DEG_TY_STAGE);
        vxStageController.setSetpoint(0);
        vyStageController.setSetpoint(Auto.VERTICAL_DEG_STAGE);
    }

    @Override
    public void initialize() {
        vxStageController.reset();
        vyStageController.reset();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (Limelight.atStage())
        {
            double vx = vxStageController.calculate(Limelight.getTx());
            // double vy = -vyStageController.calculate(Limelight.getTy());
            double vy = 0;

            SwerveSubsystem.getInstance().drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0, SwerveSubsystem.getInstance().getHeading()));
    
        }   
    }

    @Override
    public boolean isFinished() {
        return (vxStageController.atSetpoint() /*&& vyStageController.atSetpoint()*/) ||
            timer.get() >= 1;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().drive(new ChassisSpeeds());
    }
}