package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseShooter extends InstantCommand {
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public ReverseShooter() {

    addRequirements(intake);
    addRequirements(shooter);

    
  }

  @Override
  public void initialize() {
    shooter.setMotorPower(ShooterConstants.kReversePower);
    intake.setMotorPower(IntakeConstants.kReversePower);
  }
}
