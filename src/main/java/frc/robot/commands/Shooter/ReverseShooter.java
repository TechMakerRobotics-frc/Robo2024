package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseShooter extends InstantCommand {
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

  @Override
  public void initialize() {
      addRequirements(intake);
      addRequirements(shooter);
  }
  
  @Override
  public void execute() {
      intake.setMotorPower(IntakeConstants.kReversePower);
      shooter.setMotorPower(ShooterConstants.kReversePower);
  }
}
    

