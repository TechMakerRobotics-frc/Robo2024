package frc.robot.commands.Intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ReverseIntake extends InstantCommand {

    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

    @Override
    public void initialize() {
      addRequirements(intake);
    }
  
    @Override
    public void execute() {
  
        intake.setMotorPower(IntakeConstants.kReversePower);
    }
  }
    
