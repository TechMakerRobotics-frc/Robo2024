package frc.robot.commands.Claw;

import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class OutsideClaw extends InstantCommand {

    private final ClawSubsystem claw = ClawSubsystem.getInstance();

    @Override
    public void initialize() {
      addRequirements(claw);
    }
  
    @Override
    public void execute() {
  
        claw.setMotorPower(ClawConstants.kpowerOutside);
    }
  }
    
