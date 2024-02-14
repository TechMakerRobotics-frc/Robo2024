
package frc.robot.commands.Auto;

import frc.robot.commands.swervedrive.MoveXYHeading;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto2Heading extends SequentialCommandGroup {

  public Auto2Heading (SwerveSubsystem drivebase){
  {
      addCommands(
  new MoveXYHeading(0, 0, 90, drivebase));

  // para andar 2metros x = 2.1
  // para andar 2metros y = 2.2
                                        
    }
  }
}