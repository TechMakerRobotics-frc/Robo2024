
package frc.robot.commands.swervedrive.auto;


import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomoControle extends SequentialCommandGroup {

  Command defaultDriveCommand;
  public AutonomoControle (SwerveSubsystem drivebase){
  {
    drivebase.removeDefaultCommand();
      addCommands(

  new AutonomoQuadrado(drivebase),
  new AutonomoDiagonais(drivebase));
  
  // para andar 2metros x = 2.1
  // para andar 2metros y = 2.2
                                        
    }
  }
}
