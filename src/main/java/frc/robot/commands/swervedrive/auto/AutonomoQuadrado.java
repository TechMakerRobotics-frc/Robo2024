
package frc.robot.commands.swervedrive.auto;


import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomoQuadrado extends SequentialCommandGroup {

  Command defaultDriveCommand;
  public AutonomoQuadrado (SwerveSubsystem drivebase){
  {
    drivebase.removeDefaultCommand();
      addCommands(
        new MoveXYHeading(2.1, 0, 0, drivebase),
        new WaitCommand(1),
        new MoveXYHeading(0, 2.2, 0, drivebase),
        new WaitCommand(1),
        new MoveXYHeading(-2.1, 0, 0, drivebase),
        new WaitCommand(1),
        new MoveXYHeading(0, -2.2, 0, drivebase),
        new WaitCommand(1));

// para andar 2metros x = 2.1
// para andar 2metros y = 2.2

    }
  }
}