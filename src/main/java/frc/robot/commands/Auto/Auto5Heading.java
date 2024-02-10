
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.auto.MoveXYHeading;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto5Heading extends SequentialCommandGroup {

  public Auto5Heading(SwerveSubsystem drivebase) {

    addCommands(
      new MoveXYHeading(0, 0, 172, drivebase)
    
    );
  }
}