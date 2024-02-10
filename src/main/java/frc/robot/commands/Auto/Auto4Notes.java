
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.IntakeSensor;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.swervedrive.auto.MoveXYHeading;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto4Notes extends SequentialCommandGroup {

  public Auto4Notes(SwerveSubsystem drivebase) {

    addCommands(
      new StartIntake(),
      new MoveXYHeading(-3.1, 0, 0, drivebase),
      new IntakeSensor(),
      new MoveXYHeading(2.1, 0, 0, drivebase),
      new StartShooter(),
      new WaitCommand(0.5),
      new StopShooter(),
      new MoveXYHeading(-2.1, 0, 0, drivebase),
      new MoveXYHeading(0, 0, 90, drivebase),
      new StartIntake(),
      new MoveXYHeading(0, 1.6, 0, drivebase),
      new IntakeSensor(),
      new MoveXYHeading(0, -1.6, 0, drivebase),
      new MoveXYHeading(0, 0, -90, drivebase),
      new MoveXYHeading(-2.1, 0, 0, drivebase),
      new StartShooter(),
      new WaitCommand(0.5),
      new StopShooter()

    );
  }
}