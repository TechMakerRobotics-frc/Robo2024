
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PosesConstants;
import frc.robot.commands.AlignToNote;
import frc.robot.commands.AlignToNoteSensor;
import frc.robot.commands.AlignToSpeaker;

import frc.robot.commands.swervedrive.RobotGotoFieldPos;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto5Notes extends SequentialCommandGroup {

  public Auto5Notes() {
    SwerveSubsystem.getInstance().resetOdometry(PosesConstants.atSpeakerMiddle);
    SwerveSubsystem.getInstance().zeroGyro();
    addCommands(

      new RobotGotoFieldPos(PosesConstants.atSpeakerShooter,2),
      new AlignToSpeaker(3),
      new RobotGotoFieldPos(PosesConstants.atSpeakerShooter.getX()+0.5,PosesConstants.atSpeakerShooter.getY(),0,1),
      new AlignToNoteSensor(3),
      new RobotGotoFieldPos(PosesConstants.atSpeakerShooter,2),
      new AlignToSpeaker(5),
      new RobotGotoFieldPos(PosesConstants.atSpeakerShooter.getX(),PosesConstants.atSpeakerShooter.getY(),-90,2),
      new AlignToNoteSensor(3),
            new RobotGotoFieldPos(PosesConstants.atSpeakerShooter,2),

      new AlignToSpeaker(5),
      new RobotGotoFieldPos(PosesConstants.atSpeakerShooter.getX(),PosesConstants.atSpeakerShooter.getY(),90,2),
      new AlignToNoteSensor(3),
      new RobotGotoFieldPos(PosesConstants.atSpeakerShooter,2),

      new AlignToSpeaker(5)

      /*
      new MoveXYHeading(0, 1, 0),
      new StartIntake(),
      new AlignToNote(),
      new MoveXYHeading(1, 0, 0),
      new IntakeSensor(),
      new MoveXYHeading(-1, 0, 0),
      new AlignToSpeaker(),
      new StartShooter(),
      new WaitCommand(1),
      new StopShooter(),
      new MoveXYHeading(1, 0, 0),
      new MoveXYHeading(0, 0, -45),
      new MoveXYHeading(2, 0, 0),
      new StartIntake(),
      new AlignToNote(),
      new MoveXYHeading(1, 0, 0),
      new IntakeSensor(),
      new MoveXYHeading(-3, 0, 0),
      new MoveXYHeading(0, 0, 45),
      new AlignToSpeaker(),
      new StartShooter(),
      new WaitCommand(1),
      new StopShooter()*/
      );
  }
}