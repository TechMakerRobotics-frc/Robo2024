
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PosesConstants;
import frc.robot.commands.AlignToNoteSensor;
import frc.robot.commands.AlignToSpeaker;

import frc.robot.commands.swervedrive.RobotGotoFieldPos;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Poses;

public class Auto4Notes extends SequentialCommandGroup {
  Poses poses = new Poses();
  public Auto4Notes() {
    SwerveSubsystem.getInstance().resetOdometry(PosesConstants.atSpeakerMiddle);
    SwerveSubsystem.getInstance().zeroGyro();
    if(DriverStation.getAlliance().get()==Alliance.Blue){
      addCommands(
        new RobotGotoFieldPos(poses.changePose(PosesConstants.atSpeakerShooter,0,-1,0),1),
        new AlignToSpeaker(3),
        new RobotGotoFieldPos(poses.changePose(PosesConstants.atSpeakerShooter,0.5,0,0),1),
        new AlignToNoteSensor(3),
        new RobotGotoFieldPos(PosesConstants.atSpeakerShooter,2),
        new AlignToSpeaker(5),
        new RobotGotoFieldPos(poses.changePose(PosesConstants.atSpeakerShooter,0.5,0,-90),1),
        new AlignToNoteSensor(3),
        new RobotGotoFieldPos(PosesConstants.atSpeakerShooter,2),
        new AlignToSpeaker(5),
        new RobotGotoFieldPos(poses.changePose(PosesConstants.atSpeakerShooter,0.5,0,90),1),
        new AlignToNoteSensor(3),
        new RobotGotoFieldPos(PosesConstants.atSpeakerShooter,2),

        new AlignToSpeaker(5)
        );
    }
    else if(DriverStation.getAlliance().get()==Alliance.Red){
      addCommands(

      new RobotGotoFieldPos(poses.changePose(PosesConstants.atSpeakerShooter,0,-1,0),1),
      new AlignToSpeaker(3),
      new RobotGotoFieldPos(poses.changePose(PosesConstants.atSpeakerShooter,0.5,0,0),1),
      new AlignToNoteSensor(3),
      new RobotGotoFieldPos(PosesConstants.atSpeakerShooter,2),
      new AlignToSpeaker(5),
      new RobotGotoFieldPos(poses.changePose(PosesConstants.atSpeakerShooter,0.5,0,90),1),
      new AlignToNoteSensor(3),
      new RobotGotoFieldPos(PosesConstants.atSpeakerShooter,2),
      new AlignToSpeaker(5),
      new RobotGotoFieldPos(poses.changePose(PosesConstants.atSpeakerShooter,0.5,0,-90),1),
      new AlignToNoteSensor(3),
      new RobotGotoFieldPos(PosesConstants.atSpeakerShooter,2),
      new AlignToSpeaker(5)
      );
    }
    else{
        addCommands(
          new RobotGotoFieldPos(poses.changePose(PosesConstants.atSpeakerShooter,0,-1,0),1),
          new AlignToSpeaker(3)
          );
      }
    
  }
  
}