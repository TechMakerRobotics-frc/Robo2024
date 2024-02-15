// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightHelpers;

public class AlignToSpeaker extends Command {
  private static final int SpeakerTagIDBlue = 7;
  private static final int SpeakerTagIDRed = 4;
  private int SpeakerTagID;
  private LimelightHelpers.LimelightTarget_Fiducial ll = new LimelightHelpers.LimelightTarget_Fiducial();
  private Command defaultCommand;
  SwerveSubsystem drive = SwerveSubsystem.getInstance();
  Boolean finished = false;
  String commandString = "AlingToSpeaker";
  /** Creates a new AlignToSpeaker. */
  public AlignToSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        SpeakerTagID = SpeakerTagIDRed;
      }
      if (ally.get() == Alliance.Blue) {
        SpeakerTagID = SpeakerTagIDBlue;
      }
    }
    SmartDashboard.putString(commandString+"/Tag to Shoot", String.valueOf(SpeakerTagID));

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    defaultCommand = drive.getDefaultCommand();
    drive.removeDefaultCommand();
    int tag = (int)ll.fiducialID;
    SmartDashboard.putString(commandString+"/Tag On face", String.valueOf(tag));
    if(tag!=SpeakerTagID)
    {
      SmartDashboard.putString(commandString, "Mirando na tag errada");
      finished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber(commandString+"/Rotation", ll.tx);
    SmartDashboard.putNumber(commandString+"/Vertical", ll.ty);
    SmartDashboard.putNumber(commandString+"/Area", ll.ta);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setDefaultCommand(defaultCommand);
    SmartDashboard.putString(commandString, "Retornado Comando padrao");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
