
package frc.robot;

//import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToSpeaker;
import frc.robot.commands.Claw.InsideClaw;
import frc.robot.commands.Claw.OutsideClaw;
import frc.robot.commands.Claw.StopClaw;
import frc.robot.commands.Elevator.DownElevator;
import frc.robot.commands.Elevator.StopElevator;
import frc.robot.commands.Elevator.UpElevator;
import frc.robot.commands.Intake.IntakeSensor;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.Shooter.ReverseShooter;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem drivebase = SwerveSubsystem.getInstance();
    //private final PhotonVisionSubsystem photonVision = new PhotonVisionSubsystem();
    
    private final Joystick m_operatorControlller = new Joystick(OperatorConstants.kOperatorControllerPort);
    //Eventos do intake prontos para os botões
  Trigger bIntakeSensor = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonIntakeSensor);
  Trigger bReverseIntake = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonReverseIntake);
  Trigger bAlignToSpeaker = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonAlignToSpeaker);
  
  //Eventos do shooter prontos para os botões
  Trigger bStartShooter = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonStartShooter);
  Trigger bReverseShooter = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonReverseShooter);

  //Eventos da garra prontos para os botões
  Trigger bInsideClaw = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonInsideClaw);
  Trigger bOutisdeClaw = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonOutsideClaw);

  // Eventos do elevador prontos para os botões
  Trigger bUpElevator = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonUpElevator);
  Trigger bDownElevator = new JoystickButton(m_operatorControlller,OperatorConstants.kButtonDownElevator);
  Trigger bStopElevator = new JoystickButton(m_operatorControlller, OperatorConstants.kButtonStopElevator);




    CommandXboxController driverController = new CommandXboxController(0);
    //CommandXboxController operatorController = new CommandXboxController(1);
    Joystick operatorController = new Joystick(1); 
    XboxController xbox = new XboxController(0);
    Trigger twoBumper = new Trigger(
            () -> (driverController.getRawAxis(2) > 0.85 && driverController.getRawAxis(3) > 0.85));
    Command driveFieldOrientedAnglularVelocity;

    public RobotContainer() {

        driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
                () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> (driverController.getRawAxis(3) - driverController.getRawAxis(2)));

    }

    // Configura os botões do Xbox.
    void configureBindings() {

        // Controle do piloto

        driverController.povRight().onTrue(new InstantCommand(drivebase::zeroGyro));
        // driverXbox.povLeft().onTrue(new InstantCommand(drivebase::resetOdometry));
        driverController.a().onTrue(new InstantCommand(drivebase::lock));

        // Controle do operador:


         bIntakeSensor.onTrue(new IntakeSensor()); 

    bReverseIntake.onTrue(new ReverseIntake())
              .onFalse(new StopIntake());
   
    bAlignToSpeaker.onTrue(new AlignToSpeaker());

    bStartShooter.onTrue(new StartShooter())
          .onFalse(new StopShooter());
    
    bReverseShooter.onTrue(new ReverseShooter())
                .onFalse(new StopShooter());

    bInsideClaw.onTrue(new InsideClaw())
               .onFalse(new StopClaw());

    bOutisdeClaw.onTrue(new OutsideClaw())
                .onFalse(new StopClaw());
    
    bUpElevator.onTrue(new UpElevator())
                  .onFalse(new StopElevator());

    bDownElevator.onTrue(new DownElevator())
                .onFalse(new StopElevator());

   /*      operatorControllerx.()
                .onTrue(new StartShooter())
                .onFalse(new StopShooter());

        operatorController.y().onTrue(new IntakeSensor());

        operatorController.a()
                .onTrue(new ReverseShooter())
                .onFalse(new StopShooter());
                

        operatorController.b()
                .onTrue(new ReverseIntake())
                .onFalse(new StopIntake());

        operatorController.povUp()
                .onTrue(new UpElevator())
                .onFalse(new StopElevator());
        operatorController.povDown()
                .onTrue(new DownElevator())
                .onFalse(new StopElevator());
        operatorController.back().onTrue(new StopElevator());
        operatorController.povRight()
                .onTrue(new InsideClaw())
                .onFalse(new StopClaw());
        operatorController.povLeft()
                .onTrue(new OutsideClaw())
                .onFalse(new StopClaw());
        operatorController.leftBumper().onTrue(new AlignToSpeaker()); */

        twoBumper
                .onTrue(new InstantCommand(() -> xbox.setRumble(RumbleType.kBothRumble, 1)))
                .onFalse((new InstantCommand(() -> xbox.setRumble(RumbleType.kBothRumble, 0))));

        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    }

    public Command getAutonomousCommand() {

        return drivebase.getAutonomousCommand("3 notes blue");
    }

    public void setDriveMode() {
        // drivebase.setDefaultCommand();
    }
}