
package frc.robot;

//import java.io.File;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
//import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.AlignToAmp;
import frc.robot.commands.AlignToNote;
import frc.robot.commands.AlignToSpeaker;
import frc.robot.commands.Auto.Auto5Notes;
import frc.robot.commands.Claw.InsideClaw;
import frc.robot.commands.Claw.OutsideClaw;
import frc.robot.commands.Claw.StopClaw;
import frc.robot.commands.Elevator.DownElevator;
import frc.robot.commands.Elevator.StopElevator;
import frc.robot.commands.Elevator.UpElevator;
import frc.robot.commands.Intake.IntakeSensor;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.swervedrive.RobotGotoFieldPos;
import frc.robot.commands.swervedrive.drivebase.PIDTurnToAngle;
import frc.robot.commands.Shooter.ReverseShooter;
import frc.robot.commands.Shooter.StartShooter;

public class RobotContainer {
    private final SwerveSubsystem drivebase = SwerveSubsystem.getInstance();
        private final LEDSSubsystem leds = LEDSSubsystem.getInstance();
        private final ClawSubsystem claw = ClawSubsystem.getInstance();
        private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
        private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    CommandXboxController driverController = new CommandXboxController(0);
    CommandXboxController operatorController = new CommandXboxController(1);
    XboxController xbox = new XboxController(0);
    Trigger twoBumper = new Trigger(
            () -> (driverController.getRawAxis(2) > 0.85 && driverController.getRawAxis(3) > 0.85));
    Command driveFieldOrientedAngularVelocity;
    Trigger sensorClaw = new Trigger(() ->claw.getSensor());
    Trigger elevatorSensor = new Trigger(()->elevator.getLimiSwitch());
    Trigger intakeSensor = new Trigger(()->intake.getSensor());
    public RobotContainer() {

        driveFieldOrientedAngularVelocity = drivebase.driveCommand(
                () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.kLeftYDeadBand),
                () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.kLeftXDeadBand),
                () -> (driverController.getRawAxis(2) - driverController.getRawAxis(3)));

    }

    // Configura os botões do Xbox.
    void configureBindings() {
        // Controle do piloto
        driverController.povRight().onTrue(new InstantCommand(drivebase::zeroGyro));
        driverController.a().onTrue(new InstantCommand(drivebase::lock));
        driverController.leftBumper().whileTrue(new AlignToSpeaker());
        driverController.rightBumper().whileTrue(new AlignToNote());
        //driverController.x().onTrue(new AlignToAmp());
        driverController.b().onTrue(new RobotGotoFieldPos(0,0,0));

        // Controle do operador:
        operatorController.x()
                .onTrue(new StartShooter())
                .onFalse(new StopShooter());

        operatorController.y().whileTrue(new IntakeSensor());
                              
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
        
        twoBumper
                .onTrue(new InstantCommand(() -> xbox.setRumble(RumbleType.kBothRumble, 1)))
                .onFalse((new InstantCommand(() -> xbox.setRumble(RumbleType.kBothRumble, 0))));

        sensorClaw.onTrue(new InstantCommand(()-> leds.setRGB(0, 255, 0)))
        .onFalse(new InstantCommand(()->leds.setLedTeamColor()));
        elevatorSensor.onTrue(new InstantCommand(()-> leds.setRGB(127, 0, 127)))
        .onFalse(new InstantCommand(()->leds.setLedTeamColor()));
        intakeSensor.onTrue(new InstantCommand(()-> leds.setRGB(127, 127, 0)))
        .onFalse(new InstantCommand(()->leds.setLedTeamColor()));
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    }

    public Command getAutonomousCommand() {
        return new Auto5Notes();
        //return drivebase.getAutonomousCommand("3 notes blue");
    }

    public void setDriveMode() {
        // drivebase.setDefaultCommand();
    }
}