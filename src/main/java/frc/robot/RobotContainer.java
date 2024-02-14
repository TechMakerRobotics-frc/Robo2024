
package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Auto.Auto5Heading;
import frc.robot.commands.Claw.InsideClaw;
import frc.robot.commands.Claw.OutsideClaw;
import frc.robot.commands.Claw.StopClaw;
import frc.robot.commands.Elevator.DownElevator;
import frc.robot.commands.Elevator.UpElevator;
import frc.robot.commands.Intake.IntakeSensor;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final SwerveSubsystem drivebase = SwerveSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final ClawSubsystem claw = ClawSubsystem.getInstance();
  private final PhotonVisionSubsystem photonVision = new PhotonVisionSubsystem();
  private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

  // Subtitua por CommandPS4Controller ou CommandJoystick se necessário.
  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandXboxController driverXboxOperator = new CommandXboxController(1);
  XboxController xbox = new XboxController(0);
  Trigger twoBumper = new Trigger(() -> (driverXbox.getRawAxis(2) > 0.85 && driverXbox.getRawAxis(3) > 0.85));
  Command driveFieldOrientedAnglularVelocity;

  public RobotContainer() {
   
     driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> (driverXbox.getRawAxis(3)-driverXbox.getRawAxis(2)));

  }
  // Configura os botões do Xbox.
  void configureBindings() {

    // Controle do piloto

    driverXbox.povRight().onTrue(new InstantCommand(drivebase::zeroGyro));
    // driverXbox.povLeft().onTrue(new InstantCommand(drivebase::resetOdometry));
    driverXbox.a().onTrue(new InstantCommand(drivebase::lock));

    driverXbox.x().whileTrue(drivebase.aimAtTarget(photonVision.getCamera(),
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)));

    // Controle do operador:

    driverXboxOperator.x()
        .onTrue(new StartShooter())
        .onFalse(new StopShooter());

    driverXboxOperator.y().onTrue(new IntakeSensor());   // Falts conectar o sensor na RoboRio

    driverXboxOperator.a()
        .onTrue(new InstantCommand(() -> intake.setMotorPower(IntakeConstants.kReversePower), intake))
        .onTrue(new InstantCommand(() -> shooter.setMotorPower(ShooterConstants.kReversePower), shooter))
        .onFalse(new InstantCommand(() -> intake.setMotorPower(0), intake))
        .onFalse(new InstantCommand(() -> shooter.setMotorPower(0), shooter));

    driverXboxOperator.b()
        .onTrue(new InstantCommand(() -> intake.setMotorPower(IntakeConstants.kReversePower), intake))
        .onFalse(new InstantCommand(() -> intake.setMotorPower(0), intake));

    driverXboxOperator.povUp()
        // .onTrue(new UpElevator());
        .onTrue(new InstantCommand(() -> elevator.setMotorPower(1)))
        .onFalse(new InstantCommand(() -> elevator.setMotorPower(0)));
    driverXboxOperator.povDown()
        // .onTrue(new DownElevator());
        .onTrue(new InstantCommand(() -> elevator.setMotorPower(-1)))
        .onFalse(new InstantCommand(() -> elevator.setMotorPower(0)));

    driverXboxOperator.povRight()
        .onTrue(new InsideClaw())
        .onFalse(new StopClaw());

    driverXboxOperator.povLeft()
        .onTrue(new OutsideClaw())
        .onFalse(new StopClaw());

    twoBumper
        .onTrue(new InstantCommand(() -> xbox.setRumble(RumbleType.kBothRumble, 1)))
        .onFalse((new InstantCommand(() -> xbox.setRumble(RumbleType.kBothRumble, 0))));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return drivebase.getAutonomousCommand("3 notes blue");
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }
}