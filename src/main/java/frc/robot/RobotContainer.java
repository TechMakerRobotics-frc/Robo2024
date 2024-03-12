
package frc.robot;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;

//import java.io.File;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.limelight.LimelightCalcs;
import frc.robot.subsystems.limelight.LimelightProfile;
import frc.robot.util.Poses;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
//import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PosesConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants2;
import frc.robot.commands.AlignToNote;
import frc.robot.commands.AlignToSpeaker;
import frc.robot.commands.Auto.Auto4Notes;
import frc.robot.commands.Auto.AutoMiddle;
import frc.robot.commands.Auto.AutoSimple;
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
import frc.robot.commands.swervedrive.RobotGotoFieldPos;
import frc.robot.commands.Shooter.ReverseShooter;
import frc.robot.commands.Shooter.StartShooter;

public class RobotContainer {
        private final SwerveSubsystem drivebase = SwerveSubsystem.getInstance();
        private final LEDSSubsystem leds = LEDSSubsystem.getInstance();
        private final ClawSubsystem claw = ClawSubsystem.getInstance();
        private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
        private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
        private final PoseEstimatorSubsystem poseEstimator =
        new PoseEstimatorSubsystem(drivebase::getGyroYaw, drivebase::getModulePositions);
        Poses poses = new Poses();
        private final LimelightSubsystem lowLimelightSubsystem = new LimelightSubsystem();
        private final LimelightSubsystem highLimelightSubsystem = new LimelightSubsystem();
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController operatorController = new CommandXboxController(1);
        XboxController xbox = new XboxController(0);
        Trigger twoBumper = new Trigger(
                        () -> (driverController.getRawAxis(2) > 0.85 && driverController.getRawAxis(3) > 0.85));
        Command driveFieldOrientedAngularVelocity;
        Trigger sensorClaw = new Trigger(() -> claw.getSensor());
        Trigger elevatorSensor = new Trigger(() -> elevator.getLimiSwitch());
        Trigger intakeSensor = new Trigger(() -> intake.getSensor());
        Trigger AlertController = new Trigger(() -> intake.alertController());
        private final SendableChooser<Integer> m_chooser = new SendableChooser<>();
        private boolean pickingUp = false;

        public RobotContainer() {

                driveFieldOrientedAngularVelocity = drivebase.driveCommand(
                                () -> MathUtil.applyDeadband(driverController.getLeftY(),
                                                OperatorConstants.kLeftYDeadBand),
                                () -> MathUtil.applyDeadband(driverController.getLeftX(),
                                                OperatorConstants.kLeftXDeadBand),
                                () -> (driverController.getRawAxis(2) - driverController.getRawAxis(3)));
                m_chooser.setDefaultOption("Autonomo 4 notas", AutonomousConstants.kAuto4Notes);
                m_chooser.addOption("Simples saindo a direita", AutonomousConstants.kAutoSimpleRight);
                m_chooser.addOption("Simples saindo ao meio", AutonomousConstants.kAutoSimpleMiddle);
                m_chooser.addOption("Simples saindo a esquerda", AutonomousConstants.kAutoSimpleLeft);
                m_chooser.addOption("Longo saindo a direita", AutonomousConstants.kAutoLongRight);
                m_chooser.addOption("Longo saindo a esquerda", AutonomousConstants.kAutoLongLeft);

                SmartDashboard.putData("Autonomo", m_chooser);
        }

         private void configureDashboard() {
    /**** Driver tab ****/
    var driverTab = Shuffleboard.getTab("Driver");
    driverTab.addBoolean("Pickup", () -> pickingUp).withPosition(11, 3).withSize(2, 2);

    driverTab.add(new HttpCamera("limelight-high", "http://10.90.47.110:5801"))
        .withWidget(BuiltInWidgets.kCameraStream)
        .withProperties(Map.of("showCrosshair", true, "showControls", false, "rotation", "QUARTER_CCW"))
        .withSize(4, 6).withPosition(0, 0);

    /**** Vision tab ****/
    final var visionTab = Shuffleboard.getTab("Vision");

    // Pose estimation
    poseEstimator.addDashboardWidgets(visionTab);

    // Top target
    final var topLimelightCalcs = new LimelightCalcs(
        LimelightProfile.SCORE_CONE_TOP.cameraToRobot, LimelightProfile.SCORE_CONE_TOP.targetHeight);
    final var topTargetLayout = visionTab.getLayout("Top Target").withPosition(6, 0).withSize(1, 2);

    final var midTargetLayout = visionTab.getLayout("Mid Target").withPosition(7, 0).withSize(1, 2);
    
 
   
  }

        // Configura os botões do Xbox.
        void configureBindings() {
                // Controle do piloto
                driverController.povRight().onTrue(new InstantCommand(drivebase::zeroGyro));
                driverController.leftBumper().whileTrue(new AlignToSpeaker());
                driverController.rightBumper().whileTrue(new AlignToNote());
                // driverController.x().onTrue(new AlignToAmp());
                driverController.a().onTrue(new RobotGotoFieldPos(poses.changePose(drivebase.getPose(), 1, 0, 0)));
                driverController.x().onTrue(new RobotGotoFieldPos(poses.changePose(drivebase.getPose(), 0, 0, 0)));
                driverController.y().onTrue(new RobotGotoFieldPos(poses.changePose(drivebase.getPose(), 0, 0, 90)));
                driverController.b().onTrue(new RobotGotoFieldPos(poses.changePose(drivebase.getPose(), 0, 0, -90)));

                driverController.povDown().onTrue(new InstantCommand(() -> claw.setCleanerMotor(90), claw));
                driverController.povUp().onTrue(new InstantCommand(() -> claw.setCleanerMotor(0), claw));

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

                AlertController
                                .onTrue(new InstantCommand(() -> xbox.setRumble(RumbleType.kBothRumble, 1)))
                                .onFalse((new InstantCommand(() -> xbox.setRumble(RumbleType.kBothRumble, 0))));

                sensorClaw.onTrue(new InstantCommand(() -> leds.setRGB(0, 255, 0)))
                                .onFalse(new InstantCommand(() -> leds.setLedTeamColor()));
                elevatorSensor.onTrue(new InstantCommand(() -> leds.setRGB(127, 0, 127)))
                                .onFalse(new InstantCommand(() -> leds.setLedTeamColor()));
                intakeSensor.onTrue(new InstantCommand(() -> leds.setRGB(127, 127, 0)))
                                .onFalse(new InstantCommand(() -> leds.setLedTeamColor()));
                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

        }

        public Command getAutonomousCommand() {
                int autoSelected = m_chooser.getSelected();
                switch (autoSelected) {
                        case AutonomousConstants.kAuto4Notes:
                                return new Auto4Notes();
                        case AutonomousConstants.kAutoSimpleLeft:
                                return new AutoSimple(PosesConstants.atSpeakerLeft);
                        case AutonomousConstants.kAutoSimpleMiddle:
                                return new AutoSimple(PosesConstants.atSpeakerMiddle);
                        case AutonomousConstants.kAutoSimpleRight:
                                return new AutoSimple(PosesConstants.atSpeakerRight);
                        case AutonomousConstants.kAutoLongRight:
                                return new AutoMiddle(PosesConstants.atSpeakerRight);
                        case AutonomousConstants.kAutoLongLeft:
                                return new AutoMiddle(PosesConstants.atSpeakerLeft);

                        default:
                                return null;
                }
                // return drivebase.getAutonomousCommand("3 notes blue");
        }

}