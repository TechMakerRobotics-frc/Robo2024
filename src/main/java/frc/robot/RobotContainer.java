
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
import frc.robot.commands.Elevator.DownElevator;
import frc.robot.commands.Elevator.UpElevator;
import frc.robot.commands.Intake.IntakeSensor;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;

import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer
{
    private final SwerveSubsystem drivebase;
    private final IntakeSubsystem intake  = IntakeSubsystem.getInstance();
    private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private final ClawSubsystem claw = ClawSubsystem.getInstance();
    //private final PhotonVision photonVision = new PhotonVision();
    
    // Subtitua por CommandPS4Controller ou CommandJoystick se necessário.
    CommandXboxController driverXbox = new CommandXboxController(0);
    CommandXboxController driverXboxOperator = new CommandXboxController(1);
    XboxController xbox = new XboxController(0);
    Trigger twoBumper = new Trigger(()-> (driverXbox.getRawAxis(2)>0.85 && driverXbox.getRawAxis(3)>0.85 ));  

  TeleopDrive closedFieldRel;



    public RobotContainer(){
      drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
      closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                  OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                  OperatorConstants.LEFT_X_DEADBAND),
        () -> (driverXbox.getRawAxis(3)-driverXbox.getRawAxis(2)), () -> true);


    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     //* CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */

    // Configura os botões do Xbox.
    void configureBindings(){

        //Controle do piloto

        driverXbox.povRight().onTrue(new InstantCommand(drivebase::zeroGyro));
        driverXbox.povLeft().onTrue(new InstantCommand(drivebase::resetOdometry));
        driverXbox.a().onTrue(new InstantCommand(drivebase::lock));

        //Controle do operador:

        driverXboxOperator.x()
        .onTrue(new StartShooter())
        .onFalse(new StopShooter());
        
        driverXboxOperator.y().onTrue(new IntakeSensor());
        
        driverXboxOperator.a()
        .onTrue(new InstantCommand(()->intake.setMotorPower(IntakeConstants.kReversePower),intake))
        .onTrue(new InstantCommand(()->shooter.setMotorPower(ShooterConstants.kReversePower),shooter))
        .onFalse(new InstantCommand(()->intake.setMotorPower(0),intake))
        .onFalse(new InstantCommand(()->shooter.setMotorPower(0),shooter));

        driverXboxOperator.b()
        .onTrue(new InstantCommand(()->intake.setMotorPower(IntakeConstants.kReversePower),intake))
        .onFalse(new InstantCommand(()->intake.setMotorPower(0),intake));

        driverXboxOperator.povUp()
        .onTrue(new UpElevator());
        
        driverXboxOperator.povDown()
        .onTrue(new DownElevator());


        driverXboxOperator.povRight()
        .onTrue(new InsideClaw())
        .onFalse(new InstantCommand(()->claw.setMotorPower(0),claw));
        
        driverXboxOperator.povLeft()
        .onTrue(new OutsideClaw())
        .onFalse(new InstantCommand(()->claw.setMotorPower(0),claw));

        
        twoBumper
        .onTrue(new InstantCommand(()->xbox.setRumble(RumbleType.kBothRumble, 1)))
        .onFalse((new InstantCommand(()->xbox.setRumble(RumbleType.kBothRumble, 0))));
        
        drivebase.setDefaultCommand(closedFieldRel);





    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new Auto5Heading(drivebase);
  }


  public void setDriveMode(){
    //drivebase.setDefaultCommand();
  }
}