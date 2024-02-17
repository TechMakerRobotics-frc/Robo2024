
package frc.robot.commands.Auto;


import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToNote;
import frc.robot.commands.AlignToSpeaker;
import frc.robot.commands.Intake.IntakeSensor;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.swervedrive.MoveXYHeading;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto extends SequentialCommandGroup {

  public Auto (SwerveSubsystem drivebase){
  {
      addCommands(
        new AlignToSpeaker(),
        new StartShooter(),
        new WaitCommand(1),
        new StopShooter(),
        new StartIntake(),
        new AlignToNote(),
        new MoveXYHeading(-0.9, 0, 0, drivebase),
        new IntakeSensor(),
        new AlignToSpeaker(),
        new StartShooter(),
        new WaitCommand(1),
        new StopShooter());

        // Pega a nota central.

  // para andar 2metros x = 2.1
  // para andar 2metros y = 2.2
                                        
    }
  }
}

// Outros tipos de comandos para serem usados, (lembre-se das importações):

/* Permite que os comandos sejam executados até um determinado tempo limite:
    new DeadlineGroup(
        new CLASSE-USADA(driveSubsystem).withTimeout(5),
        new CLASSE-USADA(driveSubsystem).withTimeout(3)
    ),
 */

/* Executa todos os comandos paralelamente e avança quando todos terminam:

    new ParallelCommandGroup(
        new CLASSE-USADA(driveSubsystem),
        new CLASSE-USADA(driveSubsystem)
    ),

 */

 /* Um comando que executa instantaneamente quando agendado. 
    É útil para tarefas simples ou ações que não exigem uma execução contínua:
  
    new InstantCommand(() -> SmartDashboard.putString("Status", "Acabou o autônomo!")),
    new InstantCommand(() -> shooter.shoot(), shooter)
    new InstantCommand(() -> led.setColor(Color.BLUE), led)
    new InstantCommand(() -> System.out.println("Comando Instantâneo Executado!"))
    new InstantCommand(() -> armMotor.setPosition(0), armMotor)

  */