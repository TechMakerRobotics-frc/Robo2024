
package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot
{

    private Command autonomousCommand;
    
    private RobotContainer robotContainer;
    
// Este método é chamado quando o robô inicia.
    @Override
    public void robotInit(){
        robotContainer = new RobotContainer();
    }
    
/*Esse método é chamado a cada 20 ms, não importa o modo. Use isso para itens como diagnósticos 
que você quer executado durante desativado, autônomo, teleoperado e teste.*/

    @Override
    public void robotPeriodic()
    {
/*Executa o Agendador.  Isso é responsável pelos botões de sondagem, adicionando recém-agendados
comandos, executando comandos já agendados, removendo comandos concluídos ou interrompidos,
e execução de métodos periódicos do subsistema.  Isso deve ser chamado a partir do periódico do robô
para que qualquer coisa na estrutura baseada em comando funcione.*/

        CommandScheduler.getInstance().run();
    }
    
    
// Este método é chamado uma vez cada vez que o robô entra no modo disable.
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}
    
/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit()
    {
        autonomousCommand = robotContainer.getAutonomousCommand();
        
        if (autonomousCommand != null){
            autonomousCommand.schedule();
        }
    }
    
// Método chamado periódicamente durante o período autônomo
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit()
    {
/*Isso garante que o autônomo pare de funcionar quando
O Teleop começa a funcionar. Se você quer o autônomo para
continuar até ser interrompido por outro comando, remover
esta linha ou comentá-lo.*/

        robotContainer.configureBindings();
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }
    
    
// Método chamado periódicamente durante o período Teleoperado.
    @Override
    public void teleopPeriodic() {}
    
// Este método é chamado quando o robô inicia o modo teste.
    @Override
    public void testInit()
    {
//Cancela todos os comandos em execução no início do modo de teste.
        CommandScheduler.getInstance().cancelAll();
    }
    
    
// Método chamado periódicamente durante o período de teste.
    @Override
    public void testPeriodic() {}
    
    
//Este método é chamado uma vez quando o robô é iniciado pela primeira vez.
    @Override
    public void simulationInit() {}
    
    
//Este método é chamado periodicamente enquanto em simulação.
    @Override
    public void simulationPeriodic() {}

    
}

