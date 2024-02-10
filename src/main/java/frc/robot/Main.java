
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/*
NÃO adicione nenhuma variável estática a esta classe, ou qualquer inicialização. A menos que você saiba o que
você está fazendo, não modifique este arquivo, exceto para alterar a classe de parâmetro para a chamada startRobot.
 */
public final class Main
{
    private Main() {}

    /*Método de inicialização principal. Não execute nenhuma inicialização aqui.
    * Se você alterar sua classe principal Robot (nome), altere o tipo de parâmetro.*/
    
    public static void main(String... args)
    {
        RobotBase.startRobot(Robot::new);
    }
}
