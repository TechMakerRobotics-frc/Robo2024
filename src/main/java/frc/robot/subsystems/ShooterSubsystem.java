
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem instance;

  //Dois motores, um de  cada lado 
  CANSparkMax  motorDown = new CANSparkMax(ShooterConstants.kShooterDownMotor,MotorType.kBrushless);
  CANSparkMax  motorUp = new CANSparkMax (ShooterConstants.kShooterUpMotor,MotorType.kBrushless);
  RelativeEncoder encoder = motorUp.getEncoder();
  public ShooterSubsystem() {
    
    //Limpo qualquer configuração  inicial dos modulos
    motorDown.restoreFactoryDefaults();
    motorUp.restoreFactoryDefaults();

    motorDown.setOpenLoopRampRate(ShooterConstants.kRampRate);
    motorUp.setOpenLoopRampRate(ShooterConstants.kRampRate);

    //Configuro para  que o  motor se mantenha estatico quando em 0
    motorDown.setIdleMode(IdleMode.kBrake);
    motorUp.setIdleMode(IdleMode.kBrake);

    //Inverto o motor de baixo para que girem juntos
    motorDown.setInverted(false);
    motorUp.setInverted(false);
    
    
  }
  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }


  public void setMotorPower(double forward) {
  
      motorUp.set(forward);
      motorDown.set(forward);
    
  }
  public double getRPM(){
    return encoder.getVelocity();
  }
  @Override
  public void periodic(){

    SmartDashboard.putNumber("Shooter Rotacao ", encoder.getVelocity());

  }
}
