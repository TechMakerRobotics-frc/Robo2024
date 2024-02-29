package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem instance;
  boolean extending = false;
  //Dois motores, um de  cada lado 
  CANSparkMax  motorLeft = new CANSparkMax(ElevatorConstants.kElevatorLeftMotor,MotorType.kBrushless);
  CANSparkMax  motorRight = new CANSparkMax (ElevatorConstants.kElevatorRightMotor,MotorType.kBrushless);
  
  //dois encoders, um de cada motor
   RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  DigitalInput limiSwitch = new DigitalInput(ElevatorConstants.kLimitSwitch);
  //DigitalInput optical = new DigitalInput(3);
  //AnalogInput distance = new AnalogInput(0);
  /** Creates a new arm. */
  public ElevatorSubsystem() {
    
    //Limpo qualquer configuração  inicial dos modulos
    motorLeft.restoreFactoryDefaults();
    motorRight.restoreFactoryDefaults();

    //Configuro para  que o  motor se mantenha estatico quando em 0
    motorLeft.setIdleMode(IdleMode.kBrake);
    motorRight.setIdleMode(IdleMode.kBrake);
    
    //Inverto o motor da esquerda para que girem juntos
    motorLeft.setInverted(true);
    motorRight.setInverted(false);

    //Associo os encoders, seto a razão de 1 volta e zero os mesmos
    leftEncoder = motorLeft.getEncoder();
    rightEncoder = motorRight.getEncoder();
  
    resetEncoder();
    //setMotorPower(ElevatorConstants.kPowerDown);
    
  }
  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
}
  //Função principal que movimenta o braço para frente(+) e  para tras(-)
  public void setMotorPower(double power) {
    SmartDashboard.putNumber("Elevator Potencia (%)", power * 100.0);
   // extending = power>0;
    motorRight.set(power);
    motorLeft.set(power);
  }

  //Reseta os valores dos encoders, para ter a referencia atual
  public void resetEncoder(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  
  //Função  que captura  os encoders, fazendo uma media dos dois lados e dividindo pela redução
  public double getEncoder(){
    return (((rightEncoder.getPosition()+leftEncoder.getPosition())/2));}
  public boolean getLimiSwitch(){
    return !limiSwitch.get();
  }
  
  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Elevator Encoder", getEncoder());

    SmartDashboard.putBoolean("Elevator LimitSwitch", getLimiSwitch());
    if((extending && getEncoder()>ElevatorConstants.kEncoderTicksTop) || (!extending && getEncoder()<10)){
      motorRight.set(0);
      motorLeft.set(0);
    }
  }
}    
   
