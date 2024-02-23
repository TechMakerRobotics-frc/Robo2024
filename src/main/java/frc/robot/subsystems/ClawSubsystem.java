
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  private static ClawSubsystem instance;
  DigitalInput sensor = new DigitalInput(ClawConstants.kClawSensor);
  boolean extended = false;

  CANSparkMax  motor = new CANSparkMax(ClawConstants.kClawMotor,MotorType.kBrushless);
  
  public ClawSubsystem() {
    
    motor.restoreFactoryDefaults();

    //Configuro para  que o  motor se mantenha estatico quando em 0
    motor.setIdleMode(IdleMode.kCoast);
  }

  public boolean getSensor(){
    return !(sensor.get());
  }

  public static ClawSubsystem getInstance() {
    if (instance == null) {
        instance = new ClawSubsystem();
    }
    return instance;
}

  public void setMotorPower(double inside ) {
    SmartDashboard.putNumber("Claw Potencia (%)", inside * 100.0);
      motor.set(inside);
  }
}