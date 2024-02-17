// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSSubsystem extends SubsystemBase {
  /** Creates a new LEDS. */
  PWM redPwm = new PWM(0);
  PWM greenPwm = new PWM(1);
  PWM bluePwm = new PWM(2);
  int r, g, b;
  public LEDSSubsystem() {

  }

  public void setRGB(int R, int G, int B){
    
    redPwm.setPulseTimeMicroseconds(Math.abs(R));
    greenPwm.setPulseTimeMicroseconds(Math.abs(G));
    bluePwm.setPulseTimeMicroseconds(Math.abs(B));
  }
  @Override
  public void periodic() {
  
  }
}
