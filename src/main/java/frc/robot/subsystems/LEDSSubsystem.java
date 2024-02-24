// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSSubsystem extends SubsystemBase {
  SerialPort arduino = null;
  static LEDSSubsystem instance = null;
  double timeoutColor = 0;
  private int r,g,b;
  public LEDSSubsystem() {
    try{
      arduino = new SerialPort(115200,SerialPort.Port.kUSB);
    }
    catch (Exception e){
      System.out.println(e);
      try{
        arduino = new SerialPort(115200,SerialPort.Port.kUSB1);
      }
      catch (Exception e1){
        System.out.println(e1);
        try{
        arduino = new SerialPort(115200,SerialPort.Port.kUSB2);
        }
        catch (Exception e2){
          System.out.println(e2);
        }
      }
    }
    setLedTeamColor(); 
  }
public static LEDSSubsystem getInstance(){
  if(instance==null){
    instance = new LEDSSubsystem();
  }
  return instance;
}
public void setLedTeamColor(){
    if(arduino != null){
        if(DriverStation.isAutonomous()){
            setRGB(0, 255, 0);
          
        }
        else{
          if(DriverStation.getAlliance().get() == Alliance.Blue){
            setRGB(0, 0, 255);          }
          else if(DriverStation.getAlliance().get() == Alliance.Red){
            setRGB( 255,0, 0);
          }
          else{
            setRGB(0, 255, 0);
          }
      }
    }
  }

  public void setRGB(int R, int G, int B){
   r = R;
   g = G;
   b = B;
  }
  @Override
  public void periodic() {
     try{
        arduino.writeString(String.format("%03d%03d%03d\n",b,g,r));
      }
      catch (Exception e1){
        System.out.println(e1);

    }

  }
}
