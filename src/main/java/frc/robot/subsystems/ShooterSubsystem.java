// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
public class ShooterSubsystem extends SubsystemBase {

public final TalonFX LeftShooter;
public final TalonFX RightShooter;
public double KP;
public double KI;
public double KD;
public double PIDTESTspeed;

public boolean isActive;



  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    LeftShooter = new TalonFX(Constants.Shooter.LeftShooterID);
    LeftShooter.setInverted(false);
    LeftShooter.setPosition(0);
    //LeftShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,25);

    RightShooter = new TalonFX(Constants.Shooter.RightShooterID);
    RightShooter.setInverted(false);
    RightShooter.setPosition(0);

    isActive =false;
    //RightShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,25);
    //Layout = new AprilTagFieldLayout("2024-crescendo.json");

  }

public void stop(){

  LeftShooter.set(0);
  RightShooter.set(0);

}

public void setspeed(Double speed){

LeftShooter.set(speed);
RightShooter.set(speed);


}

public boolean getisActive(){

  return isActive;
}

public void setDiffSpeed(Double leftspeed, double rightspeed){

  LeftShooter.set(leftspeed);
  RightShooter.set(rightspeed);

}

public double GetRightShooterRPM(){


  return (RightShooter.getVelocity().getValueAsDouble()) * 60;

}

public double GetLeftShooterRPM(){


  return (LeftShooter.getVelocity().getValueAsDouble()) * 60;

}

public void PIDSetup(){

if (SmartDashboard.containsKey("LeftShooter(PID) KP")==false){
SmartDashboard.putNumber("LeftShooter(PID) KP", 0.7);
}

if (SmartDashboard.containsKey("LeftShooter(PID) KI") ==false){
  SmartDashboard.putNumber("LeftShooter(PID) KI", 4.0);
  }

  if (SmartDashboard.containsKey("LeftShooter(PID) KD")==false){
    SmartDashboard.putNumber("LeftShooter(PID) KD", 0.0);
    }

    if (SmartDashboard.containsKey("TEST PID (RPM) Speed")==false){
      SmartDashboard.putNumber("TEST PID (RPM) Speed", 2500);
      }



}

public void PIDUpdate(){

  KP = SmartDashboard.getNumber("LeftShooter(PID) KP", 0.7);
  KI = SmartDashboard.getNumber("LeftShooter(PID) KI", 4);
  KD = SmartDashboard.getNumber("LeftShooter(PID) KD", 0.0);
  PIDTESTspeed = SmartDashboard.getNumber("TEST PID (RPM) Speed", 0.0);

  if (PIDTESTspeed > 5000){

    PIDTESTspeed = 5000;
    SmartDashboard.putNumber("TEST PID (RPM) Speed", PIDTESTspeed);
  }
  if (PIDTESTspeed < -5000){

    PIDTESTspeed = -5000;
    SmartDashboard.putNumber("TEST PID (RPM) Speed", PIDTESTspeed);
  }


  SmartDashboard.setPersistent("LeftShooter(PID) KP");
  SmartDashboard.setPersistent("LeftShooter(PID) KI");
  SmartDashboard.setPersistent("LeftShooter(PID) KD");
  SmartDashboard.setPersistent("TEST PID (RPM) Speed");
}
  @Override
  public void periodic() {

    SmartDashboard.putNumber("Left Shooter Motor RPM", GetLeftShooterRPM());
    SmartDashboard.putNumber("Right Shooter Motor RPM ", GetRightShooterRPM());

    //SmartDashboard.putnumber("Tag7 Pose",0 )


    // This method will be called once per scheduler run
  }
}
