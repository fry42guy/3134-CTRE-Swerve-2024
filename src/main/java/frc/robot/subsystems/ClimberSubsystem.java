// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX ClimberMotor;
  /** Creates a new Climber. */
  public ClimberSubsystem() {

ClimberMotor = new TalonFX(Constants.Climber.ClimberID);
ClimberMotor.setInverted(true);



  }

  public void setspeed(Double speed){
ClimberMotor.set(speed);

  }

// public void ClimberFWD(){
//   ClimberMotor.set(ControlMode.PercentOutput, Constants.Climber.FWDSpeed);
// }

// public void ClimberREV(){
//   ClimberMotor.set(ControlMode.PercentOutput, Constants.Climber.REVSpeed);
// }

public void Stop(){

  ClimberMotor.set(0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
