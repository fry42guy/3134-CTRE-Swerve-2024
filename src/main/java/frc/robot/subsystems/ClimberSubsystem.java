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

  public final TalonFX ClimberLeftMotor;
    public final TalonFX ClimberRightMotor;
  /** Creates a new Climber. */
  public ClimberSubsystem() {

ClimberLeftMotor = new TalonFX(Constants.Climber.ClimberLeftID);
ClimberLeftMotor.setInverted(true);

ClimberRightMotor = new TalonFX(Constants.Climber.ClimberRightID);
ClimberRightMotor.setInverted(false);

  }

  public void setspeed(Double speed){
ClimberLeftMotor.set(speed);
ClimberRightMotor.set(speed);
  }

// public void ClimberFWD(){
//   ClimberMotor.set(ControlMode.PercentOutput, Constants.Climber.FWDSpeed);
// }

// public void ClimberREV(){
//   ClimberMotor.set(ControlMode.PercentOutput, Constants.Climber.REVSpeed);
// }

public void Stop(){

  ClimberLeftMotor.set(0);
  ClimberRightMotor.set(0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
