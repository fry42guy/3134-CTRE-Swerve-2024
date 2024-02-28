// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonSRX IntakeMotor;

  private final DigitalInput RangeFinder ;
  /** Creates a new Intake. */
  public IntakeSubsystem() {

IntakeMotor = new TalonSRX(Constants.Intake.IntakeMotorID);
IntakeMotor.setInverted(true);


RangeFinder = new DigitalInput(3);

//RangeFinder.setAutomaticMode(true);




  }

  public void setspeed(Double speed){
IntakeMotor.set(ControlMode.PercentOutput, speed);

  }

// public void IntakeFWD(){
//   IntakeMotor.set(ControlMode.PercentOutput, Constants.Intake.FWDSpeed);
// }

// public void IntakeREV(){
//   IntakeMotor.set(ControlMode.PercentOutput, Constants.Intake.REVSpeed);
// }

public void Stop(){

  IntakeMotor.set(ControlMode.PercentOutput,0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run


SmartDashboard.putBoolean("Intake Beam", RangeFinder.get());

  }



public boolean Note_In_Intake(){

  return RangeFinder.get();

  




}





}
