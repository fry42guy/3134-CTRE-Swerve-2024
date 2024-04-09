// Copyright (c) FIRST ano other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

  public final TalonFX ClimberLeftMotor;
    public final TalonFX ClimberRightMotor;
  /** Creates a new Climber. */
  public ClimberSubsystem() {

ClimberLeftMotor = new TalonFX(Constants.Climber.ClimberLeftID);
ClimberLeftMotor.setInverted(true);

ClimberRightMotor = new TalonFX(Constants.Climber.ClimberRightID);
ClimberRightMotor.setInverted(false);
ClimberRightMotor.setPosition(0);
ClimberLeftMotor.setPosition(0);

ConfigSoftlimits();

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

    SmartDashboard.putNumber("Left climber Encoder", ClimberLeftMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right climber Encoder", ClimberRightMotor.getPosition().getValueAsDouble());
  }


  public void ConfigSoftlimits() {


      //PivotMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,Constants.Arm.MAX_CURRENT_DRAW,Constants.Arm.MAX_CURRENT_DRAW + 5, 0.5));
TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    m_currentLimits.SupplyCurrentLimit = 35 ; // Limit to 1 amps
    m_currentLimits.SupplyCurrentThreshold = 40; // If we exceed 4 amps
    m_currentLimits.SupplyTimeThreshold = .5; // For at least 1 second
    m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

    m_currentLimits.StatorCurrentLimit = 35; // Limit stator to 20 amps
    m_currentLimits.StatorCurrentLimitEnable = true; // And enable it

    
    toConfigure.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toConfigure.CurrentLimits = m_currentLimits;

    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitThreshold(80);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitThreshold(-300);
    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    toConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      

    

    //toConfigure.MotorOutput.withMotorOutput(NeutralMode.Brake);
    

    ClimberLeftMotor.getConfigurator().apply(toConfigure);


    toConfigure.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toConfigure.CurrentLimits = m_currentLimits;

    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitThreshold(80);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitThreshold(-300);
    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
 toConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    ClimberRightMotor.getConfigurator().apply(toConfigure);

}

}
