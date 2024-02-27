// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.InvertType;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
//import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  public final TalonFX PivotMotor;
  
  private final DutyCycleEncoder HexEncoder;

  //private final Duty Hexencoder2;

  private final double MaxCurrent;

 private boolean encoderupdated = false;
  
private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    PivotMotor = new TalonFX(Constants.Arm.LeftPivotID);
    PivotMotor.setInverted(false);
    
   PivotMotor.setNeutralMode(NeutralModeValue.Brake);///////////////////////////////////////////
   
    HexEncoder = new DutyCycleEncoder(Constants.Arm.EncoderPWMID);
    MaxCurrent = Constants.Arm.MAX_CURRENT_DRAW;

    

    ConfigPivotCurrent();

     

  }

  public void ConfigPivotCurrent(){

TalonFXConfiguration toConfigure = new TalonFXConfiguration();

    if (MaxCurrent >= 5){
      //PivotMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,Constants.Arm.MAX_CURRENT_DRAW,Constants.Arm.MAX_CURRENT_DRAW + 5, 0.5));

    m_currentLimits.SupplyCurrentLimit = Constants.Arm.MAX_CURRENT_DRAW ; // Limit to 1 amps
    m_currentLimits.SupplyCurrentThreshold = Constants.Arm.MAX_CURRENT_DRAW + 5; // If we exceed 4 amps
    m_currentLimits.SupplyTimeThreshold = .5; // For at least 1 second
    m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

    m_currentLimits.StatorCurrentLimit = 25; // Limit stator to 20 amps
    m_currentLimits.StatorCurrentLimitEnable = true; // And enable it

    

    toConfigure.CurrentLimits = m_currentLimits;

    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitThreshold(245.0);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitThreshold(-2);
    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
//toConfigure.MotorOutput.withMotorOutput(NeutralMode.Brake);
    

    PivotMotor.getConfigurator().apply(toConfigure);
  }

      
      else{
       // PivotMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false,40,10,1));
      

        m_currentLimits.StatorCurrentLimitEnable = true;
         m_currentLimits.SupplyCurrentLimitEnable = true;
         toConfigure.CurrentLimits = m_currentLimits;
         PivotMotor.getConfigurator().apply(toConfigure);
      }

      

  }

  public void setspeed(Double speed){
PivotMotor.set( speed);


  }

  public void stop(){
    PivotMotor.set( 0);
  
    
      }

      public double getPivotEncoder(){

       return PivotMotor.getPosition().getValueAsDouble();

      } 




    public double Calibrate_Arm_Encoder_To_Hex_Encoder(){

      double inputangle = HexEncoder.getAbsolutePosition();
if (inputangle < .25 && encoderupdated == false) {


  
  Timer.delay(3);
  inputangle = HexEncoder.getAbsolutePosition();

  encoderupdated = true; 

}
      double CalculatedEnocderPosition = (1287.7292741*(Math.pow(inputangle,2))+(-2760.633858*inputangle)+(1338.221256));

      return CalculatedEnocderPosition;

//PivotMotor.setPosition(CalculatedEnocderPosition);

    }


    public void setpivotpoistion(){
      PivotMotor.setPosition(Calibrate_Arm_Encoder_To_Hex_Encoder());
    
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Hex Encoder Value", HexEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Pivot Motor Encoder", getPivotEncoder());
    SmartDashboard.putNumber("Pivot Motor Current",PivotMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Calc Encoder Position", Calibrate_Arm_Encoder_To_Hex_Encoder());

    // This method will be called once per scheduler run
  }
}
