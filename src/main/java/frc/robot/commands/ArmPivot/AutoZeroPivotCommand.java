// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmPivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class AutoZeroPivotCommand extends Command {
  /** Creates a new PIDPivotCommand. */
 
  private final ArmSubsystem m_ArmSubsystem;
  private double setPoint;
  private PositionVoltage m_armPositionVoltage;
  


  public AutoZeroPivotCommand(ArmSubsystem m_ArmSubsystem) {
    this.m_ArmSubsystem = m_ArmSubsystem;
  
   // m_PivotPIDController.enableContinuousInput(-1, 1);
    m_armPositionVoltage = new PositionVoltage(0);
   // .withLimitForwardMotion(!m_ArmSubsystem.m_FwdLimit.get())
   // .withLimitReverseMotion(!m_ArmSubsystem.m_RevLimit.get());
    

   
    addRequirements(m_ArmSubsystem);

    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("AutoZeroPivot Started");

configTalon();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 

 {

  setPoint = 0.0;


  SmartDashboard.putNumber("Arm Setpoint Auto", setPoint);


  m_ArmSubsystem.PivotMotor.setControl(m_armPositionVoltage.withPosition(setPoint));
    // double feedforward = 0.00;
    // double speed = m_PivotPIDController.calculate(m_ArmSubsystem.getPivotEncoder(), setPoint);
    // speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    // m_ArmSubsystem.setspeed(speed);
    // SmartDashboard.putNumber("Pivot output: ", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    System.out.println("AutoZeroPivot Ended");
    //m_ArmSubsystem.setspeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (setPoint+.5 > m_ArmSubsystem.PivotMotor.getPosition().getValueAsDouble() && m_ArmSubsystem.PivotMotor.getPosition().getValueAsDouble() > setPoint -.5){

      return true;
    }
          
    return false;

  }

  public void setPoint(double setPoint)
  {
    this.setPoint = setPoint;
  }


public void configTalon(){

TalonFXConfiguration ArmCofig = new TalonFXConfiguration();
//Rightconfig.Slot0.kV = 0.12;
ArmCofig.Slot0.kP = 2;
ArmCofig.Slot0.kI = 1;
ArmCofig.Slot0.kD = 0;

ArmCofig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


ArmCofig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
ArmCofig.TorqueCurrent.PeakReverseTorqueCurrent = -40;



m_ArmSubsystem.PivotMotor.getConfigurator().apply(ArmCofig,.05);

}



}
