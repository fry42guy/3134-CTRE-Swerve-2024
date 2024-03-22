// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmPivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.TargetCalcs2;
import frc.robot.subsystems.ArmSubsystem;

public class AutoTargetPIDPivotCommand extends Command {
  /** Creates a new PIDPivotCommand. */
 
  private final ArmSubsystem m_ArmSubsystem;
  private double setPoint;
  private PositionVoltage m_armPositionVoltage;
  
private TargetCalcs2 m_Calcs2;

private boolean withEnd;

  public AutoTargetPIDPivotCommand(ArmSubsystem m_ArmSubsystem, Boolean withEnd) {
    this.m_ArmSubsystem = m_ArmSubsystem;
  this.withEnd = withEnd;
   // m_PivotPIDController.enableContinuousInput(-1, 1);
    m_armPositionVoltage = new PositionVoltage(0);
    
m_Calcs2 = RobotContainer.m_Calcs2;
   
    addRequirements(m_ArmSubsystem);

    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   

configTalon();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 

 {

  setPoint = m_Calcs2.absbyPostSetpointSpeaker();


  SmartDashboard.putNumber("Arm Setpoint Auto", setPoint);


  m_ArmSubsystem.PivotMotor.setControl(m_armPositionVoltage.withPosition(setPoint)
  .withLimitForwardMotion(!m_ArmSubsystem.m_FwdLimit.get())
  .withLimitReverseMotion(!m_ArmSubsystem.m_RevLimit.get())
  );


  SmartDashboard.putNumber("Pivot Target", setPoint);
  SmartDashboard.putNumber("Pivot Actual", m_ArmSubsystem.PivotMotor.getPosition().getValueAsDouble());
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
    m_ArmSubsystem.setspeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    

if (withEnd){

    if (setPoint+.05 > m_ArmSubsystem.PivotMotor.getPosition().getValueAsDouble() && m_ArmSubsystem.PivotMotor.getPosition().getValueAsDouble() > setPoint -.05){

     // System.out.println(" AutotargetPIDPivotCommand at setpoint");
      return true;
    }}
         
   // System.out.println(" AutotargetPIDPivotCommand NOT AT setpoint");
    return false;

  }

  public void setPoint(double setPoint)
  {
    this.setPoint = setPoint;
  }


public void configTalon(){

TalonFXConfiguration ArmCofig = new TalonFXConfiguration();
//Rightconfig.Slot0.kV = 0.12;
ArmCofig.Slot0.kP = 3;
ArmCofig.Slot0.kI = 20;
ArmCofig.Slot0.kD = 0;

ArmCofig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


ArmCofig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
ArmCofig.TorqueCurrent.PeakReverseTorqueCurrent = -40;



m_ArmSubsystem.PivotMotor.getConfigurator().apply(ArmCofig,.05);

}



}
