// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoPIDShooterCommand extends Command {
  /** Creates a new PIDShooterCommand. */
  private PIDController m_LeftShooterPIDController;
  private final ShooterSubsystem m_ShooterSubsystem;
  private double setPoint;

  private boolean timerruning; 

  private Timer shoottime;

  private IntakeSubsystem m_IntakeSubsystem;
  
  private VelocityVoltage m_LeftVelocityVoltage;
private VelocityVoltage m_RightVelocityVoltage;


private boolean withstop;


  public AutoPIDShooterCommand(ShooterSubsystem m_ShooterSubsystem,IntakeSubsystem m_IntakeSubsystem, double rpmspeed,boolean YeswithStop) {
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.withstop = YeswithStop;
    m_LeftShooterPIDController = new PIDController(.00004, 0., 0.0);
   // m_ShooterPIDController.enableContinuousInput(-1, 1);
    m_LeftShooterPIDController.setTolerance(0.0035);
    this.setPoint = rpmspeed;
    m_LeftVelocityVoltage = new VelocityVoltage(0);
     m_RightVelocityVoltage = new VelocityVoltage(0);

     timerruning =false;
   
shoottime = new Timer();

    addRequirements(m_ShooterSubsystem,m_IntakeSubsystem);

    

    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

shoottime.reset();

    m_ShooterSubsystem.PIDSetup();

    m_ShooterSubsystem.PIDUpdate();

    m_ShooterSubsystem.isActive = true;
  
configLeftTalon();
configRightTalon();

    m_LeftShooterPIDController.setP(m_ShooterSubsystem.KP);
    m_LeftShooterPIDController.setI(m_ShooterSubsystem.KI);
    m_LeftShooterPIDController.setD(m_ShooterSubsystem.KD);

   
    //System.out.println("PID init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // double feedforward = 0.2;
    // double speed = m_LeftShooterPIDController.calculate(m_ShooterSubsystem.GetLeftShooterRPM(), m_ShooterSubsystem.PIDTESTspeed);
    // speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    // m_ShooterSubsystem.setDiffSpeed(speed,0);

 m_ShooterSubsystem.LeftShooter.setControl(m_LeftVelocityVoltage.withVelocity(setPoint/60));
 m_ShooterSubsystem.RightShooter.setControl(m_RightVelocityVoltage.withVelocity(setPoint/60));

    SmartDashboard.putNumber("LeftShooter output: ", m_ShooterSubsystem.LeftShooter.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("RightShooter output: ", m_ShooterSubsystem.RightShooter.getMotorVoltage().getValueAsDouble());

    if (setPoint * 1.05 > m_ShooterSubsystem.GetLeftShooterRPM() && m_ShooterSubsystem.GetLeftShooterRPM() > setPoint *0.95){


m_IntakeSubsystem.setspeed(Constants.Intake.FWDSpeed);

System.out.println(shoottime.get());

if (timerruning == false){

  shoottime.start();
  timerruning = true;
  
}


    }
    //System.out.println(m_ShooterSubsystem.GetLeftShooterRPM());
    //System.out.println(LeftsetPoint);
    //System.out.println(speed);
   //System.out.println(KP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    if (withstop){
    m_ShooterSubsystem.setspeed(0.0);////////////////////////
    }
    m_ShooterSubsystem.isActive = false;
    m_IntakeSubsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
if (shoottime.get() > Constants.AutoConstants.shoottime)
{
  return true;
}

    return false;

  }

  


public void configLeftTalon(){

TalonFXConfiguration Leftconfig = new TalonFXConfiguration();
//Leftconfig.Slot0.kV = 0.12;
Leftconfig.Slot0.kP = m_ShooterSubsystem.KP;
Leftconfig.Slot0.kI = m_ShooterSubsystem.KI;
Leftconfig.Slot0.kD = m_ShooterSubsystem.KD;



Leftconfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
Leftconfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

m_ShooterSubsystem.LeftShooter.getConfigurator().apply(Leftconfig,.05);

}


public void configRightTalon(){

TalonFXConfiguration Rightconfig = new TalonFXConfiguration();
//Rightconfig.Slot0.kV = 0.12;
Rightconfig.Slot0.kP = m_ShooterSubsystem.KP;
Rightconfig.Slot0.kI = m_ShooterSubsystem.KI;
Rightconfig.Slot0.kD = m_ShooterSubsystem.KD;


Rightconfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
Rightconfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

m_ShooterSubsystem.RightShooter.getConfigurator().apply(Rightconfig,.05);

}



}
