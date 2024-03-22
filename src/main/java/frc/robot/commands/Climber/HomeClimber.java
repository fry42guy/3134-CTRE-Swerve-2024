// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class HomeClimber extends Command {

  private Timer HomeTimer;
  private boolean Timerrunning;

  

  private final ClimberSubsystem m_ClimberSubsystem;
  /** Creates a new HomeClimber. */
  public HomeClimber(ClimberSubsystem m_ClimberSubsystem) {

    this.m_ClimberSubsystem = m_ClimberSubsystem;
    addRequirements(m_ClimberSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Climberinit();
   Timerrunning = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

m_ClimberSubsystem.setspeed(-.25);

if (!Timerrunning & Math.abs(m_ClimberSubsystem.ClimberLeftMotor.getVelocity().getValueAsDouble())<.05 & Math.abs(m_ClimberSubsystem.ClimberRightMotor.getVelocity().getValueAsDouble())<.05){

  HomeTimer.reset();
  HomeTimer.start();
  Timerrunning = true;

}





  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_ClimberSubsystem.Stop();
    Zeromotors();


    Timerrunning = false;
    HomeTimer.reset();
    HomeTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

if (HomeTimer.get()>1 & Timerrunning){

  return true;
}


    return false;
  }

public void Climberinit(){

  TalonFXConfiguration Motorconfig = new TalonFXConfiguration();
//Leftconfig.Slot0.kV = 0.12;

CurrentLimitsConfigs currentconfig = Motorconfig.CurrentLimits;

currentconfig.StatorCurrentLimit = 2;
currentconfig.StatorCurrentLimitEnable = true;

//Leftconfig.TorqueCurrent.PeakForwardTorqueCurrent = 1;
//Leftconfig.TorqueCurrent.PeakReverseTorqueCurrent = 1;

Motorconfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
Motorconfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

m_ClimberSubsystem.ClimberLeftMotor.getConfigurator().apply(Motorconfig);
m_ClimberSubsystem.ClimberRightMotor.getConfigurator().apply(Motorconfig);

}

public void MotorSettings(){
  TalonFXConfiguration Motorconfig = new TalonFXConfiguration();
CurrentLimitsConfigs currentconfig = Motorconfig.CurrentLimits;

currentconfig.StatorCurrentLimit = 40;
currentconfig.StatorCurrentLimitEnable = false;

Motorconfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =500;
Motorconfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -500;
Motorconfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
Motorconfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;


}

public void Zeromotors(){

m_ClimberSubsystem.ClimberLeftMotor.setPosition(0);
m_ClimberSubsystem.ClimberRightMotor.setPosition(0);



}



}
