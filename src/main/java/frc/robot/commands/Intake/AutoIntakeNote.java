// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeNote extends Command {
  private final IntakeSubsystem m_IntakeSubsystem;

  private Timer revtime;
  private Timer intaketime;

  private double ReversTime;
  private double IntakeTimeout;

  private boolean revphase;


  /** Creates a new IntakeFWD. */
  public AutoIntakeNote(IntakeSubsystem m_IntakeSubsystem, double IntakeTimeout, double ReversTime) {

    this.m_IntakeSubsystem = m_IntakeSubsystem;

revtime = new Timer();
intaketime = new Timer();
this.ReversTime = ReversTime;
this.IntakeTimeout = IntakeTimeout;
revphase = false;
    addRequirements(m_IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    revphase = false;
    revtime.reset();
    intaketime.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   


    if (intaketime.get() < IntakeTimeout ){

      if (m_IntakeSubsystem.Note_In_Intake()){
        // if (m_ShooterSubsystem.getisActive() == false){
m_IntakeSubsystem.Stop();


if (revphase == false){
       revtime.restart();
        revphase = true;
      }

m_IntakeSubsystem.setspeed(Constants.Intake.REVSpeed);





}
else {

m_IntakeSubsystem.setspeed(Constants.Intake.FWDSpeed);
    }
  }
    else {

      if (revphase == false){
       revtime.restart();
        revphase = true;
      }

m_IntakeSubsystem.setspeed(Constants.Intake.REVSpeed);




    }

    System.out.println(revtime.get());
 System.out.println(intaketime.get());
System.out.println(revphase);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_IntakeSubsystem.Stop();
revphase = false;

    revtime.reset();
    intaketime.reset();
    intaketime.stop();
    revtime.stop();


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (revtime.get() > ReversTime){

      return true;
    }




    return false;
  }
}
