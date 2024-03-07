// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class Rumble_Time extends Command {

private XboxController xbox = new XboxController(0);

private Timer delaytimer;

private double RuntimeSECONDS;


  /** Creates a new Rumble_Time. */
  public Rumble_Time(Double RuntimeSECONDS) {

    this.RuntimeSECONDS =  RuntimeSECONDS;

    delaytimer = new Timer();

      // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    delaytimer.stop();
    delaytimer.reset();
    delaytimer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    xbox.setRumble(RumbleType.kBothRumble, 1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
xbox.setRumble(RumbleType.kBothRumble, 0);
delaytimer.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (delaytimer.get() > RuntimeSECONDS){

      return true;
    }


    return false;
  }
}
