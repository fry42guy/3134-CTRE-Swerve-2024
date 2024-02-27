// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeedSame extends Command {
private final ShooterSubsystem m_ShooterSubsystem;
private Double Speed;

  /** Creates a new ShootSpeedSame. */
  public ShootSpeedSame(ShooterSubsystem m_ShooterSubsystem,Double Speed) {
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.Speed = Speed;
    addRequirements(m_ShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

m_ShooterSubsystem.setspeed(Speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_ShooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
