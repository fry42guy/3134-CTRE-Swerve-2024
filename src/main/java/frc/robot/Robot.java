// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

 
  
  //private final Field2d m_feild = new Field2d();
  
 

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
m_robotContainer.drivetrain.init();

RobotContainer.m_Calcs2.SetSpeakerTargetID();
    
   
    //PortForwarder.add(5800, "photonvision.local", 5800); // for photonvision
    
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    m_robotContainer.m_Calcs2.UpdateLocalPose(m_robotContainer.drivetrain.getrobotpose());

    if (m_robotContainer.m_Calcs2.TargetID == -5){

m_robotContainer.m_Calcs2.SetSpeakerTargetID();

m_robotContainer.drivetrain.addVisionMeasurement(null, kDefaultPeriod);


    }
m_robotContainer.m_Calcs2.getDistTo_Tag(m_robotContainer.m_Calcs2.TargetID,m_robotContainer.drivetrain.getrobotpose());

  }

  @Override
  public void disabledInit() {





  }

  @Override
  public void disabledPeriodic() {

    if (!RobotContainer.m_ArmSubsystem.encoderupdated)

    RobotContainer.m_ArmSubsystem.setpivotpoistion();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    m_robotContainer.m_Calcs2.SetSpeakerTargetID();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    RobotContainer.m_ArmSubsystem.setpivotpoistion();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

//m_robotContainer.drivetrain.LimelightUpdatePose(); //////////////////////////////////////////////////////////

  }

  @Override
  public void autonomousExit() {
 

  }

  @Override
  public void teleopInit() {
   m_robotContainer.m_Calcs2.SetSpeakerTargetID();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {


m_robotContainer.drivetrain.LimelightUpdatePose();

m_robotContainer.m_Calcs2.AbsDistToSpeaker(m_robotContainer.drivetrain.getrobotpose());


  }

  @Override
  public void teleopExit() {
   

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
