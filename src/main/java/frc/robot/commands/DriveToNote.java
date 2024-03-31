// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;



public class DriveToNote extends Command {

  private final IntakeSubsystem m_IntakeSubsystem;
  private final CommandSwerveDrivetrain m_commandSwerveDrivetrain;
  private double Timeout;
private Pose2d RoboPose;
  private Timer Runtime;
  private double FWD_mPerSec;
 private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

//private Rotation2d TargetRotation;

    // private final SwerveRequest.FieldCentricFacingAngle driveFaceinangle = new SwerveRequest.FieldCentricFacingAngle()
    //   .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //   .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

      private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    
  /** Creates a new DriveToNote. */
  public DriveToNote(CommandSwerveDrivetrain m_commandSwerveDrivetrain,IntakeSubsystem m_IntakeSubsystem, double Timeout,double FWD_mPerSec) {
    // Use addRequirements() here to declare subsystem dependencies.
this.Timeout = Timeout;
this.FWD_mPerSec = FWD_mPerSec;
Runtime = new Timer();
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.m_commandSwerveDrivetrain = m_commandSwerveDrivetrain;
    
    addRequirements(m_IntakeSubsystem);
    addRequirements(m_commandSwerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Runtime.reset();
    Runtime.start();
    m_IntakeSubsystem.setspeed(1.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

double tx = LimelightHelpers.getTX("limelight-note");

boolean note = LimelightHelpers.getTV("limelight-note");




SmartDashboard.putBoolean("Note Seen", note);

SmartDashboard.putNumber("tx of note", tx);


RoboPose = m_commandSwerveDrivetrain.getrobotpose();


//TargetRotation = RoboPose.getRotation().minus(Rotation2d.fromDegrees(tx));



if (note){



m_commandSwerveDrivetrain.setControl( robotCentric
.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
.withVelocityX( FWD_mPerSec)
.withDeadband(0)
.withVelocityY(0)


.withRotationalRate(tx*-.08));

 
}
else {

  m_commandSwerveDrivetrain.applyRequest(() -> robotCentric.withVelocityX(0)


.withVelocityY(0)

.withRotationalRate(0));

}

//.withTargetDirection(TargetRotation));



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

m_IntakeSubsystem.Stop();

m_commandSwerveDrivetrain.setControl(robotCentric
.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
.withVelocityX(0)
.withVelocityY(0)
.withRotationalRate(0));

    
    Runtime.stop();



  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Runtime.get() > Timeout) {
      return true;
    }

    if (m_IntakeSubsystem.Note_In_Intake()){
      m_IntakeSubsystem.Stop();
      return true;
    }

    return false;
  }
}
