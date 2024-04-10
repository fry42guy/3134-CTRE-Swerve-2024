package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.Utils;

import frc.robot.commands.DriveToNote;
import frc.robot.commands.Rumble_Time;
import frc.robot.commands.ArmPivot.ArmDown;
import frc.robot.commands.ArmPivot.ArmUP;
import frc.robot.commands.ArmPivot.AutoTargetPIDPivotCommand;
import frc.robot.commands.ArmPivot.AutoZeroPivotCommand;
import frc.robot.commands.ArmPivot.PIDPivotCommand;
import frc.robot.commands.ArmPivot.TargetPIDPivotCommand;
import frc.robot.commands.Climber.ClimberFWD;
import frc.robot.commands.Climber.ClimberREV;
import frc.robot.commands.Intake.AutoIntakeNote;
import frc.robot.commands.Intake.IntakeFWD;
import frc.robot.commands.Intake.IntakeFWDWithSensor;
import frc.robot.commands.Intake.IntakeREV;
import frc.robot.commands.Shooter.AutoPIDShooterCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;
import frc.robot.commands.Shooter.PIDsetRPMShooterCommand;
import frc.robot.commands.Shooter.ShootSpeedSame;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;


public class RobotContainer 
{
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
 // private final CommandXboxController m_driverController = new CommandXboxController(0); // My joystick







  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop


  private final SwerveRequest.FieldCentricFacingAngle driveFaceinangle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

      private final PhoenixPIDController turnPID = new PhoenixPIDController(10, 0, .1); //3.2 (10, 1, 0.0);


  
  // driver buttons
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);



  /* Subsystems */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
 
  public final CommandXboxController m_driverController =  new CommandXboxController(0);//Constants.OperatorConstants.kDriverControllerPort);
  public final CommandXboxController m_OperatorController =  new CommandXboxController(1);



  private final IntakeSubsystem   m_IntakeSubsystem   = new IntakeSubsystem();
  public  static ArmSubsystem      m_ArmSubsystem      = new ArmSubsystem();
  private final ShooterSubsystem  m_ShooterSubsystem  = new ShooterSubsystem();
  //private final LimelightHelpers  m_Limelight         = new LimelightHelpers();
  public static ClimberSubsystem  m_ClimberSubsystem  = new ClimberSubsystem();
  public static TargetCalcs       m_Calcs             = new TargetCalcs();

  public static TargetCalcs2       m_Calcs2             = new TargetCalcs2();

  /* Auto List */
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private void configureBindings() 
  {
  
m_ArmSubsystem.setDefaultCommand(new PIDPivotCommand(m_ArmSubsystem, 0, true));

  m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.runOnce(()-> m_ShooterSubsystem.stop()));
    drivetrain.setDefaultCommand
    (
      drivetrain.applyRequest(() -> drive.withVelocityX(-Math.pow(m_driverController.getLeftY(),3) * MaxSpeed)
      .withVelocityY(-Math.pow(m_driverController.getLeftX(),3) * MaxSpeed) // Drive left with negative X (left)
      .withRotationalRate(Math.pow(-m_driverController.getRightX(),3) * MaxAngularRate) // Drive counterclockwise with negative X (left)
      ).ignoringDisable(true));

driveFaceinangle.HeadingController = turnPID;
driveFaceinangle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

m_driverController.rightStick().toggleOnTrue(new ParallelCommandGroup( drivetrain.applyRequest(() -> driveFaceinangle.withVelocityX(-Math.pow(m_driverController.getLeftY(),3) * MaxSpeed)
.withVelocityY(-Math.pow(m_driverController.getLeftX(),3) * MaxSpeed)

.withTargetDirection(m_Calcs2.AbsRotationToTag(m_Calcs2.TargetID,drivetrain.getrobotpose()).minus(drivetrain.Getoffsetroation()))),
new AutoTargetPIDPivotCommand(m_ArmSubsystem, false)//,
//new PIDShooterCommand(m_ShooterSubsystem,4000)
).beforeStarting(new PIDsetRPMShooterCommand(m_ShooterSubsystem, 4000))
// .beforeStarting(
// drivetrain.runOnce(()-> drivetrain.ConfigLimelightAutoshoot())
// )
.until(m_driverController.axisGreaterThan(5,.3).or(m_driverController.axisLessThan(5,-.3))));


    //m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    //m_driverController.povUp().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    //m_driverController.povDown().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    m_driverController.leftBumper().whileTrue(new AutoPIDShooterCommand(m_ShooterSubsystem,m_IntakeSubsystem, Constants.Shooter.SlowSpeed,true,0.0));
    m_driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, .05).whileTrue(new AutoPIDShooterCommand(m_ShooterSubsystem,m_IntakeSubsystem, Constants.Shooter.FastSpeed,true,.25));
    m_driverController.y().whileTrue(new ArmDown(m_ArmSubsystem)); 
    m_driverController.leftStick().whileTrue(new ArmUP(m_ArmSubsystem));  

    m_driverController.rightBumper().whileTrue( new SequentialCommandGroup(  new IntakeFWDWithSensor(m_IntakeSubsystem),
    new Rumble_Time(2.0)));




    m_driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .05).whileTrue(new IntakeREV(m_IntakeSubsystem));
    m_driverController.povUp().whileTrue(new ClimberFWD(m_ClimberSubsystem));
    m_driverController.povDown().whileTrue(new ClimberREV(m_ClimberSubsystem));
    m_driverController.x().toggleOnTrue(new TargetPIDPivotCommand(m_ArmSubsystem));
    m_driverController.a().onTrue(new AutoZeroPivotCommand(m_ArmSubsystem));

    m_driverController.b().whileTrue( new SequentialCommandGroup(new ParallelCommandGroup(new AutoZeroPivotCommand(m_ArmSubsystem), new DriveToNote(drivetrain, m_IntakeSubsystem, 2, 1.5)), new Rumble_Time(2.0)));
m_driverController.povLeft().whileTrue(new PIDPivotCommand(m_ArmSubsystem, 16, false));
    //m_driverController.povLeft().onTrue(drivetrain.getAutoPath("Telli Auto").until(m_driverController.axisGreaterThan(5,.3).or(m_driverController.axisLessThan(5,-.3))));
//m_driverController.b().whileTrue(new ParallelCommandGroup(new IntakeFWDWithSensor(m_IntakeSubsystem),drivetrain.applyRequest(()-> forwardStraight.withVelocityX(1).withRotationalRate(LimelightHelpers.getTX("limelight-note")*-.1))).withTimeout(5).until(m_IntakeSubsystem.Note_In_Intake));
   // m_driverController.y().whileTrue(new AutoIntakeNote(m_IntakeSubsystem, 2  , 0.125));
   // m_driverController.b().onTrue(m_ArmSubsystem.runOnce(() -> m_ArmSubsystem.ZeroPivotPositon()));
   // m_driverController.povRight().onTrue(drivetrain.runOnce(()-> drivetrain.Uppdateseededroation()));


   m_OperatorController.y().whileTrue(new ClimberFWD(m_ClimberSubsystem));
   m_OperatorController.x().whileTrue(new ClimberREV(m_ClimberSubsystem));

   m_OperatorController.leftBumper().whileTrue(new AutoPIDShooterCommand(m_ShooterSubsystem, m_IntakeSubsystem,m_Calcs2.getlobspeed() ,true, 0.0));






  //   intakeButton.whileTrue(new setIntake(Constants.Intake.Speed, m_intake));
  //   reverseIntake.whileTrue(new setIntake((-Constants.Intake.Speed + 0.25), m_intake));
  //   shooterButton.whileTrue(new setShooter(Constants.Shooter.shooterSpeed, m_shooter));
  //   //aimButton.whileTrue(new aimCamera(0, 2, m_vision, drivetrain));
  //   setSourceButton.whileTrue(new setArmTo(Constants.Aiming.Source, m_arm, "Source"));
  //   setAmpButton.whileTrue(new setArmTo(Constants.Aiming.Amp , m_arm, "Amp"));
  //   //setHomeButton.whileTrue(new setArmTo(Constants.Aiming.Home, m_arm, "home"));
  //   setShootButton.whileTrue(new setArmTo(Constants.Aiming.Position, m_arm, "Position"));
  //   halfpowerShootButton.whileTrue(new setShooter(Constants.Shooter.shooterSpeed / 2, m_shooter));
  //   farbackButton.whileTrue(new setArmTo(Constants.Aiming.Farback, m_arm, "Farback"));
  }

  public RobotContainer() 
  {
    // NamedCommands.registerCommand("intake", new setIntake(Constants.Intake.Speed, m_intake).withTimeout(1));
    // NamedCommands.registerCommand("shoot", new setShooter(Constants.Shooter.shooterSpeed, m_shooter).withTimeout(.75));
    // NamedCommands.registerCommand("reverseintake", new setIntake(-Constants.Intake.Speed * 0.25, m_intake).withTimeout(0.1));
    // NamedCommands.registerCommand("setShoot", new setArmTo(Constants.Aiming.Position, m_arm, "Position").withTimeout(5) );
    // NamedCommands.registerCommand("Home", new setArmTo(Constants.Aiming.Home, m_arm, "home").withTimeout(2));
    // NamedCommands.registerCommand("Farback", new setArmTo(Constants.Aiming.Farback, m_arm, "Farback").withTimeout(5));

    if (Utils.isSimulation()){
NamedCommands.registerCommand("AutoAim&Shoot", new PrintCommand("AutoAim&Shoot"));
NamedCommands.registerCommand("AutoAim&ShootStop",new PrintCommand("AutoAim&ShootStop"));
NamedCommands.registerCommand("Arm_To_Zero",new PrintCommand("Arm_To_Zero"));
 NamedCommands.registerCommand("Run_Note_Intake",new PrintCommand("Run_Note_Intake"));
    }

else {
NamedCommands.registerCommand("AutoAim&Shoot", 
new SequentialCommandGroup(
      (new AutoTargetPIDPivotCommand(m_ArmSubsystem,true)),
      (new AutoPIDShooterCommand(m_ShooterSubsystem,m_IntakeSubsystem,4000,false,0.0))
    ));

NamedCommands.registerCommand("AutoAim&ShootStop", 
  new SequentialCommandGroup(
      (new AutoTargetPIDPivotCommand(m_ArmSubsystem,true)),
      (new AutoPIDShooterCommand(m_ShooterSubsystem,m_IntakeSubsystem,4000,true,0.0))
    ));

NamedCommands.registerCommand("Arm_To_Zero", new AutoZeroPivotCommand(m_ArmSubsystem));
NamedCommands.registerCommand("Run_Note_Intake", new AutoIntakeNote(m_IntakeSubsystem, 3  , .125));

NamedCommands.registerCommand("Just Aim", new AutoTargetPIDPivotCommand(m_ArmSubsystem,true));



NamedCommands.registerCommand("AlignandAim",new ParallelCommandGroup( drivetrain.applyRequest(() -> driveFaceinangle.withVelocityX(0)
.withVelocityY(0)
//.withTargetDirection(m_Calcs2.AbsRotationToTag(m_Calcs2.TargetID,drivetrain.getrobotpose()).minus(drivetrain.Getoffsetroation()))),
.withTargetDirection(m_Calcs2.AbsRotationToTag(m_Calcs2.TargetID,drivetrain.getrobotpose()).minus(drivetrain.Getoffsetroation()))),
new AutoTargetPIDPivotCommand(m_ArmSubsystem, false)
.withTimeout(1.5)));

NamedCommands.registerCommand("AlignandAimShoot",new ParallelCommandGroup(drivetrain.applyRequest(() -> driveFaceinangle.withVelocityX(0)
.withVelocityY(0)
//.withTargetDirection(m_Calcs2.AbsRotationToTag(m_Calcs2.TargetID,drivetrain.getrobotpose()).minus(drivetrain.Getoffsetroation()))),
.withTargetDirection(m_Calcs2.AbsRotationToTag(m_Calcs2.TargetID,drivetrain.getrobotpose()).minus(drivetrain.Getoffsetroation()))).withTimeout(1),



new SequentialCommandGroup(
  (new AutoTargetPIDPivotCommand(m_ArmSubsystem,true)),
  (new AutoPIDShooterCommand(m_ShooterSubsystem,m_IntakeSubsystem,4000,false,0.0))
)));


NamedCommands.registerCommand("AlignandAimShootStop",new ParallelCommandGroup(drivetrain.applyRequest(() -> driveFaceinangle.withVelocityX(0)
.withVelocityY(0)
//.withTargetDirection(m_Calcs2.AbsRotationToTag(m_Calcs2.TargetID,drivetrain.getrobotpose()).minus(drivetrain.Getoffsetroation()))),
.withTargetDirection(m_Calcs2.AbsRotationToTag(m_Calcs2.TargetID,drivetrain.getrobotpose()).minus(drivetrain.Getoffsetroation()))).withTimeout(1),



new SequentialCommandGroup(
  (new AutoTargetPIDPivotCommand(m_ArmSubsystem,true)),
  (new AutoPIDShooterCommand(m_ShooterSubsystem,m_IntakeSubsystem,4000,true,0.0))
)));



NamedCommands.registerCommand("DrivetoNote", new DriveToNote(drivetrain, m_IntakeSubsystem, 2, 1.5));
// NamedCommands.registerCommand("DrivetoNote",new ParallelCommandGroup(new IntakeFWDWithSensor(m_IntakeSubsystem),
// drivetrain.applyRequest(()-> forwardStraight.withVelocityX(1.5).withRotationalRate(LimelightHelpers.getTX("limelight-note")*-.08))
// .until(m_IntakeSubsystem.Note_In_Intake))
// .andThen(drivetrain.applyRequest(()-> forwardStraight.withVelocityX(0))).until(m_IntakeSubsystem.Note_In_Intake));

NamedCommands.registerCommand("WarmUpShooter", new PIDsetRPMShooterCommand(m_ShooterSubsystem, 4000));

}


    configureBindings();
    SmartDashboard.putData("Autonomous", m_chooser);
    m_chooser.setDefaultOption("SC Two Note", drivetrain.getAutoPath("SC Two Note"));
    m_chooser.addOption("SC Three Note", drivetrain.getAutoPath("SC Three Note"));
     m_chooser.addOption("Tripple Note", drivetrain.getAutoPath("Tripple Note"));
     m_chooser.addOption("Drive Back", drivetrain.getAutoPath("Test Auto2"));
     m_chooser.addOption("Quad Note", drivetrain.getAutoPath("Test Auto3"));
     m_chooser.addOption("Long Auto one", drivetrain.getAutoPath("Long Auto one"));
     m_chooser.addOption("Sit and Shoot", drivetrain.getAutoPath("Sit and Shoot"));
     //m_chooser.addOption("SC Sorce Long 1", drivetrain.getAutoPath("SC Sorce Long 1"));
     //m_chooser.addOption("SC Sorce Shoot", drivetrain.getAutoPath("SC Sorce Shoot"));
    // m_chooser.addOption("Align and Aim Test", drivetrain.getAutoPath("Align and Aim Test"));
     m_chooser.addOption("Full Auto Three Note",drivetrain.getAutoPath("Full Auto Three Note"));
     m_chooser.addOption("Full Auto Four Note", drivetrain.getAutoPath("Full Auto Four Note"));
     m_chooser.addOption("Full Auto Long Three Note", drivetrain.getAutoPath("Full Auto Long Three Note"));
     m_chooser.addOption("Full Auto Long Two Note", drivetrain.getAutoPath("Full Auto Long Two Note"));
     //m_chooser.addOption("Test Auto4", drivetrain.getAutoPath("Test Auto4"));
     
    // m_chooser.addOption("(Right) Shoot, Drive Back and Intake", drivetrain.getAutoPath("!rsdin"));
    // m_chooser.addOption("3 Note Far, Towards Center", drivetrain.getAutoPath("3 Note Far"));
    // m_chooser.addOption("2 Note Far, Toward Amp", drivetrain.getAutoPath("2 Note Far"));
    // m_chooser.addOption("Test", drivetrain.getAutoPath("test"));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
