package frc.robot;

import java.io.File;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;

public class Constants 
{
    public static final double stickDeadband = 0.1;
    public static final int blueTarget = 4;
    public static final int redTarget = 7;
  

  
    public static final class AutoConstants {

        public static double shoottime = 1.0; // Time intake runs to shoot

    }

    public static class Climber{

        public static final int ClimberID = 50;
        public static final Double UpSpeed = .5;
        public static final Double DownSpeed = -.5;
      }
    
    
    public static class Arm{
    
      public static final int LeftPivotID = 49;
      public static final int RightPivotID = 47;
      public static final int EncoderPWMID = 9;
      public static final double ArmUpSpeed = .75;
      public static final double ArmDownSpeed = -.75;
    
      public static final double MAX_CURRENT_DRAW = 25;
    
    public static final double Shooting_SetPoint1 = 0 ;
    public static final double Shooting_SetPoint2 = 0 ;
    public static final double FloorPickup_SetPoint = 0 ;
    
    // public static final double RawAngleHex_To_EncoderConversion_Rate = 0 ;
    
    
    }
    
    
      public static class Intake{
    
        public static final int IntakeMotorID = 44;
     
      public static final Double FWDSpeed = 1.0; 
      public static final Double REVSpeed = -1.0;
    
      }
    
    public static class Shooter{
    
      public static final int LeftShooterID = 40;
      public static final int RightShooterID = 46;
    
      public static final Double SlowSpeed = .375;
      public static final Double FastSpeed = .75;
      public static final Double SameSpeed = .75; //Not Used
      public static final Double TopSpeed = 0.6; //Not used
      public static final Double BottomSpeed = 0.4;//Not used
    }
    
 
    public static final class Aiming 
    {
        public static final double Home = -1.25;
        public static final double Amp = 13.3;
        public static final double Source = 16.7;
        public static final double Position = 4.5;
        public static final double Farback = 7.25;
    }
  public static final Path APRILTAG_FIELD_LAYOUT_PATH =
      new File(Filesystem.getDeployDirectory(), "2024-crescendo.json").toPath();
      
      public static class OperatorConstants
      {public static final int kDriverControllerPort = 0;
    
        // Joystick Deadband
        public static final double LEFT_X_DEADBAND  = 0.1;
        public static final double LEFT_Y_DEADBAND  = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT    = 6;
      }
}
