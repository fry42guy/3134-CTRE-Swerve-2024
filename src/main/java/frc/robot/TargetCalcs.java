// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimelightHelpers;

/** Add your docs here. */
public class TargetCalcs {





public AprilTagFieldLayout Layout;

public AprilTag targeTag;

public LimelightHelpers limelight;

public TargetCalcs(){

  try {
      Layout = new AprilTagFieldLayout(Constants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      Layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
    }


} 
/*
*Get tredgetory to target basedon robot postion
*/
public Rotation2d GetRotaiontoAprilTag(int TagID ){

   

Optional<Pose3d> oAprilTagpos3d = Layout.getTagPose(TagID);

//Pose2d AprilTagpos2d = convert3DTo2D(AprilTagpos3d);

Pose2d AprilTagpos2d = new Pose2d();

if (oAprilTagpos3d.isPresent()){

 AprilTagpos2d = oAprilTagpos3d.get().toPose2d();
    
}

//Pose2d relativePose = calculateRelativePose(getLimeLightBotPose(), AprilTagpos2d);

double dx = AprilTagpos2d.getTranslation().getX() - getLimeLightBotPose().getTranslation().getX();
double dy = AprilTagpos2d.getTranslation().getY() - getLimeLightBotPose().getTranslation().getY();

Rotation2d HeadtingtoTarget = new Rotation2d(Math.atan2(dy,dx) - getLimeLightBotPose().getRotation().getRadians());

SmartDashboard.putNumber("Align Heading", HeadtingtoTarget.getDegrees());
return HeadtingtoTarget;

}



public double GetDistanceToTag(int TagID){



Optional<Pose3d> AprilTagpos3d = Layout.getTagPose(TagID);

double distance = 0;

if (AprilTagpos3d.isPresent()){

    distance = getLimeLightBotPose().getTranslation().getDistance(AprilTagpos3d.get().toPose2d().getTranslation());
    
}

SmartDashboard.putNumber("limelightxpos",getLimeLightBotPose().getX());
SmartDashboard.putNumber("limelightypos",getLimeLightBotPose().getY());
SmartDashboard.putNumber("Tag7xpos",AprilTagpos3d.get().getX());
SmartDashboard.putNumber("Tag7ypos",AprilTagpos3d.get().getY());
//Pose2d AprilTagpos2d = convert3DTo2D(AprilTagpos3d);

//Pose2d relativePose = calculateRelativePose(getLimeLightBotPose(), AprilTagpos2d);


//double distance = Math.sqrt(Math.pow(relativePose.getX(),2) + Math.pow(relativePose.getY(), 2));

return distance;
}









public static Pose2d convert3DTo2D(Pose3d pose3d) {
    // Extract x, y, and rotation from the 3D pose
    double x = pose3d.getTranslation().getX();
    double y = pose3d.getTranslation().getY();
    Rotation2d rotation = pose3d.getRotation().toRotation2d();

    // Create a 2D pose
    return new Pose2d(x, y, rotation);
}

public Pose2d calculateRelativePose(Pose2d robotPose, Pose2d aprilTagPose) {
    // Calculate the difference in positions
    Translation2d relativePosition = aprilTagPose.getTranslation().minus(robotPose.getTranslation());

    // Calculate the difference in orientations
    Rotation2d relativeRotation = robotPose.getRotation().minus(aprilTagPose.getRotation());

SmartDashboard.putNumber("Apriltag Rotation" , aprilTagPose.getRotation().getDegrees());

SmartDashboard.putNumber("Robot Rotation" , robotPose.getRotation().getDegrees());

    // Return the relative pose
    return new Pose2d(relativePosition, relativeRotation);
}



public Pose2d getLimeLightBotPose(){



    return LimelightHelpers.getBotPose2d_wpiBlue("limelight");

}

public double CalcArmEncoderViaTarget(int Tagid){


//double armEncoder = ((GetDistanceToTag(Tagid) * 21.59922323) + -14.59293246);

double armEncoder = ((56.511 * Math.log(GetDistanceToTag(Tagid))) + 4.947);

SmartDashboard.putNumber("Arm Encoder setpoint", armEncoder);


    return armEncoder;
}


public double PIDDriveRotateSpeed(int TagID){

  Double Raw =  GetRotaiontoAprilTag(TagID).getDegrees() * -.05;


if (Raw > .5 ){

    Raw = .5;
}

if (Raw < -.5){

    Raw = -.5;
}


if (Math.abs(  GetRotaiontoAprilTag(TagID).getDegrees()) < 2){
    Raw = 0.0;
}

Double rotatespeed  = Raw;
  return rotatespeed;

}



public Double Calcperiodic() {
    // TODO Auto-generated method stub



    return GetRotaiontoAprilTag(7).getDegrees();
}



}
