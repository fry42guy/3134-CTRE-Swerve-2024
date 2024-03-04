// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

/** Add your docs here. */
public class TargetCalcs2 {
public AprilTagFieldLayout Layout;

public LimelightHelpers m_visionSystem;

public AprilTag targeTag;

public Integer TargetID;


public TargetCalcs2(){

  try {
      Layout = new AprilTagFieldLayout(Constants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      Layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
    }


} 

public Pose2d GetApriltagePose(int TagID){


Optional<Pose3d> oAprilTagpos3d = Layout.getTagPose(TagID);

Pose2d AprilTagpos2d = new Pose2d();

if (oAprilTagpos3d.isPresent()){

 AprilTagpos2d = oAprilTagpos3d.get().toPose2d();
    
}

return AprilTagpos2d;
}


public double getDistTo_Tag(int TagId, Pose2d RobotPose2D){
 Pose2d TagPose = GetApriltagePose(TagId);

 Pose2d RoboPose = RobotPose2D;

 double distanceinMeters = RoboPose.getTranslation().getDistance(TagPose.getTranslation());

 double distFT = Units.metersToFeet(distanceinMeters);

 SmartDashboard.putNumber("Dist to Traget", distFT);

return distFT;

}



public Rotation2d AbsRotationToTag(int TagId, Pose2d RobotPose2D){

  Pose2d TagPose = GetApriltagePose(TagId);

Pose2d RoboPose = RobotPose2D;

Translation2d targeTranslation2d = TagPose.getTranslation();

Translation2d relativeTranslation = targeTranslation2d.minus(RoboPose.getTranslation());


Rotation2d rotationtotarget = new Rotation2d(relativeTranslation.getX(),relativeTranslation.getY());


//Transform2d Transfrom = new Transform2d(RoboPose,TagPose);

//SmartDashboard.putNumber("Rotation angle", Transfrom.getRotation().getDegrees());

SmartDashboard.putString("Tag Pose",TagPose.toString());

//Transfrom.getRotation();




    //return Transfrom.getRotation();
    return rotationtotarget;
}

public void periodic(){}

public void SetSpeakerTargetID(){

 Optional<Alliance> ally = DriverStation.getAlliance();
if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {
        
TargetID = 4;

    }
    if (ally.get() == Alliance.Blue) {
        
      TargetID = 7;
    }
}
else {

  TargetID = -5;
    
}

}






public void LimelightUpdatePose(){

  PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
  
  SmartDashboard.putNumber("Limelight Feild Pos",limelightMeasurement.pose.getX());

  //SmartDashboard.putString("BluePOS", LimelightHelpers.getBotPose2d_wpiBlue("limelight").);
  
   SmartDashboard.putString("Tag Count", limelightMeasurement.toString());
  if(limelightMeasurement.tagCount >= 1)
  {
    // this.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    // this.addVisionMeasurement(
    //     limelightMeasurement.pose,
    //     limelightMeasurement.timestampSeconds);

        
    
  }
}



}
