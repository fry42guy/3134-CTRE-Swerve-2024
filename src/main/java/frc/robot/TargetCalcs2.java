// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

/** Add your docs here. */
public class TargetCalcs2 {
public AprilTagFieldLayout Layout;

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

public Rotation2d AbsRotationToTag(int TagId, Pose2d RobotPose2D){

  Pose2d TagPose = GetApriltagePose(TagId);

Pose2d RoboPose = RobotPose2D;


Transform2d Transfrom = new Transform2d(RoboPose,TagPose);

Transfrom.getRotation();




    return Transfrom.getRotation();
}

public void periodic(){}

public void SetSpeakerTargetID(){

 Optional<Alliance> ally = DriverStation.getAlliance();
if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {
        
TargetID = 7;

    }
    if (ally.get() == Alliance.Blue) {
        
      TargetID = 4;
    }
}
else {

  TargetID = -5;
    
}

}

}
