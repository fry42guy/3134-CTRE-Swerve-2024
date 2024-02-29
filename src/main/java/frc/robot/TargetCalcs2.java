// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

/** Add your docs here. */
public class TargetCalcs2 {
public AprilTagFieldLayout Layout;

public AprilTag targeTag;


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

public Rotation2d RotationToTag(int TagId, Pose2d RobotPose2D){

  //GetApriltagePose(TagID)





    return new Rotation2d();
}

public void periodic(){}



}
