// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


/** Add your docs here. */
public class PoseEstimator {

// public PoseEstimator m_PoseEstimator = new PoseEstimator(null);

 private final Field2d field2d = new Field2d();
 private Pose2d poseTracker;



 public PoseEstimator(Pose2d poseTracker){

        this.poseTracker = poseTracker;




 }





  public void resetToPose(Pose2d pose) {
    poseTracker = pose;




}}
