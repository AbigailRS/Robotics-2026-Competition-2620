// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class CameraSystem {

    public String cameraName = "";
    public PhotonCamera photonCamera;
    public Translation3d robotToCamTranslation;
    public Rotation3d robotToCamRotation;
    public final AprilTagFieldLayout kTagLayout;
    public final Transform3d kRobotToCam;
    public final PhotonPoseEstimator photonEstimator;
    public Optional<EstimatedRobotPose> poseEstimate = Optional.empty();
    public PhotonPipelineResult result;
    
    CameraSystem(String name, Translation3d robotToCamTranslation, Rotation3d robotToCamRotation){
        this.cameraName = name;
        this.robotToCamTranslation = robotToCamTranslation;
        this.robotToCamRotation = robotToCamRotation;

        kRobotToCam = new Transform3d(new Translation3d(), new Rotation3d());
        kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

        photonCamera = new PhotonCamera(name);
    }

    Pose2d getRobotPose(){
        for(var result: photonCamera.getAllUnreadResults()){
            if(photonCamera.isConnected()){
                result = photonCamera.getLatestResult();
                poseEstimate = photonEstimator.estimateCoprocMultiTagPose(result);
                if(poseEstimate.isEmpty()){
                    poseEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
                }
                
            }

        }
    }
    
}
