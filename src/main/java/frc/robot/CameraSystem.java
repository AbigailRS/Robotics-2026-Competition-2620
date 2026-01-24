// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

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
    public Pose2d currentPose2d;

    StructPublisher<Pose2d> cameraPosePublisher;
    
    CameraSystem(String name, Translation3d robotToCamTranslation, Rotation3d robotToCamRotation){
        this.cameraName = name;
        this.robotToCamTranslation = robotToCamTranslation;
        this.robotToCamRotation = robotToCamRotation;

        kRobotToCam = new Transform3d(robotToCamTranslation, robotToCamRotation);
        kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
        currentPose2d = new Pose2d();

        photonCamera = new PhotonCamera(name);
        cameraPosePublisher = NetworkTableInstance.getDefault().getStructTopic(cameraName + " Pose Log", Pose2d.struct).publish();

    }

    public Pose2d getRobotPose(){
        for(var result: photonCamera.getAllUnreadResults()){
            if(photonCamera.isConnected()){
                result = photonCamera.getLatestResult();
                poseEstimate = photonEstimator.estimateCoprocMultiTagPose(result);
                if(poseEstimate.isEmpty()){
                    poseEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
                }
                    currentPose2d = poseEstimate.orElse(null).estimatedPose.toPose2d();
                
            }
        }

        cameraPosePublisher.set(currentPose2d);
        return currentPose2d;
    }

}