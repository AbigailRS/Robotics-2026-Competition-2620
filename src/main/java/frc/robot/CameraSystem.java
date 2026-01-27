// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    public PhotonTrackedTarget target;
    private Matrix<N3, N1> curStdDevs;

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
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : photonCamera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());

        }

        visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = getEstimationStdDevs();

                currentPose2d = est.estimatedPose.toPose2d();
            });
        cameraPosePublisher.set(currentPose2d);
        return currentPose2d;
    }

    public boolean hasTarget(){
        if(photonCamera.getLatestResult() == null){
            return false;
        }
        return photonCamera.getLatestResult().hasTargets();
    }


    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

}