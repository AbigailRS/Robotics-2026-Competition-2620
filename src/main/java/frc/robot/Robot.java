// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private Timer updatePoseTimer = new Timer();
    StructPublisher<Pose2d> cameraPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Camera Pose Log", Pose2d.struct).publish();
    StructPublisher<Pose2d> LLPosePublisher = NetworkTableInstance.getDefault().getStructTopic("LL Pose Log", Pose2d.struct).publish();
    StructPublisher<Pose2d> combinedPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Combined Pose Log", Pose2d.struct).publish();

    private final RobotContainer m_robotContainer;
    private final Vision cam1, cam2, cam3, cam4;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        updatePoseTimer.start();
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        cam1 = new Vision(m_robotContainer.drivetrain::addVisionMeasurement, Constants.CAMERA_1_NAME, new Transform3d(
            new Translation3d(Constants.CAMERA_1_TRANSLATION_X, Constants.CAMERA_1_TRANSLATION_Y, Constants.CAMERA_1_TRANSLATION_Z), 
            new Rotation3d(Constants.CAMERA_1_ROTATION_ROLL, Constants.CAMERA_1_ROTATION_PITCH, Constants.CAMERA_1_ROTATION_YAW)));
        cam2 = new Vision(m_robotContainer.drivetrain::addVisionMeasurement, Constants.CAMERA_2_NAME, new Transform3d(
            new Translation3d(Constants.CAMERA_2_TRANSLATION_X, Constants.CAMERA_2_TRANSLATION_Y, Constants.CAMERA_2_TRANSLATION_Z), 
            new Rotation3d(Constants.CAMERA_2_ROTATION_ROLL, Constants.CAMERA_2_ROTATION_PITCH, Constants.CAMERA_2_ROTATION_YAW)));
        cam3 = new Vision(m_robotContainer.drivetrain::addVisionMeasurement, Constants.CAMERA_3_NAME, new Transform3d(
            new Translation3d(Constants.CAMERA_3_TRANSLATION_X, Constants.CAMERA_3_TRANSLATION_Y, Constants.CAMERA_3_TRANSLATION_Z), 
            new Rotation3d(Constants.CAMERA_3_ROTATION_ROLL, Constants.CAMERA_3_ROTATION_PITCH, Constants.CAMERA_3_ROTATION_YAW)));
        cam4 = new Vision(m_robotContainer.drivetrain::addVisionMeasurement, Constants.CAMERA_4_NAME, new Transform3d(
            new Translation3d(Constants.CAMERA_4_TRANSLATION_X, Constants.CAMERA_4_TRANSLATION_Y, Constants.CAMERA_4_TRANSLATION_Z), 
            new Rotation3d(Constants.CAMERA_4_ROTATION_ROLL, Constants.CAMERA_4_ROTATION_PITCH, Constants.CAMERA_4_ROTATION_YAW)));

    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        if(!DriverStation.isDisabled()){
            cam1.periodic();
            cam2.periodic();
            cam3.periodic();
            cam4.periodic();
        }

        // First, tell Limelight your robot's current orientation
        double robotYaw = m_robotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble();
        LimelightHelpers.SetRobotOrientation(Constants.PRIMARY_LL_NAME, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Get the pose estimate
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.PRIMARY_LL_NAME);

        // Add it to your pose estimator
        if(LimelightHelpers.getTargetCount(Constants.PRIMARY_LL_NAME) > 0){
            m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
            m_robotContainer.drivetrain.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds
        );
        }
        
        LLPosePublisher.set(limelightMeasurement.pose);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        LimelightHelpers.SetThrottle(Constants.PRIMARY_LL_NAME, 200);
        LimelightHelpers.SetThrottle(Constants.SECONDARY_LL_NAME, 200);
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        LimelightHelpers.SetThrottle(Constants.PRIMARY_LL_NAME, 0);
        LimelightHelpers.SetThrottle(Constants.SECONDARY_LL_NAME, 0);
    }

    @Override
    public void teleopPeriodic() {
        LimelightHelpers.SetThrottle(Constants.PRIMARY_LL_NAME, 0);
        LimelightHelpers.SetThrottle(Constants.SECONDARY_LL_NAME, 0);
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
