// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        Pose2d pose1 = m_robotContainer.photonCamera1.getRobotPose();
        Matrix<N3, N1> stdDevs1 = VecBuilder.fill(0.001, 0.001, 0.001);

        if(m_robotContainer.photonCamera1.photonCamera.isConnected() && m_robotContainer.photonCamera1.hasTarget()){
            m_robotContainer.drivetrain.addVisionMeasurement(pose1, Utils.getSystemTimeSeconds(), stdDevs1);
        }
        
        Pose2d pose2 = m_robotContainer.photonCamera2.getRobotPose();
        Matrix<N3, N1> stdDevs2 = VecBuilder.fill(0.001, 0.001, 0.001);

        if(m_robotContainer.photonCamera2.photonCamera.isConnected() && m_robotContainer.photonCamera2.hasTarget()){
             m_robotContainer.drivetrain.addVisionMeasurement(pose2, Utils.getCurrentTimeSeconds(), stdDevs2);
        }

        Pose2d pose3 = m_robotContainer.photonCamera3.getRobotPose();
        Matrix<N3, N1> stdDevs3 = VecBuilder.fill(0.001, 0.001, 0.001);

        if(m_robotContainer.photonCamera3.photonCamera.isConnected() && m_robotContainer.photonCamera2.hasTarget()){
             m_robotContainer.drivetrain.addVisionMeasurement(pose3, Utils.getCurrentTimeSeconds(), stdDevs3);
        }

        Pose2d pose4 = m_robotContainer.photonCamera4.getRobotPose();
        Matrix<N3, N1> stdDevs4 = VecBuilder.fill(0.001, 0.001, 0.001);

        if(m_robotContainer.photonCamera4.photonCamera.isConnected() && m_robotContainer.photonCamera4.hasTarget()){
             m_robotContainer.drivetrain.addVisionMeasurement(pose4, Utils.getCurrentTimeSeconds(), stdDevs4);
        }

    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

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
    }

    @Override
    public void teleopPeriodic() {
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
