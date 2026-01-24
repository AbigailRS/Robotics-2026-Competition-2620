// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CameraSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AddVisionMeasurements extends Command {
  /** Creates a new AddVisionMeasurements. */
  CommandSwerveDrivetrain drivetrain;
  CameraSystem leftCamera;
  CameraSystem rightCamera;

  public AddVisionMeasurements(CommandSwerveDrivetrain drivetrain, CameraSystem rightCamera, CameraSystem leftCamera) {
    this.drivetrain = drivetrain;
    this.leftCamera = leftCamera;
    this.rightCamera = rightCamera;
    addRequirements(this.drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      drivetrain.addVisionMeasurement(leftCamera.getRobotPose(), Utils.getCurrentTimeSeconds());
      drivetrain.addVisionMeasurement(rightCamera.getRobotPose(), Utils.getCurrentTimeSeconds());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
