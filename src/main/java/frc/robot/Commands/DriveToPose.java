// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  CommandSwerveDrivetrain driveTrain;
  Pose2d targetPose, pose;

  PIDController xController, yController, rController;
  double xSpeed, ySpeed, rSpeed;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /** Creates a new DriveToPose. */
  public DriveToPose(CommandSwerveDrivetrain driveTrain, Pose2d targetPose) {
    this.driveTrain = driveTrain;
    this.targetPose = targetPose;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pose = driveTrain.samplePoseAt(Utils.currentTimeToFPGATime(Utils.getCurrentTimeSeconds())).get();
    
    xSpeed = xController.calculate(pose.getX(), targetPose.getX());
    ySpeed = xController.calculate(pose.getY(), targetPose.getY());
    rSpeed = xController.calculate(pose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());

    driveTrain.applyRequest(() ->
        drive.withVelocityX(-xSpeed * Constants.MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-ySpeed * Constants.MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-rSpeed * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(pose.getX() - targetPose.getX()) < 0.1 && Math.abs(pose.getY() - targetPose.getY()) < 0.1 && Math.abs(pose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < 0.1){
        return true;
    }
    return false;
  }
}
