// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Cameras;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CameraSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EnableCamera extends Command {

  CameraSystem cam1, cam2, cam3, cam4;
  /** Creates a new EnableCamera. */
  public EnableCamera(CameraSystem cam1, CameraSystem cam2, CameraSystem cam3, CameraSystem cam4) {
    this.cam1 = cam1;
    this.cam2 = cam2;
    this.cam3 = cam3;
    this.cam4 = cam4;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cam1.setCamOn(true);
    cam2.setCamOn(true);
    cam3.setCamOn(true);
    cam4.setCamOn(true);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
