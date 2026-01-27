// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackTarget extends Command {
  /** Creates a new TrackTarget. */

  CommandSwerveDrivetrain drivetrain;
  Turret turret;

  InterpolatingDoubleTreeMap xSpeedToVectorMap;
  InterpolatingDoubleTreeMap ySpeedToVectorMap;

  public TrackTarget(CommandSwerveDrivetrain drivetrain, Turret turret) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    addRequirements(turret);

    xSpeedToVectorMap.put(1.0, 1.0);
    xSpeedToVectorMap.put(2.0, 2.0);

    ySpeedToVectorMap.put(1.0, 1.0);
    ySpeedToVectorMap.put(2.0, 2.0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightHelpers.SetFidcuial3DOffset("turret", xSpeedToVectorMap.get(drivetrain.getSpeedx()), ySpeedToVectorMap.get(drivetrain.getSpeedy()), 0);
    turret.setTurretRotateOnTarget();
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
