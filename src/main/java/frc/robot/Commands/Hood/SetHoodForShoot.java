// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Hood;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldZoneManager;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hoods;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetHoodForShoot extends Command {
  /** Creates a new SetHoodForShoot. */

  Hoods hoods;
  CommandSwerveDrivetrain driveTrain;

  InterpolatingDoubleTreeMap leftHoodIMap = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap rightHoodIMap = new InterpolatingDoubleTreeMap();

  public SetHoodForShoot(Hoods hoods, CommandSwerveDrivetrain driveTrain) {
    this.hoods = hoods;
    this.driveTrain = driveTrain;
    addRequirements(hoods);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftHoodIMap.put(1.0, 0.99);
    leftHoodIMap.put(2.0, 0.90);
    leftHoodIMap.put(3.0, 0.85);
    leftHoodIMap.put(5.0, 0.5);

    rightHoodIMap.put(1.0, 0.99);
    rightHoodIMap.put(2.0, 0.90);
    rightHoodIMap.put(3.0, 0.85);
    rightHoodIMap.put(5.0, 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    Translation2d projectedPosition = driveTrain.getState().Pose.getTranslation().plus(
      new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond).times(Constants.LATENCY_CONSTANT));

    hoods.setLeftServoPosition(leftHoodIMap.get(FieldZoneManager.getDistanceTogoal(projectedPosition)));
    hoods.setRightServoPosition(rightHoodIMap.get(FieldZoneManager.getDistanceTogoal(projectedPosition)));
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
