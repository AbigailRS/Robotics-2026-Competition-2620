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
public class RetractHoods extends Command {
  /** Creates a new SetHoodForShoot. */

  Hoods hoods;
  CommandSwerveDrivetrain driveTrain;

  InterpolatingDoubleTreeMap leftHoodIMap = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap rightHoodIMap = new InterpolatingDoubleTreeMap();

  public RetractHoods(Hoods hoods) {
    this.hoods = hoods;
    addRequirements(hoods);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    hoods.setLeftServoPosition(Constants.HOOD_LEFT_LOW_POSITION);
    hoods.setRightServoPosition(Constants.HOOD_RIGHT_LOW_POSITION);
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
