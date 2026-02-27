// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hoods;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TESTSetHoodsHigh extends Command {
  /** Creates a new TESTSetServoHigh. */
  Hoods hoodSubsystem;
  public TESTSetHoodsHigh(Hoods hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(this.hoodSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hoodSubsystem.setLeftServoPosition(Constants.HOOD_LEFT_HIGH_POSITION);
    hoodSubsystem.setRightServoPosition(Constants.HOOD_RIGHT_HIGH_POSITION);
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
