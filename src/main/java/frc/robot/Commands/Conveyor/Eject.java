// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.bigRockIntake;
import frc.robot.subsystems.rockDestroyerInxder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Eject extends Command {
  /** Creates a new Eject. */
  rockDestroyerInxder indexer;

  public Eject(rockDestroyerInxder indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.setConveryVoltage(-1.0);
    indexer.setLeftRockSumusherVoltage(-1.0);
    indexer.setRightRockSmusherVoltage(-1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setConveryVoltage(0.0);
    indexer.setLeftRockSumusherVoltage(0.0);
    indexer.setRightRockSmusherVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
