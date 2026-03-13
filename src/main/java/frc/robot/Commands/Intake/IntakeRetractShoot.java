// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.bigRockIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeRetractShoot extends Command {

  bigRockIntake rockIntake;
  boolean retractMode = false;

  /** Creates a new IntakeExtendPos. */
  public IntakeRetractShoot(bigRockIntake rockIntake) {
    this.rockIntake = rockIntake;
    addRequirements(rockIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rockIntake.setUpdatedPID(1.0, 0, 0);
    if((rockIntake.intakeInPosition() || rockIntake.intakeRetracted()) && retractMode){
      rockIntake.setExtendPosition(Constants.EXTEND_POSITION_OSCILLATE);
      retractMode = false;
    }
    else if((rockIntake.intakeInPosition() || rockIntake.intakeRetracted()) && !retractMode){
      rockIntake.setExtendPosition(Constants.EXTEND_POSITION_IN);
      retractMode = true;
    }
    rockIntake.setIntakeVoltage(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rockIntake.setUpdatedPID(1.0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return rockIntake.intakeInPosition() && DriverStation.isAutonomousEnabled();
    return false;
  }
}
