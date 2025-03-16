// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands; 

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAlgaeCommand extends Command {
  private AlgaeSubsystem algaeSubsystem;
  private double encoderSetPoint;

  /** Creates a new SetAlgaeCommand. */
  public SetAlgaeCommand(AlgaeSubsystem algaeSubsystem, double encoderSetPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeSubsystem = algaeSubsystem;
    this.encoderSetPoint = encoderSetPoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeSubsystem.algaePivot.set(algaeSubsystem.algaePivotPID.calculate(algaeSubsystem.algaeEncoderDistance, encoderSetPoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double aimError = Math.abs(algaeSubsystem.algaeEncoderDistance - encoderSetPoint);
    if (aimError <= Constants.AlgaeConstants.algaeTolerance) {
        return true;
    }
    return false;
  }
}
