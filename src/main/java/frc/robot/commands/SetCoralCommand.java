// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralCommand extends Command {
  private CoralSubsystem coralSubsystem;;
  private double encoderSetPoint;
  private boolean right;
  /** Creates a new CoralPreset. */
  public SetCoralCommand(CoralSubsystem coralSubsystem, double encoderSetPoint, boolean right) {
    this.coralSubsystem = coralSubsystem;
    this.encoderSetPoint = encoderSetPoint;
    this.right = right;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralSubsystem.coralPivotRight.set(coralSubsystem.coralPID.calculate(coralSubsystem.coralPivotEncoderDistanceRight, encoderSetPoint));
    coralSubsystem.coralPivotLeft.set(coralSubsystem.coralPID.calculate(coralSubsystem.coralPivotEncoderDistanceLeft, encoderSetPoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralSubsystem.coralPivotRight.set(0);
    coralSubsystem.coralPivotLeft.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double aimErrorRight = Math.abs(coralSubsystem.coralPivotEncoderDistanceRight - encoderSetPoint);
    double aimErrorLeft = Math.abs(coralSubsystem.coralPivotEncoderDistanceLeft - encoderSetPoint);
    if(right)
      if(aimErrorRight <= Constants.CoralConstants.coralTolerance) {
          return true;
      }
    else{
      if (aimErrorLeft <= Constants.CoralConstants.coralTolerance){
        return true;
      }
    }
    return false;
  }
}
