// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualCoralCommand extends Command {
  /** Creates a new ManualCoralCommand. */
  private CoralSubsystem coralSubsystem;
  private CommandPS5Controller operator;
  private double pivotSpeedRight;
  private double pivotSpeedLeft;
  private String side;
  public ManualCoralCommand(CoralSubsystem coralSubsystem, CommandPS5Controller operator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralSubsystem = coralSubsystem;
    this.operator = operator;
    this.side = side;

    addRequirements(coralSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSpeedRight = MathUtil.applyDeadband(-operator.getRightY(), Constants.Operator.rightStick.Y);
    coralSubsystem.coralPivotRight.set(pivotSpeedRight);

    pivotSpeedLeft = MathUtil.applyDeadband(-operator.getLeftY(), Constants.Operator.leftStick.Y);
    coralSubsystem.coralPivotLeft.set(pivotSpeedLeft);

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
    return false;
  }
}
