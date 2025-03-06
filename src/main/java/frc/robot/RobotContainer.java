// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.SetElevatorCommand;
import frc.robot.commands.SpinCoralUntilHeldCommand;
import frc.robot.commands.ManualClimberCommand;
import frc.robot.commands.ManualCoralCommand;
import frc.robot.commands.SetCoralCommand;
import frc.robot.commands.SetElevatorCommand;
import frc.robot.commands.SetElevatorCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public SwerveDrive swerveDrive;
  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public ElevatorSubsystem elevatorSubsystem;
  public CoralSubsystem coralSubsystem;
  public ClimberSubsystem climberSubsystem;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driver = new CommandXboxController(Constants.Driver.id);
  public final CommandPS5Controller operator = new CommandPS5Controller(Constants.Operator.id);

  public ManualCoralCommand manualCoralCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    elevatorSubsystem = new ElevatorSubsystem();
    coralSubsystem = new CoralSubsystem();
    climberSubsystem = new ClimberSubsystem();

    manualCoralCommand = new ManualCoralCommand(coralSubsystem, operator);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /* translation controls for the robot */
    DoubleSupplier speedMod = () -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) == 1 ? 1.5 : 1;
    /* NOTE: the division is used to reduce the speed of the robot when the left trigger is held */
    DoubleSupplier translationX = () -> -MathUtil.applyDeadband(driver.getLeftY(), Constants.Driver.leftStick.Y) / speedMod.getAsDouble();
    DoubleSupplier translationY = () -> -MathUtil.applyDeadband(driver.getLeftX(), Constants.Driver.leftStick.X) / speedMod.getAsDouble();

    /* rotation controls for the robot */
    DoubleSupplier angularRotationX = () -> -MathUtil.applyDeadband(driver.getRawAxis(4), Constants.Driver.rightStick.X) / speedMod.getAsDouble();

    Command driverControls = swerveSubsystem.driveCommand(translationX, translationY, angularRotationX);
    swerveSubsystem.setDefaultCommand(driverControls);

    driver.y().onTrue(new InstantCommand(swerveSubsystem::zeroGyro)); //zero gyro command
    operator.triangle().onTrue(new SequentialCommandGroup(
       new SetElevatorCommand(elevatorSubsystem, Constants.ElevatorConstants.ElevatorL4),
       new SetCoralCommand(coralSubsystem, Constants.CoralConstants.coralL4)
      )); // L4 Preset (untested)
    operator.square().onTrue(new SequentialCommandGroup(
      new SetElevatorCommand(elevatorSubsystem, Constants.ElevatorConstants.ElevatorL3),
      new SetCoralCommand(coralSubsystem, Constants.CoralConstants.coralL3)
    )); //L3 Preset (untested)
    operator.cross().onTrue(new SequentialCommandGroup(
      new SetElevatorCommand(elevatorSubsystem, Constants.ElevatorConstants.ElevatorL2),
      new SetCoralCommand(coralSubsystem, Constants.CoralConstants.coralL2)
    )); //L2 Preset (untested)
    operator.circle().onTrue(new SequentialCommandGroup(
      new SetElevatorCommand(elevatorSubsystem, Constants.ElevatorConstants.ElevatorL1),
      new SetCoralCommand(coralSubsystem, Constants.CoralConstants.coralL1)
    )); //L1 Preset (untested)
    operator.touchpad().onTrue(new SequentialCommandGroup(
      new SetElevatorCommand(elevatorSubsystem, 0.5),
      new SetCoralCommand(coralSubsystem, 0.3),
      new SpinCoralUntilHeldCommand(coralSubsystem, -0.5).withTimeout(10)
    ));

    operator.povUp().onTrue(new ManualClimberCommand(climberSubsystem, 0.6)); //speed is random rn, untested
    operator.povDown().onTrue(new ManualClimberCommand(climberSubsystem, -0.6)); //speed is random rn, untested
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(swerveSubsystem);
    return swerveSubsystem.getAutonomousCommand("New Auto");
 }
}
