// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  public TalonFX LElevator;
  //public TalonFX RElevator;
  final Follower RElevator;
  public DutyCycleEncoder elevatorEncoder;
  public PIDController elevatorPID;

  public double ElevatorEncoderDistance;

  //public TalonFX wristMotor;

  /** Creates a new ElavatorSubsystem. */
  public ElevatorSubsystem() {
    LElevator = new TalonFX(Constants.ElevatorConstants.LElevatorID);
    //RElevator = new TalonFX(Constants.ElevatorConstants.RElevatorID);

    RElevator = new Follower(Constants.ElevatorConstants.LElevatorID, true);

   // wristMotor = new TalonFX(Constants.ElevatorConstants.wristMotorID);
    elevatorEncoder = new DutyCycleEncoder(Constants.ElevatorConstants.ElevatorEncoderDIOPort); 
    elevatorPID = Constants.ElevatorConstants.ElevatorPID;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ElevatorEncoderDistance = Math.abs(elevatorEncoder.get());
  }
}
