// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralSubsystem extends SubsystemBase {
  public SparkMax coralSpinnyRight;
  public SparkMax coralPivotRight;
  public SparkMax coralSpinnyLeft;
  public SparkMax coralPivotLeft;

  public DutyCycleEncoder coralPivotEncoderRight;
  public DutyCycleEncoder coralPivotEncoderLeft;

  public DigitalInput coralDetector;
  public boolean holdingCoral;

  public PIDController coralPID;
  public double coralPivotEncoderDistanceRight;
  public double coralPivotEncoderDistanceLeft;

  public ShuffleboardTab tab;
  public GenericEntry hasCoral;

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {
    coralSpinnyRight = new SparkMax(Constants.CoralConstants.coralSpinnyRightID, MotorType.kBrushless);
    coralPivotRight = new SparkMax(Constants.CoralConstants.coralPivotRightID, MotorType.kBrushless);
    coralSpinnyLeft = new SparkMax(Constants.CoralConstants.coralSpinnyLeftID, MotorType.kBrushless);
    coralPivotLeft = new SparkMax(Constants.CoralConstants.coralPivotLeftID, MotorType.kBrushless);
    
    coralPivotEncoderRight = new DutyCycleEncoder(Constants.CoralConstants.coralPivotEncoderRightDIOPort);
    coralPivotEncoderLeft = new DutyCycleEncoder(Constants.CoralConstants.coralPivotEncoderLeftDIOPort);

    coralPID = Constants.CoralConstants.coralPID;

    coralDetector = new DigitalInput(Constants.CoralConstants.coralDetectorID);

    tab = Shuffleboard.getTab("Coral Subsystem");
    hasCoral = tab.add("Holdng Coral", false).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    coralPivotEncoderDistanceRight = coralPivotEncoderRight.get();
    coralPivotEncoderDistanceLeft = coralPivotEncoderLeft.get();
    holdingCoral = coralDetector.get();
    if(holdingCoral) hasCoral.setBoolean(true);
      else hasCoral.setBoolean(false);
  }
}
