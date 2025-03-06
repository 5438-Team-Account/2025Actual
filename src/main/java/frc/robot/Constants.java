// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utils.StickDeadband;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Controller;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Controller Operator = new Controller(
    /* NOTE: this is a PS5 Controller */
    1, /* id */
    new StickDeadband(0.1, 0.1), /* left stick deadband */
    new StickDeadband(0.1, 0.1)); /* right stick deadband+ */

  public static final Controller Driver = new Controller(
    /* NOTE: this is a Xbox Controller */
    0, /* id */
    new StickDeadband(0.1, 0.1), /* left stick deadband */
    new StickDeadband(0.1, 0.1)); /* right stick deadband */

  //public static final Rotation3d gyroOffset = new Rotation3d(0,0,90);
  public static final double MAX_SPEED  = 4.5; //in meters/sec

  public static class ElevatorConstants{
    //ALL THE FOLLOWING ID's ARE TEMPORARY
    public static final int LElevatorID = 23;
    public static final int RElevatorID = 24;
    //public static final int wristMotorID = 2;

    public static final int ElevatorEncoderDIOPort = 0;
    public static final PIDController ElevatorPID = new PIDController(8.5, 0, 0.2);

    public static final double elevatorTolerance = 0.008;

    public static final int ElevatorL4 = 0;
    public static final int ElevatorL3 = 0;
    public static final int ElevatorL2 = 0;
    public static final int ElevatorL1 = 0;
    public static final int ElevatorTroff = 0;
  }

  public static class CoralConstants{
    public static final int coralSpinnyRightID = 17;
    public static final int coralPivotRightID = 18;
    public static final int coralSpinnyLeftID = 19;
    public static final int coralPivotLeftID = 21;

    public static final int coralDetectorID = 4;

    public static final int coralPivotEncoderRightDIOPort = 1;
    public static final int coralPivotEncoderLeftDIOPort = 1;
    public static final PIDController coralPID = new PIDController(8.5, 0, 0.2);

    public static final double coralTolerance = 0.008;

    public static final double coralL4 = 69; //angle for coral during L4 preset
    public static final double coralL3 = 50; //angle for coral during L4 preset
    public static final double coralL2 = 50; //angle for coral during L4 preset
    public static final double coralL1 = 20; //angle for coral during L4 preset
    
  }

  public static class ClimberConstants{
    public static final int climberID = 6;
  }

  public static class AlgaeConstants{
    public static final int algaeLeftID = 5438;
    public static final int algaeRightID = 8345;

    public static final int algaePivotID = 65;
    public static final int algaePivotEncoderDIOPort = 75;
    public static final PIDController algaePivotPID = new PIDController(8.5, 0, 0.2);

    public static final int algaeDetectorID = 1234;
  }
}
