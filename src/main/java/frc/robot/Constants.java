// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map.Entry;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.AutoCommands.kReefPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldMirror;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class kDrive {
    public static final Mass ROBOT_FULL_MASS = Kilograms.of(25.034);
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(2.881);
    public static final double WHEEL_COF = 1.2;
  }

  public static final class kAuto {
    public static final boolean PRINT_AUTO_TIME = false;

    /** When this is true the robot will set it's position where the path starts when the auto is selected. */
    public static final boolean RESET_ODOM_ON_CHANGE = true;

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0 , 0.0, 0.0);
    public static final PIDConstants ROTATION_PID    = new PIDConstants(5.0 , 0.0, 0.0);
  }

  public static final class kAutoAlign {
    public static final PIDConstants ALIGN_PID = new PIDConstants(12.0, 0.0, 0.5);

    public static final LinearVelocity     MAX_AUTO_ALIGN_VELOCITY     = MetersPerSecond         .of(3.5);
    public static final LinearAcceleration MAX_AUTO_ALIGN_ACCELERATION = MetersPerSecondPerSecond.of(10.0);

    public static final Distance TRANSLATION_TOLLERANCE = Centimeters.of(2.0);
    public static final Angle    ROTATION_TOLLERANCE    = Degrees    .of(1.0);

    public static final PathConstraints PATH_FIND_CONSTRAINTS = new PathConstraints(
        TunerConstants.kSpeedAt12Volts,
        MAX_AUTO_ALIGN_ACCELERATION,
        RadiansPerSecond.of(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / Drive.DRIVE_BASE_RADIUS),
        DegreesPerSecondPerSecond.of(720.0)
    );

    public static final Pose2d PROCESSOR_TARGET = new Pose2d(11.568, 7.500, Rotation2d.fromDegrees(-90.000));

    public static final Time VELOCITY_TIME_ADJUSTEDMENT = Milliseconds.of(5000);
    public static final int  TIME_ADJUSTMENT_TIMEOUT = 10;

    public static final class kReef {
      public static final Transform2d LEFT_OFFSET_TO_BRANCH = new Transform2d(0.315, 0.167, new Rotation2d());
      public static final Transform2d RIGHT_OFFSET_TO_BRANCH = new Transform2d(0.315, -0.167, new Rotation2d());

      public static final HashMap<kReefPosition, Pose2d> TARGETS = new HashMap<>();
      static {
        TARGETS.put(kReefPosition.CLOSE_LEFT,   new Pose2d(3.668, 5.428, Rotation2d.fromDegrees(-60.000)));
        TARGETS.put(kReefPosition.FAR_LEFT,     new Pose2d(5.335, 5.392, Rotation2d.fromDegrees(-120.000)));
        TARGETS.put(kReefPosition.FAR,          new Pose2d(6.150, 4.026, Rotation2d.fromDegrees(180.000)));
        TARGETS.put(kReefPosition.CLOSE,        new Pose2d(2.850, 4.026, Rotation2d.fromDegrees(0.000)));

        TARGETS.put(kReefPosition.CLOSE_RIGHT,  FieldMirror.mirrorPose(TARGETS.get(kReefPosition.CLOSE_LEFT)));
        TARGETS.put(kReefPosition.FAR_RIGHT,    FieldMirror.mirrorPose(TARGETS.get(kReefPosition.FAR_LEFT)));
      }

      public static final HashMap<String, Pose2d> BRANCHES = new HashMap<>();
      static {
        for (Entry<kReefPosition, Pose2d> entry : TARGETS.entrySet()) {
            BRANCHES.put(entry.getKey().name() + ".L", entry.getValue().transformBy( LEFT_OFFSET_TO_BRANCH));
            BRANCHES.put(entry.getKey().name() + ".R", entry.getValue().transformBy(RIGHT_OFFSET_TO_BRANCH));
        }
      }
    }

    public static final class kStation {
        private static final Distance DISTANCE_RAMPS = Inches .of( 8.000);
        private static final Angle    STATION_ANGLE  = Degrees.of(-54.000);

        public static final Pose2d LEFT_STATION  = new Pose2d(1.498, 7.274, Rotation2d.fromDegrees(STATION_ANGLE.in(Degrees)));
        public static final Pose2d RIGHT_STATION = FieldMirror.mirrorPose(LEFT_STATION);

        public static final Transform2d STATIONS_OFFSET = new Transform2d(
            0.0,
            -DISTANCE_RAMPS.in(Meters),
            new Rotation2d()
        );
    }
  }

  public static final class kArmPivot {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 0.0;

    public static final double   ARM_GEARING      = 10.0/1.0;
    public static final Distance ARM_DRUM_RADIUS  = Inches.of(0.944);
    public static final MomentOfInertia ARM_MOI   = KilogramSquareMeters.of(0.10);
    public static final Distance ARM_LENGTH       = Inches.of(12.0);
    public static final Mass     ARM_MASS         = Pounds.of(11.0);

    public static final Angle minAngles = Radians.of(0);
    public static final Angle maxAngles = Radians.of(90);

    public static final PIDConstants SIMULATED_PID_VALUES = new PIDConstants(1.0, 0.0, 0.1);
  }

    public static enum ScoringLevel {
        LEVEL1(Meters.of(0.05), Degrees.of(15.0)),
        LEVEL2(Meters.of(0.18), Degrees.of(20.0)),
        LEVEL3(Meters.of(0.38), Degrees.of(20.0)),
        LEVEL4(Meters.of(0.68), Degrees.of(30.0));

        public final Distance elevatorSetpoint;
        public final Angle pivotAngle;

        private ScoringLevel(Distance elevatorSetpoint, Angle pivotAngle) {
            this.elevatorSetpoint = elevatorSetpoint;
            this.pivotAngle = pivotAngle;
        }
    }

  public static final class kEndEffector {
      public static final int ENDEFFECTOR_MOTOR_ID = 0;
      public static final int CURRENT_LIMIT = 30;
      public static final double VOLTAGE_INTAKE = 3;
      public static final double VOLTAGE_SCORE = -3;
      public static final int TIMOFFLIGHT_SENSORID = 0;
      public static final int TIMEOFFLIGHT_DISTANCE_VALIDATION = 80;

  }

  public static final class kElevator {
    public static final int MAIN_MOTOR_ID = 20;
    public static final int FOLLOWER_MOTOR_ID = 21;
    public static final double CURRENT_LIMIT = 30.0;
    public static final double kGearing = 9.0/1.0;
    public static final double kCircumfrence = 2 * Math.PI * 0.0199;
    public static final double kRotationConverter = kCircumfrence / kGearing;
    public static final PIDConstants TALONFX_PID = new PIDConstants(0.78, 0, 0);
    public static final PIDConstants SIM_PID = new PIDConstants(10, 0, 0);
    public static final Mass ELEVATOR_MASS = Pound.of(52.95);
    public static final Distance ELEVATOR_DRUMRADIUS = Inches.of(1.751/2);
    public static final double ELEVATOR_MIN_HEIGHT = 0.0;
    public static final double ELEVATOR_MAX_HEIGHT = 0.684;
  }

  public static final class kVision {
    public static final String CAM_NAME = "limelight";

    public static final int FIDUCIAL_TRUST_THRESHOLD = 1;

    /**
     * Frames allowed without latency update before flagged as disconnected
     */
    public static final int DISCONNECTION_TIMEOUT = 5;
  }
}