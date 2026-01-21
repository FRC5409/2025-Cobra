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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(
          TunerConstants.DrivetrainConstants.Pigeon2Id,
          TunerConstants.DrivetrainConstants.CANBusName);

  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final StatusSignal<Angle> pitch = pigeon.getPitch();
  private final StatusSignal<Angle> roll = pigeon.getRoll();

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  private final Queue<Double> pitchPositionQueue;
  private final Queue<Double> pitchTimestampQueue;
  private final StatusSignal<AngularVelocity> pitchVelocity = pigeon.getAngularVelocityYWorld();

  private final Queue<Double> rollPositionQueue;
  private final Queue<Double> rollTimestampQueue;
  private final StatusSignal<AngularVelocity> rollVelocity = pigeon.getAngularVelocityYWorld();



  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    pitch.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    roll.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);

    yawVelocity.setUpdateFrequency(100.0);
    pitchVelocity.setUpdateFrequency(100.0);
    rollVelocity.setUpdateFrequency(100.0);

    pigeon.optimizeBusUtilization();

    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());

    pitchTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    pitchPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getPitch());

    rollTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    rollPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getRoll());

  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);

    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble());
    inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());

    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(pitchVelocity.getValueAsDouble());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(rollVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    inputs.odometryPitchTimestamps =
        pitchTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();    

    inputs.odometryRollTimestamps =
        rollTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    inputs.odometryPitchPositions = 
        pitchPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    inputs.odometryRollPositions =
        rollPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    inputs.isOnBump = Math.abs(inputs.pitchPosition.getDegrees()) > 7.0; // Arbitrary threshold for bump detection -> Should be a constant for comp robot

    yawTimestampQueue.clear();
    yawPositionQueue.clear();

    pitchTimestampQueue.clear();
    pitchPositionQueue.clear();

    rollTimestampQueue.clear();
    rollPositionQueue.clear();

    inputs.tilt = Math.acos(Math.cos(pitch.getValueAsDouble() * Math.PI / 180.0) * Math.cos(roll.getValueAsDouble() * Math.PI / 180.0));

  }
}
