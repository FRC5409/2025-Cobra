// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.kElevator;
import frc.robot.Constants.kEndEffector;
import frc.robot.Constants.kElevator.kSetpoints;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.collector.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SubPickup extends SequentialCommandGroup {
  /** Creates a new SubPickup. */
  public SubPickup(Elevator sys_elevator, EndEffector sys_endEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(Commands.runOnce(() -> sys_elevator.ElevatorGo(kSetpoints.kPickUp)),
    Commands.waitSeconds(0.1), 
    Commands.runOnce(() -> sys_endEffector.runUntilCoralDetected(kEndEffector.VOLTAGE_SCORE)), 
    Commands.waitSeconds(1), 
    Commands.runOnce(() -> sys_elevator.ElevatorGo(kSetpoints.kLOW))

    
    );
  }
}
