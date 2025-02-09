package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlBoard {
    private class HIDHandler extends CommandGenericHID {
        public HIDHandler(int port) {
            super(port);
        }
    }

    private HIDHandler[] HIDs;

    /**
     * Creates a new Control board
     * @param port1 The first port of the device
     * @param port2 The second port of the device
     * @throws IllegalArgumentException if the two ports match
     */
    public ControlBoard(int port1, int port2) {
        if (port1 == port2) throw new IllegalArgumentException("Controlboard was given two of the same port!");

        HIDs = new HIDHandler[] {
            new HIDHandler(port1),
            new HIDHandler(port2)
        };
    }

    /**
     * Gets the button attached to the ID
     * @param id The Button ID of the control board [0, 31]
     * @return The trigger related to that ID
     * @throws IllegalArgumentException if an invalid ID was given
     */
    public Trigger button(int id) {
        if (id < 0 || id >= 32) throw new IllegalArgumentException("No such button ID exists on the control board!");

        return id < 16 ? HIDs[0].button(id) : HIDs[1].button(id % 16);
    }

    /**
     * @return True if the device is connected
     */
    public boolean isConnected() {
        return HIDs[0].isConnected() && HIDs[1].isConnected();
    }
}
