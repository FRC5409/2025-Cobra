package frc.robot.subsystems.vision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionInputsAutoLogged extends VisionIO.VisionInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Tx", tx);
    table.put("Ty", ty);
    table.put("Ta", ta);
    table.put("HasTarget", hasTarget);
    table.put("TargetId", targetId);
  }

  @Override
  public void fromLog(LogTable table) {
    tx = table.get("Tx", tx);
    ty = table.get("Ty", ty);
    ta = table.get("Ta", ta);
    hasTarget = table.get("HasTarget", hasTarget);
    targetId = table.get("TargetId", targetId);
  }

  public VisionInputsAutoLogged clone() {
    VisionInputsAutoLogged copy = new VisionInputsAutoLogged();
    copy.tx = this.tx;
    copy.ty = this.ty;
    copy.ta = this.ta;
    copy.hasTarget = this.hasTarget;
    copy.targetId = this.targetId;
    return copy;
  }
}
