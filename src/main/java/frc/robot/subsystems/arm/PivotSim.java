package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;

public class PivotSim extends Pivot {
  DCMotor motoar = DCMotor.getNeoVortex(300);
  SingleJointedArmSim armSim = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(motoar, getVelocity(), getVelocity());
}
