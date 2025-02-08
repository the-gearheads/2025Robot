package frc.robot.subsystems.arm;

import static frc.robot.constants.ArmConstants.ARM_LENGTH;
import static frc.robot.constants.ArmConstants.MAX_ANGLE;
import static frc.robot.constants.ArmConstants.MIN_ANGLE;
import static frc.robot.constants.ArmConstants.PIVOT_GEAR_RATIO;
import static frc.robot.constants.ArmConstants.PIVOT_MOI_EST;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotSim extends Pivot {
  DCMotor motoar = DCMotor.getNeoVortex(2);
  SingleJointedArmSim armSim = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(motoar, PIVOT_MOI_EST, PIVOT_GEAR_RATIO), motoar, PIVOT_GEAR_RATIO, ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, true, 15);
  SparkFlexSim pivotSim = new SparkFlexSim(pivot, motoar);

  public PivotSim() {
    super();
  }

  @Override
  public void simulationPeriodic() {
    var out = pivotSim.getAppliedOutput();
    armSim.setInputVoltage(out * RoboRioSim.getVInVoltage());
    armSim.update(0.02);
    pivotSim.iterate(armSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02); // might not work theres a separate iterate for abs encoder that might need to be called????
  }



}
