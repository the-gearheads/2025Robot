package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.INTAKE_GEAR_RATIO;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ArmvatorPosition;
import frc.robot.util.Polygon;

public class IntakeSim extends Intake {
  FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.1, INTAKE_GEAR_RATIO), DCMotor.getNeo550(1));

  private static final double FL = Vision.field.getFieldLength();
  private static final double FW = Vision.field.getFieldWidth();

  Swerve swerve;
  Superstructure superstructure;
  GamePiece simPiece = GamePiece.EMPTY;

  Polygon feederStation1Polygon = new Polygon(
      new double[] {0.0, 0.0, 2.0},
      new double[] {0.0, 2.0, 2.0}
  );

  Polygon feederStation2Polygon = new Polygon(
      new double[] {0.0, 0.0, 2.0},
      new double[] {FW, FW-2.0, FW}
  );

  public IntakeSim(Swerve swerve, Superstructure superstructure) {
    super();
    this.swerve = swerve;
    this.superstructure = superstructure;
  }

  private boolean isInFeederStation(Translation2d pose) {
    boolean shouldFlip = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    // this also would ideally need to be flipped on Y but we don't care about that here
    Translation2d flippedPose = shouldFlip ? new Translation2d(FL - pose.getX(), pose.getY()) : pose;
    return feederStation1Polygon.contains(flippedPose) || feederStation2Polygon.contains(flippedPose);
  }

  public double getVelocity() {
    return intakeSim.getAngularVelocityRPM();
  }

  @Override
  public void simulationPeriodic() {
    intakeSim.update(0.02);
    switch(simPiece) {
      case EMPTY:
        if(getVelocity() > 10) {
          if(isInFeederStation(swerve.getPose().getTranslation())) {
            simPiece = GamePiece.CORAL;
          } else {
            var pos = superstructure.getClosestArmvatorPosition();
            if(pos == ArmvatorPosition.AlgaeL2 || pos == ArmvatorPosition.AlgaeL3 || pos == ArmvatorPosition.GROUND_INTAKE) {
              simPiece = GamePiece.ALGAE;
            }
          }
        }
        break;
      case CORAL:
      case ALGAE:
        if(getVelocity() < -10) {
          simPiece = GamePiece.EMPTY;
        }
      break;
    }
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    Logger.recordOutput("Intake/Volts", volts);
    intakeSim.setInputVoltage(volts);
  }
  
  @Override
  public GamePiece getGamePiecePhosphorus() {
    return simPiece;
  }

  public void resetSimGamePiece(GamePiece gamePiece) {
    simPiece = gamePiece;
  }
}
