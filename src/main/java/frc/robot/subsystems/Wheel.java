package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheel extends SubsystemBase{
   // private final SparkMax motor = new SparkMax(1, MotorType.kBrushless);
    DCMotor motor = DCMotor.getNEO(1);
    LinearSystem<N1, N1, N1> FlywheelSystem = LinearSystemId.createFlywheelSystem(motor, 0.2, 1);
    FlywheelSim driveSim = new FlywheelSim(FlywheelSystem, motor, 0.2);
    double pos = 0;
    double lastError = 0.0;
    double integral = 0.0;

    public Wheel() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //motor.set(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    double deltaT = 0.02;
   // System.out.println("Velocity:" + driveSim.getAngularVelocityRadPerSec());
    pos += driveSim.getAngularVelocityRadPerSec() * deltaT;
    //System.out.println("Position:" + pos);
    PIDControlPoint(pos, 10, deltaT);
    driveSim.update(deltaT);
  }

  public void SetVoltage(double volts) {
     //motor.setVoltage(volts);
  }
  
  private double CalculatePID(double error, double kP, double dE, double dt, double kD, double kI){
    
      double pTerm = error * kP;

      integral += error * dt;
      double iTerm = integral * kI;

      double derivative = (error - lastError) / dt;
      double dTerm = derivative * kD;

      double output = pTerm + iTerm + dTerm;

      lastError = error;
      return output;
  }

  public void PIDControlPoint(double currentPos, double targetPos, double dt){
    double kP = 5;
    double dE = 2;
    double kD = 3;
    double kI= 0;

    double targetVoltage = CalculatePID(targetPos - currentPos, kP,dE, dt, kD, kI);
    targetVoltage = MathUtil.clamp(targetVoltage, -12, 12);
    driveSim.setInputVoltage(targetVoltage);

    System.out.println("Target:" + targetVoltage + "Position:" + pos);
    
  }
}
