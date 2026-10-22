package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Wheel;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command{
     @SuppressWarnings("PMD.UnusedPrivateField")
  private final Wheel m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand() {
    m_subsystem = Robot.wheel;
    addRequirements(Robot.wheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Running...");
    // set wheel state as ON 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.SetVoltage(10.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stoping...");
    // set wheel state as IDLE
    m_subsystem.SetVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
