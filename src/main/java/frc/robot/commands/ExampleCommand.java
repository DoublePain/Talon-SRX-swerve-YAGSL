package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class ExampleCommand extends Command {
  private final SwerveSubsystem m_swerveSubsystem;
  private final DoubleSupplier m_translationX;
  private final DoubleSupplier m_translationY;
  private final DoubleSupplier m_angularRotationX;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param translationX The X-axis translation input (forward/backward).
   * @param translationY The Y-axis translation input (left/right).
   * @param angularRotationX The angular velocity input (rotation).
   */
  public ExampleCommand(SwerveSubsystem subsystem, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    m_swerveSubsystem = subsystem;
    m_translationX = translationX;
    m_translationY = translationY;
    m_angularRotationX = angularRotationX;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Call the driveCommand in the subsystem with translation and rotation inputs
    m_swerveSubsystem.driveCommand(m_translationX, m_translationY, m_angularRotationX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}