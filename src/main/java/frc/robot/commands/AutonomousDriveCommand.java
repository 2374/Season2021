package frc.robot.commands;

// import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import org.frcteam2910.common.robot.Utilities;

public class AutonomousDriveCommand extends CommandBase {
    // The subsystem to associate the commands with see invocation in
    // RobotContainer.java
    private DrivetrainSubsystem m_subsystem;
    private Timer m_timer;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutonomousDriveCommand(DrivetrainSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        m_timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        
        m_subsystem.autoTest();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("END AUTO");
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > 2; // exit after running for 2 seconds
    }
}
