package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeToolConstants.AlgaeToolPosition;
import frc.robot.Constants.CoralToolConstants.CoralToolPosition;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TunnelCageConstants;
import frc.robot.Constants.ElevatorConstants.StageLevel;
import frc.robot.Constants.FieldConstants.ReefConstants.TargetSide;
import frc.robot.Constants.TunnelCageConstants.PinPosition;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.ClimbCage;
import frc.robot.commands.CoralToolAutomaticInit;
import frc.robot.commands.DropAtLevel;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.ElevatorAutomaticInit;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.LoadFromStation;
import frc.robot.commands.LockIntoCage;
import frc.robot.commands.MoveCageIntoPlace;
import frc.robot.commands.RemoveAlgae;
import frc.robot.commands.StowSubsystems;
import frc.robot.commands.TunnelCageAutomaticInit;
import frc.robot.subsystems.AlgaeTool;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.CoralTool;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.ManipulatorWheels;
import frc.robot.subsystems.TunnelCage;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController m_driverController = new CommandXboxController(
            DriveConstants.kXboxControllerID);
    private final CommandJoystick m_turnStick = new CommandJoystick(DriveConstants.kTurnStickID);
    private final CommandJoystick m_throttleStick = new CommandJoystick(DriveConstants.kThrottleStickID);

    private double driverMulti = 1;
    private final SendableChooser<Command> m_chooser;

    // /* Subsystems */
    private Base m_base;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_base = new Base();

        if (m_base != null) {
            m_base.setDefaultCommand(getBaseDefaultCommand());
        }

        registerNamedCommands();

        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoChooser", m_chooser);

        configureButtonBindings();
    }

    private void registerNamedCommands() {
    }

    private Command getBaseDefaultCommand() {
        return new RunCommand(() -> {
            // double dir_x = m_driverController.getLeftX();
            // double dir_y = m_driverController.getLeftY();

            double dir_x = m_throttleStick.getX();
            double dir_y = m_throttleStick.getY();

            // Convert cartesian vector to polar for circular deadband
            double dir_r = Math.sqrt(Math.pow(dir_x, 2) + Math.pow(dir_y, 2)); // norm of vector
            double dir_theta = Math.atan2(dir_y, dir_x); // direction of vector (rad)

            // Cap norm and add deadband
            if (dir_r < DriveConstants.kControllerMovementDeadband) {
                dir_r = 0.0;
            } else if (dir_r > 1.0) {
                dir_r = 1.0;
            } else {
                dir_r = (dir_r - DriveConstants.kControllerMovementDeadband) /
                        (1 - DriveConstants.kControllerMovementDeadband);
            }
            dir_r *= dir_r;

            double turn = 0;

            turn = MathUtil.applyDeadband(m_turnStick.getX(),
                    DriveConstants.kControllerRotationDeadband);
            turn *= (turn < 0) ? -turn : turn;

            double x = -dir_r * Math.sin(dir_theta);
            x *= DriveConstants.kMaxTeleopSpeed * DriveConstants.kGeneralSpeedMulti;
            if (Math.abs(x) >= DriveConstants.kMaxTeleopSpeed * driverMulti) {
                x = DriveConstants.kMaxTeleopSpeed * driverMulti * (x < 0 ? -1 : 1);
            }

            double y = -dir_r * Math.cos(dir_theta);
            y *= DriveConstants.kMaxTeleopSpeed * DriveConstants.kGeneralSpeedMulti;
            if (Math.abs(y) >= DriveConstants.kMaxTeleopSpeed * driverMulti) {
                y = DriveConstants.kMaxTeleopSpeed * driverMulti * (y < 0 ? -1 : 1);
            }

            turn *= -DriveConstants.kMaxTeleopRotateSpeed * DriveConstants.kGeneralSpeedMulti;
            if (Math.abs(turn) >= DriveConstants.kMaxTeleopRotateSpeed * driverMulti) {
                turn = DriveConstants.kMaxTeleopRotateSpeed * driverMulti * (turn < 0 ? -1 : 1);
            }

            m_base.drive(new Translation2d(x, y), turn, false);
        }, m_base);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(m_base);
        return m_chooser.getSelected();
    }

    public void resetGyroOffsetEstimatedPose() {
        m_base.resetGyroOffset(true);
    }

    public void setNeutralModeSwerve(NeutralModeValue neutralMode) {
        m_base.setNeutralMode(neutralMode);
    }
}
