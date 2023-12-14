
package frc.robot;

import frc.robot.commands.ArmControl;
import frc.robot.commands.CatcherControl;
import frc.robot.commands.CatcherOn;
import frc.robot.commands.SetEncoderZero;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.armSet.TurnToDoubleSubstation;
import frc.robot.commands.armSet.TurnToHighNode;
import frc.robot.commands.armSet.TurnToLowNode;
import frc.robot.commands.auto.AutoCatch;
import frc.robot.commands.auto.AutoDelay;
import frc.robot.commands.auto.AutoNeckDown;
import frc.robot.commands.auto.AutoRelease;
import frc.robot.commands.auto.AutoTurnBack;
import frc.robot.commands.auto.AutoTurnToHighNode;
import frc.robot.commands.auto.AutoTurnUp;
import frc.robot.commands.ledControl.PurpleLedControl;
import frc.robot.commands.ledControl.YelloLedControl;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Catcher;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	private final XboxController m_driveController = new XboxController(
			Constants.ControllerConstants.DRIVE_CONTROLLER_PORT);
	private final XboxController m_operatorController = new XboxController(
			Constants.ControllerConstants.OPERATOR_CONTROLLER_PORT);

	// The robot's subsystems and commands are defined here...

	private final Catcher m_chatcher = new Catcher();
	private final Arm m_arm = new Arm();
	private final Swerve s_Swerve = new Swerve();

	// Replace with CommandPS4Controller or CommandJoystick if needed

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		setDefaultCommand();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		new Trigger(m_operatorController::getAButton).toggleOnTrue(new CatcherOn(m_chatcher));
		new Trigger(m_operatorController::getLeftStickButton).toggleOnTrue(
				new SetEncoderZero(
						m_chatcher,
						m_arm,
						m_operatorController::getRightTriggerAxis,
						m_operatorController::getLeftTriggerAxis,
						m_operatorController::getRightY,
						m_operatorController::getRightBumper,
						m_operatorController::getLeftBumper));

		new Trigger(m_operatorController::getYButton).whileTrue(new TurnToHighNode(m_arm, m_chatcher));
		new Trigger(m_operatorController::getXButton)
				.whileTrue(new TurnToLowNode(m_arm, m_chatcher));
		new Trigger(m_operatorController::getBButton).whileTrue(new TurnToDoubleSubstation(m_arm, m_chatcher));

		new Trigger(m_driveController::getAButton).onTrue(new YelloLedControl(m_chatcher));
		new Trigger(m_driveController::getBButton).onTrue(new PurpleLedControl(m_chatcher));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	Command getAutonomousCommand() {

		return autoBalance();
	}

	private void setDefaultCommand() {
		m_arm.setDefaultCommand(new ArmControl(m_arm, m_operatorController::getRightY,
				m_operatorController::getRightTriggerAxis, m_operatorController::getLeftTriggerAxis));
		m_chatcher.setDefaultCommand(
				new CatcherControl(m_chatcher, m_operatorController::getRightBumper,
						m_operatorController::getLeftBumper));

		s_Swerve.setDefaultCommand(
				new TeleopSwerve(
						s_Swerve,
						m_driveController::getLeftY,
						m_driveController::getLeftX,
						m_driveController::getRightX,
						m_driveController::getYButton,
						m_driveController::getRightTriggerAxis));
	}

	private Command auto1() {
		return new AutoTurnToHighNode(m_arm, m_chatcher)
				.andThen(new AutoDelay(3))
				.andThen(new AutoNeckDown(m_chatcher));
	}

	private Command autoLine() {
		return new AutoTurnToHighNode(m_arm, m_chatcher)
				.andThen(new AutoDelay(0.6))
				.andThen(new AutoNeckDown(m_chatcher))
				.andThen(new AutoDelay(0.2))
				.andThen(new AutoDelay(0.5).andThen(new AutoTurnBack(m_arm, m_chatcher)).alongWith(pathBack));
	}

	private Command autoSet1To1ToSet2() {
		return new AutoTurnToHighNode(m_arm, m_chatcher)
				.andThen(new AutoDelay(0.6))
				.andThen(new AutoNeckDown(m_chatcher))
				.andThen(new AutoDelay(0.2))
				.andThen(new AutoDelay(0.5).andThen(new AutoTurnBack(m_arm, m_chatcher)).alongWith(pathSet1To1))
				.andThen(new AutoDelay(0.5))
				.andThen(new AutoCatch(m_chatcher))
				.andThen(new AutoDelay(0.2))
				.andThen(path1ToSet2.alongWith(new AutoDelay(1).andThen(new AutoTurnToHighNode(m_arm, m_chatcher))))
				.andThen(new AutoRelease(m_chatcher))
				.andThen(new AutoDelay(0.8))
				.andThen(new AutoTurnUp(m_arm, m_chatcher));
	}

	private Command autoBalance() {
		return new AutoTurnToHighNode(m_arm, m_chatcher)
				.andThen(new AutoDelay(0.6))
				.andThen(new AutoNeckDown(m_chatcher))
				.andThen(new AutoDelay(0.2))
				.andThen(new AutoDelay(0.5).andThen(new AutoTurnBack(m_arm, m_chatcher))
						.alongWith(pathSetToBal.andThen(pathSetToBal2)));
	}

	private PathPlannerTrajectory PATH_SET1_TO_1 = PathPlanner.loadPath(
			"PATH_SET1_TO_1",
			3,
			3);

	private PathPlannerTrajectory PATH_1_TO_SET2 = PathPlanner.loadPath(
			"PATH_1_TO_SET2",
			3,
			3);

	private PathPlannerTrajectory PATH_SET_TO_BAL = PathPlanner.loadPath(
			"PATH_SET_TO_BAL",
			2.5,
			3);

	private PathPlannerTrajectory PATH_SET_TO_BAL2 = PathPlanner.loadPath(
			"PATH_SET_TO_BAL2",
			2.5,
			3);

	private PathPlannerTrajectory PATH_BACK = PathPlanner.loadPath(
			"PATH_BACK",
			3,
			3);

	private PPSwerveControllerCommand pathBack = new PPSwerveControllerCommand(
			PATH_BACK,
			s_Swerve::getPose,
			Constants.SwerveConstants.swerveKinematics,
			new PIDController(0.53, 0, 0),
			new PIDController(0, 0, 0),
			new PIDController(5, 0, 0),
			s_Swerve::setModuleStates,
			s_Swerve);

	private PPSwerveControllerCommand pathSet1To1 = new PPSwerveControllerCommand(
			PATH_SET1_TO_1,
			s_Swerve::getPose,
			Constants.SwerveConstants.swerveKinematics,
			new PIDController(0.45, 0, 0),
			new PIDController(-0.03, 0, 0),
			new PIDController(5, 0, 0),
			s_Swerve::setModuleStates,
			s_Swerve);
	private PPSwerveControllerCommand path1ToSet2 = new PPSwerveControllerCommand(
			PATH_1_TO_SET2,
			s_Swerve::getPose,
			Constants.SwerveConstants.swerveKinematics,
			new PIDController(-0.6, 0, 0),
			new PIDController(0, 0, 0),
			new PIDController(5, 0, 0),
			s_Swerve::setModuleStates,
			s_Swerve);

	// -0.18 -0.04
	private PPSwerveControllerCommand pathSetToBal = new PPSwerveControllerCommand(
			PATH_SET_TO_BAL,
			s_Swerve::getPose,
			Constants.SwerveConstants.swerveKinematics,
			new PIDController(0.7, 0, 0),
			new PIDController(0., 0, 0),
			new PIDController(5, 0, 0),
			s_Swerve::setModuleStates,
			s_Swerve);

	private PPSwerveControllerCommand pathSetToBal2 = new PPSwerveControllerCommand(
			PATH_SET_TO_BAL2,
			s_Swerve::getPose,
			Constants.SwerveConstants.swerveKinematics,
			new PIDController(-0.6, 0, 0),
			new PIDController(0., 0, 0),
			new PIDController(5, 0, 0),
			s_Swerve::setModuleStates,
			s_Swerve);

	// private Trajectory trajectoryTest() {
	// return TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,new
	// Rotation2d(0)),
	// List.of(new Translation2d(0,0.5),
	// new Translation2d(0,0.75)),
	// new Pose2d(0,1,new Rotation2d(180)), null);

	// }
}
