package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

//import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
	public SwerveDriveOdometry swerveOdometry;
	public SwerveModule[] mSwerveMods;
	// public Pigeon2 gyro;
	public AHRS n_Ahrs;

	public Swerve() {
		// gyro = new Pigeon2(Constants.Swerve.pigeonID);
		// gyro.configFactoryDefault();
		// zeroGyro();

		n_Ahrs = new AHRS(SPI.Port.kMXP);
		n_Ahrs.calibrate();
		// n_Ahrs.enableBoardlevelYawReset(false);
		zeroGyro();
		// n_Ahrs.zeroYaw();
		// gyro = new AHRS(SPI.Port.kMXP);
		// gyroEnabled = true;
		// n_Ahrs.enableLogging(true);
		// n_Ahrs.resetDisplacement();

		mSwerveMods = new SwerveModule[] {
				new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
				new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
				new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
				new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
		};
		Timer.delay(1);
		resetModulesToAbsolute();
		swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getModulePositions());
		SmartDashboard.putNumber("xP", 0);
		SmartDashboard.putNumber("yP", 0);
		SmartDashboard.putNumber("rP", 0);
	}

	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
				fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
						translation.getX(),
						translation.getY(),
						rotation,
						getYaw())
						: new ChassisSpeeds(
								translation.getX(),
								translation.getY(),
								rotation));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

		for (SwerveModule mod : mSwerveMods) {
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
		}
	}

	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

		for (SwerveModule mod : mSwerveMods) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], false);
		}
	}

	public Pose2d getPose() {
		return swerveOdometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : mSwerveMods) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : mSwerveMods) {
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public void zeroGyro() {
		// gyro.setYaw(0);
		n_Ahrs.zeroYaw();

	}

	public Rotation2d getYaw() {
		// return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360
		// -gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
		// double diretion = 360 - n_Ahrs.getAngle() + 180;
		// diretion = diretion % 360;

		// return Rotation2d.fromDegrees(diretion);
		return Rotation2d.fromDegrees(360 - n_Ahrs.getAngle());
	}

	// public double a = Math.IEEEremainder(n_Ahrs.getAngle(), 360.0);

	public void resetModulesToAbsolute() {
		for (SwerveModule mod : mSwerveMods) {
			mod.resetToAbsolute();
		}
	}

	@Override
	public void periodic() {
		/*
		 * if (DriverStation.isDisabled()){
		 * resetModulesToAbsolute();
		 * }
		 */

		swerveOdometry.update(getYaw(), getModulePositions());
		SmartDashboard.putNumber("x", swerveOdometry.getPoseMeters().getX());
		SmartDashboard.putNumber("y", swerveOdometry.getPoseMeters().getY());
		SmartDashboard.putNumber("diretion", swerveOdometry.getPoseMeters().getRotation().getDegrees());

		// for(SwerveModule mod : mSwerveMods){
		// SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
		// mod.getCanCoder().getDegrees());
		// SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
		// mod.getPosition().angle.getDegrees());
		// SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
		// mod.getState().speedMetersPerSecond);
		// }
	}
}