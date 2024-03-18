package frc.robot.subsystems.drive;

import java.util.List;
import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;

public class Swerve extends SubsystemBase {

    private static class Constants {

        public static final double WHEEL_BASE_INCHES = 24;
        public static final double WHEEL_BASE_METERS = Units.inchesToMeters(WHEEL_BASE_INCHES);
        public static final double TRACK_WIDTH_INCHES = 24;
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH_INCHES);
        // (FL, FR, RL, RR) to follow WPILib Convention
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(-TRACK_WIDTH_METERS / 2, WHEEL_BASE_METERS / 2),
                new Translation2d(TRACK_WIDTH_METERS / 2, WHEEL_BASE_METERS / 2),
                new Translation2d(-TRACK_WIDTH_METERS / 2, -WHEEL_BASE_METERS / 2),
                new Translation2d(TRACK_WIDTH_METERS / 2, WHEEL_BASE_METERS / 2));

        // From https://www.revrobotics.com/rev-21-3005/
        // Reductions are always measured as turns-in/turns-out
        public static final double STEERING_MOTOR_REDUCTION = 9424.0 / 203.0;
        public static final double DRIVE_MOTOR_PINION_TEETH = 14;
        // Spot check - with a 12 tooth pinion this is ~5.5:1, as stated by the docs
        public static final double DRIVE_MOTOR_REDUCTION = (22.0 / DRIVE_MOTOR_PINION_TEETH) * (45.0 / 15.0);
        public static final double DRIVE_WHEEL_DIAMETER_INCHES = 3;
        public static final double DRIVE_WHEEL_CIRCUMFRENCE_INCHES = DRIVE_WHEEL_DIAMETER_INCHES * Math.PI;
        public static final double DRIVE_WHEEL_CIRCUMFRENCE_METERS = Units
                .inchesToMeters(DRIVE_WHEEL_CIRCUMFRENCE_INCHES);

        // Spec is 15.76 ft/s on a 14t pinion, derated slightly
        public static final double DRIVE_MAX_VELOCITY_METERS_PER_SECOND = Units.feetToMeters(15);

        // TODO: Calculate based on max linear velocity with all wheels oriented tangent
        // to the drive
        public static final double DRIVE_MAX_OMEGA_RADIANS_PER_SECOND = Math.PI;

        public static final ChassisSpeeds MAX_CHASSIS_SPEEDS = new ChassisSpeeds(DRIVE_MAX_VELOCITY_METERS_PER_SECOND,
                DRIVE_MAX_VELOCITY_METERS_PER_SECOND, DRIVE_MAX_OMEGA_RADIANS_PER_SECOND);

        // Just a guess since I can't find supporting documentation - we should only
        // ever be commanding 90 degree rotation and we should be able to do that in
        // ~200ms
        public static final double MODULE_MAX_SLEW_PER_SECOND = (Math.PI / 2) / 0.2;
        public static final double SERVICE_DT = 0.02;
    }

    // 2 public members - CommandFactories and State
    // Constants are private and subsystem local!
    public static final class State {

        // Only public members are functions that get or change the internal state but
        // do not control actuation
        private final Swerve thisSubsystem;

        private State(Swerve thisSubsystem) {
            this.thisSubsystem = thisSubsystem;
        }

        public ChassisSpeeds getMaxSpeeds() {
            return Constants.MAX_CHASSIS_SPEEDS;
        }

        public List<SwerveModuleState> getDesiredSwerveModuleStates() {
            return thisSubsystem.modules.stream().map(m -> m.getDesiredState()).toList();
        }

        public List<SwerveModulePosition> getSwerveModulePositions() {
            return thisSubsystem.modules.stream().map(m -> m.getPosition()).toList();

        }

        public SwerveModulePosition[] getSwerveModulePositionArray() {
            List<SwerveModulePosition> modulePositions = this.getSwerveModulePositions();
            var modulePositionArray = new SwerveModulePosition[modulePositions.size()];
            for (int i = 0; i < modulePositions.size(); i++) {
                modulePositionArray[i] = modulePositions.get(i);
            }
            return modulePositionArray;
        }

        public Rotation2d getRotation2d() {
            return Rotation2d.fromDegrees(thisSubsystem.gyro.getAngle());
        }

        public Rotation2d getDesiredHeading() {
            return thisSubsystem.desiredHeading;
        }

        public void setDesiredHeading(Rotation2d rot) {
            thisSubsystem.desiredHeading = rot;
        }

        public Pose2d getPose() {
            return thisSubsystem.poseEstimator.getEstimatedPosition();
        }

    }

    public class CommandFactories {
        // Only public members are command factories

        private final Swerve thisSubsystem;

        private CommandFactories(Swerve thisSubsystem) {
            this.thisSubsystem = thisSubsystem;
        }

        public Command driveFieldOriented(Supplier<ChassisSpeeds> speedSupplier) {
            return Commands.run(() -> {
                var speeds = speedSupplier.get();
                var relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, thisSubsystem.state.getRotation2d());
                thisSubsystem.setDesiredRobotRelativeSpeeds(relativeSpeeds);

            }, thisSubsystem).withName("Swerve/driveFieldOriented");
        }

    }

    public final State state;
    public final CommandFactories commands;

    private static final class Module {
        private final CANSparkMax drive, steer;
        private final RelativeEncoder driveEncoder;
        private final AbsoluteEncoder steerEncoder;
        private SwerveModuleState desired;

        public Module(int driveID, int turnID, Rotation2d zeroOffset) {
            drive = driveFactory(driveID, false);
            driveEncoder = drive.getEncoder();
            steer = steerFactory(
                    turnID,
                    zeroOffset, true, false, false);
            steerEncoder = steer.getAbsoluteEncoder();
            desired = getState();
        }

        private static CANSparkMax driveFactory(int deviceID, boolean motorInverted) {
            var drive = new CANSparkMax(deviceID, MotorType.kBrushless);
            Util.handleREVLibErr(drive.restoreFactoryDefaults());
            Util.handleREVLibErr(drive.setIdleMode(IdleMode.kBrake));
            Util.handleREVLibErr(drive.enableVoltageCompensation(12));

            // WPILib setInverted doesn't allow returning an error :'(
            drive.setInverted(motorInverted);
            Util.handleREVLibErr(drive.getLastError());

            // native units are Rotations, RPM
            // we want meters/ meters per second
            var encoder = drive.getEncoder();
            Util.handleREVLibErr(
                    encoder.setPositionConversionFactor(
                            Constants.DRIVE_WHEEL_CIRCUMFRENCE_METERS / Constants.DRIVE_MOTOR_REDUCTION));
            Util.handleREVLibErr(encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor() / 60));

            var controller = drive.getPIDController();
            Util.handleREVLibErr(controller.setFeedbackDevice(encoder));
            // In units of output fraction [-1, 1] per m/s of setpoint
            Util.handleREVLibErr(controller.setFF(1.0 / Constants.DRIVE_MAX_VELOCITY_METERS_PER_SECOND));
            // In units of output fraction [-1, 1] per m/s of error
            Util.handleREVLibErr(controller.setP(3));

            return drive;
        }

        private static CANSparkMax steerFactory(int deviceID, Rotation2d zeroOffset, boolean headingEncoderInverted,
                boolean relativeEncoderInverted, boolean motorInverted) {
            var steer = new CANSparkMax(deviceID, MotorType.kBrushless);
            Util.handleREVLibErr(steer.restoreFactoryDefaults());
            Util.handleREVLibErr(steer.setIdleMode(IdleMode.kBrake));
            Util.handleREVLibErr(steer.enableVoltageCompensation(12));

            // WPILib setInverted doesn't allow returning an error :'(
            steer.setInverted(motorInverted);
            Util.handleREVLibErr(steer.getLastError());

            // Assume NEO550, don't smoke the poor motor
            Util.handleREVLibErr(steer.setSmartCurrentLimit(20));

            // The duty cycle encoder's native units are [0, 1) rotations
            var heading_encoder = steer.getAbsoluteEncoder(Type.kDutyCycle);
            // Rotations -> rotations, RPM -> rotations per second
            Util.handleREVLibErr(
                    heading_encoder.setVelocityConversionFactor(heading_encoder.getPositionConversionFactor() / 60));
            Util.handleREVLibErr(heading_encoder.setInverted(headingEncoderInverted));

            // Each module has its own zero offset:
            // if we're passed a zero offset of -0.25, wrap to 0.75.
            // if passed an offset of 1.25, wrap to .25
            var wrappedZeroOffset = MathUtil.inputModulus(zeroOffset.getRotations(), 0, 1);
            Util.handleREVLibErr(heading_encoder.setZeroOffset(wrappedZeroOffset));

            // KISS - use the absolute encoder for feedback to mitigate backlash concerns.
            // just setting up the relative encoder with the right settings so we can use it
            // if needed
            var motor_encoder = steer.getEncoder();
            Util.handleREVLibErr(motor_encoder.setInverted(relativeEncoderInverted));
            // Rotations -> rotations, RPM -> rotations per second
            Util.handleREVLibErr(
                    motor_encoder.setPositionConversionFactor(1.0 / Constants.STEERING_MOTOR_REDUCTION));
            Util.handleREVLibErr(
                    motor_encoder.setVelocityConversionFactor(motor_encoder.getPositionConversionFactor() / 60));

            // Seed the relative encoder with the true heading
            Util.handleREVLibErr(motor_encoder.setPosition(heading_encoder.getPosition()));

            // Configire PID
            var controller = steer.getPIDController();
            Util.handleREVLibErr(controller.setFeedbackDevice(heading_encoder));
            Util.handleREVLibErr(controller.setPositionPIDWrappingEnabled(true));
            Util.handleREVLibErr(controller.setPositionPIDWrappingMinInput(0));
            Util.handleREVLibErr(controller.setPositionPIDWrappingMaxInput(1));

            // In units of output fraction [-1, 1] per rotation of error
            Util.handleREVLibErr(controller.setP(3));
            // In units of output fraction [-1, 1] per rotation of error per millisecond
            Util.handleREVLibErr(controller.setD(.001));

            // Enable the controller
            Util.handleREVLibErr(controller.setReference(heading_encoder.getPosition(), ControlType.kPosition));

            return steer;
        }

        public Rotation2d getRotation2d() {
            return new Rotation2d(steerEncoder.getPosition());
        }

        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(driveEncoder.getPosition(), getRotation2d());
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(
                    drive.getEncoder().getVelocity(),
                    getRotation2d());
        }

        public void setDesiredState(SwerveModuleState desired) {
            this.desired = desired;
        }

        public SwerveModuleState getDesiredState() {
            return this.desired;
        }

        public void service() {
            var currentRotation = getRotation2d();
            // Assumption here - all "equivalent" states are actually equivalent
            // for our purposes. Lift this optimize call if that's not true
            var optimized = SwerveModuleState.optimize(desired, currentRotation);
            var rotationDelta = optimized.angle.minus(currentRotation);

            // Do a little baby motion profile by figuring out how long it
            // should take us to do this rotation and moving one timestep along
            // that line. When DT > timeToComplete, we'll go all the way to the end
            // e.x. rotation change of pi/2 can happen in 200ms
            var timeToCompleteSeconds = Math.abs(rotationDelta.getRadians()) / Constants.MODULE_MAX_SLEW_PER_SECOND;
            var limitedSlewRotation = currentRotation.interpolate(
                    optimized.angle,
                    MathUtil.clamp(Constants.SERVICE_DT / timeToCompleteSeconds, 0, 1));

            Util.handleREVLibErr(
                    drive.getPIDController().setReference(desired.speedMetersPerSecond, ControlType.kVelocity));
            Util.handleREVLibErr(
                    steer.getPIDController().setReference(limitedSlewRotation.getRadians(), ControlType.kPosition));
        }
    }

    private final List<Module> modules;
    private final ADIS16470_IMU gyro;
    private final SwerveDrivePoseEstimator poseEstimator;

    private Rotation2d desiredHeading;
    private PIDController headingController;

    public Swerve() {
        this.state = new State(this);
        this.commands = new CommandFactories(this);

        /*
         * The zeroing jig provided with MaxSwerve orients each module to:
         *
         * |------------|
         * |----....^...|
         * .....\...|...|
         * ......\..|...|
         * .......\.....|
         * ........\----|
         *
         * Where the arrow indicates the wheel direction.
         * When applied to all 4 modules though, this gives:
         * - Module 1 (FL) facing west
         * - Module 2 (FR) facing north
         * - Module 3 (RL) facing south
         * - Module 4 (RR) facing east
         *
         * Our constants for each module are captured from the use of the jig,
         * then we apply offsets to get them to face north
         */

        modules = List.of(
                new Module(11, 12, Rotation2d.fromRotations(0 + -0.25)),
                new Module(21, 22, Rotation2d.fromRotations(0 + +0.00)),
                new Module(31, 32, Rotation2d.fromRotations(0 + +0.50)),
                new Module(41, 42, Rotation2d.fromRotations(0 + +0.25)));
        gyro = new ADIS16470_IMU();

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.kinematics,
                state.getRotation2d(),
                state.getSwerveModulePositionArray(),
                // TODO: starting pose from choreo/pathplanner
                new Pose2d());

        desiredHeading = state.getRotation2d();
        headingController = new PIDController(
                3, // Radians/s / radian of error
                0,
                0);
        // units of rotation
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        modules.forEach(m -> m.service());
        poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                state.getRotation2d(),
                state.getSwerveModulePositionArray());
    }

    private void setDesiredRobotRelativeSpeeds(ChassisSpeeds speeds) {
        desiredHeading = desiredHeading
                .plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * Constants.SERVICE_DT));
        var headingCorrection = headingController.calculate(state.getRotation2d().getRadians(),
                desiredHeading.getRadians());
        speeds.omegaRadiansPerSecond += headingCorrection;
        var discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.SERVICE_DT);
        var states = Constants.kinematics.toSwerveModuleStates(discreteSpeeds);
        setDesiredModuleStates(states);
    }

    private void setDesiredModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DRIVE_MAX_VELOCITY_METERS_PER_SECOND);
        for (int i = 0; i < states.length; i++) {
            modules.get(i).setDesiredState(states[i]);
        }
    }
}
