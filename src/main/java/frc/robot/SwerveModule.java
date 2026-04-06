package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class SwerveModule {
    final double wheelRadius = 0.0508;
    final double encoderResolution = 4096.0;
    final double moduleMaxAngularVelocity = Math.PI;
    final double moduleMaxAngularAcceleration = Math.PI * 2.0;

    public SparkMax driveMotor;
    public SparkMax rotateMotor;
    public CANcoder rotateEncoder;
    public PIDController drivePIDController;
    public ProfiledPIDController rotatePIDController;

    public SimpleMotorFeedforward driveFeedforward;
    public SimpleMotorFeedforward rotateFeedforward;

    public SwerveModule(int driveMotorID, int rotateMotorID, int encoderID, double[] drivePID, double[] rotatePID) {
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        rotateMotor = new SparkMax(rotateMotorID, MotorType.kBrushless);
        rotateEncoder = new CANcoder(encoderID);

        drivePIDController = new PIDController(drivePID[0], drivePID[1], drivePID[2]);

        rotatePIDController = new ProfiledPIDController(rotatePID[0], rotatePID[1], rotatePID[2],
                new Constraints(moduleMaxAngularVelocity, moduleMaxAngularAcceleration));

        rotatePIDController.enableContinuousInput(-Math.PI, Math.PI);

        driveFeedforward = new SimpleMotorFeedforward(1, 3);
        rotateFeedforward = new SimpleMotorFeedforward(1, 1.25);

        rotateEncoder.setPosition(0); // Tell wheel its zeroed

        rotatePIDController.setTolerance(0.02);
    }

    private Rotation2d getRotationValue() {
        return new Rotation2d(rotateEncoder.getPosition().getValueAsDouble() * Math.PI * 2);
    }

    public SwerveModuleState getState() {
        double wheelSpeed = driveMotor.getEncoder().getVelocity();
        return new SwerveModuleState(wheelSpeed, getRotationValue());
    }

    public SwerveModulePosition getPosition() {
        double wheelDistance = driveMotor.getEncoder().getPosition();
        return new SwerveModulePosition(wheelDistance, getRotationValue());
    }

    public void setDrivePIDGains(double[] drivePID) {
        drivePIDController.setPID(drivePID[0], drivePID[1], drivePID[2]);
    }

    public void setRotatePIDGains(double[] rotatePID) {
        rotatePIDController.setPID(rotatePID[0], rotatePID[1], rotatePID[2]);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d encoderRotation = getRotationValue();

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(encoderRotation);

        double delta = desiredState.angle.minus(encoderRotation).getRadians();
        desiredState.speedMetersPerSecond *= Math.cos(delta);

        double currentSpeed = driveMotor.getEncoder().getVelocity();
        
        double driveOutput = drivePIDController.calculate(currentSpeed, desiredState.speedMetersPerSecond);
        double driveFeedforwardOutput = driveFeedforward.calculate(desiredState.speedMetersPerSecond);

        double rotateOutput = rotatePIDController.calculate(
            getRotationValue().getRadians(), desiredState.angle.getRadians()
        );
        double rotateFeedforwardOutput = rotateFeedforward.calculate(rotatePIDController.getSetpoint().velocity);
    
        // Two statements below set new targets for the Swerve drive motor controller
        driveMotor.setVoltage(driveOutput + driveFeedforwardOutput);
        rotateMotor.setVoltage(rotateOutput + rotateFeedforwardOutput);
    }
        
}
