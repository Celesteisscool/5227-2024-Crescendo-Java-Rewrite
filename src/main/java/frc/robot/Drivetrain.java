package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Drivetrain {
    double maxSpeed = 3.0; // meters per second
    double maxAngularSpeed = Math.PI;

    private final double[] frontLeftDrivePID = { 0.0, 0.0, 0.0 };
    private final double[] frontRightDrivePID = { 0.0, 0.0, 0.0 };
    private final double[] rearLeftDrivePID = { 0.0, 0.0, 0.0 };
    private final double[] rearRightDrivePID = { 0.0, 0.0, 0.0 };

    private final double[] frontLeftRotatePID = { 6.0, 0.0, 0.0 };
    private final double[] frontRightRotatePID = { 6.0, 0.0, 0.0 };
    private final double[] rearLeftRotatePID = { 6.0, 0.0, 0.0 };
    private final double[] rearRightRotatePID = { 6.0, 0.0, 0.0 };

    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule rearLeft;
    SwerveModule rearRight;

    Pigeon2 gyro = new Pigeon2(32);

    SwerveDriveKinematics swerveDriveKinematics;
    SwerveDriveOdometry swerveDriveOdometry;

    public Drivetrain() {

        Translation2d frontLeftLocation = new Translation2d(0.301, 0.301);
        Translation2d frontRightLocation = new Translation2d(0.301, -0.301);
        Translation2d rearLeftLocation = new Translation2d(-0.301, 0.301);
        Translation2d rearRightLocation = new Translation2d(-0.301, -0.301);

        frontLeft = new SwerveModule(5, 6, 11, frontLeftDrivePID, frontLeftRotatePID);
        frontRight = new SwerveModule(3, 4, 10, frontRightDrivePID, frontRightRotatePID);
        rearLeft = new SwerveModule(7, 8, 12, rearLeftDrivePID, rearLeftRotatePID);
        rearRight = new SwerveModule(1, 2, 9, rearRightDrivePID, rearRightRotatePID);
        SwerveModulePosition[] swerveModulePositions = { frontLeft.getPosition(), frontRight.getPosition(),
                rearLeft.getPosition(), rearRight.getPosition() };

        swerveDriveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, rearLeftLocation,
                rearRightLocation);
        swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics,
                new Rotation2d(gyro.getYaw().getValueAsDouble()), swerveModulePositions);

    }

    public SwerveModuleState[] getStatesBasedOffSpeeds(double xSpeed, double ySpeed, double rotSpeed,
            boolean fieldRelative, double periodSeconds, Rotation2d gyroOffset) {
        Rotation2d currentYaw = new Rotation2d(Math.toRadians(gyro.getYaw().getValueAsDouble())).rotateBy(gyroOffset);
        edu.wpi.first.math.kinematics.ChassisSpeeds chassisSpeeds = fieldRelative
                ? edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                        currentYaw)
                : new edu.wpi.first.math.kinematics.ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        edu.wpi.first.math.kinematics.ChassisSpeeds discretized = edu.wpi.first.math.kinematics.ChassisSpeeds
                .discretize(chassisSpeeds, periodSeconds);
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(discretized);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
        return swerveModuleStates;
    }

    public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void updateOdometry(double periodSeconds) {
        SwerveModulePosition[] swerveModulePositions = { frontLeft.getPosition(), frontRight.getPosition(),
                rearLeft.getPosition(), rearRight.getPosition() };
        swerveDriveOdometry.update(new Rotation2d(gyro.getYaw().getValueAsDouble()), swerveModulePositions);
    }

    public void updatePID() {
        frontLeft.setRotatePIDGains(frontLeftRotatePID);
        frontRight.setRotatePIDGains(frontRightRotatePID);
        rearLeft.setRotatePIDGains(rearLeftRotatePID);
        rearRight.setRotatePIDGains(rearRightRotatePID);
    }

    public double[] getSwerveStatesForDashboard() {
        double[] array = { (frontLeft.rotateEncoder.getPosition().getValueAsDouble() * Math.PI * 2),
                frontLeft.getState().speedMetersPerSecond,
                (frontRight.rotateEncoder.getPosition().getValueAsDouble() * Math.PI * 2),
                frontRight.getState().speedMetersPerSecond,
                (rearLeft.rotateEncoder.getPosition().getValueAsDouble() * Math.PI * 2),
                rearLeft.getState().speedMetersPerSecond,
                (rearRight.rotateEncoder.getPosition().getValueAsDouble() * Math.PI * 2),
                rearRight.getState().speedMetersPerSecond };

        return array;
    }

    public SwerveModuleState[] xState() {
        SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(0));
        SwerveModuleState[] states = { state, state, state, state };
        return states;

    }

    public void driveWithJoystick(boolean xState) {
        double xSpeed = Classes.Controls.getDriveX() * maxSpeed;
        double ySpeed = Classes.Controls.getDriveY() * maxSpeed;
        double rotSpeed = Classes.Controls.getDriveRot() * maxSpeed;

        SwerveModuleState[] states;
        if (xState) {
            states = xState();
        } else {
            states = getStatesBasedOffSpeeds(xSpeed, ySpeed, rotSpeed, true, rotSpeed, new Rotation2d(0));
        }

        setDesiredStates(states);
    }

}
