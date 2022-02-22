package frc.robot.common;

import static frc.robot.Constants.DriveTrain;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class Odometry {
    private final DifferentialDriveOdometry odometry;
    private final Gyro gyro;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    public Odometry(
            Gyro gyro,
            DifferentialDriveOdometry odometry,
            RelativeEncoder leftEncoder,
            RelativeEncoder rightEncoder) {
        this.gyro = gyro;
        this.odometry = odometry;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;

        leftEncoder.setPositionConversionFactor(DriveTrain.DRIVE_ENCODER_CONSTANT);
        leftEncoder.setVelocityConversionFactor(DriveTrain.DRIVE_ENCODER_CONSTANT / 60);

        rightEncoder.setPositionConversionFactor(DriveTrain.DRIVE_ENCODER_CONSTANT);
        rightEncoder.setVelocityConversionFactor(DriveTrain.DRIVE_ENCODER_CONSTANT / 60);

        resetEncoders();
    }

    public void update() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    public double getLeftEncoderDistance() {
        return leftEncoder.getPosition();
    }
    public double getRightEncoderDistance() {
        return rightEncoder.getPosition();
    }

    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void reset(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }
}