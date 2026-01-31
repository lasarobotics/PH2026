package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AimUtil;
import frc.robot.Constants;
import frc.robot.GameHelpers;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

    private static ShooterSubsystem s_shooterSubsystem;
    private TalonFX m_shooterMotorOne;
    private TalonFX m_shooterMotorTwo;
    private TalonFX m_shooterMotorThree;
    private TalonFX m_indexerMotor;
    private TalonFX m_hoodMotor;

    private final MotionMagicVelocityVoltage m_shooterRequest;
    private final MotionMagicVelocityVoltage m_indexerRequest;
    private final MotionMagicVoltage m_hoodRequest;

    public static ShooterSubsystem getInstance() {
        if (s_shooterSubsystem == null) {
            s_shooterSubsystem = new ShooterSubsystem();
        }
        return s_shooterSubsystem;
    }

    private ShooterSubsystem() {
        m_shooterMotorOne = new TalonFX(Constants.ShooterSubsystem.SHOOTER_ONE_MOTOR_ID);
        m_shooterMotorOne = new TalonFX(Constants.ShooterSubsystem.SHOOTER_TWO_MOTOR_ID);
        m_shooterMotorOne = new TalonFX(Constants.ShooterSubsystem.SHOOTER_THREE_MOTOR_ID);
        m_indexerMotor = new TalonFX(Constants.ShooterSubsystem.INDEXER_MOTOR_ID);
        m_hoodMotor = new TalonFX(Constants.ShooterSubsystem.HOOD_MOTOR_ID);

        m_shooterRequest = new MotionMagicVelocityVoltage(0);
        m_indexerRequest = new MotionMagicVelocityVoltage(0);
        m_hoodRequest = new MotionMagicVoltage(0);

        // TODO set up configs
        // for reference:
        // https://github.com/lasarobotics/PH2025/blob/master/src/main/java/frc/robot/subsystems/lift/LiftSubsystem.java#L1359-L1422
        TalonFXConfiguration shooterOneConfig = new TalonFXConfiguration();
        TalonFXConfiguration shooterTwoConfig = new TalonFXConfiguration();
        TalonFXConfiguration shooterThreeConfig = new TalonFXConfiguration();

        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        m_shooterMotorOne.getConfigurator().apply(shooterOneConfig);
        m_shooterMotorTwo.getConfigurator().apply(shooterTwoConfig);
        m_shooterMotorThree.getConfigurator().apply(shooterThreeConfig);
        m_indexerMotor.getConfigurator().apply(indexerConfig);
        m_hoodMotor.getConfigurator().apply(hoodConfig);
    }

    /**
     * Periodic shooter logic. Basically:
     * <ul>
     * <li>Always adjust hood
     * <li>If holding shoot/pass button:
     * <ul>
         * <li>Set shoot motor to shoot speed
         * <li>Toggle indexer motor based on if ready to shoot/pass
     * </ul>
     * <li>If not holding button, set shoot motor to hold speed
     * and stop indexer
     * </ul
     * @param shooting If shoot button is held
     * @param passing If pass button is held
     */
    public void shooterPeriodic(boolean shooting, boolean passing) {
        adjustHood();
        if (shooting || passing) {
            shoot();
            if ((readyToShoot() && shooting) ||
                (readyToPass() && passing)) {
                index();
            } else {
                stopIndexer();
            }
        } else {
            holdShooter();
            stopIndexer();
        }
    }

    /**
     * Stops all motors.
     */
    public void stopEverything() {
        stopShooter();
        stopIndexer();
        stopHood();
    }

    /**
     * Stop the {@link #m_indexerMotor indexer motor}.
     */
    public void stopIndexer() {
        m_indexerMotor.setControl(
            m_indexerRequest.withVelocity(0)
        );
    }

    /**
     * Set the speed of the {@link #m_indexerMotor indexer motor}
     * to the (constant)
     * {@link Constants.ShooterSubsystem#indexerHoldSpeed indexer hold speed}.
     */
    public void index() {
        m_indexerMotor.setControl(
            m_indexerRequest.withVelocity(Constants.ShooterSubsystem.INDEXER_MOTOR_SPEED)
        );
    }

    /**
     * Stop the {@link #m_shooterMotorOne shooter motors} (coast to 0).
     */
    public void stopShooter() {
        m_shooterMotorOne.setVoltage(0);
        m_shooterMotorTwo.setVoltage(0);
        m_shooterMotorThree.setVoltage(0);
    }

    /**
     * Set the speed of one {@link #m_shooterMotorOne shooter motor}
     * to the (constant)
     * {@link Constants.ShooterSubsystem#SHOOTER_HOLD_SPEED shooter hold speed}
     * and let the others coast.
     */
    public void holdShooter() {
        m_shooterMotorOne.setControl(
            m_shooterRequest.withVelocity(Constants.ShooterSubsystem.SHOOTER_HOLD_SPEED)
        );
        m_shooterMotorTwo.setVoltage(0);
        m_shooterMotorThree.setVoltage(0);
    }

    /**
     * Set the speed of the {@link #m_shooterMotorOne shooter motors}
     * to the desired shooting speed
     * according to {@link #wantedShooterSpeed()}
     */
    public void shoot() {
        m_shooterMotorOne.setControl(
            m_shooterRequest.withVelocity(wantedShooterSpeed())
        );
        m_shooterMotorTwo.setControl(
            m_shooterRequest.withVelocity(wantedShooterSpeed())
        );
        m_shooterMotorThree.setControl(
            m_shooterRequest.withVelocity(wantedShooterSpeed())
        );
    }

    // TODO see if every motor should be checked
    // TODO update javadoc
    /**
     * 
     * @return If the shooter motor is {@link Constants.ShooterSubsystem#SHOOTER_SPEED_TOLERANCE near}
     * the desired speed according to {@link #wantedShooterSpeed()}
     */
    public boolean atShootSpeed() {
        return m_shooterMotorOne.getVelocity().isNear(
            wantedShooterSpeed(),
            Constants.ShooterSubsystem.SHOOTER_SPEED_TOLERANCE
        );
    }

    /**
     * Sets the target position of the hood to the current hood position
     * (i.e. stop the hood)
     */
    public void stopHood() {
        double pos = m_hoodMotor.getPosition().getValueAsDouble();
        m_hoodMotor.setControl(m_hoodRequest.withPosition(pos));
    }

    /**
     * Set the setpoint of the {@link #m_hoodMotor hood motor}
     * to the desired hood position
     * according to {@link #wantedHoodPosition()}
     */
    public void adjustHood() {
        m_hoodMotor.setControl(
            m_hoodRequest.withPosition(wantedHoodPosition())
        );
    }

    /**
     * Checks that the robot can make it in if it shoots right now
     * and that the drivetrain is at the wanted rotation
     * @return If robot is in a good position to shoot
     */
    public boolean readyToShoot() {
        return (
            (
                GameHelpers.scoringTimeLeft() - Constants.Drive.HANG_TIME
                >= Constants.ShooterSubsystem.SHOOTER_TIME_MARGIN
            ) &&
            atShootSpeed() &&
            DriveSubsystem.getInstance().atWantedRotation()
        );
    }

    /**
     * Checks that the robot can make it in the alliance zone if it shoots right now
     * and that the drivetrain is at the wanted rotation
     * (doesn't check if hub is active)
     * @return If robot is in a good position to shoot
     */
    public boolean readyToPass() {
        return atShootSpeed() && DriveSubsystem.getInstance().atWantedRotation();
    }

    /**
     * Get the current wanted ball velocity from AimUtil
     * and convert to rotations per second.
     * @return The wanted rotations per second
     */
    public double wantedShooterSpeed() {
        LinearVelocity ballVelocity = AimUtil.getBallVelocity();
        double radiansPerSecond =
            2 * ballVelocity.in(MetersPerSecond)
            / Constants.ShooterSubsystem.SHOOTER_RADIUS.in(Meters);
        double rotationsPerSecond =
            radiansPerSecond / (2 * Math.PI);
        return rotationsPerSecond;
    }

    // TODO implement
    // return a number of rotations
    public double wantedHoodPosition() {
        Angle theta = AimUtil.getExitAngle();
        return 0;
    }

    /**
     * Converts a given angle to the number of rotations
     * the hood motor would be at at that angle
     * @param angle The angle to convert
     * @return The number of rotations
     */
    public double hoodAngleToRotations(Angle angle) {
        double base = angle.in(Rotations);
        double rotations = base * Constants.ShooterSubsystem.HOOD_TO_MOTOR_RATIO;
        return rotations;
    }

    public void close() {
        m_shooterMotorOne.close();
        m_shooterMotorTwo.close();
        m_shooterMotorThree.close();
        m_indexerMotor.close();
        m_hoodMotor.close();
    }
}