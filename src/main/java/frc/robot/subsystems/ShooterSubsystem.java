package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex shooterMotor;
    private final SparkFlexConfig shooterMotorConfig = new SparkFlexConfig();

    private final SparkFlex indexMotor;
    private final SparkFlexConfig indexMotorConfig = new SparkFlexConfig();


    private final SparkFlex intakeMotor;
    private final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

    private final RelativeEncoder shooterEncoder;

    public ShooterSubsystem() {
        shooterMotor = new SparkFlex(33, MotorType.kBrushless);
        indexMotor = new SparkFlex(34, MotorType.kBrushless);
        intakeMotor = new SparkFlex(35, MotorType.kBrushless);

        shooterMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40);
        indexMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        intakeMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        shooterEncoder = shooterMotor.getEncoder();
        
        shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexMotor.configure(indexMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        shooterEncoder.setPosition(0);
    }

    public void setShooter(double power) {
        shooterMotor.set(power);
    }

    public void stopShooter() {
        shooterMotor.set(0);
    }

    public void setIndex(double power) {
        indexMotor.set(power);
    }

    public void stopIndex() {
        indexMotor.set(0);
    }

    public void setIntake(double power) {
        intakeMotor.set(power);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }
    
    public void allStop() {
        stopShooter();
        stopIndex();
        stopIntake();
    }

}
