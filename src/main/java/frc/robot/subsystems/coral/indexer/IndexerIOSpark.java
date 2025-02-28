package frc.robot.subsystems.coral.indexer;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;

public class IndexerIOSpark implements IndexerIO {
    private final SparkMax indexMotor;
    private final CANrange canRange;

    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public IndexerIOSpark() {
        indexMotor = new SparkMax(IndexerConstants.indexMotorID, MotorType.kBrushless);
        canRange = new CANrange(IndexerConstants.canRangeID);

        SparkMaxConfig indexMotorConfig = new SparkMaxConfig();
        indexMotorConfig
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12.0)
                .inverted(IndexerConstants.motorInvereted)
                .smartCurrentLimit(20); // NEO 550 -> Lower current limit

        indexMotor.configure(indexMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void disableIndexer() {
        indexMotor.set(0);
    }

    @Override
    public void enableIndexer() {
        indexMotor.set(IndexerConstants.indexMotorSpeed);
    }

    

    @Override
    public Trigger getAutoLoadTrigger() {
        return new Trigger(canRange.getIsDetected()::getValue);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(indexMotor, indexMotor::getAppliedOutput, (value) -> inputs.indexMotorAppliedOutput = value);
        inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
        inputs.autoLoadTriggerTripped = canRange.getIsDetected().getValue();
    }

}
