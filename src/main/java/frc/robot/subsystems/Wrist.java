package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.operationCommands.SuperstructureCommand;
import frc.robot.util.ActuatorMap;
import frc.robot.util.Constants;

public class Wrist extends Subsystem {

    private final TalonSRX wristMotor;

    public enum WristState{
        INTAKE_FRONT, INTAKE_BACK, HIGH_FRONT, HIGH_BACK, PARALLEL_TO_GROUND
    }


    int lastPos;
    private boolean isManual = false;

    Wrist() {

        wristMotor = new TalonSRX(ActuatorMap.wristPort);

        /* Factory default hardware to prevent unexpected behavior */
        wristMotor.configFactoryDefault();

        wristMotor.setNeutralMode(NeutralMode.Brake);

        /* Configure Sensor Source for Pirmary PID */
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog,
                Constants.wristPIDLoopIdx,
                Constants.wristTimeoutMs);

        wristMotor.setSensorPhase(true);
        wristMotor.setInverted(false);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,
                10, Constants.wristTimeoutMs);
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
                10, Constants.wristTimeoutMs);

        /* Set the peak and nominal outputs */
        wristMotor.configNominalOutputForward(0, Constants.wristTimeoutMs);
        wristMotor.configNominalOutputReverse(0, Constants.wristTimeoutMs);
        wristMotor.configPeakOutputForward(.4, Constants.wristTimeoutMs);
        wristMotor.configPeakOutputReverse(-.4, Constants.wristTimeoutMs);

        /* Set Motion Magic gains in slot0 - see documentation */
        wristMotor.selectProfileSlot(Constants.wristSlotIdx, Constants.wristPIDLoopIdx);
        wristMotor.config_kF(Constants.wristSlotIdx, Constants.wristF, Constants.wristTimeoutMs);
        wristMotor.config_kP(Constants.wristSlotIdx, Constants.wristP, Constants.wristTimeoutMs);
        wristMotor.config_kI(Constants.wristSlotIdx, Constants.wristI, Constants.wristTimeoutMs);
        wristMotor.config_kD(Constants.wristSlotIdx, Constants.wristD, Constants.wristTimeoutMs);
        wristMotor.config_kD(Constants.wristSlotIdx, Constants.wristD, Constants.wristTimeoutMs);

        /* Set acceleration and vcruise velocity - see documentation */
        wristMotor.configMotionCruiseVelocity(Constants.wristMaxVel, Constants.wristTimeoutMs);
        wristMotor.configMotionAcceleration(Constants.wristMaxAccel, Constants.wristTimeoutMs);

        lastPos = wristMotor.getSelectedSensorPosition();

        SmartDashboard.putBoolean("Enable Wrist", true);
    }


    void handle(SuperstructureCommand sCommand) {

        if(SmartDashboard.getBoolean("Enable Wrist", true)) {
            if(sCommand.getEmergencyCommand().getTrigger()){
                wristMotor.set(ControlMode.PercentOutput, sCommand.getEmergencyCommand().getWristVal());
                lastPos = wristMotor.getSelectedSensorPosition();
                SmartDashboard.putNumber("State", 1);
                isManual = true;
            }
            else if(sCommand.getScoreState().isNewState()){
                wristMotor.set(ControlMode.MotionMagic, sCommand.getScoreState().getWristDesiredPos());
                SmartDashboard.putNumber("Wrist Desired Position", sCommand.getScoreState().getWristDesiredPos());
                SmartDashboard.putNumber("State", 2);
                isManual = false;
            }
            else if(isManual){
                wristMotor.set(ControlMode.MotionMagic, lastPos);
                SmartDashboard.putNumber("Wrist Desired Position", lastPos);
                SmartDashboard.putNumber("State", 3);
            }
            else{
                wristMotor.set(ControlMode.MotionMagic, sCommand.getScoreState().getWristDesiredPos());
                SmartDashboard.putNumber("Wrist Desired Position", sCommand.getScoreState().getWristDesiredPos());
                SmartDashboard.putNumber("State", 4);
                isManual = false;
            }
        }

        SmartDashboard.putNumber("Wrist Actual Position Angle", wristMotor.getSelectedSensorPosition());

    }

    @SuppressWarnings("unused")
    boolean isInPosition(){
        return wristMotor.getClosedLoopError() < Constants.wristAllowedError;
    }

    @Override
    public void zeroSensors() {
        wristMotor.setSelectedSensorPosition(Constants.wristZeroPos, Constants.wristPIDLoopIdx, Constants.wristTimeoutMs);
    }

    @Override
    public void stop() {

    }
}
