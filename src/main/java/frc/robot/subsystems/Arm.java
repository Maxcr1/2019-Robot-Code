package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.operationCommands.SuperstructureCommand;
import frc.robot.util.ActuatorMap;
import frc.robot.util.Constants;

public class Arm extends Subsystem{

    private final TalonSRX arm;

    Arm() {

        arm = new TalonSRX(ActuatorMap.armMaster);
        VictorSPX armSlave = new VictorSPX(ActuatorMap.armSlave);
        armSlave.set(ControlMode.Follower, ActuatorMap.armMaster);

        arm.setNeutralMode(NeutralMode.Brake);
        armSlave.setNeutralMode(NeutralMode.Brake);

        /* Factory default hardware to prevent unexpected behavior */
        arm.configFactoryDefault();

        /* Configure Sensor Source for Pirmary PID */
        arm.configSelectedFeedbackSensor(FeedbackDevice.Analog,
                Constants.armPIDLoopIdx,
                Constants.armTimeoutMs);

        arm.setSensorPhase(false);
        arm.setInverted(true);
        armSlave.setInverted(true);



        /* Set relevant frame periods to be at least as fast as periodic rate */
        arm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.armTimeoutMs);
        arm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.armTimeoutMs);

        /* Set the peak and nominal outputs */
        arm.configNominalOutputForward(0, Constants.armTimeoutMs);
        arm.configNominalOutputReverse(0, Constants.armTimeoutMs);
        arm.configPeakOutputForward(1, Constants.armTimeoutMs);
        arm.configPeakOutputReverse(-1, Constants.armTimeoutMs);

        /* Set Motion Magic gains in slot0 - see documentation */
        arm.selectProfileSlot(Constants.armSlotIdx, Constants.armPIDLoopIdx);
        arm.config_kP(Constants.armSlotIdx, Constants.armP, Constants.armTimeoutMs);
        arm.config_kI(Constants.armSlotIdx, Constants.armI, Constants.armTimeoutMs);
        arm.config_kD(Constants.armSlotIdx, Constants.armD, Constants.armTimeoutMs);

        /* Set acceleration and vcruise velocity - see documentation */
        arm.configMotionCruiseVelocity(Constants.armMaxVel, Constants.armTimeoutMs);
        arm.configMotionAcceleration(Constants.armMaxAccel, Constants.armTimeoutMs);

        SmartDashboard.putBoolean("Enable Arm", true);
    }

    void handle(SuperstructureCommand sCommand) {
        arm.config_kF(0, Math.abs(Math.sin(Math.toRadians(getArmAngle()))), 10);
        if(sCommand.getEmergencyCommand().getEmergencyActive()){
            arm.set(ControlMode.PercentOutput, sCommand.getEmergencyCommand().getArmVal());
        }
        else{
            if(SmartDashboard.getBoolean("Enable Arm", true)) {
                arm.set(ControlMode.MotionMagic, sCommand.getScoreState().getArmDesiredPos());
                SmartDashboard.putNumber("Arm Desired Position", sCommand.getScoreState().getArmDesiredPos());
            }
        }

        SmartDashboard.putNumber("Arm Current Position", arm.getSelectedSensorPosition());

    }

    boolean isInPosition(){
        return arm.getClosedLoopError() < Constants.armAllowedError;
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void stop() {
        arm.set(ControlMode.PercentOutput, 0);
    }

    double getArmAngle(){
        double x1 = Constants.armP90;
        double y1 = 90;
        double x2 = Constants.armP270;
        double y2 = 270;
        double m = (y2-y1)/(x2-x1);
        double b = y1 - m*x1;
        return (m*arm.getSelectedSensorPosition() + b);
    }

}
