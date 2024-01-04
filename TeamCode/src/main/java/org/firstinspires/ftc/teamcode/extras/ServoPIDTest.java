package org.firstinspires.ftc.teamcode.extras;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.ArmV2;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@TeleOp
@Config
public class ServoPIDTest extends LinearOpMode {
    ArmV2 arm = null;
    public static double
            error_servoOne, error_servoTwo, error_diffOne, error_diffTwo, error_prevOne, error_prevTwo, error_intOne, error_intTwo, output_servoOne, output_servoTwo;

    public static double servo_kp = 2, servo_ki, servo_kd;
    @Override
    public void runOpMode() throws InterruptedException {
        arm = new ArmV2(hardwareMap, telemetry);

        AnalogInput wristAnalogInput = hardwareMap.get(AnalogInput.class, "wristAnalogInput");
        AnalogInput armOneAnalogInput = hardwareMap.get(AnalogInput.class, "armOneAnalogInput");
        AnalogInput armTwoAnalogInput = hardwareMap.get(AnalogInput.class, "armTwoAnalogInput");

        while (opModeInInit()){
            ArmV2.SetArmPosition(0.15,0.155);
        }
        waitForStart();
        while (opModeIsActive()){
            double wristPosition = wristAnalogInput.getVoltage() / 3.3 * 360;
            double armOnePosition = armOneAnalogInput.getVoltage() / 3.3 * 360;
            double armTwoPosition = armTwoAnalogInput.getVoltage() / 3.3 * 360;

            if (gamepad1.x){
                List<Double> armposition = servo_pid(servo_kp, servo_ki, servo_kd, 67, 295, armOnePosition, armTwoPosition);
                double armpos2 = armposition.get(1);
                double value = armpos2 * 0.15/67;

                ArmV2.SetArmPosition(value, 0.155);
            }
            if (gamepad1.y){
                ArmV2.SetArmPosition(0.5, 0.155);
            }
            if (gamepad1.a){
                ArmV2.SetArmPosition(0.3, 0.155);
            }
//            if (gamepad1.b){
//                ArmV2.SetArmPosition(0.6, 0.155);
//            }

            telemetry.addData("armOnePosition", armOnePosition);
            telemetry.addData("armTwoPosition", armTwoPosition);

            telemetry.addData("armServoOne", ArmV2.armServoOne.getPosition());
            telemetry.addData("armServoTwo", ArmV2.armServoTwo.getPosition());
            telemetry.addData("wristServo", ArmV2.wristServo.getPosition());
            telemetry.update();
        }

    }
    public List<Double> servo_pid(double kp_servo, double ki_servo, double kd_servo, double targetOne, double targetTwo, double armOnePosition, double armTwoPosition) {
        error_servoOne = targetOne - armOnePosition;
        error_diffOne = error_servoOne - error_prevOne;
        error_intOne = error_servoOne + error_prevOne;
        output_servoOne = kp_servo * error_servoOne + kd_servo * error_diffOne + ki_servo * error_intOne;

        error_servoTwo = targetTwo - armTwoPosition;
        error_diffTwo = error_servoTwo - error_prevTwo;
        error_intTwo = error_servoTwo + error_prevTwo;
        output_servoTwo = kp_servo * error_servoTwo + kd_servo * error_diffTwo + ki_servo * error_intTwo;

        error_prevOne = error_servoOne;
        error_prevTwo = error_servoTwo;

        List<Double> ArmPosition = new ArrayList<>();
        ArmPosition.add(output_servoOne);
        ArmPosition.add(output_servoTwo);
        return ArmPosition;
    }
}
