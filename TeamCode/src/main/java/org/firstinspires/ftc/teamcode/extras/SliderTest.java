package org.firstinspires.ftc.teamcode.extras;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Config
public class SliderTest extends LinearOpMode {
    public static DcMotorEx
            sliderMotorOne = null, sliderMotorTwo = null;
    public static double motorPowerUP = 0.8, motorPowerDOWN = 0.8;
    public static int levelOne = 200, levelZero = 0, levelNeg = -10;
    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target;
    public static double kp = 3.5, ki, kd = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        sliderMotorOne = hardwareMap.get(DcMotorEx.class, "sliderMotorOne");
        sliderMotorTwo = hardwareMap.get(DcMotorEx.class, "sliderMotorTwo");

        sliderMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sliderMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //motor directions
        sliderMotorOne.setDirection(DcMotorEx.Direction.FORWARD);
        sliderMotorTwo.setDirection(DcMotorEx.Direction.FORWARD);

        //reset motor encoders
        sliderMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.x){
                output_power = lifter_pid(kp, ki, kd, levelOne);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                extendTo(levelOne, output_power);
            }
            if(gamepad1.y){
                output_power = lifter_pid(kp, ki, kd, levelZero);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                extendTo(levelZero, output_power);
            }
            if(gamepad1.b){
                output_power = lifter_pid(kp, ki, kd, levelNeg);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                extendTo(levelNeg, output_power);
            }
            telemetry.addData("SliderMotorOne tick count", sliderMotorOne.getCurrentPosition());
            telemetry.addData("SliderMotorTwo tick count", sliderMotorTwo.getCurrentPosition());
            telemetry.addData("SliderMotorOne Current", sliderMotorOne.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SliderMotorTwo Current", sliderMotorTwo.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
   public void extendTo(int targetPos,double pow){
        sliderMotorOne.setTargetPosition(targetPos);
        sliderMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderMotorOne.setPower(pow);

        sliderMotorTwo.setTargetPosition(targetPos);
        sliderMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderMotorTwo.setPower(pow);

    }
    public double lifter_pid(double kp_lifter, double ki_lifter, double kd_lifter, int target)
    {
        lifter_posL = sliderMotorOne.getCurrentPosition();
        lifter_posR = sliderMotorTwo.getCurrentPosition();
        error_lifter = target - lifter_posL;
        error_diff = error_lifter - errorprev;
        error_int = error_lifter + errorprev;
        output_lifter = kp_lifter*error_lifter + kd_lifter*error_diff +ki_lifter*error_int;
        error_lifterR = target - lifter_posR;
        error_diffR = error_lifterR - errorprevR;
        error_intR = error_lifterR + errorprevR;
        output_lifterR = kp_lifter*error_lifterR + kd_lifter*error_diffR +ki_lifter*error_intR;
        errorprev = error_lifter;
        errorprevR = error_lifterR;
        return Math.abs(output_lifter);
    }
}
