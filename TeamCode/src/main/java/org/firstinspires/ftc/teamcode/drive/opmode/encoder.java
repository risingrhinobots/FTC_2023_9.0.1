package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Presets ")
public class encoder extends OpMode {
    DcMotor motor;
    DcMotor motor2;
    double ticks = 537.7;
    double newTarget;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        telemetry.addData("Hardware: ", "Initialized");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            encoder(1);
        }
        if(gamepad1.y){
            encodertwo(1);
        }
        telemetry.addData("Motor Ticks: ", motor.getCurrentPosition());
        telemetry.addData("Motor2 Ticks: ", motor2.getCurrentPosition());
        if(gamepad1.b){
            tracker();
        }

    }
    public void encoder(int turnage){
        newTarget = ticks;
        motor.setTargetPosition(-491);
        motor.setPower(0.3);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(-1043);
        motor2.setPower(0.3);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void encodertwo(int turnage){
        newTarget = ticks;
        motor.setTargetPosition(-1500);
        motor.setPower(0.3);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(-1500);
        motor2.setPower(0.3);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void tracker(){
        motor.setTargetPosition(-918);
        motor.setPower(0.8);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(257);
        motor2.setPower(0.8);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}