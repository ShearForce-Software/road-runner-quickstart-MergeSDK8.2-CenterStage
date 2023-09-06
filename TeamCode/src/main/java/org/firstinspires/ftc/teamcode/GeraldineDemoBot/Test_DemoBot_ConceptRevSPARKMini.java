/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.GeraldineDemoBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 *
 * This OpMode executes a basic Tank Drive Teleop for a two wheeled robot using two REV SPARKminis.
 * To use this example, connect two REV SPARKminis into servo ports on the Expansion Hub. On the
 * robot configuration, use the drop down list under 'Servos' to select 'REV SPARKmini Controller'
 * and name them 'left_drive' and 'right_drive'.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Demo Bot SPARKminiMotor & Servo Drive Test")
//@Disabled
public class Test_DemoBot_ConceptRevSPARKMini extends LinearOpMode {

    // Declare OpMode members.
    DcMotorSimple armDrive = null;
    DcMotorSimple turretDrive = null;
    Servo servo_left = null;
    Servo servo_right = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        armDrive  = hardwareMap.get(DcMotorSimple.class, "arm");
        turretDrive = hardwareMap.get(DcMotorSimple.class, "turret");
        servo_left = hardwareMap.get(Servo.class, "left_servo");
        servo_right = hardwareMap.get(Servo.class, "right_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backward when connected directly to the battery
        armDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        turretDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        servo_left.setDirection(Servo.Direction.FORWARD);
        servo_right.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double armPower = -gamepad1.left_stick_y;
            double turretPower  =  gamepad1.right_stick_x;

            // Send calculated power to motors
            armDrive.setPower(armPower);
            turretDrive.setPower(turretPower);

            if (gamepad1.left_bumper) {
                servo_left.setPosition(1);
                servo_right.setPosition((0));
            } else if (gamepad1.right_bumper) {
                servo_left.setPosition(0);
                servo_right.setPosition((1));
            } else {
                servo_left.setPosition(.5);
                servo_right.setPosition((.5));
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "arm (%.2f), turret (%.2f)", armPower, turretPower);
            telemetry.addData("Servos", "left (%.2f), right (%.2f)", servo_left.getPosition(), servo_right.getPosition());
            telemetry.update();
        }
    }
}
