/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constants.AutoMods;
import org.firstinspires.ftc.teamcode.subsystem.drive.OldDrive;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;
import org.firstinspires.ftc.teamcode.vision.SpikeDetectionNew;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous (name = "Extendo Blue Close")
public class AutoExtendoBlueClose extends LinearOpMode{

    private SpikeDetectionNew spikeDetect;
    private VisionPortal portal;
    OldDrive drive;
    LinearSlide slide;
    AutoExtendoCloseTemplates templates;

    @Override
    public void runOpMode() throws InterruptedException {
        AutoMods.teamRed = false;
        AutoMods.isFar = false;
        slide = new LinearSlide(hardwareMap);
        drive = new OldDrive(hardwareMap, this);
        drive.botHeading = 0;
        templates = new AutoExtendoCloseTemplates(slide,drive,false);


        spikeDetect = new SpikeDetectionNew();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(spikeDetect)
                .build();

        slide.turnPlaceAuto();
        slide.grabAll();
        slide.turnRot(slide.droneServo, 1);
        while (opModeInInit()) {
            telemetry.addData("Right Claw", "Yellow Pixel");
            telemetry.addData("Left Claw", "Purple Pixel");
            telemetry.addData("Block Location", spikeDetect.getPos());
            telemetry.update();
        }


        waitForStart();
        drive.imu.resetYaw();
        SpikeDetectionNew.Position position = spikeDetect.getPos();
        portal.close();
        slide.turnFloorEx();
        templates.purplePixel(position);
        templates.yellowPixelIn(position);
        templates.parkCenter(position);
        slide.setAutoPos(0,0);
    }
//
//    public void purplePixel(SpikeDetectionNew.Position position){
//        drive.rotateAndMoveInches(0,10,0,0.5,0.5);
//        switch (position){
//            case RIGHT:
//                drive.rotateAndMoveInches(-25, 0,0,0,0.4);
//                slide.setAutoExtendo((int)(18/slide.INCHESPERTICK));
//                slide.turnFloorEx();
//                wait(0.25);
//                slide.ungrabL();
//                wait(0.25);
//                slide.turnPlaceEx();
//                slide.setAutoExtendo(0);
//                wait(0.5);
//                drive.rotateAndMoveInches(0,0,0,0,0.5);
//                drive.rotateAndMoveInches(0,-2.5,0,0.5,0.75);
//                break;
//            case CENTER:
//                slide.setAutoExtendo((int)(22.0/slide.INCHESPERTICK));
//                slide.turnFloorEx();
//                wait(0.25);
//                slide.ungrabL();
//                wait(0.25);
//                slide.turnPlaceEx();
//                slide.setAutoExtendo(0);
//                wait(0.5);
//                drive.rotateAndMoveInches(0,0,0,0,0.5);
//                drive.rotateAndMoveInches(0,-2,0,0.5,0.75);
//                break;
//            case LEFT:
//                drive.rotateAndMoveInches(20, 0,0,0,0.4);
//                slide.setAutoExtendo((int)(18/slide.INCHESPERTICK));
//                slide.turnFloorEx();
//                wait(0.25);
//                slide.ungrabL();
//                wait(0.25);
//                slide.turnPlaceEx();
//                slide.setAutoExtendo(0);
//                wait(0.5);
//                drive.rotateAndMoveInches(0,0,0,0,0.5);
//                drive.rotateAndMoveInches(0,-3.5,0,0.5,0.75);
//                break;
//        }
//    }
//
//    public void yellowPixelIn(SpikeDetectionNew.Position position){
//        drive.rotateAndMoveInches(0,0,39,0.5,0.5);
//        switch(position){
//            case LEFT:
//                drive.rotateAndMoveInches(90,30.5,0,0.5,0.25);
//                slide.setArmPos(350, LinearSlide.LOW_ROT-10);
//                slide.turnPlaceEx();
//                drive.rotateAndMoveInches(90,0,16,0.25,0.2);
//                wait(0.75);
//                break;
//
//            case RIGHT:
//                drive.rotateAndMoveInches(90,17,0,0.5,0.25);
//                slide.setArmPos(350, LinearSlide.LOW_ROT-10);
//                slide.turnPlaceEx();
//                drive.rotateAndMoveInches(90,0,16,0.25,0.2);
//                wait(0.75);
//                break;
//
//            case CENTER:
//                drive.rotateAndMoveInches(90,22,0,0.5,0.25);
//                slide.setArmPos(300, LinearSlide.LOW_ROT-5);
//                slide.turnPlaceEx();
//                drive.rotateAndMoveInches(90,0,17,0.25,0.2);
//                wait(0.5);
//                break;
//        }
//        slide.ungrabR();
//        wait(0.5);
//        slide.setAutoPos(0,0);
//    }
//    public void parkCenter(SpikeDetectionNew.Position position){
//        switch(position){
//            case LEFT:
//                drive.rotateAndMoveInches(90,0,-10,0.4,0.2);
//                drive.rotateAndMoveInches(90,20,0,0.4,0.2);
////                drive.rotateAndMoveInches(0,0,17,0.4,0.4);
//                break;
//            case CENTER:
//                drive.rotateAndMoveInches(90,27,-10,0.4,0.2);
////                drive.rotateAndMoveInches(0,0,17,0.4,0.4);
//                break;
//            case RIGHT:
//                drive.rotateAndMoveInches(90,36,-10,0.4,0.2);
////                drive.rotateAndMoveInches(0,0,17,0.4,0.4);
//                break;
//        }
//        drive.rotateAndMoveInches(0,0,17,0.4,0.4);
//        slide.setAutoPos(0,0);
//    }
//    public void wait (double t) {
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        while (timer.time() < t) {idle();}
//    }
}
