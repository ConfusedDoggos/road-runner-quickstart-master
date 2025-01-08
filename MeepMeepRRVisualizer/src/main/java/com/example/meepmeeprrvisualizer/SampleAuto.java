package com.example.meepmeeprrvisualizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SampleAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100,75, Math.toRadians(360), Math.toRadians(270), 14)
                .build();

        /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(30, 60, Math.PI))
                .waitSeconds(2)
                .setTangent(0)
                .splineTo(new Vector2d(55, 55), Math.PI / 4)
                .waitSeconds(5)
                //.turnTo(-Math.PI / 2)
                .setTangent(-3 * Math.PI / 4)
                .splineTo(new Vector2d(50, 40), -Math.PI / 2)
                .waitSeconds(3)
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(55, 55), Math.PI / 4)
                .waitSeconds(5)
                .setTangent(-3 * Math.PI / 4)
                .splineTo(new Vector2d(60, 40), -Math.PI / 2)
                .waitSeconds(3)
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(55, 55), Math.PI / 4)
                .waitSeconds(2)
                .lineToY(55)
                .build());*/

      //myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                 /*.waitSeconds(1/timemultiplier)
              .strafeToConstantHeading(new Vector2d(30,55))
              .setTangent(0)
                  .splineTo(new Vector2d(55, 55), Math.PI / 4)
                  .waitSeconds(5/timemultiplier)
                  //Lift slide, deliver, bring back wrist (if possible, collapse during next move
                  .turnTo(-Math.PI / 2)
                  .lineToY(40)
                  .strafeToConstantHeading(new Vector2d(50, 40))
                  .waitSeconds(3/timemultiplier)
                  //Extend Horizontal arm (after stopping??), bring down wrist to pick up sample on ground. Transfer can likely occur on way back
                  .setTangent(Math.PI / 2)
                  .splineTo(new Vector2d(55, 55), Math.PI / 4)
                              .waitSeconds(2/timemultiplier)
                              .lineToY(55)*/
              /*turnTo(0)
              .lineToX(20)
              .turnTo(Math.PI)
              .lineToX(30)
              .setTangent(Math.PI)
              .splineTo(new Vector2d(10,50),-Math.PI/2)*/
                  /*.lineToX(10)
                      .turnTo(Math.toRadians(90))
                      .lineToY(10)
                      .turnTo(Math.toRadians(270))
                      .setTangent(Math.toRadians(270))
                      .splineTo(new Vector2d(0,0),Math.toRadians(180))
                          .build());

        /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-30, 60, Math.PI))
                //.waitSeconds(1)
                //.setTangent(0)
                //.splineTo(new Vector2d)

                .build());*/
        //Specimen Auto Path
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-5, 62, Math.toRadians(90)))
                .waitSeconds(3)
                .lineToY(32,null,new ProfileAccelConstraint(-70,100))
                .waitSeconds(1.5)
                .lineToY(35)
                .strafeTo(new Vector2d(-35,35),null,new ProfileAccelConstraint(-70,100))
                .strafeTo(new Vector2d(-35,12),null,new ProfileAccelConstraint(-70,100))
                .strafeTo(new Vector2d(-45,12),null,new ProfileAccelConstraint(-70,100))
                .strafeTo(new Vector2d(-45,55),null,new ProfileAccelConstraint(-70,100))
                /*.setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-50,12),Math.toRadians(180),null,new ProfileAccelConstraint(-70,100))
                .setTangent(Math.toRadians(90))
                .lineToY(55,null,new ProfileAccelConstraint(-70,100))*/
                .setTangent(Math.toRadians(90))
                .turnTo(Math.toRadians(270))
                .lineToY(62)
                .waitSeconds(1)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-2,32,Math.toRadians(90)),Math.toRadians(270))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-35,62,Math.toRadians(270)),Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(2,32,Math.toRadians(90)),Math.toRadians(270))
                .waitSeconds(1.5)
                        .lineToY(35)
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}