package com.example.meepmeeprrvisualizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SampleAuto {
    static double timemultiplier = 1;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60*timemultiplier, 60*timemultiplier, Math.toRadians(180)*timemultiplier, Math.toRadians(180)*timemultiplier, 14)
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

      myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(30, 60, Math.PI))
                 .waitSeconds(1/timemultiplier)
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
                              .lineToY(55)
              /*turnTo(0)
              .lineToX(20)
              .turnTo(Math.PI)
              .lineToX(30)
              .setTangent(Math.PI)
              .splineTo(new Vector2d(10,50),-Math.PI/2)*/
                          .build());

        /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-30, 60, Math.PI))
                //.waitSeconds(1)
                //.setTangent(0)
                //.splineTo(new Vector2d)

                .build());*/

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}