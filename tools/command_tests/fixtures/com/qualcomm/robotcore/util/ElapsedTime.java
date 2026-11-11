package com.qualcomm.robotcore.util;
public class ElapsedTime {
 public static long now; private long start=now;
 public void reset(){start=now;} public double milliseconds(){return now-start;}
}
