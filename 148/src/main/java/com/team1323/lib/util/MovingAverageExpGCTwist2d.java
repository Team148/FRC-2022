// /* GRANT TESTING */

// package com.team1323.lib.util;

// import com.team254.lib.geometry.Twist2d;

// import java.util.ArrayList;

// /**
//  * Helper class for storing and calculating a moving average of the Twist2d class
//  */
// public class MovingAverageExpGCTwist2d {
//     ArrayList<Twist2d> twists = new ArrayList<Twist2d>();
//     private int maxSize;

//     //New EMA fudget factor
//     private double emaFudge = 0.1; // 0.5 means equal bias between cur and prev
    

//     public MovingAverageExpGCTwist2d(int maxSize) {
//         this.maxSize = maxSize;
//     }

//     public synchronized void add(Twist2d twist) {
//         twists.add(twist);
//         if (twists.size() > maxSize) {
//             twists.remove(0);
//         }
//     }

//     public synchronized Twist2d getAverage() {
//         if (getSize() == 0)
//             return Twist2d.identity();

//         double size = getSize();

//         //EMA formula GC test
//         // Get a few twist objects
//         Twist2d twist_cur = twists.get(maxSize-1);
//         Twist2d twist_minus_1 = twists.get(maxSize-2);
//         Twist2d twist_minus_2 = twists.get(maxSize-3);
//         Twist2d twist_minus_3 = twists.get(maxSize-4);
//         Twist2d twist_minus_4 = twists.get(maxSize-5);

//         // Assign x/y/t values for each
//         double dx_current = twist_current.dx;
//         double dy_current = twist_current.dy;
//         double dt_current = twist_current.dtheta;
//         double dx_previous = twist_previous.dx;
//         double dy_previous = twist_previous.dy;
//         double dt_previous = twist_previous.dtheta; 

//         // Calculate EMA values
//         // https://www.investopedia.com/terms/e/ema.asp
//         // (this is not actually a correct EMA calculation... it's a slightly weighted SMA of 2 values)
//         double newEma_X = (dx_current * emaFudge) + (dx_previous * (1-emaFudge));
//         double newEma_Y = (dy_current * emaFudge) + (dy_previous * (1-emaFudge));
//         double newEma_T = (dt_current * emaFudge) + (dt_previous * (1-emaFudge));

//         //Save current run as previous, to be used in EMA formula (no longer used)
//         // prevEma_X = newEma_X;
//         // prevEma_Y = newEma_Y;
//         // prevEma_T = newEma_T;

//         // output
//         return new Twist2d(newEma_X, newEma_Y, newEma_T);
//     }

//     public int getSize() {
//         return twists.size();
//     }

//     public boolean isUnderMaxSize() {
//         return getSize() < maxSize;
//     }

//     public void clear() {
//         twists.clear();
//     }

// }