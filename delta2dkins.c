/********************************************************************
* Description: deltakins.c
*   Kinematics for 3 axis Delta machine
*   Derived from a work of mzavatsky at Trossen Robotics
*                          at forums.trossenrobotics.com
* Author: Jozsef Papp / hg5bsd
*    and: István Ábel at kvarc.extra.hu
* License: coding and fitting it into EMC2 from our part is GPL alike
*          but we won't place the code itself under GPL because we
*          don't know if this would hurt or not the delta robot
*          inventors rights, if you are sure that it doesn't hurt,
*          then you are free to place it under GPL ( in this case place
*          your name in the comment lines, and keep original comments )
* System: realtime Linux, EMC2
* Copyright (c) 2010 All rights reserved.
* Last change:                   2010.07.14.14.28.
* modified by cncbasher	 compiles correctly in 2.5 and dev  2013.06.20
********************************************************************/
#include "kinematics.h"             /* these decls */
#include "rtapi_math.h"

 /* robot geometry  /Ref - mzavatsky: Delta robot kinematics/

e = side length of end effector triangle, middle arm - "re"
f = side length of base triangle, middle drive joints - "rf"
re = length of end effector arm
rf = length of drive arm

sample:
e = 115.0;
f = 457.3;
re = 232.0;
rf = 112.0;  */

#ifdef RTAPI
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

struct haldata {
    hal_float_t *e, *f, *re, *rf;
} *haldata = 0;

#define delta_e (*(haldata->e))
#define delta_f (*(haldata->f))
#define delta_re (*(haldata->re))
#define delta_rf (*(haldata->rf))

#else
double delta_e, delta_f, delta_re, delta_rf;
#endif

 /* trigonometric constants */
 const double sqrt3 = 1.7320508075688772935274463415059;   /* sqrt(3.0);*/
#ifndef PI
#define PI 3.14159265358979323846
#endif
 const double sin120 = 0.86602540378443864676372317075294; /* (sqrt3)/2;*/
 const double cos120 = -0.5;
 const double tan60 = 1.7320508075688772935274463415059;   /* sqrt3;*/
 const double sin30 = 0.5;
 const double tan30 = 0.57735026918962576450914878050196;  /* 1/(sqrt3);*/


 /* forward kinematics: (joints[0], joints[1], joints[2]) -> (pos->tran.x,
pos->tran.y, pos->tran.z)
 // returned status: 0=OK, -1=non-existing position*/

int kinematicsForward( const double *joints,
		      EmcPose *pos,
		      const KINEMATICS_FORWARD_FLAGS *fflags,
		      KINEMATICS_INVERSE_FLAGS *iflags)

 {
     //
// DESCRIPTION
//  Solves the Direct Kinematics problem of a delta-2 robot. 
// PARAMETERS
//  KinemParams [IN] : Kinematics parameters [rf,lf,le,re].
//  Joints      [IN] : Position [j1,j2]' in JCS (radians).
//  TCP0       [OUT] : Pose [x,y,z]' in RCS.
// RETURN
//  Ret : Success (1) or error (<0)
//
double rf,lf,le,re,theta1,theta2,ac,d1,xG1p,zG1p,xG2p,zG2p;
double G2p,x0,z0,r0,x1,y1,r1,dx,dy,d,a,x2,y2;
  // Inputs
  // --------------------------------------------
  
  rf = delta_rf;
  lf = delta_f
  le = delta_e
  re = delta_re
  
  theta1 = joints(0);
  theta2 = Joints(2);
  
  // Solve kinematics
  // --------------------------------------------
  
  ac = rf - re;
  
  // 1. Calcualte G1' and G2'
  // ///////////////////////////////
  
  // 1.1. G1'
  
  d1   = ac + lf*cos(theta1);  
  xG1p = d1;
  zG1p = -lf*sin(theta1);
   
  G1p = [xG1p zG1p];
  
  // 1.2. G2' 
  
  d2   = ac + lf*cos(theta2);  
  xG2p = -d2;
  zG2p = -lf*sin(theta2);
  
  G2p = [xG2p zG2p];
   
  // 2. TCP-0 as intersection of 2 circles
  // ///////////////////////////////
  
  // Center and radius of the circles
  x0 = xG1p;  y0 = zG1p;  r0 = le;
  x1 = xG2p;  y1 = zG2p;  r1 = le;
  
  // dx and dy are the vertical and horizontal distances between
  // the circle centers, they are the components of d.
  dx = x1 - x0;
  dy = y1 - y0;
  
  // Determine the straight-line distance between the centers. 
  d = sqrt( (dy*dy) + (dx*dx) );

  // Check for solvability
  if d > (r0 + r1)
    // ERROR - No solution. circles do not intersect.
    return -1;
  end 

  if d < abs(r0 - r1)    
    // ERROR - No solution. one circle is contained in the other 
    return -2;
  end

  // 'point 2' is the point where the line through the circle
  // intersection points crosses the line between the circle
  // centers.  
  
  // Determine the distance from point 0 to point 2. 
  a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0*d) ;
  
  // Determine the coordinates of point 2. 
  x2 = x0 + (dx * a/d);
  y2 = y0 + (dy * a/d);
  
  // Determine the distance from point 2 to either of the
  // intersection points.
  h = sqrt((r0*r0) - (a*a));
  
  // Now determine the offsets of the intersection points from
  // point 2.
  // Determine the absolute intersection points. 
  
  // Select P1 or P2
  // We have to chose smaller point

  xi_1 = x2 - dy*(h/d);
  yi_1 = y2 + dx*(h/d);
  
  x0 = xi_1;
  z0 = yi_1;
  
  // Return TCP-0
  // --------------------------------------------
     pos->tran.x = x0
    pos->tran.y = 0;
     pos->tran.z = z0;

	
  return 0;
     
    
    //double t, theta1, theta2, theta3, x2, x3, y1, y2, y3, z1, z2, z3, w1, w2, w3, a, b, c, d, a1, a2, b1, b2, dnm;

     //t = (delta_f-delta_e)*tan30/2;

     ///* float dtr = pi/(float)180.0; TO_RAD */

     //theta1 = joints[0] * TO_RAD;
     //theta2 = joints[1] * TO_RAD;
     //theta3 = joints[2] * TO_RAD;

     //y1 = -(t + delta_rf*cos(theta1));
     //z1 = -delta_rf*sin(theta1);

     //y2 = (t + delta_rf*cos(theta2))*sin30;
     //x2 = y2*tan60;
     //z2 = -delta_rf*sin(theta2);

     //y3 = (t + delta_rf*cos(theta3))*sin30;
     //x3 = -y3*tan60;
     //z3 = -delta_rf*sin(theta3);

     //dnm = (y2-y1)*x3-(y3-y1)*x2;

     //w1 = y1*y1 + z1*z1;
     //w2 = x2*x2 + y2*y2 + z2*z2;
     //w3 = x3*x3 + y3*y3 + z3*z3;

     ///* x = (a1*z + b1)/dnm*/
     //a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     //b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

     ///* y = (a2*z + b2)/dnm;*/
     //a2 = -(z2-z1)*x3+(z3-z1)*x2;
     //b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

     ///* a*z^2 + b*z + c = 0*/
     //a = a1*a1 + a2*a2 + dnm*dnm;
     //b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     //c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - delta_re*delta_re);

     ///* discriminant*/
     //d = b*b - (double)4.0*a*c;
     //if (d < 0) return -1; /* non-existing point*/

     //pos->tran.z = -(double)0.5*(b+sqrt(d))/a;
     //pos->tran.x = (a1*pos->tran.z + b1)/dnm;
     //pos->tran.y = (a2*pos->tran.z + b2)/dnm;

     //return 0;

 }




 /* inverse kinematics: (pos->tran.x, pos->tran.y, pos->tran.z) ->
(joints[0], joints[1], joints[2])
    returned status: 0=OK, -1=non-existing position*/
int kinematicsInverse(const EmcPose *pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS *iflags,
		      KINEMATICS_FORWARD_FLAGS *fflags)

 {
     double x0,y0,z0,theta1,theta2,x,z ;
     int status;
     x0 = pos->tran.x;
     y0 = pos->tran.y;
     z0 = pos->tran.z;
     theta1 = 0;
     theta2 = 0;
     x = x0;
     z = z0;
    status  = delta_calcJointAngle(x,0,z,&theta1)
    x = x0*c180 - z0*s180;
    z = z0;
    status  = delta_calcJointAngle(x,0,z,&theta2)
    if (status == 0) joints[0] = theta1;
    if (status == 0) joints[1] = 0;
    if (status == 0) joints[2] = theta2;
    //double x0, y0, z0, theta1, theta2, theta3;
    //int status;
     //x0 = pos->tran.x;
     //y0 = pos->tran.y;
     //z0 = pos->tran.z;

     //theta1 = theta2 = theta3 = 0;

     //status = delta_calcAngleYZ(x0, y0, z0, &theta1);
     //if (status == 0) joints[0] = theta1;
     //if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120,y0*cos120-x0*sin120, z0, &theta2); /*rotate coords to +120 deg*/
     //if (status == 0) joints[1] = theta2;
     //if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120,y0*cos120+x0*sin120, z0, &theta3); /*rotate coords to -120 deg*/
     //if (status == 0) joints[2] = theta3;

     return status;
 }
int delta_calcJointAngle( double x0, double y0, double z0,double *theta)
{
    *theta = 0;
    double x_pos,z_pos,xF,zF,xF,xF;
    double x0,x1,y0,y1,r0,r1,dx,dy;
    double d,a,xi_1,yi_1,xG,zG;
    x_pos = x0;
    z_pos = z0;
    if(!z_pos){
    // ERROR - This is necessary to avoid division by zero.
        return -1;
        *theta = 0;
   }
        xF = delta_rf;
        zF = 0;
        xE = x_pos + delta_re;
        zE = z_pos;
          
  // Calculate Gi point 
  // It is the intersection of circles (Fi,lf) and (Ei,le)  
  // --------------------------------------------
  
  // Parameters of the circles: circle 0 (x0,y0r0) and circle 1 (x1,y1,r1)
        x0 = xF;
        x1 = xE;
        y0 = zF;
        y1 = zE;
        r0 = delta_f;
        r1 = delta_e;
  // dx and dy are the vertical and horizontal distances between
  // the circle centers, they are the components of d.
    dx = x1 - x0;
    dy = y1 - y0;
  // Determine the straight-line distance between the centers.
    d = sqrt((dy * dy) + (dx * dx));
      // Check for solvability
    if(d > (r0+r1){
         // ERROR - No solution. Circles do not intersect.
        return -2;
        theta = 0;
    }
    if (d < abs(r0 - r1))  {  
    // ERROR - No solution. One circle is contained in the other 
    *theta =0;
    return -3;
}
// 'point 2' is the point where the line through the circle
// intersection points crosses the line between the circle
// centers.  
// Determine the distance from point 0 to point 2. 
    a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0*d) ;
      
  // Determine the coordinates of point 2. 
  x2 = x0 + (dx * a/d);
  y2 = y0 + (dy * a/d);
// Determine the distance from point 2 to either of the
// intersection points.
  h = sqrt((r0*r0) - (a*a));
// Now determine the offsets of the intersection points from
// point 2.
// Determine the absolute intersection points. 
// Select P1 or P2
// We must chose smaller point
  xi_1 = x2 - dy*(h/d);
  yi_1 = y2 + dx*(h/d);
  xG = xi_1;   zG = yi_1;

  // Caculate theta using Fi and Gi points
  double angle = atan(-zG,(xG - xF));
  *theta = angle;  
    return 0

 }

 /* implemented for these kinematics as giving joints preference */
int kinematicsHome(EmcPose * world,
		   double *joint,
		   KINEMATICS_FORWARD_FLAGS * fflags,
		   KINEMATICS_INVERSE_FLAGS * iflags)
{
    *fflags = 0;
    *iflags = 0;

    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
}

#ifdef RTAPI
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");


int comp_id;
int rtapi_app_main(void) {
    int res = 0;

    comp_id = hal_init("deltakins");
    if(comp_id < 0) return comp_id;

    haldata = hal_malloc(sizeof(struct haldata));
    if(!haldata) goto error;

    if((res = hal_pin_float_new("deltakins.e", HAL_IO, &(haldata->e),comp_id)) != 0) goto error;
    if((res = hal_pin_float_new("deltakins.f", HAL_IO, &(haldata->f),comp_id)) != 0) goto error;
    if((res = hal_pin_float_new("deltakins.re", HAL_IO, &(haldata->re),comp_id)) != 0) goto error;
    if((res = hal_pin_float_new("deltakins.rf", HAL_IO, &(haldata->rf),comp_id)) != 0) goto error;

    delta_e = delta_f = delta_re = delta_rf = 1.0;

    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
#endif


