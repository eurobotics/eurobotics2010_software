#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include <aversive.h>
#include <aversive/error.h>

#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>

#ifndef HOST_VERSION
#error only for host
#endif

#define M_2PI (M_PI*2)
#define E_USER_STRAT 200

#define ROBOT_X 1000
#define ROBOT_Y 1000
#define ROBOT_A 0.5   /* radian */
#define ROBOT_A_DEG ((int)((ROBOT_A*180)/3.14159))

#define OPP_X 2000
#define OPP_Y 1500

#define DST_X 1500
#define DST_Y 2000

#define PLAYGROUND_X_MIN 350
#define PLAYGROUND_X_MAX 2650
#define PLAYGROUND_Y_MIN 300
#define PLAYGROUND_Y_MAX 1800

#define PLAYGROUND_X_CENTER 1500
#define PLAYGROUND_Y_CENTER 1100

#define N_CORNS            18
#define CORN_EDGE_NUMBER   6
#define CORN_RADIUS        (243)
#define OFFSET_AVOID_RAMPE 50


struct corn_t{
   int16_t x;
   int16_t y;
   uint8_t oa_flag;
   uint8_t special_poly;
};

struct corn_t corn[18] = {
#if 1	
	[0] = { 150+450*0, 2100-(128+250*1), 0, 0},
	[1] = { 150+450*0, 2100-(128+250*3), 0, 0},
	[2] = { 150+450*0, 2100-(128+250*5), 0, 0},

	[3] = { 150+450*1, 2100-(128+250*0), 0, 0},
	[4] = { 150+450*1, 2100-(128+250*2), 1, 0},
	[5] = { 150+450*1, 2100-(128+250*4), 1, 2},

	[6] = { 150+450*2, 2100-(128+250*1), 1, 1},
	[7] = { 150+450*2, 2100-(128+250*3), 1, 0},

	[8] = { 150+450*3, 2100-(128+250*0), 0, 0},
	[9] = { 150+450*3, 2100-(128+250*2), 1, 0},

	[10] = { 3000-(150+450*2), 2100-(128+250*1), 1, 1},
	[11] = { 3000-(150+450*2), 2100-(128+250*3), 1, 0},

	[12] = { 3000-(150+450*1), 2100-(128+250*0), 0, 0},
	[13] = { 3000-(150+450*1), 2100-(128+250*2), 1, 0},
	[14] = { 3000-(150+450*1), 2100-(128+250*4), 1, 3},

	[15] = { 3000-(150+450*0), 2100-(128+250*1), 0, 0},
	[16] = { 3000-(150+450*0), 2100-(128+250*3), 0, 0},
	[17] = { 3000-(150+450*0), 2100-(128+250*5), 0, 0},
#endif

#if 0
//	[0] = { 150+450*0, 2100-(128+250*1)}, /* 0 */
//	[1] = { 150+450*0, 2100-(128+250*3)}, /* 0 */
//	[2] = { 150+450*0, 2100-(128+250*5)}, /* 0 */

//	[3] = { 150+450*1, 2100-(128+250*0)}, /* 0 */
	[0] = { 150+450*1, 2100-(128+250*2)}, /* 0 */
	[1] = { 150+450*1, 2100-(128+250*4)}, /* 0 */

	[2] = { 150+450*2, 2100-(128+250*1)}, /* 0 */
	[3] = { 150+450*2, 2100-(128+250*3)}, /* 0 */

//	[8] = { 150+450*3, 2100-(128+250*0)}, /* 0 */
	[4] = { 150+450*3, 2100-(128+250*2)}, /* 0 */

	[5] = { 3000-(150+450*2), 2100-(128+250*1)}, /* 0 */
	[6] = { 3000-(150+450*2), 2100-(128+250*3)}, /* 0 */

//	[12] = { 3000-(150+450*1), 2100-(128+250*0)}, /* 0 */
	[7] = { 3000-(150+450*1), 2100-(128+250*2)}, /* 0 */
	[8] = { 3000-(150+450*1), 2100-(128+250*4)}, /* 0 */

//	[15] = { 3000-(150+450*0), 2100-(128+250*1)}, /* 0 */
//	[16] = { 3000-(150+450*0), 2100-(128+250*3)},
//	[17] = { 3000-(150+450*0), 2100-(128+250*5)},
#endif
};



int16_t robot_x = ROBOT_X;
int16_t robot_y = ROBOT_Y;
double robot_a = ROBOT_A;
int16_t robot_a_deg = ROBOT_A_DEG;

int16_t opp_x = OPP_X;
int16_t opp_y = OPP_Y;

int16_t dst_x = DST_X;
int16_t dst_y = DST_Y;

uint8_t num_corn_in_path;
uint8_t corn_in_path_flag[N_CORNS];

double norm(double x, double y)
{
	return sqrt(x*x + y*y);
}

/* return the distance between two points */
int16_t distance_between(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	int32_t x,y;
	x = (x2-x1);
	x = x*x;
	y = (y2-y1);
	y = y*y;
	return sqrt(x+y);
}

void rotate(double *x, double *y, double rot)
{
	double l, a;
	
	l = norm(*x, *y);
	a = atan2(*y, *x);

	a += rot;
	*x = l * cos(a);
	*y = l * sin(a);
}


void set_rotated_pentagon(poly_t *pol, int16_t radius,
			      int16_t x, int16_t y, uint8_t special)
{

	double c_a, s_a;
	uint8_t i;
	double px1, py1, px2, py2;
	double a_rad;

	//a_rad = atan2(y - robot_y, x - robot_x);
   a_rad = 0;

	/* generate pentagon  */
	c_a = cos(-2*M_PI/CORN_EDGE_NUMBER);
	s_a = sin(-2*M_PI/CORN_EDGE_NUMBER);

	/*
	px1 = radius;
	py1 = 0;
	*/
	px1 = radius * cos(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));
	py1 = radius * sin(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));


	for (i = 0; i < CORN_EDGE_NUMBER; i++){
		
		if(special == 1 && (i==0||i==1||i==2))
		   oa_poly_set_point(pol, x + px1, PLAYGROUND_Y_MAX+2, i);
		else if(special == 2 && (i==3))
		   oa_poly_set_point(pol, x + px1, y + py1 - OFFSET_AVOID_RAMPE, i);
		else if(special == 3 && (i==5))
		   oa_poly_set_point(pol, x + px1, y + py1 - OFFSET_AVOID_RAMPE, i);
		else
		   oa_poly_set_point(pol, x + px1, y + py1, i);
		
		px2 = px1*c_a + py1*s_a;
		py2 = -px1*s_a + py1*c_a;

		px1 = px2;
		py1 = py2;
	}

}

void set_rotated_pentagon_pts(point_t *pt, int16_t radius,
			      int16_t x, int16_t y, uint8_t special)
{

	double c_a, s_a;
	uint8_t i;
	double px1, py1, px2, py2;
	double a_rad;

	//a_rad = atan2(y - robot_y, x - robot_x);
   a_rad = 0;

	/* generate pentagon  */
	c_a = cos(-2*M_PI/CORN_EDGE_NUMBER);
	s_a = sin(-2*M_PI/CORN_EDGE_NUMBER);

	/*
	px1 = radius;
	py1 = 0;
	*/
	px1 = radius * cos(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));
	py1 = radius * sin(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));


	for (i = 0; i < CORN_EDGE_NUMBER; i++){
	
	   if(special == 1 && (i==0||i==1||i==2)){
		   pt[i].x = x + px1;
		   pt[i].y = PLAYGROUND_Y_MAX+2;
		}
		else if(special == 2 && (i==3)){
		   pt[i].x = x + px1;
		   pt[i].y = y + py1 - OFFSET_AVOID_RAMPE;
      }
		else if(special == 3 && (i==5)){
		   pt[i].x = x + px1;
		   pt[i].y = y + py1 - OFFSET_AVOID_RAMPE;
      }
		else{	
		   pt[i].x = x + px1;
		   pt[i].y = y + py1;		
      }

	   px2 = px1*c_a + py1*s_a;
	   py2 = -px1*s_a + py1*c_a;
      
		px1 = px2;
		py1 = py2;
	}

}

void set_rhombus(poly_t *pol,
               int16_t w, int16_t l,
			      int16_t x, int16_t y)
{

   oa_poly_set_point(pol, x, y+l, 0);
   oa_poly_set_point(pol, x-w, y, 1);
   oa_poly_set_point(pol, x, y-l, 2);
   oa_poly_set_point(pol, x+w, y, 3);
}

void set_rotated_poly(poly_t *pol, int16_t w, int16_t l,
		      int16_t x, int16_t y)
{
	double tmp_x, tmp_y;
	//double a_rad;

	//a_rad = atan2(y - robot_y, x - robot_x);

	/* point 1 */
	tmp_x = w;
	tmp_y = l;
	//rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 0);
	
	/* point 2 */
	tmp_x = -w;
	tmp_y = l;
	//rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 1);
	
	/* point 3 */
	tmp_x = -w;
	tmp_y = -l;
	//rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 2);
	
	/* point 4 */
	tmp_x = w;
	tmp_y = -l;
	//rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 3);
}



void set_corns_poly_in_path(poly_t **pol, int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
  uint8_t i;
  int16_t ret;
  poly_t poly_corn;
  point_t poly_corn_pts[CORN_EDGE_NUMBER];
	point_t init_pt, dst_pt, intersect_corn_pt;
   
  init_pt.x = x0;
	init_pt.y = y0;
   
  dst_pt.x = x1;
  dst_pt.y = y1;

  poly_corn.l = CORN_EDGE_NUMBER;
  poly_corn.pts = poly_corn_pts;
     
   
	for(i=0; i<N_CORNS; i++)
	{	
	   if(corn[i].oa_flag)
	   {      
	      set_rotated_pentagon_pts(poly_corn_pts, CORN_RADIUS, 
	                               corn[i].x, corn[i].y, corn[i].special_poly);

         ret = is_crossing_poly(init_pt, dst_pt, &intersect_corn_pt, &poly_corn);
                              
         if(ret==1 && corn_in_path_flag[i]==0)
		   {
		      corn_in_path_flag[i] = 1;
		      num_corn_in_path++;

           *(pol+i) = oa_new_poly(CORN_EDGE_NUMBER);
            set_rotated_pentagon(*(pol+i), CORN_RADIUS,
                                 corn[i].x, corn[i].y, corn[i].special_poly);            

            //*(pol+i) = oa_new_poly(4);
	         //set_rhombus(*(pol+i), 440, 240, corn[i].x, corn[i].y);
	         
	         NOTICE(E_USER_STRAT,"num_corn_in_path %d", num_corn_in_path);
	      }
	   }
	}
	

 
}


#define RAMPE_X (1500)
#define RAMPE_Y (PLAYGROUND_Y_MIN)
void set_rampe_poly(poly_t *pol)
{
 //   oa_poly_set_point(pol, 740-220, PLAYGROUND_Y_MIN-2, 0); 
//   oa_poly_set_point(pol, 3000-(740-220), PLAYGROUND_Y_MIN-2, 1);
//   oa_poly_set_point(pol, 3000-(740-220), 2100-(1600-11-150), 2);
//   oa_poly_set_point(pol, 740-220, 2100-(1600-11-150), 3);

   oa_poly_set_point(pol, 650, PLAYGROUND_Y_MIN-2, 0); 
   oa_poly_set_point(pol, 3000-(650), PLAYGROUND_Y_MIN-2, 1);
   oa_poly_set_point(pol, 3000-(650), 700, 2);
   oa_poly_set_point(pol, 650, 700, 3);
}


/* /!\ half size */
#define O_WIDTH  360
#define O_LENGTH 400

void set_opponent_poly(poly_t *pol, int16_t w, int16_t l)
{
	int16_t x, y;
	x = opp_x;
	y = opp_y;
	DEBUG(E_USER_STRAT, "oponent at: %d %d", x, y);
	
	/* place poly even if invalid, because it's -100 */
	set_rotated_poly(pol, w, l, x, y);
}

/* don't care about polygons further than this distance for
 * strat_escape	//set_central_disc_poly(pol_disc);
 */
#define ESCAPE_POLY_THRES 1000

/* don't reduce opp if opp is too far */
#define REDUCE_POLY_THRES 600

/* has to be longer than any poly */
#define ESCAPE_VECT_LEN 3000


/* go in playground, loop until out of poly */
/* XXX return end timer??? */
static int8_t go_in_area(void)
{
	point_t poly_pts_area[4];
	poly_t poly_area;

	point_t robot_pt, center_pt, dst_pt;

	robot_pt.x = robot_x;
	robot_pt.y = robot_y;
	center_pt.x = PLAYGROUND_X_CENTER;
	center_pt.y = PLAYGROUND_Y_CENTER;

	/* Go in playground */
	if (!is_in_boundingbox(&robot_pt)){
		NOTICE(E_USER_STRAT, "not in playground %d, %d", robot_x, robot_y);

		poly_area.l = 4;
		poly_area.pts = poly_pts_area;
		poly_pts_area[0].x = PLAYGROUND_X_MIN;
		poly_pts_area[0].y = PLAYGROUND_Y_MIN;

		poly_pts_area[1].x = PLAYGROUND_X_MAX;
		poly_pts_area[1].y = PLAYGROUND_Y_MIN;

		poly_pts_area[2].x = PLAYGROUND_X_MAX;
		poly_pts_area[2].y = PLAYGROUND_Y_MAX;

		poly_pts_area[3].x = PLAYGROUND_X_MIN;
		poly_pts_area[3].y = PLAYGROUND_Y_MAX;

		
		is_crossing_poly(robot_pt, center_pt, &dst_pt, &poly_area);
		NOTICE(E_USER_STRAT, "pt dst %d, %d", dst_pt.x, dst_pt.y);
		
		/* XXX */
		robot_pt.x = dst_pt.x;
		robot_pt.y = dst_pt.y;

		robot_x = dst_pt.x;
		robot_y = dst_pt.y;

		NOTICE(E_USER_STRAT, "GOTO %d,%d",
		       dst_pt.x, dst_pt.y);

		return 1;
	}

	return 0;
}


/*
 * Escape from polygons if needed.
 */
static int8_t escape_from_poly(poly_t *pol_rampe,
			       int16_t opp_x, int16_t opp_y, 
			       int16_t opp_w, int16_t opp_l, 
			       poly_t *pol_opp)
{
	uint8_t in_opp = 0, in_corn = 0, in_rampe = 0;
	double escape_dx = 0, escape_dy = 0;
	double corn_dx = 0, corn_dy = 0;
	double opp_dx = 0, opp_dy = 0;
	double rampe_dx = 0, rampe_dy = 0;
	double len;
	uint8_t i;
	
	poly_t pol_corn;
	point_t pol_corn_pts[CORN_EDGE_NUMBER];

	point_t robot_pt, opp_pt, corn_pt, rampe_pt, dst_pt;
	point_t intersect_corn_pt, intersect_opp_pt, intersect_rampe_pt;

	robot_pt.x = robot_x;
	robot_pt.y = robot_y;
	opp_pt.x = opp_x;
	opp_pt.y = opp_y;
	rampe_pt.x = RAMPE_X;
   rampe_pt.y = RAMPE_Y;

	/* escape from corns poly */
  pol_corn.l = CORN_EDGE_NUMBER;
  pol_corn.pts = pol_corn_pts;  
	
	for(i=0; i<N_CORNS; i++){
	   if(corn[i].oa_flag){
  
	      corn_pt.x = corn[i].x;
	      corn_pt.y = corn[i].y;

    	   set_rotated_pentagon_pts(pol_corn_pts, CORN_RADIUS, 
    	                                 corn[i].x, corn[i].y, corn[i].special_poly);

	      if (is_point_in_poly(&pol_corn, robot_x, robot_y) == 1){
		      in_corn = 1;
		      break;
	      }
	   }	
	}
	
	/* escape from opponent poly */
	if (is_point_in_poly(pol_opp, robot_x, robot_y) == 1)
		in_opp = 1;

   /* escape from rampe */
   if (is_point_in_poly(pol_rampe, robot_x, robot_y) == 1){
	   in_rampe = 1;
   }	


	if (in_corn == 0 && in_opp == 0 && in_rampe == 0) {
		NOTICE(E_USER_STRAT, "no need to escape");
		return 0;
	}
	
	NOTICE(E_USER_STRAT, "in_corn=%d, in_opp=%d, in_rampe=%d", in_corn, in_opp, in_rampe);
	
	/* process escape vector */
	if (distance_between(robot_x, robot_y, corn[i].x, corn[i].y) < ESCAPE_POLY_THRES) {
		corn_dx = robot_x - corn[i].x;
		corn_dy = robot_y - corn[i].y;
		NOTICE(E_USER_STRAT, " robot is near corn %d: vect=%2.2f,%2.2f",
		       i, corn_dx, corn_dy);
		len = norm(corn_dx, corn_dy);
		if (len != 0) {
			corn_dx /= len;
			corn_dy /= len;
		}
		else {
			corn_dx = 1.0;
			corn_dy = 0.0;
		}
		escape_dx += corn_dx;
		escape_dy += corn_dy;
	}

	if (distance_between(robot_x, robot_y, opp_x, opp_y) < ESCAPE_POLY_THRES) {
		opp_dx = robot_x - opp_x;
		opp_dy = robot_y - opp_y;
		NOTICE(E_USER_STRAT, " robot is near opp: vect=%2.2f,%2.2f",
		       opp_dx, opp_dy);
		len = norm(opp_dx, opp_dy);
		if (len != 0) {
			opp_dx /= len;
			opp_dy /= len;
		}
		else {
			opp_dx = 1.0;
			opp_dy = 0.0;
		}
		escape_dx += opp_dx;
		escape_dy += opp_dy;
	}
	
	if (distance_between(robot_x, robot_y, RAMPE_X, RAMPE_Y) < ESCAPE_POLY_THRES) {
		rampe_dx = robot_x - RAMPE_X;
		rampe_dy = robot_y - RAMPE_Y;
		NOTICE(E_USER_STRAT, " robot is near rampe: vect=%2.2f,%2.2f",
		       rampe_dx, rampe_dy);
		len = norm(rampe_dx, rampe_dy);
		if (len != 0) {
			rampe_dx /= len;
			rampe_dy /= len;
		}
		else {
			rampe_dx = 1.0;
			rampe_dy = 0.0;
		}
		escape_dx += rampe_dx;
		escape_dy += rampe_dy;
	}


	/* normalize escape vector */
	len = norm(escape_dx, escape_dy);
	if (len != 0) {
		escape_dx /= len;
		escape_dy /= len;
	}
	else {
		if (&pol_corn != NULL) {
			/* rotate 90° */
			escape_dx = corn_dy;
			escape_dy = corn_dx;
		}
		else if (pol_opp != NULL) {
			/* rotate 90° */
			escape_dx = opp_dy;
			escape_dy = opp_dx;
		}
		else if (pol_rampe != NULL) {
			/* rotate 90° */
			escape_dx = rampe_dy;
			escape_dy = rampe_dx;
		}
		else { /* should not happen */
			opp_dx = 1.0;
			opp_dy = 0.0;
		}
	}

	NOTICE(E_USER_STRAT, " escape vect = %2.2f,%2.2f",
	       escape_dx, escape_dy);

	/* process the correct len of escape vector */

	dst_pt.x = robot_pt.x + escape_dx * ESCAPE_VECT_LEN;
	dst_pt.y = robot_pt.y + escape_dy * ESCAPE_VECT_LEN;

	NOTICE(E_USER_STRAT, "robot pt %d %d \r\ndst point %d,%d",
	       robot_pt.x, robot_pt.y, 
	       dst_pt.x, dst_pt.y);

	if (in_corn) {
		if (is_crossing_poly(robot_pt, dst_pt, &intersect_corn_pt,
				     &pol_corn) == 1) {
			/* we add 2 mm to be sure we are out of th polygon */
			dst_pt.x = intersect_corn_pt.x + escape_dx * 4;
			dst_pt.y = intersect_corn_pt.y + escape_dy * 4;

			if (is_point_in_poly(pol_opp, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_rampe, dst_pt.x, dst_pt.y) != 1){


				if (!is_in_boundingbox(&dst_pt))
					return -1;
				
				robot_x = dst_pt.x;
				robot_y = dst_pt.y;

				/* XXX */
				NOTICE(E_USER_STRAT, "GOTO %d,%d (%d %d)",
				       robot_x, robot_y,
				       is_point_in_poly(pol_opp, robot_x, robot_y),
				       is_point_in_poly(pol_rampe, robot_x, robot_y));
				return 0;
			}
		}
	}

	if (in_opp) {
		if (is_crossing_poly(robot_pt, dst_pt, &intersect_opp_pt,
				     pol_opp) == 1) {
			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_opp_pt.x + escape_dx * 2;
			dst_pt.y = intersect_opp_pt.y + escape_dy * 2;


			if ((is_point_in_poly(&pol_corn, dst_pt.x, dst_pt.y) && in_corn) != 1 &&
			    is_point_in_poly(pol_rampe, dst_pt.x, dst_pt.y) != 1 ){


				if (!is_in_boundingbox(&dst_pt))
					return -1;
				
				robot_x = dst_pt.x;
				robot_y = dst_pt.y;

				/* XXX */
				NOTICE(E_USER_STRAT, "GOTO %d,%d (%d %d)",
				       robot_x, robot_y,
				       is_point_in_poly(&pol_corn, robot_x, robot_y),
				       is_point_in_poly(pol_rampe, robot_x, robot_y));

				return 0;
			}
		}
	}
	
	if (in_rampe) {
		if (is_crossing_poly(robot_pt, dst_pt, &intersect_rampe_pt,
				     pol_rampe) == 1) {
			// we add 2 mm to be sure we are out of th polygon 
			dst_pt.x = intersect_rampe_pt.x + escape_dx * 2;
			dst_pt.y = intersect_rampe_pt.y + escape_dy * 2;
			if (is_point_in_poly(pol_opp, dst_pt.x, dst_pt.y) != 1 &&
			    (is_point_in_poly(&pol_corn, dst_pt.x, dst_pt.y) && in_corn) != 1) {

				if (!is_in_boundingbox(&dst_pt))
					return -1;
				
				robot_x = dst_pt.x;
				robot_y = dst_pt.y;

				// XXX
				NOTICE(E_USER_STRAT, "GOTO %d,%d (%d %d)",
				       robot_x, robot_y,
				       is_point_in_poly(pol_opp, robot_x, robot_y),
				       is_point_in_poly(&pol_corn, robot_x, robot_y));
				return 0;
			}
		}
	}

	/* should not happen */
	return -1;
}




static int8_t goto_and_avoid(int16_t x, int16_t y)
{
	int8_t len = -1, i, num_corn_in_path_save;
	point_t *p;
	poly_t *pol_corn_in_path[N_CORNS], *pol_opp, *pol_rampe;
   
   poly_t pol_corn;
   point_t pol_corn_pts[CORN_EDGE_NUMBER];
   
	int16_t posx, posy;
	int8_t ret;
	int16_t opp_w;
	int16_t opp_l;
	point_t p_dst;

	opp_w = O_WIDTH;
	opp_l = O_LENGTH;

	posx = robot_x;
	posy = robot_y;
	
	oa_init();

   set_corns_poly_in_path(&pol_corn_in_path[0], posx, posy, x, y);
   
   pol_rampe = oa_new_poly(4);
   set_rampe_poly(pol_rampe);

	pol_opp = oa_new_poly(4);
	set_opponent_poly(pol_opp, O_WIDTH, O_LENGTH);

	go_in_area();

	p_dst.x = x;
	p_dst.y = y;
	if (!is_in_boundingbox(&p_dst)) {
		NOTICE(E_USER_STRAT, " dst is not in playground");
		return -1;
	}


   pol_corn.l = CORN_EDGE_NUMBER;
   pol_corn.pts = pol_corn_pts;

	for(i=0; i<N_CORNS; i++){ 
	   if(corn[i].oa_flag){
	   
    	   set_rotated_pentagon_pts(pol_corn_pts, CORN_RADIUS, 
    	                            corn[i].x, corn[i].y, corn[i].special_poly);
    	                                    
	      if (is_point_in_poly(&pol_corn, x, y)) {
		      NOTICE(E_USER_STRAT, " dst is in corn %d", i);
		      return -1;
	      }
	   }
   }

	if (is_point_in_poly(pol_opp, x, y)) {
		NOTICE(E_USER_STRAT, " dst is in opp");
		return -1;
	}
	
   if (is_point_in_poly(pol_rampe, x, y)) {
	   NOTICE(E_USER_STRAT, " dst is in rampe");
	   return -1;
   }



	while (opp_w && opp_l) {

		ret = escape_from_poly( &pol_rampe[0],
				                   opp_x, opp_y, 
				                   opp_w, opp_l, 
				                   pol_opp);  

      if (ret == 0) {

repeat_oa:      
			oa_reset();
			oa_start_end_points(robot_x, robot_y, x, y);
			oa_dump();
	
			len = oa_process();
			if (len > 0){
				/* any corn else in path? */
				num_corn_in_path_save = num_corn_in_path;
            p = oa_get_path();
            set_corns_poly_in_path(&pol_corn_in_path[0], robot_x, robot_y, p->x, p->y);
         	for (i=0 ; i<(len-1) ; i++) {
               set_corns_poly_in_path(&pol_corn_in_path[0], p->x, p->y, (p+1)->x, (p+1)->y);
               p++;
				}
								
				if(num_corn_in_path_save != num_corn_in_path){
   				NOTICE(E_USER_STRAT,"repeat oa");   				
				   goto repeat_oa;
				}
				else
				   break;
			}
			else 
			if(len >= 0)
			   break;
		}
		
		if (distance_between(robot_x, robot_y, opp_x, opp_y) < REDUCE_POLY_THRES ) {
			if (opp_w == 0)
				opp_l /= 2;
			opp_w /= 2;
		}
		else {
			NOTICE(E_USER_STRAT, "oa_process() returned %d", len);

			// XXX
			return -1;
		}

		NOTICE(E_USER_STRAT, "reducing opponent %d %d", opp_w, opp_l);
		set_opponent_poly(pol_opp, opp_w, opp_l);
	}
	
	p = oa_get_path();
	for (i=0 ; i<len ; i++) {
		DEBUG(E_USER_STRAT, "With avoidance %d: x=%d y=%d", i, p->x, p->y);
		p++;
	}

	return 0;
}

/* log function, add a command to configure
 * it dynamically */
void mylog(struct error * e, ...) 
{
	va_list ap;

	va_start(ap, e);

	vfprintf(stdout, e->text, ap);
	printf_P(PSTR("\r\n"));
	va_end(ap);
}

#ifdef HOST_VERSION
int main(int argc, char **argv)
#else
int main(void)
#endif
{
   uint8_t i;
   
#ifdef HOST_VERSION
	if (argc != 8) {
		printf("bad args\n");
		return -1;
	}
	robot_x = atoi(argv[1]);
	robot_y = atoi(argv[2]);
	robot_a_deg = atoi(argv[3]);
	robot_a = (((double)robot_a_deg*M_PI)/3.14159);
	dst_x = atoi(argv[4]);
	dst_y = atoi(argv[5]);
	opp_x = atoi(argv[6]);
	opp_y = atoi(argv[7]);
#endif
   
   /* corns in path */
   num_corn_in_path = 0;
   for(i=0; i<N_CORNS; i++)
      corn_in_path_flag[i] = 0;

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);
	
	polygon_set_boundingbox(PLAYGROUND_X_MIN, PLAYGROUND_Y_MIN,
				PLAYGROUND_X_MAX, PLAYGROUND_Y_MAX);

	DEBUG(E_USER_STRAT, "robot at: %d %d %d", robot_x, robot_y, robot_a_deg);
	goto_and_avoid(dst_x, dst_y);

	return 0;
}
