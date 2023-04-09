#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

//key memory address definitions
#define SDRAM_BASE            0xC0000000
#define FPGA_ONCHIP_BASE      0xC8000000
#define FPGA_CHAR_BASE        0xC9000000
#define LEDR_BASE             0xFF200000
#define HEX3_HEX0_BASE        0xFF200020
#define HEX5_HEX4_BASE        0xFF200030
#define SW_BASE               0xFF200040
#define KEY_BASE              0xFF200050
#define TIMER_BASE            0xFF202000
#define PIXEL_BUF_CTRL_BASE   0xFF203020
#define CHAR_BUF_CTRL_BASE    0xFF203030

//VGA colour definitions
#define WHITE 0xFFFF
#define YELLOW 0xFFE0
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define GREY 0xC618
#define PINK 0xFC18
#define ORANGE 0xFC00
#define BLACK 0x0000

//VGA Resolution definitions
#define RESOLUTION_X 320
#define RESOLUTION_Y 240


//animation constant definitions
#define BOX_LEN 2
#define NUM_BOXES 8
#define SATELLITE_SIZE      3

//Constants for satellite shifting and scalings
#define r_x_shift           64000000
#define r_x_scale           294102
#define r_y_shift           43456477
#define r_y_scale           366528

//structure definitions for vectors
typedef struct _2D_double_vector_struct
{
  double x;
  double y;
} vector; //2d vector

typedef struct _3D_double_vector_struct
{
  double x;
  double y;
  double z;
} vectorX; //3d vector

//VGA subroutine initializations
void clear_box(int x, int y, int size);
void clear_drawn(int x0, int y0, int x1, int y1);
void clear_screen();
void draw_line(int x0, int y0, int x1, int y1, short int line_color);
void draw_body(int x, int y, int size, short int color);
void plot_pixel(int x, int y, short int line_color);
void swap(int* val1, int* val2);
void wait_for_vsync();

//Vector Structure subroutine definitions
void initialize_vector (vector* self);
void initialize_vectorX (vectorX* self);
vector scale_vector (vector self, double scale);
vectorX scale_vectorX (vectorX self, double scale);
vector add_vector (vector a, vector b);
vectorX add_vectorX (vectorX a, vectorX b);
vector sub_vector (vector a, vector b);
vectorX sub_vectorX (vectorX a, vectorX b);
vector rsb_vector (vector a, vector b);
vectorX rsb_vectorX (vectorX a, vectorX b);
double mag_vector (vector self);
double mag_vectorX (vectorX self);
double dot_prod_vector (vector a, vector b);
double dot_prod_vectorX (vectorX a, vectorX b);
vectorX cross_prod_vector (vector a, vector b);
vectorX cross_prod_vectorX (vectorX a, vectorX b);
double angle_vector (vector a, vector b);  //in radians
double angle_vectorX (vectorX a, vectorX b); //in radians
vector unit_vector (vector self);
vectorX unit_vectorX (vectorX self);
vector make2D_vectorX(vectorX self);
vectorX make3D_vector(vector self);

// Condition Checks subroutine initializaiton
void check_hole_body_touch(int x_body, int y_body, int x_hole_body, int y_hole_body, bool *loop_condition);
void check_out_bounds(int x, int y, bool *pause_display_condition);

// Back-end subroutine initializations







volatile int pixel_buffer_start; // global variable

int main(void)
{
  //=======VGA initializations======
  volatile int * pixel_ctrl_ptr = (int *)0xFF203020;
  *(pixel_ctrl_ptr + 1) = 0xC8000000; //on-chip memory -> back-buffer
  wait_for_vsync(); //swapping front and back buffers
  pixel_buffer_start = *pixel_ctrl_ptr; //initialize a pointer to the pixel buffer, used by drawing functions
  clear_screen(); // pixel_buffer_start points to the pixel buffer
  *(pixel_ctrl_ptr + 1) = 0xC0000000; //set back pixel buffer to start of SDRAM memory
  pixel_buffer_start = *(pixel_ctrl_ptr + 1); // we draw on the back buffer
  clear_screen();


  
  


	//=======Begin Body of Program======
  //Back-end parameter initializations (to be replaced and initialized using switches and keys)
  vector launch_pos, launch_vel;
  initialize_vector(&launch_pos);
  launch_pos.x = RESOLUTION_X/2 - 32;
  launch_pos.y = RESOLUTION_Y/2;
  initialize_vector(&launch_vel);
  launch_vel.y = 12;


  double hole_mass = 100;
	vector hole_pos;
	initialize_vector(&hole_pos);
  hole_pos.x = RESOLUTION_X/2;
  hole_pos.y = RESOLUTION_Y/2;

	
  double satellite_mass = 10;
	vector sat_pos;
	initialize_vector(&sat_pos);
	sat_pos = launch_pos;
	
	
	vector sat_vel;
	initialize_vector(&sat_vel);
	sat_vel = launch_vel;
	
	vector r;
	initialize_vector(&r);
	
	vector rhat;
	initialize_vector(&rhat);
	
  double rmag;
  double G = 46.08; //see additional documentation to see derivation of this parameter

	vector Fgrav;
	initialize_vector(&Fgrav);

  double dt = 1/60; //screen refresh is 60hz

  /*
  //Displayed objects location declarations
	vector sat1_prev;
	sat1_prev.x = 0;
	sat1_prev.y = 0;
	vector sat1_prev2;
	sat1_prev2.x = 0;
	sat1_prev2.y = 0;
  */


  bool loop_condition = true;
  //bool pause_display_condition = false;


  //initialize display for 1st frame of animation
  draw_body(hole_pos.x, hole_pos.y, 5, YELLOW);
  wait_for_vsync();
  pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer 
  draw_body(hole_pos.x, hole_pos.y, 5, YELLOW);
  draw_body(sat_pos.x, sat_pos.y, SATELLITE_SIZE, WHITE);
  wait_for_vsync();
  pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer 

  /* Superloop */
  while (loop_condition)
  {
    //check_hole_body_touch(sat_pos_scaled.x, sat_pos_scaled.y, hole_pos.x, hole_pos.y, &loop_condition);
    //check_out_bounds(sat_pos_scaled.x, sat_pos_scaled.y, &pause_display_condition);
    

    /* STEP 1: Erase any boxes and lines that were drawn in the previous previous iteration */
    clear_box(sat_pos.x, sat_pos.y, SATELLITE_SIZE);

    /* STEP 2: Increment Values at Current Location */
    r.x = sat_pos.x - hole_pos.x;
    r.y = sat_pos.y - hole_pos.y;

    rmag = mag_vector(r);
      
		rhat = unit_vector (r);
        
		Fgrav.x = -1 * G * ((hole_mass * satellite_mass) / (rmag * rmag)) * rhat.x;
		Fgrav.y = -1 * G * ((hole_mass * satellite_mass) / (rmag * rmag)) * rhat.y;

    sat_vel.x = sat_vel.x + (Fgrav.x / satellite_mass) * dt;
    sat_vel.y = sat_vel.y + (Fgrav.y / satellite_mass) * dt;

    sat_pos.x = sat_pos.x + sat_vel.x * dt;
    sat_pos.y = sat_pos.y + sat_vel.y * dt;
    draw_body(sat_pos.x, sat_pos.y, SATELLITE_SIZE, WHITE);

    /* STEP 3: Wait for VSYNC */
    wait_for_vsync();
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer   


    printf("%0.5lf\n", sat_pos.x);

    /* ===================================================================================================*/  
  }
}







/*
HELPER FUNCTION DEFINITIONS
*/

//VGA subroutines
void draw_body(int x, int y, int size, short int color){
    for (int i = x; i < x + size; i++){
        for (int j = y; j < y + size; j++){
            plot_pixel(i, j, color);
        }
    }
}


void wait_for_vsync(){
    volatile int * pixel_ctrl_ptr = (int *) PIXEL_BUF_CTRL_BASE;
    volatile int status;
    *pixel_ctrl_ptr = 1;
    status = *(pixel_ctrl_ptr + 3);
    while ((status &0x01) != 0x0) {
        status = *(pixel_ctrl_ptr + 3);
    }
}

void plot_pixel(int x, int y, short int line_color){
    *(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}

void clear_screen(){
    for (int x = 0; x < RESOLUTION_X; x++){
        for (int y = 0; y < RESOLUTION_Y; y++){
            plot_pixel(x, y, BLACK);
        }
    }
}

void clear_drawn(int x0, int y0, int x1, int y1){
    draw_line(x0, y0, x1, y1, BLACK);
}

void swap(int* val1, int* val2){
    int val1_temp = *val1;
    *val1 = *val2;
    *val2 = val1_temp;
}

void draw_line(int x0, int y0, int x1, int y1, short int line_color){
    bool is_steep = abs(y1 - y0) > abs(x1 - x0);
    if (is_steep){
        swap(&x0, &y0);
        swap(&x1, &y1);
    }

    if (x0 > x1){
        swap(&x0, &x1);
        swap(&y0, &y1);
    }
    int deltax = x1 - x0;
    int deltay = abs(y1 - y0);
    int error = -1*(deltax / 2);
    
    int y = y0;
    int y_step = 0;

    if (y0 < y1){
        y_step = 1;
    }
    else {
        y_step = -1;
    }

    for (int x = x0; x != x1; x++){
        if (is_steep){
            plot_pixel(y, x, line_color);
        }
        else {
            plot_pixel(x, y, line_color);
        }
        error += deltay;
        if (error > 0){
            y += y_step;
            error -= deltax;
        }
    }
}

void clear_box(int x, int y, int size){
    for (int i = x; i < x + size; i++){
        for (int j = y; j < y + size; j++){
            plot_pixel(i, j, BLACK);
        }
    }
}

//Condition Subroutines
void check_out_bounds(int x, int y, bool *pause_display_condition){
    // box hit y (vertical) border
    for (int i = x; i < x + 3; i++){
        if (i < 0 || i > RESOLUTION_X-1){
            *pause_display_condition = true;
            return;
        }
    }
    
    // box hit x (horizontal) border
    for (int i = y; i < y + 3; i++){
        if (i < 0 || i > RESOLUTION_Y-1){
            *pause_display_condition = true;
            return;
        }
    }
    *pause_display_condition = false;
}

void check_hole_body_touch(int x_body, int y_body, int x_hole_body, int y_hole_body, bool *loop_condition){
    for (int x = x_hole_body; x < x_hole_body + 5; x++){
        for (int y = y_hole_body; y < y_hole_body + 5; y++){
            if (((x_body == x) && (y_body == y)) || ((x_body + 1 == x) && (y_body == y)) || ((x_body + 2 == x) && (y_body == y)) ||
                ((x_body == x) && (y_body + 1 == y)) || ((x_body + 1 == x) && (y_body + 1 == y)) || ((x_body + 2 == x) && (y_body + 1 == y)) ||
                ((x_body == x) && (y_body + 2 == y)) || ((x_body + 1 == x) && (y_body + 2 == y)) || ((x_body + 2 == x) && (y_body + 2 == y)))                
                *loop_condition = false;
        }
    }
}

//Vector Subroutines
void initialize_vector (vector* self) {
  (*self).x = 0;
  (*self).y = 0;
}
void initialize_vectorX (vectorX* self) {
  (*self).x = 0;
  (*self).y = 0;
  (*self).z = 0;
}


vector scale_vector (vector self, double scale) {
  vector scaled;
  
  scaled.x = scale * self.x;
  scaled.y = scale * self.y;
  return scaled;
}
vectorX scale_vectorX (vectorX self, double scale) {
  vectorX scaled;
  
  scaled.x = scale * self.x;
  scaled.y = scale * self.y;
  scaled.z = scale * self.z;
  return scaled;
}


vector add_vector (vector a, vector b) {
  vector sum;
  
  sum.x = a.x + b.x;
  sum.y = a.y + b.y;
  return sum;
}
vectorX add_vectorX (vectorX a, vectorX b) {
  vectorX sum;
  
  sum.x = a.x + b.x;
  sum.y = a.y + b.y;
  sum.z = a.z + b.z;
  return sum;
}


vector sub_vector (vector a, vector b) {
  vector sum;
  
  sum.x = a.x + -1* b.x;
  sum.y = a.y + -1* b.y;
  return sum;
}
vectorX sub_vectorX (vectorX a, vectorX b) {
  vectorX sum;
  
  sum.x = a.x + -1* b.x;
  sum.y = a.y + -1* b.y;
  sum.z = a.z + -1* b.z;
  return sum;
}


vector rsb_vector (vector a, vector b) {
  vector sum;
  
  sum.x = -1 * a.x + b.x;
  sum.y = -1 * a.y + b.y;
  
  return sum;
}
vectorX rsb_vectorX (vectorX a, vectorX b) {
  vectorX sum;
  
  sum.x = -1 * a.x + b.x;
  sum.y = -1 * a.y + b.y;
  sum.z = -1 * a.z + b.z;
  
  return sum;
}


double mag_vector (vector self) { 
  double x = self.x;
  double y = self.y;

  return sqrt(pow(x, 2) + pow(y, 2));
}
double mag_vectorX (vectorX self) { 
  double x = self.x;
  double y = self.y;
  double z = self.z;

  return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}


double dot_prod_vector (vector a, vector b) { 
  
  double a1 = a.x;
  double a2 = a.y;

  double b1 = b.x;
  double b2 = b.y;
  
  return a1*b1 + a2*b2;
}
double dot_prod_vectorX (vectorX a, vectorX b) { 
  
  double a1 = a.x;
  double a2 = a.y;
  double a3 = a.z;

  double b1 = b.x;
  double b2 = b.y;
  double b3 = b.z; 
  
  return a1*b1 + a2*b2 + a3*b3;
}


vectorX cross_prod_vector (vector a, vector b) {
  vectorX product;
  
  double a1 = a.x;
  double a2 = a.y;
  double a3 = 0;

  double b1 = b.x;
  double b2 = b.y;
  double b3 = 0;

  product.x = a2*b3 - a3*b2;
  product.y = a3*b1 - a1*b3;
  product.z = a1*b2 - a2*b1;
  
  return product;
}
vectorX cross_prod_vectorX (vectorX a, vectorX b) {
  vectorX product;
  
  double a1 = a.x;
  double a2 = a.y;
  double a3 = a.z;

  double b1 = b.x;
  double b2 = b.y;
  double b3 = b.z;

  product.x = a2*b3 - a3*b2;
  product.y = a3*b1 - a1*b3;
  product.z = a1*b2 - a2*b1;
  
  return product;
}


double angle_vector (vector a, vector b) {
  double dotAB = dot_prod_vector(a, b);
  double magA = mag_vector(a);
  double magB = mag_vector(b);

  return acos(dotAB/(magA * magB)); //in radians
}
double angle_vectorX (vectorX a, vectorX b) {
  double dotAB = dot_prod_vectorX(a, b);
  double magA = mag_vectorX(a);
  double magB = mag_vectorX(b);

  return acos(dotAB/(magA * magB)); //in radians
}


vector unit_vector (vector self) {
  double mag = mag_vector(self);
  if ( mag == 0) {
    vector out;
    initialize_vector(&out);
    return out;
  }
  return scale_vector(self, 1/mag_vector(self));
}
vectorX unit_vectorX (vectorX self) {
  double mag = mag_vectorX(self);
  if ( mag == 0) {
    vectorX out;
    initialize_vectorX(&out);
    return out;
  }
  return scale_vectorX(self, 1/mag_vectorX(self));
}


vector make2D_vectorX(vectorX self) {
  vector final;
  final.x = self.x;
  final.y = self.y;

  return final;
}
vectorX make3D_vector(vector self) {
  vectorX final;
  final.x = self.x;
  final.y = self.y;
  final.z = 0;

  return final;
}
