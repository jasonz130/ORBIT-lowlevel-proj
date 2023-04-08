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
void clear_box(int x, int y);
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
void check_cen_body_touch(int x_body, int y_body, int x_cen_body, int y_cen_body, bool *loop_condition);


// Back-end subroutine initializations
void elliptical_orbit (double init_pos, double init_velocity, double central_mass);
void check_out_bounds(int x, int y, bool *pause_display_condition);






volatile int pixel_buffer_start; // global variable

int main(void)
{
  //Back-end parameter initializations (to be replaced and initialized using switches and keys)
  double init_pos = -24000000;
  double init_vel = -2000;
  double central_mass = 6 * pow(10, 24);


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
	vector cen_pos;
	initialize_vector(&cen_pos);
  cen_pos.x = r_x_shift / r_x_scale;
  cen_pos.y = r_y_shift / r_y_scale;

	
	vector sat_pos;
	initialize_vector(&sat_pos);
	sat_pos.x = init_pos;
	
	vector sat_pos_scaled;
	initialize_vector(&sat_pos_scaled);
	
	
	vector sat_vel;
	initialize_vector(&sat_vel);
	sat_vel.y = init_vel;
	
  double satellite_mass = 15000.0;
	
	vector r;
	initialize_vector(&r);
	
	vector rhat;
	initialize_vector(&rhat);
	
  double rmag;
  double G = 6.67 * pow(10, -11);

	vector Fgrav;
	initialize_vector(&Fgrav);

  double dt = 600;
  double t = 0.0;

  //Displayed objects location declarations
	vector sat1_prev;
	sat1_prev.x = 0;
	sat1_prev.y = 0;
	vector sat1_prev2;
	sat1_prev2.x = 0;
	sat1_prev2.y = 0;


  // DRAW INITIAL CENTRAL BODY


  bool loop_condition = true;
  bool pause_display_condition = false;

  /* Superloop */
  while (loop_condition)
  {
    check_cen_body_touch(sat_pos_scaled.x, sat_pos_scaled.y, cen_pos.x, cen_pos.y, &loop_condition);
    check_out_bounds(sat_pos_scaled.x, sat_pos_scaled.y, &pause_display_condition);
    

    /* STEP 1: Erase any boxes and lines that were drawn in the previous previous iteration */
    if (pause_display_condition == false) clear_box(sat1_prev2.x, sat1_prev2.y);

    /* STEP 2: Draw current body */
    if (pause_display_condition == false){
      draw_body(sat_pos_scaled.x, sat_pos_scaled.y, SATELLITE_SIZE, WHITE);
    }
    else {
      // X borders
      for (int x_idx = 0; x_idx < RESOLUTION_X; x_idx++){
          for (int y_idx = 0; y_idx < 15; y_idx++){
              plot_pixel(x_idx, y_idx, BLACK);
          }
          for (int y_idx = 220; y_idx < RESOLUTION_Y; y_idx++){
              plot_pixel(x_idx, y_idx, BLACK);
          }
      }
      // Y borders
      for (int y_idx = 0; y_idx < RESOLUTION_Y; y_idx++){
          for (int x_idx = 0; x_idx < 20; x_idx++){
              plot_pixel(x_idx, y_idx, BLACK);
          }
          for (int x_idx = 300; x_idx < RESOLUTION_X; x_idx++){
              plot_pixel(x_idx, y_idx, BLACK);
          }
      }
    }

    /* STEP 3: Set previous previous location to previous */
    sat1_prev2.x = sat1_prev.x;
    sat1_prev2.y = sat1_prev.y;

    /* STEP 4: Set previous location to current */
    sat1_prev.x = sat_pos_scaled.x;
    sat1_prev.y = sat_pos_scaled.y;

    draw_body(cen_pos.x, cen_pos.y, 5, YELLOW);
    
    /* ============================================ Back-end =============================================*/

    /* STEP 5: Increment current location */
    r.x = sat_pos.x - cen_pos.x;
    r.y = sat_pos.y - cen_pos.y;

    rmag = mag_vector(r);
      
		rhat = unit_vector (r);
        
		Fgrav.x = -1 * G * ((central_mass * satellite_mass) / (rmag * rmag)) * rhat.x;
		Fgrav.y = -1 * G * ((central_mass * satellite_mass) / (rmag * rmag)) * rhat.y;

    sat_vel.x = sat_vel.x + (Fgrav.x / satellite_mass) * dt;
    sat_vel.y = sat_vel.y + (Fgrav.y / satellite_mass) * dt;

    sat_pos.x = sat_pos.x + sat_vel.x * dt;
    sat_pos.y = sat_pos.y + sat_vel.y * dt;

    sat_pos_scaled.x = round((sat_pos.x + r_x_shift) / r_x_scale);
    sat_pos_scaled.y = round((sat_pos.y + r_y_shift) / r_y_scale);

    t = t + dt;

    printf("%0.5lf\n", sat_pos_scaled.x);

    /* ===================================================================================================*/

    /* STEP 6: APPLY VSYNC */
    wait_for_vsync(); // swap front and back buffers on VGA vertical sync
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer   
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

void clear_box(int x, int y){
    for (int i = x; i < x + 3; i++){
        for (int j = y; j < y + 3; j++){
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

void check_cen_body_touch(int x_body, int y_body, int x_cen_body, int y_cen_body, bool *loop_condition){
    for (int x = x_cen_body; x < x_cen_body + 5; x++){
        for (int y = y_cen_body; y < y_cen_body + 5; y++){
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
