#ifndef VECTORS_H
#define VECTORS_H
#include <math.h>

typedef struct _2D_double_vector_struct
{
  double x;
  double y;
} vector;
typedef struct _3D_double_vector_struct
{
  double x;
  double y;
  double z;
} vectorX;

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


#endif /* VECTORS_H */