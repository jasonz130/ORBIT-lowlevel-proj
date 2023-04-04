#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <strings.h>
#include "vectors.h"



/*test driver code
int main(void) {
  printf("Performing Test 1 on VectorX Typedef:\n");
  vectorX test1;
  test1.x = 1;
  test1.y = 2;
  test1.z = 3;

  vectorX test2;
  test2.x = 3;
  test2.y = 5;
  test2.z = 7;

  vectorX scaleX = scale_vectorX(test1, 2);
  vectorX sumX = add_vectorX(test1, test2);
  vectorX subX = sub_vectorX(test1, test2);
  vectorX rsbX = rsb_vectorX(test1, test2);
  double dotX = dot_prod_vectorX(test1, test2);
  vectorX crossX = cross_prod_vectorX(test1, test2);
  double angleX = angle_vectorX(test1, test2);
  double magX = mag_vectorX(test1);

  printf(" Scaled %lf %lf %lf \n Sum %lf %lf %lf \n Sub %lf %lf %lf \n RSB %lf %lf %lf \n Dot %lf \n Cross %lf %lf %lf \n Angle %lf \n Mag %lf\n", scaleX.x, scaleX.y, scaleX.z, sumX.x, sumX.y, sumX.z, subX.x, subX.y, subX.z, rsbX.x, rsbX.y, rsbX.z, dotX, crossX.x, crossX.y, crossX.z, angleX, magX);

  printf("Performing Test 2 on Vector Typedef:\n");
  
  vector test3 = make2D_vectorX(test1);

  vector test4 = make2D_vectorX(test2);

  vector scale = scale_vector(test3, 2);
  vector sum = add_vector(test3, test4);
  vector sub = sub_vector(test3, test4);
  vector rsb = rsb_vector(test3, test4);
  double dot = dot_prod_vector(test3, test4);
  vectorX cross = cross_prod_vector(test3, test4);
  double angle = angle_vector(test3, test4);
  double mag = mag_vector(test3);
  
  printf(" Scaled %lf %lf \n Sum %lf %lf \n Sub %lf %lf \n RSB %lf %lf \n Dot %lf \n Cross %lf %lf %lf \n Angle %lf \n Mag %lf \n", scale.x, scale.y, sum.x, sum.y, sub.x, sub.y, rsb.x, rsb.y, dot, cross.x, cross.y, cross.z, angle, mag);
  return 0;
}
*/

struct _dynamic_list {
  size_t size;
  size_t capacity;
  vector* data;
};

//typedef struct _dynamic_list intList;
typedef struct _dynamic_list vectorList;


vectorList initialize_VectorList(vectorList* head) {
  head->size = 0;
  head->capacity = 1;
  head->data = (vector*)malloc(sizeof(vector));
}

int getSize_VectorList (vectorList* self) {
  return self->size;
}

int getCapacity_VectorList (vectorList* self) {
  return self->capacity;
}

//add item to last index
int push_back_VectorList(vectorList* self, vector addition)
{  
  int size = getSize_VectorList(self);
  int capacity = getCapacity_VectorList(self);
  if (size == capacity) {

    // if not then grow the array by double

    vector* temp = (vector*) realloc(self->data, capacity*2 *sizeof(vector));

    if (temp == NULL ) return -1; //exception: cannot allocate more memory, but original "self" still valid
    //else
    
    self->data = temp;
    free(temp);
    self->capacity = capacity * 2;
  }

  // insert element
  self->data[size] = addition;
  // increment the size or last_index+1
  (self->size)++;
  return 1;
}

// Deleting element at last stored index
void pop_back(vectorList* self)
{
  // delete last index by copying all other items but it
  int size = getSize_VectorList(self);
  int capacity = getCapacity_VectorList(self);

  vector* temp = (vector*) malloc( (size-1) * sizeof(vector));
  for (int i = 0; i < size-1; i++) {
    temp[i] = self->data[i];
  }
  
  //delete old array
  free(self->data);
  //reassign self
  self->data = temp;

  // Decrement the array size
  (self->size)--;

  // Reduce if the container half element of its
  // capacity
  if (size == (capacity / 2)) {
      self->data = (vector*) realloc(self->data, capacity/2 *sizeof(vector));
  }
}



int main(void) {
  
  return 0;
}
