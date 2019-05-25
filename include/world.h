#ifndef WORLD_H
#define WORLD_H

#include "wind.h"
#include "current.h"
#include "wave.h"
#include "asv.h"

struct World
{
  struct Wind* wind;
  struct Current* current;
  struct Wave* wave;
  struct Asv asv;
};

void world_init(struct World* world, char* filename);

void world_set_frame(struct World* world, double time);

void world_clean(struct World* world); 

#endif // WORLD_H
