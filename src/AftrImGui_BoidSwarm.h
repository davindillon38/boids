#pragma once
#include "AftrConfig.h"
#ifdef  AFTR_CONFIG_USE_IMGUI

#include <functional>

namespace Aftr
{

class AftrImGui_BoidSwarm
{
public:
   void draw();

   // Flocking weights
   float separationWeight = 1.5f;
   float alignmentWeight = 1.0f;
   float cohesionWeight = 1.0f;
   float boundaryWeight = 2.0f;
   float fleeWeight = 3.0f;

   // Radii
   float separationRadius = 2.0f;
   float neighborRadius = 5.0f;
   float fearRadius = 8.0f;
   float boundaryRadius = 25.0f;

   // Speeds
   float maxSpeed = 0.3f;
   float predatorSpeed = 0.15f;

   // Simulation control
   bool isPaused = false;
   bool resetRequested = false;

private:
   void draw_boid_controls();
};

}

#endif
