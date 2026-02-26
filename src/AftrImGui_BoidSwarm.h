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
   float alignmentWeight = 2.0f;
   float cohesionWeight = 1.2f;
   float boundaryWeight = 1.0f;
   float fleeWeight = 3.0f;
   float obstacleWeight = 3.0f;
   float noiseStrength = 0.4f;

   // Radii
   float separationRadius = 2.0f;
   float neighborRadius = 5.0f;
   float fearRadius = 8.0f;
   float boundaryRadius = 25.0f;

   // Speeds
   float maxSpeed = 0.3f;
   float predatorSpeed = 0.45f;

   // Boid / predator count
   int numBoids = 1000;
   int numPredators = 1;

   // Simulation control
   bool isPaused = false;
   bool resetRequested = false;
   bool showObstacles = true;

private:
   void draw_boid_controls();
};

}

#endif
