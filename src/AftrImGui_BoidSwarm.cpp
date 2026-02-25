#include "AftrImGui_BoidSwarm.h"
#include "AftrImGuiIncludes.h"

void Aftr::AftrImGui_BoidSwarm::draw()
{
   this->draw_boid_controls();
}

void Aftr::AftrImGui_BoidSwarm::draw_boid_controls()
{
   if( ImGui::Begin( "Boid Controls" ) )
   {
      // Simulation controls
      if( ImGui::Button( this->isPaused ? "Resume" : "Pause" ) )
         this->isPaused = !this->isPaused;

      ImGui::SameLine();
      if( ImGui::Button( "Reset Swarm" ) )
         this->resetRequested = true;

      ImGui::Separator();
      ImGui::Text( "Flocking Weights" );
      ImGui::SliderFloat( "Separation", &this->separationWeight, 0.0f, 5.0f );
      ImGui::SliderFloat( "Alignment", &this->alignmentWeight, 0.0f, 5.0f );
      ImGui::SliderFloat( "Cohesion", &this->cohesionWeight, 0.0f, 5.0f );
      ImGui::SliderFloat( "Boundary", &this->boundaryWeight, 0.0f, 10.0f );
      ImGui::SliderFloat( "Flee", &this->fleeWeight, 0.0f, 10.0f );

      ImGui::Separator();
      ImGui::Text( "Radii" );
      ImGui::SliderFloat( "Separation Radius", &this->separationRadius, 0.5f, 10.0f );
      ImGui::SliderFloat( "Neighbor Radius", &this->neighborRadius, 1.0f, 20.0f );
      ImGui::SliderFloat( "Fear Radius", &this->fearRadius, 2.0f, 30.0f );
      ImGui::SliderFloat( "Boundary Radius", &this->boundaryRadius, 10.0f, 100.0f );

      ImGui::Separator();
      ImGui::Text( "Speed" );
      ImGui::SliderFloat( "Max Boid Speed", &this->maxSpeed, 0.05f, 1.0f );
      ImGui::SliderFloat( "Predator Speed", &this->predatorSpeed, 0.02f, 0.5f );

      ImGui::End();
   }
}
