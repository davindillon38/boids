#pragma once

#include "GLView.h"
#include "AftrImGui_MenuBar.h"
#include "AftrImGui_WO_Editor.h"
#include "AftrImGui_BoidSwarm.h"
#include "Vector.h"
#include <vector>

namespace Aftr
{
   class Camera;
   class WOImGui;

class GLViewBoidSwarm : public GLView
{
public:
   static GLViewBoidSwarm* New( const std::vector< std::string >& outArgs );
   virtual ~GLViewBoidSwarm();
   virtual void updateWorld() override;
   virtual void loadMap() override;
   virtual void onResizeWindow( GLsizei width, GLsizei height ) override;
   virtual void onMouseDown( const SDL_MouseButtonEvent& e ) override;
   virtual void onMouseUp( const SDL_MouseButtonEvent& e ) override;
   virtual void onMouseMove( const SDL_MouseMotionEvent& e ) override;
   virtual void onKeyDown( const SDL_KeyboardEvent& key ) override;
   virtual void onKeyUp( const SDL_KeyboardEvent& key ) override;

protected:
   GLViewBoidSwarm( const std::vector< std::string >& args );
   virtual void onCreate();

   void initComputeShader();
   void initRenderShader();
   void initBoidBuffers();
   void renderBoids();
   void resetSimulation();

   WOImGui* gui = nullptr;
   AftrImGui_MenuBar menu;
   AftrImGui_WO_Editor wo_editor;
   AftrImGui_BoidSwarm boid_gui;

   // Compute shader
   GLuint computeProgram = 0;
   GLuint ssbo[2] = { 0, 0 }; // double-buffered
   int readIdx = 0;

   // Render shader (vertex + fragment for instanced boid drawing)
   GLuint renderProgram = 0;
   GLuint boidVAO = 0;
   GLuint boidVBO = 0;
   GLuint boidEBO = 0;

   // Aquarium sphere
   WO* aquarium = nullptr;

   // Obstacles (vertical pillars)
   static constexpr int NUM_OBSTACLES = 5;
   Vector obstaclePositions[NUM_OBSTACLES] = {
      Vector( 0, 0, 0 ),       // center pillar
      Vector( 10, 8, 0 ),      // front-right
      Vector( -10, 8, 0 ),     // front-left
      Vector( -7, -10, 0 ),    // back-left
      Vector( 8, -9, 0 )       // back-right
   };
   float obstacleAvoidRadius = 4.0f;
   WO* obstacleWOs[NUM_OBSTACLES] = {};
};

} //namespace Aftr
