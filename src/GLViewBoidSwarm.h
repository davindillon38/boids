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

struct Boid {
   WO* wo = nullptr;
   Vector vel;
   Vector acc;
};

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

   WO* createBoidWO( float scale, aftrColor4ub color );
   void spawnBoids();
   void updateBoids();
   void orientToVelocity( WO* wo, const Vector& pos, const Vector& vel );

   WOImGui* gui = nullptr;
   AftrImGui_MenuBar menu;
   AftrImGui_WO_Editor wo_editor;
   AftrImGui_BoidSwarm boid_gui;

   std::vector< Boid > boids;
   Boid predator;

   static constexpr int INITIAL_BOID_COUNT = 50;
};

} //namespace Aftr
