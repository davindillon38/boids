#include "GLViewBoidSwarm.h"

#include "WorldList.h"
#include "ManagerOpenGLState.h"
#include "Axes.h"
#include "PhysicsEngineODE.h"

#include "WO.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WOImGui.h"
#include "AftrImGuiIncludes.h"
#include "AftrGLRendererBase.h"
#include "MGLIndexedGeometry.h"
#include "IndexedGeometryTriangles.h"

#include <cstdlib>
#include <cmath>

using namespace Aftr;

GLViewBoidSwarm* GLViewBoidSwarm::New( const std::vector< std::string >& args )
{
   GLViewBoidSwarm* glv = new GLViewBoidSwarm( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewBoidSwarm::GLViewBoidSwarm( const std::vector< std::string >& args ) : GLView( args )
{
}


void GLViewBoidSwarm::onCreate()
{
   if( this->pe != NULL )
   {
      this->pe->setGravityNormalizedVector( Vector( 0, 0, -1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV );
}


GLViewBoidSwarm::~GLViewBoidSwarm()
{
}


void GLViewBoidSwarm::updateWorld()
{
   GLView::updateWorld();

   if( this->boid_gui.resetRequested )
   {
      this->boid_gui.resetRequested = false;
      this->spawnBoids();
   }

   if( !this->boid_gui.isPaused )
      this->updateBoids();
}


void GLViewBoidSwarm::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height );
}


void GLViewBoidSwarm::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewBoidSwarm::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewBoidSwarm::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewBoidSwarm::onKeyDown( const SDL_KeyboardEvent& key )
{
   GLView::onKeyDown( key );
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );
}


void GLViewBoidSwarm::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
}


WO* GLViewBoidSwarm::createBoidWO( float scale, aftrColor4ub color )
{
   // Tetrahedron: nose at +X, wider tail at -X
   std::vector< Vector > verts;
   verts.push_back( Vector( 1.0f * scale, 0.0f, 0.0f ) );            // 0: nose
   verts.push_back( Vector( -0.5f * scale, 0.4f * scale, 0.2f * scale ) );  // 1: top-left tail
   verts.push_back( Vector( -0.5f * scale, -0.4f * scale, 0.2f * scale ) ); // 2: top-right tail
   verts.push_back( Vector( -0.5f * scale, 0.0f, -0.3f * scale ) );         // 3: bottom tail

   std::vector< unsigned int > indices;
   // Top face
   indices.push_back( 0 ); indices.push_back( 1 ); indices.push_back( 2 );
   // Left face
   indices.push_back( 0 ); indices.push_back( 3 ); indices.push_back( 1 );
   // Right face
   indices.push_back( 0 ); indices.push_back( 2 ); indices.push_back( 3 );
   // Back face
   indices.push_back( 1 ); indices.push_back( 3 ); indices.push_back( 2 );

   // Per-vertex colors: nose bright, tail slightly darker
   aftrColor4ub noseColor = color;
   aftrColor4ub tailColor( (uint8_t)(color.r * 0.6f), (uint8_t)(color.g * 0.6f),
                           (uint8_t)(color.b * 0.6f), color.a );
   std::vector< aftrColor4ub > colors;
   colors.push_back( noseColor ); // nose
   colors.push_back( tailColor ); // tail verts
   colors.push_back( tailColor );
   colors.push_back( tailColor );

   WO* wo = WO::New();
   MGLIndexedGeometry* mgl = MGLIndexedGeometry::New( wo );
   IndexedGeometryTriangles* geo = IndexedGeometryTriangles::New( verts, indices, colors );
   mgl->setIndexedGeometry( geo );
   wo->setModel( mgl );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   return wo;
}


static float randFloat( float lo, float hi )
{
   return lo + static_cast<float>( std::rand() ) / ( static_cast<float>( RAND_MAX / ( hi - lo ) ) );
}

static Vector randVecInSphere( float radius )
{
   // Rejection sampling for uniform distribution in a sphere
   Vector v;
   do {
      v = Vector( randFloat( -1, 1 ), randFloat( -1, 1 ), randFloat( -1, 1 ) );
   } while( v.x * v.x + v.y * v.y + v.z * v.z > 1.0f );
   return Vector( v.x * radius, v.y * radius, v.z * radius );
}


void GLViewBoidSwarm::spawnBoids()
{
   // Remove existing boid WOs from worldLst
   for( auto& b : this->boids )
      if( b.wo != nullptr )
         this->worldLst->eraseViaWOptr( b.wo );
   if( this->predator.wo != nullptr )
      this->worldLst->eraseViaWOptr( this->predator.wo );

   this->boids.clear();

   // Spawn boids
   for( int i = 0; i < INITIAL_BOID_COUNT; ++i )
   {
      Boid b;
      b.wo = this->createBoidWO( 0.5f, aftrColor4ub( 0, 180, 220, 255 ) ); // teal
      b.wo->setLabel( "Boid" );

      Vector pos = randVecInSphere( 15.0f );
      b.wo->setPosition( pos );

      b.vel = randVecInSphere( 0.1f );
      b.acc = Vector( 0, 0, 0 );

      this->worldLst->push_back( b.wo );
      this->boids.push_back( b );
   }

   // Spawn predator
   this->predator.wo = this->createBoidWO( 1.5f, aftrColor4ub( 220, 40, 40, 255 ) ); // red
   this->predator.wo->setLabel( "Predator" );
   this->predator.wo->setPosition( randVecInSphere( 20.0f ) );
   this->predator.vel = randVecInSphere( 0.05f );
   this->predator.acc = Vector( 0, 0, 0 );
   this->worldLst->push_back( this->predator.wo );
}


void GLViewBoidSwarm::orientToVelocity( WO* wo, const Vector& pos, const Vector& vel )
{
   float speed = std::sqrt( vel.x * vel.x + vel.y * vel.y + vel.z * vel.z );
   if( speed < 0.0001f ) return;

   Vector fwd( vel.x / speed, vel.y / speed, vel.z / speed );

   // Choose an up hint that isn't parallel to fwd
   Vector upHint( 0, 0, 1 );
   float dot = fwd.x * upHint.x + fwd.y * upHint.y + fwd.z * upHint.z;
   if( std::fabs( dot ) > 0.99f )
      upHint = Vector( 0, 1, 0 );

   // right = fwd x upHint
   Vector right;
   right.x = fwd.y * upHint.z - fwd.z * upHint.y;
   right.y = fwd.z * upHint.x - fwd.x * upHint.z;
   right.z = fwd.x * upHint.y - fwd.y * upHint.x;
   float rLen = std::sqrt( right.x * right.x + right.y * right.y + right.z * right.z );
   right.x /= rLen; right.y /= rLen; right.z /= rLen;

   // up = right x fwd
   Vector up;
   up.x = right.y * fwd.z - right.z * fwd.y;
   up.y = right.z * fwd.x - right.x * fwd.z;
   up.z = right.x * fwd.y - right.y * fwd.x;

   // Mat4 is column-major: columns are X(fwd), Y(right), Z(up)
   Mat4 pose( fwd, right, up );
   pose.setPosition( pos );
   wo->setPose( pose );
}


void GLViewBoidSwarm::updateBoids()
{
   const float sepW = this->boid_gui.separationWeight;
   const float aliW = this->boid_gui.alignmentWeight;
   const float cohW = this->boid_gui.cohesionWeight;
   const float bndW = this->boid_gui.boundaryWeight;
   const float fleW = this->boid_gui.fleeWeight;
   const float sepR = this->boid_gui.separationRadius;
   const float neiR = this->boid_gui.neighborRadius;
   const float feaR = this->boid_gui.fearRadius;
   const float bndR = this->boid_gui.boundaryRadius;
   const float mSpd = this->boid_gui.maxSpeed;
   const float pSpd = this->boid_gui.predatorSpeed;

   const size_t n = this->boids.size();
   Vector predPos = this->predator.wo->getPosition();

   // --- Update each boid ---
   for( size_t i = 0; i < n; ++i )
   {
      Boid& bi = this->boids[i];
      Vector pos = bi.wo->getPosition();
      bi.acc = Vector( 0, 0, 0 );

      Vector separation( 0, 0, 0 );
      Vector alignSum( 0, 0, 0 );
      Vector cohesionSum( 0, 0, 0 );
      int sepCount = 0;
      int neiCount = 0;

      for( size_t j = 0; j < n; ++j )
      {
         if( i == j ) continue;
         Vector other = this->boids[j].wo->getPosition();
         float dx = pos.x - other.x;
         float dy = pos.y - other.y;
         float dz = pos.z - other.z;
         float dist = std::sqrt( dx * dx + dy * dy + dz * dz );

         // Separation
         if( dist < sepR && dist > 0.001f )
         {
            float strength = ( sepR - dist ) / sepR; // stronger when closer
            separation.x += ( dx / dist ) * strength;
            separation.y += ( dy / dist ) * strength;
            separation.z += ( dz / dist ) * strength;
            sepCount++;
         }

         // Alignment + Cohesion (same neighbor radius)
         if( dist < neiR )
         {
            alignSum.x += this->boids[j].vel.x;
            alignSum.y += this->boids[j].vel.y;
            alignSum.z += this->boids[j].vel.z;

            cohesionSum.x += other.x;
            cohesionSum.y += other.y;
            cohesionSum.z += other.z;
            neiCount++;
         }
      }

      // Apply separation
      if( sepCount > 0 )
      {
         bi.acc.x += separation.x * sepW;
         bi.acc.y += separation.y * sepW;
         bi.acc.z += separation.z * sepW;
      }

      // Apply alignment: steer toward average velocity
      if( neiCount > 0 )
      {
         float avgVx = alignSum.x / neiCount;
         float avgVy = alignSum.y / neiCount;
         float avgVz = alignSum.z / neiCount;
         bi.acc.x += ( avgVx - bi.vel.x ) * aliW;
         bi.acc.y += ( avgVy - bi.vel.y ) * aliW;
         bi.acc.z += ( avgVz - bi.vel.z ) * aliW;
      }

      // Apply cohesion: steer toward center of neighbors
      if( neiCount > 0 )
      {
         float cx = cohesionSum.x / neiCount;
         float cy = cohesionSum.y / neiCount;
         float cz = cohesionSum.z / neiCount;
         bi.acc.x += ( cx - pos.x ) * cohW;
         bi.acc.y += ( cy - pos.y ) * cohW;
         bi.acc.z += ( cz - pos.z ) * cohW;
      }

      // Boundary containment
      float distFromOrigin = std::sqrt( pos.x * pos.x + pos.y * pos.y + pos.z * pos.z );
      if( distFromOrigin > bndR )
      {
         float overshoot = distFromOrigin - bndR;
         bi.acc.x += ( -pos.x / distFromOrigin ) * overshoot * bndW;
         bi.acc.y += ( -pos.y / distFromOrigin ) * overshoot * bndW;
         bi.acc.z += ( -pos.z / distFromOrigin ) * overshoot * bndW;
      }

      // Predator avoidance
      float pdx = pos.x - predPos.x;
      float pdy = pos.y - predPos.y;
      float pdz = pos.z - predPos.z;
      float predDist = std::sqrt( pdx * pdx + pdy * pdy + pdz * pdz );
      if( predDist < feaR && predDist > 0.001f )
      {
         float strength = ( feaR - predDist ) / predDist; // inverse: panic when close
         bi.acc.x += ( pdx / predDist ) * strength * fleW;
         bi.acc.y += ( pdy / predDist ) * strength * fleW;
         bi.acc.z += ( pdz / predDist ) * strength * fleW;
      }

      // Integrate velocity
      bi.vel.x += bi.acc.x * 0.05f; // scale down acceleration for smooth motion
      bi.vel.y += bi.acc.y * 0.05f;
      bi.vel.z += bi.acc.z * 0.05f;

      // Clamp speed
      float speed = std::sqrt( bi.vel.x * bi.vel.x + bi.vel.y * bi.vel.y + bi.vel.z * bi.vel.z );
      if( speed > mSpd )
      {
         bi.vel.x = ( bi.vel.x / speed ) * mSpd;
         bi.vel.y = ( bi.vel.y / speed ) * mSpd;
         bi.vel.z = ( bi.vel.z / speed ) * mSpd;
      }

      // Update position
      Vector newPos( pos.x + bi.vel.x, pos.y + bi.vel.y, pos.z + bi.vel.z );
      this->orientToVelocity( bi.wo, newPos, bi.vel );
   }

   // --- Update predator: chase flock centroid ---
   this->predator.acc = Vector( 0, 0, 0 );
   if( n > 0 )
   {
      Vector centroid( 0, 0, 0 );
      for( size_t i = 0; i < n; ++i )
      {
         Vector p = this->boids[i].wo->getPosition();
         centroid.x += p.x;
         centroid.y += p.y;
         centroid.z += p.z;
      }
      centroid.x /= (float)n;
      centroid.y /= (float)n;
      centroid.z /= (float)n;

      float dx = centroid.x - predPos.x;
      float dy = centroid.y - predPos.y;
      float dz = centroid.z - predPos.z;
      float dist = std::sqrt( dx * dx + dy * dy + dz * dz );
      if( dist > 0.01f )
      {
         this->predator.acc.x = ( dx / dist ) * 0.5f;
         this->predator.acc.y = ( dy / dist ) * 0.5f;
         this->predator.acc.z = ( dz / dist ) * 0.5f;
      }
   }

   // Predator boundary containment
   float predDistOrigin = std::sqrt( predPos.x * predPos.x + predPos.y * predPos.y + predPos.z * predPos.z );
   if( predDistOrigin > bndR )
   {
      float overshoot = predDistOrigin - bndR;
      this->predator.acc.x += ( -predPos.x / predDistOrigin ) * overshoot * bndW;
      this->predator.acc.y += ( -predPos.y / predDistOrigin ) * overshoot * bndW;
      this->predator.acc.z += ( -predPos.z / predDistOrigin ) * overshoot * bndW;
   }

   // Integrate predator
   this->predator.vel.x += this->predator.acc.x * 0.05f;
   this->predator.vel.y += this->predator.acc.y * 0.05f;
   this->predator.vel.z += this->predator.acc.z * 0.05f;

   float pSpeed = std::sqrt( this->predator.vel.x * this->predator.vel.x +
                             this->predator.vel.y * this->predator.vel.y +
                             this->predator.vel.z * this->predator.vel.z );
   if( pSpeed > pSpd )
   {
      this->predator.vel.x = ( this->predator.vel.x / pSpeed ) * pSpd;
      this->predator.vel.y = ( this->predator.vel.y / pSpeed ) * pSpd;
      this->predator.vel.z = ( this->predator.vel.z / pSpeed ) * pSpd;
   }

   Vector newPredPos( predPos.x + this->predator.vel.x,
                      predPos.y + this->predator.vel.y,
                      predPos.z + this->predator.vel.z );
   this->orientToVelocity( this->predator.wo, newPredPos, this->predator.vel );
}


void Aftr::GLViewBoidSwarm::loadMap()
{
   this->worldLst = new WorldList();
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE( 1000.0 );
   ManagerOpenGLState::GL_NEAR_PLANE( 0.1f );
   ManagerOpenGLState::enableFrustumCulling( false );
   Axes::isVisible = false;
   this->glRenderer->isUsingShadowMapping( false );

   // Camera pulled back to see the swarm
   this->cam->setPosition( 0, 0, 60 );

   {
      // Light
      float ga = 0.2f;
      ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
      WOLight* light = WOLight::New();
      light->isDirectionalLight( true );
      light->setPosition( Vector( 0, 0, 100 ) );
      light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
      light->setLabel( "Light" );
      worldLst->push_back( light );
   }

   {
      // SkyBox — water theme
      std::string skyBox = ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_water+6.jpg";
      WO* wo = WOSkyBox::New( skyBox, this->getCameraPtrPtr() );
      wo->setPosition( Vector( 0, 0, 0 ) );
      wo->setLabel( "Sky Box" );
      wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      worldLst->push_back( wo );
   }

   // Spawn the boid swarm and predator
   std::srand( static_cast<unsigned int>( std::time( nullptr ) ) );
   this->spawnBoids();

   // ImGui
   {
      this->gui = WOImGui::New( nullptr );
      gui->setLabel( "My Gui" );

      auto woEditFunc = [this]() { this->wo_editor.draw( this->getLastSelectionQuery(), *this->getWorldContainer(), this->getCamera_functor() ); };
      auto showDemoWindow_ImGui = [this]() { ImGui::ShowDemoWindow(); };
      auto showDemoWindow_AftrDemo = [this]() { WOImGui::draw_AftrImGui_Demo( this->gui ); };
      auto showDemoWindow_ImGuiPlot = [this]() { ImPlot::ShowDemoWindow(); };
      auto show_boid_controls = [this]() { this->boid_gui.draw(); };

      this->gui->subscribe_drawImGuiWidget(
         [=,this]()
         {
            menu.attach( "Edit", "Show WO Editor", woEditFunc );
            menu.attach( "Demos", "Show Default ImGui Demo", showDemoWindow_ImGui );
            menu.attach( "Demos", "Show Default ImPlot Demo", showDemoWindow_ImGuiPlot );
            menu.attach( "Demos", "Show Aftr ImGui w/ Markdown & File Dialogs", showDemoWindow_AftrDemo );
            menu.attach( "Boids", "Boid Controls", show_boid_controls, true );
            menu.draw();
         } );
      this->worldLst->push_back( this->gui );
   }
}
