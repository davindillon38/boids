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
#include "IndexedGeometrySphereTriStrip.h"
#include "ManagerEnvironmentConfiguration.h"

#include <cstdlib>
#include <cmath>
#include <ctime>
#include <vector>
#include <iostream>

using namespace Aftr;

// ============================================================
// GLSL Shader Sources (inline, following ChaosGame pattern)
// ============================================================

static const char* computeShaderSource = R"(
#version 430
layout(local_size_x = 256) in;

struct BoidData {
    vec4 pos; // xyz=position, w=type (0=boid, 1=predator)
    vec4 vel; // xyz=velocity, w=unused
};

layout(std430, binding = 0) readonly  buffer BoidInput  { BoidData boidsIn[];  };
layout(std430, binding = 1)           buffer BoidOutput { BoidData boidsOut[]; };

uniform int   u_numBoids;
uniform float u_sepWeight;
uniform float u_aliWeight;
uniform float u_cohWeight;
uniform float u_bndWeight;
uniform float u_fleWeight;
uniform float u_obsWeight;
uniform float u_sepRadius;
uniform float u_neiRadius;
uniform float u_feaRadius;
uniform float u_bndRadius;
uniform float u_maxSpeed;
uniform float u_predSpeed;
uniform float u_dt;
uniform int   u_numObstacles;
uniform vec4  u_obstacles[3]; // xyz=position, w=avoidance radius

void main() {
    uint idx = gl_GlobalInvocationID.x;
    uint totalEntities = uint(u_numBoids) + 1u;
    if (idx >= totalEntities) return;

    vec3 myPos = boidsIn[idx].pos.xyz;
    vec3 myVel = boidsIn[idx].vel.xyz;
    bool isPredator = (idx == uint(u_numBoids));

    vec3 acc = vec3(0.0);

    if (!isPredator) {
        // ---- Boid flocking rules ----
        vec3 separation = vec3(0.0);
        vec3 alignSum   = vec3(0.0);
        vec3 cohesionSum = vec3(0.0);
        int sepCount = 0;
        int neiCount = 0;

        for (uint j = 0u; j < uint(u_numBoids); ++j) {
            if (j == idx) continue;
            vec3 other = boidsIn[j].pos.xyz;
            vec3 diff  = myPos - other;
            float dist = length(diff);

            // Separation
            if (dist < u_sepRadius && dist > 0.001) {
                float strength = (u_sepRadius - dist) / u_sepRadius;
                separation += normalize(diff) * strength;
                sepCount++;
            }

            // Alignment + Cohesion
            if (dist < u_neiRadius) {
                alignSum   += boidsIn[j].vel.xyz;
                cohesionSum += other;
                neiCount++;
            }
        }

        if (sepCount > 0)
            acc += separation * u_sepWeight;

        if (neiCount > 0) {
            vec3 avgVel = alignSum / float(neiCount);
            acc += (avgVel - myVel) * u_aliWeight;

            vec3 center = cohesionSum / float(neiCount);
            acc += (center - myPos) * u_cohWeight;
        }

        // Boundary containment
        float distOrigin = length(myPos);
        if (distOrigin > u_bndRadius) {
            float overshoot = distOrigin - u_bndRadius;
            acc += (-myPos / distOrigin) * overshoot * u_bndWeight;
        }

        // Predator avoidance
        vec3 predPos  = boidsIn[u_numBoids].pos.xyz;
        vec3 predDiff = myPos - predPos;
        float predDist = length(predDiff);
        if (predDist < u_feaRadius && predDist > 0.001) {
            float strength = (u_feaRadius - predDist) / predDist;
            acc += normalize(predDiff) * strength * u_fleWeight;
        }

        // Obstacle avoidance
        for (int o = 0; o < u_numObstacles; ++o) {
            vec3  obsPos    = u_obstacles[o].xyz;
            float obsRadius = u_obstacles[o].w;
            vec3  obsDiff   = myPos - obsPos;
            float obsDist   = length(obsDiff);
            if (obsDist < obsRadius && obsDist > 0.001) {
                float strength = (obsRadius - obsDist) / obsDist;
                acc += normalize(obsDiff) * strength * u_obsWeight;
            }
        }

        // Integrate
        myVel += acc * u_dt;
        float speed = length(myVel);
        if (speed > u_maxSpeed)
            myVel = normalize(myVel) * u_maxSpeed;

    } else {
        // ---- Predator: chase flock centroid ----
        vec3 centroid = vec3(0.0);
        for (uint j = 0u; j < uint(u_numBoids); ++j)
            centroid += boidsIn[j].pos.xyz;
        centroid /= float(u_numBoids);

        vec3 toCentroid = centroid - myPos;
        float dist = length(toCentroid);
        if (dist > 0.01)
            acc = normalize(toCentroid) * 0.5;

        // Boundary
        float distOrigin = length(myPos);
        if (distOrigin > u_bndRadius) {
            float overshoot = distOrigin - u_bndRadius;
            acc += (-myPos / distOrigin) * overshoot * u_bndWeight;
        }

        // Obstacle avoidance
        for (int o = 0; o < u_numObstacles; ++o) {
            vec3  obsPos    = u_obstacles[o].xyz;
            float obsRadius = u_obstacles[o].w;
            vec3  obsDiff   = myPos - obsPos;
            float obsDist   = length(obsDiff);
            if (obsDist < obsRadius && obsDist > 0.001) {
                float strength = (obsRadius - obsDist) / obsDist;
                acc += normalize(obsDiff) * strength * u_obsWeight;
            }
        }

        myVel += acc * u_dt;
        float speed = length(myVel);
        if (speed > u_predSpeed)
            myVel = normalize(myVel) * u_predSpeed;
    }

    myPos += myVel;

    boidsOut[idx].pos = vec4(myPos, boidsIn[idx].pos.w);
    boidsOut[idx].vel = vec4(myVel, 0.0);
}
)";

static const char* boidVertexShaderSource = R"(
#version 430

layout(location = 0) in vec3 aVertex;

struct BoidData {
    vec4 pos;
    vec4 vel;
};

layout(std430, binding = 0) readonly buffer BoidBuffer {
    BoidData boids[];
};

uniform mat4  u_view;
uniform mat4  u_proj;
uniform float u_scale;
uniform int   u_instanceOffset;
uniform vec4  u_color;

out vec3 vNormal;
out vec4 vColor;

mat3 rotationFromVelocity(vec3 vel) {
    float speed = length(vel);
    if (speed < 0.0001) return mat3(1.0);

    vec3 fwd = vel / speed;
    vec3 upHint = vec3(0.0, 0.0, 1.0);
    if (abs(dot(fwd, upHint)) > 0.99)
        upHint = vec3(0.0, 1.0, 0.0);

    vec3 right = normalize(cross(fwd, upHint));
    vec3 up    = cross(right, fwd);

    return mat3(fwd, right, up);
}

void main() {
    int boidIdx = gl_InstanceID + u_instanceOffset;
    vec3 boidPos = boids[boidIdx].pos.xyz;
    vec3 boidVel = boids[boidIdx].vel.xyz;

    mat3 rot = rotationFromVelocity(boidVel);
    vec3 worldPos = boidPos + rot * (aVertex * u_scale);

    gl_Position = u_proj * u_view * vec4(worldPos, 1.0);

    vNormal = rot * normalize(aVertex);
    vColor  = u_color;
}
)";

static const char* boidFragmentShaderSource = R"(
#version 430

in vec3 vNormal;
in vec4 vColor;
out vec4 FragColor;

void main() {
    vec3 lightDir = normalize(vec3(0.3, 0.3, 1.0));
    float diffuse = max(dot(normalize(vNormal), lightDir), 0.0);
    float ambient = 0.35;
    float lighting = ambient + diffuse * 0.65;

    FragColor = vec4(vColor.rgb * lighting, vColor.a);
}
)";

// ============================================================
// Helpers
// ============================================================

static GLuint compileShader( GLenum type, const char* source )
{
   GLuint shader = glCreateShader( type );
   glShaderSource( shader, 1, &source, nullptr );
   glCompileShader( shader );

   GLint success;
   glGetShaderiv( shader, GL_COMPILE_STATUS, &success );
   if( !success )
   {
      char log[1024];
      glGetShaderInfoLog( shader, 1024, nullptr, log );
      std::cout << "Shader compile error:\n" << log << std::endl;
   }
   return shader;
}

static GLuint linkProgram( GLuint* shaders, int count )
{
   GLuint prog = glCreateProgram();
   for( int i = 0; i < count; ++i )
      glAttachShader( prog, shaders[i] );
   glLinkProgram( prog );

   GLint success;
   glGetProgramiv( prog, GL_LINK_STATUS, &success );
   if( !success )
   {
      char log[1024];
      glGetProgramInfoLog( prog, 1024, nullptr, log );
      std::cout << "Program link error:\n" << log << std::endl;
   }

   for( int i = 0; i < count; ++i )
      glDeleteShader( shaders[i] );
   return prog;
}

static float randFloat( float lo, float hi )
{
   return lo + static_cast<float>( std::rand() ) / ( static_cast<float>( RAND_MAX / ( hi - lo ) ) );
}

static Vector randVecInSphere( float radius )
{
   Vector v;
   do {
      v = Vector( randFloat( -1, 1 ), randFloat( -1, 1 ), randFloat( -1, 1 ) );
   } while( v.x * v.x + v.y * v.y + v.z * v.z > 1.0f );
   return Vector( v.x * radius, v.y * radius, v.z * radius );
}

// GPU-side boid data (matches GLSL struct layout)
struct BoidGPU {
   float px, py, pz, type; // pos.xyz, pos.w
   float vx, vy, vz, pad;  // vel.xyz, vel.w
};

// ============================================================
// GLViewBoidSwarm
// ============================================================

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

   // GL context is ready — initialize compute + render shaders
   initComputeShader();
   initRenderShader();
   initBoidBuffers();
   resetSimulation();

   std::cout << "BoidSwarm compute shader initialized with " << boid_gui.numBoids << " boids." << std::endl;
}

GLViewBoidSwarm::~GLViewBoidSwarm()
{
   if( computeProgram ) glDeleteProgram( computeProgram );
   if( renderProgram )  glDeleteProgram( renderProgram );
   if( ssbo[0] ) glDeleteBuffers( 2, ssbo );
   if( boidVAO ) glDeleteVertexArrays( 1, &boidVAO );
   if( boidVBO ) glDeleteBuffers( 1, &boidVBO );
   if( boidEBO ) glDeleteBuffers( 1, &boidEBO );
}

// ============================================================
// Shader & Buffer Initialization
// ============================================================

void GLViewBoidSwarm::initComputeShader()
{
   GLuint cs = compileShader( GL_COMPUTE_SHADER, computeShaderSource );
   computeProgram = linkProgram( &cs, 1 );
}

void GLViewBoidSwarm::initRenderShader()
{
   GLuint shaders[2];
   shaders[0] = compileShader( GL_VERTEX_SHADER, boidVertexShaderSource );
   shaders[1] = compileShader( GL_FRAGMENT_SHADER, boidFragmentShaderSource );
   renderProgram = linkProgram( shaders, 2 );
}

void GLViewBoidSwarm::initBoidBuffers()
{
   // Create double-buffered SSBOs (sized later in resetSimulation)
   glGenBuffers( 2, ssbo );

   // Tetrahedron mesh: nose at +X, wider tail at -X
   float verts[] = {
       1.0f,  0.0f,  0.0f,   // v0: nose
      -0.5f,  0.4f,  0.2f,   // v1: top-left tail
      -0.5f, -0.4f,  0.2f,   // v2: top-right tail
      -0.5f,  0.0f, -0.3f    // v3: bottom tail
   };
   unsigned int indices[] = {
      0, 1, 2,  // top face
      0, 3, 1,  // left face
      0, 2, 3,  // right face
      1, 3, 2   // back face
   };

   glGenVertexArrays( 1, &boidVAO );
   glGenBuffers( 1, &boidVBO );
   glGenBuffers( 1, &boidEBO );

   glBindVertexArray( boidVAO );

   glBindBuffer( GL_ARRAY_BUFFER, boidVBO );
   glBufferData( GL_ARRAY_BUFFER, sizeof( verts ), verts, GL_STATIC_DRAW );

   glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, boidEBO );
   glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( indices ), indices, GL_STATIC_DRAW );

   glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof( float ), (void*)0 );
   glEnableVertexAttribArray( 0 );

   glBindVertexArray( 0 );
}

void GLViewBoidSwarm::resetSimulation()
{
   int n = boid_gui.numBoids;
   int total = n + 1; // +1 for predator

   std::vector<BoidGPU> data( total );
   for( int i = 0; i < n; ++i )
   {
      Vector p = randVecInSphere( 15.0f );
      Vector v = randVecInSphere( 0.1f );
      data[i] = { p.x, p.y, p.z, 0.0f, v.x, v.y, v.z, 0.0f };
   }
   // Predator (last element)
   Vector pp = randVecInSphere( 20.0f );
   Vector pv = randVecInSphere( 0.05f );
   data[n] = { pp.x, pp.y, pp.z, 1.0f, pv.x, pv.y, pv.z, 0.0f };

   GLsizeiptr bufSize = total * sizeof( BoidGPU );

   glBindBuffer( GL_SHADER_STORAGE_BUFFER, ssbo[0] );
   glBufferData( GL_SHADER_STORAGE_BUFFER, bufSize, data.data(), GL_DYNAMIC_DRAW );

   glBindBuffer( GL_SHADER_STORAGE_BUFFER, ssbo[1] );
   glBufferData( GL_SHADER_STORAGE_BUFFER, bufSize, nullptr, GL_DYNAMIC_DRAW );

   glBindBuffer( GL_SHADER_STORAGE_BUFFER, 0 );
   readIdx = 0;
}

// ============================================================
// Per-Frame Update
// ============================================================

void GLViewBoidSwarm::updateWorld()
{
   GLView::updateWorld();

   if( boid_gui.resetRequested )
   {
      boid_gui.resetRequested = false;
      resetSimulation();
   }

   if( boid_gui.isPaused )
      return;

   int n = boid_gui.numBoids;
   int writeIdx = 1 - readIdx;

   glUseProgram( computeProgram );

   // Set uniforms
   glUniform1i( glGetUniformLocation( computeProgram, "u_numBoids" ),  n );
   glUniform1f( glGetUniformLocation( computeProgram, "u_sepWeight" ), boid_gui.separationWeight );
   glUniform1f( glGetUniformLocation( computeProgram, "u_aliWeight" ), boid_gui.alignmentWeight );
   glUniform1f( glGetUniformLocation( computeProgram, "u_cohWeight" ), boid_gui.cohesionWeight );
   glUniform1f( glGetUniformLocation( computeProgram, "u_bndWeight" ), boid_gui.boundaryWeight );
   glUniform1f( glGetUniformLocation( computeProgram, "u_fleWeight" ), boid_gui.fleeWeight );
   glUniform1f( glGetUniformLocation( computeProgram, "u_obsWeight" ), boid_gui.obstacleWeight );
   glUniform1f( glGetUniformLocation( computeProgram, "u_sepRadius" ), boid_gui.separationRadius );
   glUniform1f( glGetUniformLocation( computeProgram, "u_neiRadius" ), boid_gui.neighborRadius );
   glUniform1f( glGetUniformLocation( computeProgram, "u_feaRadius" ), boid_gui.fearRadius );
   glUniform1f( glGetUniformLocation( computeProgram, "u_bndRadius" ), boid_gui.boundaryRadius );
   glUniform1f( glGetUniformLocation( computeProgram, "u_maxSpeed" ),  boid_gui.maxSpeed );
   glUniform1f( glGetUniformLocation( computeProgram, "u_predSpeed" ), boid_gui.predatorSpeed );
   glUniform1f( glGetUniformLocation( computeProgram, "u_dt" ),        0.05f );

   // Obstacle uniforms
   glUniform1i( glGetUniformLocation( computeProgram, "u_numObstacles" ), NUM_OBSTACLES );
   for( int i = 0; i < NUM_OBSTACLES; ++i )
   {
      std::string name = "u_obstacles[" + std::to_string( i ) + "]";
      glUniform4f( glGetUniformLocation( computeProgram, name.c_str() ),
                   obstaclePositions[i].x, obstaclePositions[i].y,
                   obstaclePositions[i].z, obstacleAvoidRadius );
   }

   // Bind SSBOs: read from readIdx, write to writeIdx
   glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 0, ssbo[readIdx] );
   glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 1, ssbo[writeIdx] );

   // Dispatch one thread per entity
   glDispatchCompute( ( n + 1 + 255 ) / 256, 1, 1 );

   // Barrier: ensure compute writes are visible to vertex shader reads
   glMemoryBarrier( GL_SHADER_STORAGE_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT );

   // Swap buffers
   readIdx = writeIdx;

   glUseProgram( 0 );
}

// ============================================================
// Rendering (called from ImGui callback, like ChaosGame)
// ============================================================

void GLViewBoidSwarm::renderBoids()
{
   if( !renderProgram ) return;

   int n = boid_gui.numBoids;

   Mat4 view = this->cam->getCameraViewMatrix();
   Mat4 proj = this->cam->getCameraProjectionMatrix();

   glUseProgram( renderProgram );

   glUniformMatrix4fv( glGetUniformLocation( renderProgram, "u_view" ), 1, GL_FALSE, view.getPtr() );
   glUniformMatrix4fv( glGetUniformLocation( renderProgram, "u_proj" ), 1, GL_FALSE, proj.getPtr() );

   // Bind the latest SSBO for the vertex shader to read positions/velocities
   glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 0, ssbo[readIdx] );

   glEnable( GL_DEPTH_TEST );
   glBindVertexArray( boidVAO );

   // Draw boids: teal, small
   glUniform4f( glGetUniformLocation( renderProgram, "u_color" ), 0.0f, 0.7f, 0.85f, 1.0f );
   glUniform1f( glGetUniformLocation( renderProgram, "u_scale" ), 0.5f );
   glUniform1i( glGetUniformLocation( renderProgram, "u_instanceOffset" ), 0 );
   glDrawElementsInstanced( GL_TRIANGLES, 12, GL_UNSIGNED_INT, 0, n );

   // Draw predator: red, large
   glUniform4f( glGetUniformLocation( renderProgram, "u_color" ), 0.85f, 0.15f, 0.15f, 1.0f );
   glUniform1f( glGetUniformLocation( renderProgram, "u_scale" ), 1.5f );
   glUniform1i( glGetUniformLocation( renderProgram, "u_instanceOffset" ), n );
   glDrawElementsInstanced( GL_TRIANGLES, 12, GL_UNSIGNED_INT, 0, 1 );

   glBindVertexArray( 0 );
   glUseProgram( 0 );
}

// ============================================================
// Event Handlers
// ============================================================

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

// ============================================================
// Scene Setup
// ============================================================

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

   this->cam->setPosition( 0, 0, 60 );

   std::srand( static_cast<unsigned int>( std::time( nullptr ) ) );

   // ---- Light ----
   {
      float ga = 0.2f;
      ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
      WOLight* light = WOLight::New();
      light->isDirectionalLight( true );
      light->setPosition( Vector( 0, 0, 100 ) );
      light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
      light->setLabel( "Light" );
      worldLst->push_back( light );
   }

   // ---- SkyBox (water theme) ----
   {
      std::string skyBox = ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_water+6.jpg";
      WO* wo = WOSkyBox::New( skyBox, this->getCameraPtrPtr() );
      wo->setPosition( Vector( 0, 0, 0 ) );
      wo->setLabel( "Sky Box" );
      wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      worldLst->push_back( wo );
   }

   // ---- Obstacle boxes ----
   {
      std::string cubeModel = ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl";
      for( int i = 0; i < NUM_OBSTACLES; ++i )
      {
         WO* obs = WO::New( cubeModel, Vector( 1.5f, 1.5f, 1.5f ), MESH_SHADING_TYPE::mstFLAT );
         obs->setPosition( obstaclePositions[i] );
         obs->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
         obs->upon_async_model_loaded( [obs]()
            {
               ModelMeshSkin& skin = obs->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 );
               skin.setAmbient( aftrColor4f( 0.15f, 0.35f, 0.15f, 1.0f ) );
               skin.setDiffuse( aftrColor4f( 0.2f, 0.5f, 0.25f, 1.0f ) );
               skin.setSpecular( aftrColor4f( 0.1f, 0.2f, 0.1f, 1.0f ) );
               skin.setSpecularCoefficient( 5 );
            } );
         obs->setLabel( "Obstacle" );
         worldLst->push_back( obs );
         obstacleWOs[i] = obs;
      }
   }

   // ---- Translucent aquarium sphere ----
   {
      this->aquarium = WO::New();
      MGLIndexedGeometry* mglSphere = MGLIndexedGeometry::New( this->aquarium );
      IndexedGeometrySphereTriStrip* geoSphere = IndexedGeometrySphereTriStrip::New(
         boid_gui.boundaryRadius, 32, 32, true, true );
      mglSphere->setIndexedGeometry( geoSphere );
      this->aquarium->setModel( mglSphere );
      this->aquarium->setPosition( Vector( 0, 0, 0 ) );
      this->aquarium->renderOrderType = RENDER_ORDER_TYPE::roTRANSPARENT;
      this->aquarium->setLabel( "Aquarium" );

      // Semi-transparent blue tint
      this->aquarium->getModel()->getSkin().setAmbient( aftrColor4f( 0.2f, 0.4f, 0.7f, 0.12f ) );
      this->aquarium->getModel()->getSkin().setDiffuse( aftrColor4f( 0.3f, 0.5f, 0.8f, 0.12f ) );

      this->worldLst->push_back( this->aquarium );
   }

   // ---- ImGui ----
   {
      this->gui = WOImGui::New( nullptr );
      gui->setLabel( "My Gui" );

      auto woEditFunc = [this]() {
         this->wo_editor.draw( this->getLastSelectionQuery(),
                               *this->getWorldContainer(),
                               this->getCamera_functor() );
      };
      auto showDemoWindow_ImGui    = [this]() { ImGui::ShowDemoWindow(); };
      auto showDemoWindow_AftrDemo = [this]() { WOImGui::draw_AftrImGui_Demo( this->gui ); };
      auto showDemoWindow_ImGuiPlot = [this]() { ImPlot::ShowDemoWindow(); };
      auto show_boid_controls = [this]() { this->boid_gui.draw(); };

      this->gui->subscribe_drawImGuiWidget(
         [=,this]()
         {
            // Render boids inside the ImGui callback (same pattern as ChaosGame)
            this->renderBoids();

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
