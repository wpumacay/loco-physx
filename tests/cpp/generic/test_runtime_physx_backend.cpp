
#include <loco.h>
#include <gtest/gtest.h>

TEST( TestLocoRuntimeMujocoBackend, TestRuntimeMujocoBackend )
{
    loco::InitUtils();

    auto scenario = std::make_unique<loco::TScenario>();
    auto runtime = std::make_unique<loco::TRuntime>( loco::config::physics::PHYSX,
                                                     loco::config::rendering::NONE );

    auto simulationRef = runtime->CreateSimulation( scenario.get() );
    simulationRef->Step();
    simulationRef->Reset();
    simulationRef->Pause();
    simulationRef->Resume();
    EXPECT_EQ( simulationRef->backendId(), "PHYSX" );

    auto visualizerRef = runtime->CreateVisualizer( scenario.get() );
    auto cameraRef = visualizerRef->CreateCamera( "cam_orbit_0", 
                                                  loco::visualizer::eVizCameraType::ORBIT,
                                                  { 3.0f, 3.0f, 3.0f },
                                                  { 0.0f, 0.0f, 0.0f } );
    auto lightRef = visualizerRef->CreateLight( "light_point_0",
                                                loco::visualizer::eVizLightType::POINT,
                                                { 0.4f, 0.4f, 0.4f },
                                                { 0.8f, 0.8f, 0.8f },
                                                { 0.8f, 0.8f, 0.8f } );
    visualizerRef->Render();
    visualizerRef->Reset();
    EXPECT_EQ( visualizerRef->backendId(), "null" );
    EXPECT_TRUE( visualizerRef->HasCameraNamed( "cam_orbit_0" ) );
    EXPECT_TRUE( visualizerRef->HasLightNamed( "light_point_0" ) );
    EXPECT_TRUE( visualizerRef->GetCameraByName( "cam_orbit_0" ) != nullptr );
    EXPECT_TRUE( visualizerRef->GetLightByName( "light_point_0" ) != nullptr );
    EXPECT_TRUE( tinymath::allclose( visualizerRef->GetCameraByName( "cam_orbit_0" )->position(), cameraRef->position() ) );
    EXPECT_TRUE( tinymath::allclose( visualizerRef->GetLightByName( "light_point_0" )->ambient(), lightRef->ambient() ) );

    runtime->DestroySimulation();
    runtime->DestroyVisualizer();
}