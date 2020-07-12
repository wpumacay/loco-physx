
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_physx.h>

TEST( TestLocoPhysxSimulation, TestPhysxSimulationFunctionality )
{
    auto scenario = std::make_unique<loco::TScenario>();
    auto body_obj = std::make_unique<loco::primitives::TCapsule>( "capsule", 0.2f, 0.5f, loco::TVec3( 1.0f, 1.0f, 1.0f ), loco::TMat3() );
    scenario->AddSingleBody( std::move( body_obj ) );

    auto simulation = std::make_unique<loco::TPhysxSimulation>( scenario.get() );
    simulation->Initialize();
    simulation->Step();
    simulation->Reset();
    simulation->Pause();
    simulation->Resume();
    EXPECT_EQ( simulation->backendId(), "PHYSX" );
    EXPECT_TRUE( simulation->px_foundation() != nullptr );
    EXPECT_TRUE( simulation->px_physics() != nullptr );
    EXPECT_TRUE( simulation->px_scene() != nullptr );
    EXPECT_TRUE( simulation->px_cpu_dispatcher() != nullptr );
}