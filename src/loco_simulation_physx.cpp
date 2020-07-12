
#include <loco_simulation_physx.h>

using namespace physx;

namespace loco {

    TPhysxSimulation::TPhysxSimulation( TScenario* scenario_ref )
        : TISimulation( scenario_ref )
    {
        m_BackendId = "PHYSX";

        m_PxFoundation = std::unique_ptr<PxFoundation, px::PxFoundationDeleter>( 
                                    PxCreateFoundation( PX_PHYSICS_VERSION, m_PxAllocator, m_PxErrorCallback ) );
        m_PxPhysics = std::unique_ptr<PxPhysics, px::PxPhysicsDeleter>(
                                    PxCreatePhysics( PX_PHYSICS_VERSION, *m_PxFoundation, PxTolerancesScale(), false, NULL ) );
        m_PxDispatcher = std::unique_ptr<PxDefaultCpuDispatcher, px::PxDefaultCpuDispatcherDeleter>(
                                    PxDefaultCpuDispatcherCreate( 0 ) );

        PxSceneDesc px_scene_desc( m_PxPhysics->getTolerancesScale() );
        px_scene_desc.gravity = px::vec3_to_px( m_Gravity );
        px_scene_desc.cpuDispatcher = m_PxDispatcher.get();
        px_scene_desc.filterShader = PxDefaultSimulationFilterShader;
        m_PxScene = std::unique_ptr<PxScene, px::PxSceneDeleter>( m_PxPhysics->createScene( px_scene_desc ) );

        PxCookingParams px_cooking_params( m_PxPhysics->getTolerancesScale() );
        m_PxCooking = std::unique_ptr<PxCooking, px::PxCookingDeleter>( 
                                        PxCreateCooking( PX_PHYSICS_VERSION, *m_PxFoundation, px_cooking_params ) );
        if ( !m_PxCooking )
            LOCO_CORE_ERROR( "TPhysxSimulation >>> Unable to initialize cooking library" );
    }

    TPhysxSimulation::~TPhysxSimulation()
    {
        m_PxScene = nullptr;
        m_PxDispatcher = nullptr;
        m_PxPhysics = nullptr;
        m_PxCooking = nullptr;
        m_PxFoundation = nullptr;
    }

    bool TPhysxSimulation::_InitializeInternal()
    {
        return true;
    }

    void TPhysxSimulation::_PreStepInternal()
    {
        // Do nothing here, as call to wrappers is enough (made in base)
    }

    void TPhysxSimulation::_SimStepInternal( const TScalar& dt )
    {
        LOCO_CORE_ASSERT( m_PxScene, "TPhysSimulation::_SimStepInternal >>> PxScene object is required, but got nullptr instead" );
        const double sim_step_time = ( dt <= 0 ) ? m_FixedTimeStep : dt;
        const ssize_t sim_num_substeps = ssize_t(sim_step_time / m_FixedTimeStep);
        for ( ssize_t i = 0; i < sim_num_substeps; i++ )
        {
            m_PxScene->simulate( m_FixedTimeStep );
            m_WorldTime += m_FixedTimeStep;
        }
        // @todo: check if fetch-results should be called once per simulate call
        m_PxScene->fetchResults( true );
    }

    void TPhysxSimulation::_PostStepInternal()
    {

    }

    void TPhysxSimulation::_ResetInternal()
    {
        // Do nothing here, as call to wrappers is enough (made in base)
    }

    void TPhysxSimulation::_SetTimeStepInternal( const TScalar& time_step )
    {
        // Do nothing here, as the fixed timestep is set in each call to scene->simulate
    }

    void TPhysxSimulation::_SetGravityInternal( const TVec3& gravity )
    {
        LOCO_CORE_ASSERT( m_PxScene, "TPhysxSimulation::_SetGravityInternal >>> PxScene object is required, but got nullptr instead" );
        m_PxScene->setGravity( px::vec3_to_px( gravity ) );
    }

    extern "C" TISimulation* simulation_create( TScenario* scenario_ref )
    {
        return new loco::TPhysxSimulation( scenario_ref );
    }
}