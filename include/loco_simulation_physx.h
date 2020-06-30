#pragma once

#include <loco_common_physx.h>
#include <loco_simulation.h>

using namespace physx;

namespace loco {
    class TIVisualizer;
}

namespace loco {
namespace px {

    class TPhysxSimulation : public TISimulation
    {
    public :

        TPhysxSimulation( TScenario* scenario_ref );

        TPhysxSimulation( const TPhysxSimulation& other ) = delete;

        TPhysxSimulation& operator=( const TPhysxSimulation& other ) = delete;

        ~TPhysxSimulation();

        PxPhysics* px_physics() { return m_PxPhysics.get(); }

        const PxPhysics* px_physics() const { return m_PxPhysics.get(); }

        PxFoundation* px_foundation() { return m_PxFoundation.get(); }

        const PxFoundation* px_foundation() const { return m_PxFoundation.get(); }

        PxScene* px_scene() { return m_PxScene.get(); }

        const PxScene* px_scene() const { return m_PxScene.get(); }

        PxDefaultCpuDispatcher* px_cpu_dispatcher() { return m_PxDispatcher.get(); }

        const PxDefaultCpuDispatcher* px_cpu_dispatcher() const { return m_PxDispatcher.get(); }

    protected :

        bool _InitializeInternal() override;

        void _PreStepInternal() override;

        void _SimStepInternal( const TScalar& dt ) override;

        void _PostStepInternal() override;

        void _ResetInternal() override;

        void _SetTimeStepInternal( const TScalar& time_step ) override;

        void _SetGravityInternal( const TVec3& gravity ) override;

    private :

        std::unique_ptr<PxFoundation, PxFoundationDeleter> m_PxFoundation;

        std::unique_ptr<PxPhysics, PxPhysicsDeleter> m_PxPhysics;

        std::unique_ptr<PxScene, PxSceneDeleter> m_PxScene;

        std::unique_ptr<PxDefaultCpuDispatcher, PxDefaultCpuDispatcherDeleter> m_PxDispatcher;

        PxDefaultAllocator m_PxAllocator;

        PxDefaultErrorCallback m_PxErrorCallback;
    };

    extern "C" TISimulation* simulation_create( TScenario* scenario_ref );
}}