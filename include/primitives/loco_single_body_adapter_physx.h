#pragma once

#include <loco_common.h>
#include <primitives/loco_single_body_adapter.h>

namespace loco {
namespace primitives {
    class TSingleBody;
}}

namespace loco {
namespace primitives {

    class TPhysxSingleBodyAdapter : public TISingleBodyAdapter
    {
    public :

        TPhysxSingleBodyAdapter( TSingleBody* body_ref );

        TPhysxSingleBodyAdapter( const TPhysxSingleBodyAdapter& other ) = delete;

        TPhysxSingleBodyAdapter& operator=( const TPhysxSingleBodyAdapter& other ) = delete;

        ~TPhysxSingleBodyAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetTransform( const TMat4& transform ) override;

        void SetLinearVelocity( const TVec3& linear_vel ) override;

        void SetAngularVelocity( const TVec3& angular_vel ) override;

        void SetForceCOM( const TVec3& force ) override;

        void SetTorqueCOM( const TVec3& torque ) override;

        void GetTransform( TMat4& dst_transform ) override;

        void GetLinearVelocity( TVec3& dst_linear_vel ) override;

        void GetAngularVelocity( TVec3& dst_angular_vel ) override;

        void SetPhysxPhysics( PxPhysics* px_physics_ref );

        void SetPhysxCooking( PxCooking* px_cooking_ref );

        void SetPhysxScene( PxScene* px_scene_ref ) { m_PxSceneRef = px_scene_ref; }

        PxRigidActor* physx_rigid_actor() { return m_PxRigidActorObj.get(); }

        const PxRigidActor* physx_rigid_actor() const { return m_PxRigidActorObj.get(); }

    private :

        bool _IsRigidStatic();

    private :

        PxPhysics* m_PxPhysicsRef = nullptr;

        PxScene* m_PxSceneRef = nullptr;

        PxCooking* m_PxCookingRef = nullptr;

        std::unique_ptr<PxRigidActor, px::PxRigidActorDeleter> m_PxRigidActorObj = nullptr;
    };
}}