#pragma once

#include <loco_common_physx.h>
#include <primitives/loco_single_body_collider_adapter.h>

namespace loco {
namespace primitives {
    class TSingleBodyCollider;
}}

namespace loco {
namespace primitives {

    constexpr float LOCO_PHYSX_HFIELD_BASE = 1.0f;

    class TPhysxSingleBodyColliderAdapter : public TISingleBodyColliderAdapter
    {
    public :

        TPhysxSingleBodyColliderAdapter( TSingleBodyCollider* collider_ref );

        TPhysxSingleBodyColliderAdapter( const TPhysxSingleBodyColliderAdapter& other ) = delete;

        TPhysxSingleBodyColliderAdapter& operator=( const TPhysxSingleBodyColliderAdapter& other ) = delete;

        ~TPhysxSingleBodyColliderAdapter();

        void Build() override;

        void Initialize() override;

        void ChangeSize( const TVec3& new_size ) override;

        void ChangeVertexData( const std::vector<float>& vertices, const std::vector<int>& faces ) override;

        void ChangeElevationData( const std::vector<float>& heights ) override;

        void ChangeCollisionGroup( int collision_group ) override;

        void ChangeCollisionMask( int collision_mask ) override;

        void ChangeFriction( const TScalar& friction ) override;

        void SetPhysxPhysics( PxPhysics* px_physics_ref ) { m_PxPhysicsRef = px_physics_ref; }

        void SetPhysxCooking( PxCooking* px_cooking_ref ) { m_PxCookingRef = px_cooking_ref; }

        void SetPhysxRigidActor( PxRigidActor* px_rigid_actor_ref ) { m_PxRigidActorRef = px_rigid_actor_ref; }

        PxMaterial* physx_material() { return m_PxMaterialObj.get(); }

        const PxMaterial* physx_material() const { return m_PxMaterialObj.get(); }

        PxShape* physx_shape() { return m_PxShapeObj.get(); }

        const PxShape* physx_shape() const { return m_PxShapeObj.get(); }

    private :

        PxPhysics* m_PxPhysicsRef = nullptr;

        PxCooking* m_PxCookingRef = nullptr;

        PxRigidActor* m_PxRigidActorRef = nullptr;

        std::unique_ptr<PxMaterial, px::PxMaterialDeleter> m_PxMaterialObj = nullptr;

        std::unique_ptr<PxShape, px::PxShapeDeleter> m_PxShapeObj = nullptr;
    };
}}