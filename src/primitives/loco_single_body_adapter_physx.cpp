
#include <primitives/loco_single_body_adapter_physx.h>

namespace loco {
namespace primitives {

    TPhysxSingleBodyAdapter::TPhysxSingleBodyAdapter( TSingleBody* body_ref )
        : TISingleBodyAdapter( body_ref )
    {
        LOCO_CORE_ASSERT( body_ref, "TPhysxSingleBodyAdapter >>> adaptee (body) should be a valid \
                          reference (nullptr given)" );
        LOCO_CORE_ASSERT( body_ref->collider(), "TPhysxSingleBodyAdapter >>> body {0} doesn't have \
                          a valid collider (found nullptr)", body_ref->name() );
    }

    TPhysxSingleBodyAdapter::~TPhysxSingleBodyAdapter()
    {
        m_PxPhysicsRef = nullptr;
        m_PxSceneRef = nullptr;
        m_PxCookingRef = nullptr;
    }

    void TPhysxSingleBodyAdapter::Build()
    {

    }

    void TPhysxSingleBodyAdapter::Initialize()
    {

    }

    void TPhysxSingleBodyAdapter::Reset()
    {

    }

    void TPhysxSingleBodyAdapter::SetTransform( const TMat4& transform )
    {

    }

    void TPhysxSingleBodyAdapter::SetLinearVelocity( const TVec3& linear_vel )
    {

    }

    void TPhysxSingleBodyAdapter::SetAngularVelocity( const TVec3& angular_vel )
    {

    }

    void TPhysxSingleBodyAdapter::SetForceCOM( const TVec3& force )
    {

    }

    void TPhysxSingleBodyAdapter::SetTorqueCOM( const TVec3& torque )
    {

    }

    void TPhysxSingleBodyAdapter::GetTransform( TMat4& dst_transform )
    {

    }

    void TPhysxSingleBodyAdapter::GetLinearVelocity( TVec3& dst_linear_vel )
    {

    }

    void TPhysxSingleBodyAdapter::GetAngularVelocity( TVec3& dst_angular_vel )
    {

    }

    void TPhysxSingleBodyAdapter::SetPhysxPhysics( PxPhysics* px_physics_ref )
    {
        m_PxPhysicsRef = px_physics_ref;
        if ( auto px_collider_adapter = dynamic_cast<TPhysxSingleBodyColliderAdapter*>( m_ColliderAdapter.get() ) )
            px_collider_adapter->SetPhysxPhysics( px_physics_ref );
    }

    void TPhysxSingleBodyAdapter::SetPhysxCooking( PxCooking* px_cooking_ref )
    {
        m_PxCookingRef = px_cooking_ref;
        if ( auto px_collider_adapter = dynamic_cast<TPhysxSingleBodyColliderAdapter*>( m_ColliderAdapter.get() ) )
            px_collider_adapter->SetPhysxCooking( px_cooking_ref );
    }

    bool TPhysxSingleBodyAdapter::_IsRigidStatic()
    {
        return false;
    }
}}