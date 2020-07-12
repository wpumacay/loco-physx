
#include <primitives/loco_single_body_collider_adapter_physx.h>

namespace loco {
namespace primitives {

    TPhysxSingleBodyColliderAdapter::TPhysxSingleBodyColliderAdapter( TSingleBodyCollider* collider_ref )
        : TISingleBodyColliderAdapter( collider_ref ) {}

    TPhysxSingleBodyColliderAdapter::~TPhysxSingleBodyColliderAdapter()
    {
        m_PxPhysicsRef = nullptr;
        m_PxRigidActorRef = nullptr;
        m_PxMaterialObj = nullptr;
        m_PxShapeObj = nullptr;
    }

    void TPhysxSingleBodyColliderAdapter::Build()
    {
        LOCO_CORE_ASSERT( m_PxPhysicsRef, "TPhysxSingleBodyColliderAdapter::Build >>>  must have a valid PxPhysics \
                          to construct colliders. Error found while processing collider {0}", m_ColliderRef->name() );

        const float static_friction = m_ColliderRef->friction();
        const float dynamic_friction = m_ColliderRef->friction();
        const float restitution_coeff = 0.7f;
        m_PxMaterialObj = std::unique_ptr<PxMaterial, px::PxMaterialDeleter>( 
                                    m_PxPhysicsRef->createMaterial( static_friction, dynamic_friction, restitution_coeff ) );
        m_PxShapeObj = px::CreateCollisionShape( m_PxPhysicsRef, m_PxMaterialObj.get(), m_ColliderRef->data() );
        m_PxRigidActorRef->attachShape( *m_PxShapeObj );
        const auto rigid_type = m_PxRigidActorRef->getType();
        const auto shape_type = m_ColliderRef->shape();

        if ( rigid_type == PxActorType::Enum::eRIGID_DYNAMIC && 
             ( shape_type == eShapeType::PLANE || shape_type == eShapeType::HEIGHTFIELD ) )
            LOCO_CORE_ERROR( "TPhyxSingleBodyColliderAdapter::Build >>> shape-type {0} can't be used with "
                             "physx-rigid-actors of dynamic type", loco::ToString( shape_type ) );

        if ( rigid_type == PxActorType::Enum::eRIGID_DYNAMIC )
        {
            if ( auto px_rigid_dynamic = m_PxRigidActorRef->is<PxRigidDynamic>() )
            {
                float body_density = m_ColliderRef->data().density;
                if ( auto parent_body = m_ColliderRef->parent() )
                {
                    const float body_mass = parent_body->data().inertia.mass;
                    if ( body_mass > loco::EPS )
                        body_density = body_mass / px::compute_primitive_volume( shape_type, m_ColliderRef->size() );
                }
                PxRigidBodyExt::updateMassAndInertia( *px_rigid_dynamic, body_density );
            }
        }
    }

    void TPhysxSingleBodyColliderAdapter::Initialize()
    {

    }

    void TPhysxSingleBodyColliderAdapter::ChangeSize( const TVec3& new_size )
    {

    }

    void TPhysxSingleBodyColliderAdapter::ChangeVertexData( const std::vector<float>& vertices, const std::vector<int>& faces )
    {

    }

    void TPhysxSingleBodyColliderAdapter::ChangeElevationData( const std::vector<float>& heights )
    {

    }

    void TPhysxSingleBodyColliderAdapter::ChangeCollisionGroup( int collision_group )
    {

    }

    void TPhysxSingleBodyColliderAdapter::ChangeCollisionMask( int collision_mask )
    {

    }

    void TPhysxSingleBodyColliderAdapter::ChangeFriction( const TScalar& friction )
    {

    }
}}