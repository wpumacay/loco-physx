
#include <loco_common_physx.h>

namespace loco {
namespace px {

    PxVec3 vec3_to_px( const TVec3& vec )
    {
        return PxVec3( vec.x(), vec.y(), vec.z() );
    }

    PxVec4 vec4_to_px( const TVec4& vec )
    {
        return PxVec4( vec.x(), vec.y(), vec.z(), vec.w() );
    }

    PxMat33 mat3_to_px( const TMat3& mat )
    {
        return PxMat33( PxVec3( mat( 0, 0 ), mat( 1, 0 ), mat( 2, 0 ) ),
                        PxVec3( mat( 0, 1 ), mat( 1, 1 ), mat( 2, 1 ) ),
                        PxVec3( mat( 0, 2 ), mat( 1, 2 ), mat( 2, 2 ) ) );
    }

    PxMat44 mat4_to_px( const TMat4& mat )
    {
        return PxMat44( PxVec4( mat( 0, 0 ), mat( 1, 0 ), mat( 2, 0 ), mat( 3, 0 ) ),
                        PxVec4( mat( 0, 1 ), mat( 1, 1 ), mat( 2, 1 ), mat( 3, 1 ) ),
                        PxVec4( mat( 0, 2 ), mat( 1, 2 ), mat( 2, 2 ), mat( 3, 2 ) ),
                        PxVec4( mat( 0, 3 ), mat( 1, 3 ), mat( 2, 3 ), mat( 3, 3 ) ) );
    }

    PxTransform tf_to_px( const TMat4& tf )
    {
        return PxTransform( mat4_to_px( tf ) );
    }

    TVec3 vec3_from_px( const PxVec3& vec )
    {
        return TVec3( vec.x, vec.y, vec.z );
    }

    TVec4 vec4_from_px( const PxVec4& vec )
    {
        return TVec4( vec.x, vec.y, vec.z, vec.w );
    }

    TMat3 mat3_from_px( const PxMat33& mat )
    {
        return TMat3( { mat( 0, 0 ), mat( 0, 1 ), mat( 0, 2 ),
                        mat( 1, 0 ), mat( 1, 1 ), mat( 1, 2 ),
                        mat( 2, 0 ), mat( 2, 1 ), mat( 2, 2 ) } );
    }

    TMat4 mat4_from_px( const PxMat44& mat )
    {
        return TMat3( { mat( 0, 0 ), mat( 0, 1 ), mat( 0, 2 ), mat( 0, 3 ),
                        mat( 1, 0 ), mat( 1, 1 ), mat( 1, 2 ), mat( 1, 3 ),
                        mat( 2, 0 ), mat( 2, 1 ), mat( 2, 2 ), mat( 2, 3 ),
                        mat( 3, 0 ), mat( 3, 1 ), mat( 3, 2 ), mat( 3, 3 ) } );
    }

    TMat4 tf_from_px( const PxTransform& tf )
    {
        return mat4_from_px( PxMat44( tf ) );
    }

    double compute_primitive_volume( const eShapeType& shape, const TVec3& size )
    {
        /**/ if ( shape == eShapeType::BOX ) return size.x() * size.y() * size.z();
        else if ( shape == eShapeType::SPHERE ) return (4. / 3.) * loco::PI * size.x() * size.x() * size.x();
        else if ( shape == eShapeType::CYLINDER ) return loco::PI * size.x() * size.x() * size.y();
        else if ( shape == eShapeType::CAPSULE ) return loco::PI * size.x() * size.x() * size.y() + (4. / 3.) * loco::PI * size.x() * size.x() * size.x();
        else if ( shape == eShapeType::ELLIPSOID ) return (4. / 3.) * loco::PI * size.x() * size.y() * size.z();

        LOCO_CORE_ERROR( "compute_primitive_volume >>> unsupported shape: {0}", ToString( shape ) );
        return 1.0;
    }

    std::unique_ptr<PxShape, PxShapeDeleter> CreateCollisionShape(
                                                    PxPhysics* px_physics,
                                                    PxMaterial* px_material,
                                                    const TShapeData& data )
    {
        PxShape* px_shape = nullptr;
        const auto shape_type = data.type;
        /**/ if ( shape_type == eShapeType::BOX )
        {
            const float half_x = 0.5f * data.size.x();
            const float half_y = 0.5f * data.size.y();
            const float half_z = 0.5f * data.size.z();
            auto box_geometry = PxBoxGeometry( half_x, half_y, half_z );
            px_shape = px_physics->createShape( box_geometry, *px_material );
        }
        else if ( shape_type == eShapeType::SPHERE )
        {
            const float radius = data.size.x();
            auto sphere_geometry = PxSphereGeometry( radius );
            px_shape = px_physics->createShape( sphere_geometry, *px_material );
        }
        else if ( shape_type == eShapeType::CYLINDER )
        {
            // @todo: implement me
        }
        else if ( shape_type == eShapeType::CAPSULE )
        {
            const float radius = data.size.x();
            const float half_height = 0.5f * data.size.y();
            auto capsule_geometry = PxCapsuleGeometry( radius, half_height );
            px_shape = px_physics->createShape( capsule_geometry, *px_material );
            // Set local pose such that capsule-axis is aligned with z-axis
            PxTransform local_pose( PxQuat( PxHalfPi, PxVec3( 0.0f, 1.0f, 0.0f ) ) );
            px_shape->setLocalPose( local_pose );
        }
        else if ( shape_type == eShapeType::ELLIPSOID )
        {
            // @todo: implement me
        }
        else if ( shape_type == eShapeType::MESH )
        {
            // @todo: implement me
        }
        else if ( shape_type == eShapeType::HFIELD )
        {
            // @todo: implement me
        }

        return std::unique_ptr<PxShape, px::PxShapeDeleter>( px_shape );
    }
}}