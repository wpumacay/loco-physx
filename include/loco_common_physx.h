#pragma once

#include <loco_common.h>
#include <loco_data.h>
#include <PxPhysicsAPI.h>

using namespace physx;

#define LOCO_PHYSX_UNIQUE_PTR_DELETER(X)        \
    struct X##Deleter                           \
    {                                           \
        void operator() ( X* px_obj_pointer )   \
        {                                       \
            if ( px_obj_pointer )               \
                px_obj_pointer->release();      \
        }                                       \
    };

namespace loco {
namespace px {

    LOCO_PHYSX_UNIQUE_PTR_DELETER(PxFoundation)
    LOCO_PHYSX_UNIQUE_PTR_DELETER(PxPhysics)
    LOCO_PHYSX_UNIQUE_PTR_DELETER(PxDefaultCpuDispatcher)
    LOCO_PHYSX_UNIQUE_PTR_DELETER(PxScene)
    LOCO_PHYSX_UNIQUE_PTR_DELETER(PxMaterial)
    LOCO_PHYSX_UNIQUE_PTR_DELETER(PxShape)
    LOCO_PHYSX_UNIQUE_PTR_DELETER(PxRigidDynamic)

    PxVec3 vec3_to_px( const TVec3& vec );
    PxVec4 vec4_to_px( const TVec4& vec );
    PxMat33 mat3_to_px( const TMat3& mat );
    PxMat44 mat4_to_px( const TMat4& mat );
    PxTransform tf_to_px( const TMat4& tf );

    TVec3 vec3_from_px( const PxVec3& vec );
    TVec4 vec4_from_px( const PxVec4& vec );
    TMat3 mat3_from_px( const PxMat33& mat );
    TMat4 mat4_from_px( const PxMat44& mat );
    TMat4 tf_from_px( const PxTransform& tf );

    // @todo: move to loco-core
    double compute_primitive_volume( const eShapeType& shape, const TVec3& size );

    std::unique_ptr<PxShape, PxShapeDeleter> CreateCollisionShape(
                                                    PxPhysics* px_physics,
                                                    PxMaterial* px_material,
                                                    const TShapeData& data );
}}