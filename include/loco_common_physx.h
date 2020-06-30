#pragma once

#include <loco_common.h>
#include <loco_data.h>
#include <PxPhysicsAPI.h>

using namespace physx;

namespace loco {
namespace px {

    struct PxFoundationDeleter
    {
        void operator() ( PxFoundation* px_foundation );
    };

    struct PxPhysicsDeleter
    {
        void operator() ( PxPhysics* px_physics );
    };

    struct PxDefaultCpuDispatcherDeleter
    {
        void operator() ( PxDefaultCpuDispatcher* px_cpu_dispatcher );
    };

    struct PxSceneDeleter
    {
        void operator() ( PxScene* px_scene );
    };

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
}}