
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
}}