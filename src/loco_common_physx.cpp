
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

    std::unique_ptr<PxShape, PxShapeDeleter> CreateCollisionShape( PxPhysics* px_physics,
                                                                   PxCooking* px_cooking,
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
            px_shape = CreateConvexHullShape( px_physics, px_cooking, px_material, CreateCylinderVertices( data ) );
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
            px_shape = CreateConvexHullShape( px_physics, px_cooking, px_material, CreateEllipsoidVertices( data ) );
        }
        else if ( shape_type == eShapeType::CONVEX_MESH )
        {
            px_shape = CreateConvexHullShape( px_physics, px_cooking, px_material, CreateConvexMeshVertices( data ) );
        }
        else if ( shape_type == eShapeType::TRIANGULAR_MESH )
        {
            // @todo: implement me
        }
        else if ( shape_type == eShapeType::HEIGHTFIELD )
        {
            // @todo: implement me
        }

        return std::unique_ptr<PxShape, px::PxShapeDeleter>( px_shape );
    }

    PxShape* CreateConvexHullShape( PxPhysics* px_physics,
                                    PxCooking* px_cooking,
                                    PxMaterial* px_material,
                                    const std::vector<TVec3>& vertices )
    {
        PxConvexMeshDesc px_mesh_desc;
        px_mesh_desc.points.count = vertices.size();
        px_mesh_desc.points.stride = sizeof(TVec3);
        px_mesh_desc.points.data = vertices.data();
        px_mesh_desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

        PxDefaultMemoryOutputStream px_data_buf;
        PxConvexMeshCookingResult::Enum px_cooking_result;
        if ( !px_cooking->cookConvexMesh( px_mesh_desc, px_data_buf, &px_cooking_result ) )
        {
            /**/ if ( px_cooking_result == PxConvexMeshCookingResult::Enum::eZERO_AREA_TEST_FAILED )
                LOCO_CORE_ERROR( "CreateConvexHullShape >>> Convex mesh cooking failed, algorithm couldn't find 4 initial vertices without a small triangle" );
            else if ( px_cooking_result == PxConvexMeshCookingResult::Enum::ePOLYGONS_LIMIT_REACHED )
                LOCO_CORE_ERROR( "CreateConvexHullShape >>> Convex mesh cooking succeeded, but reached limit of 255 polygons" );
            else if ( px_cooking_result == PxConvexMeshCookingResult::Enum::eFAILURE )
                LOCO_CORE_ERROR( "CreateConvexHullShape >>> Unknown error happened while cooking a mesh" );
            return nullptr;
        }

        PxDefaultMemoryInputData px_input_buf( px_data_buf.getData(), px_data_buf.getSize() );
        PxConvexMesh* px_convex_mesh = px_physics->createConvexMesh( px_input_buf );
        return px_physics->createShape( PxConvexMeshGeometry( px_convex_mesh ), *px_material );
    }

    std::vector<TVec3> CreateCylinderVertices( const TShapeData& data )
    {
        constexpr ssize_t ndiv = 10;
        constexpr ssize_t num_vertices = 2 * ndiv;
        std::vector<TVec3> vertices( num_vertices );

        const float radius = data.size.x();
        const float height = data.size.y();
        for ( ssize_t i = 0; i < ndiv; i++ )
        {
            float x = radius * std::cos( 2.0f * loco::PI * ( ( (float) i ) / ndiv ) );
            float y = radius * std::sin( 2.0f * loco::PI * ( ( (float) i ) / ndiv ) );
            vertices[2 * i + 0].x() = x; vertices[2 * i + 0].y() = y; vertices[2 * i + 0].z() = -0.5f * height;
            vertices[2 * i + 1].x() = x; vertices[2 * i + 1].y() = y; vertices[2 * i + 1].z() = 0.5f * height;
        }
        return vertices;
    }

    std::vector<TVec3> CreateEllipsoidVertices( const TShapeData& data )
    {
        constexpr ssize_t ndiv_1 = 14;
        constexpr ssize_t ndiv_2 = 14;
        constexpr ssize_t num_vertices = ( ndiv_1 + 1 ) * ( ndiv_2 + 1 ); // keep num-vertices below 255
        std::vector<TVec3> vertices( num_vertices );

        for ( ssize_t i = 0; i <= ndiv_1; i++ )
        {
            for ( ssize_t j = 0; j <= ndiv_2; j++ )
            {
                float ang_theta = 2.0 * loco::PI * (float)i / ndiv_1;
                float ang_phi = -0.5 * loco::PI + loco::PI * (float)j / ndiv_2;
                vertices[i * ( ndiv_2 + 1 ) + j].x() = data.size.x() * std::cos( ang_theta ) * std::cos( ang_phi );
                vertices[i * ( ndiv_2 + 1 ) + j].y() = data.size.y() * std::sin( ang_theta ) * std::cos( ang_phi );
                vertices[i * ( ndiv_2 + 1 ) + j].z() = data.size.z() * std::sin( ang_phi );
            }
        }
        return vertices;
    }

    std::vector<TVec3> CreateConvexMeshVertices( const TShapeData& data )
    {
        std::vector<TVec3> mesh_vertices;
        if ( data.mesh_data.filename != "" )
            _CollectMeshVerticesFromFile( mesh_vertices, data );
        else if ( data.mesh_data.vertices.size() > 0 && data.mesh_data.faces.size() > 0 )
            _CollectMeshVerticesFromUser( mesh_vertices, data );
        else
            LOCO_CORE_ERROR( "CreateConvexMeshVertices >>> Tried to construct a mesh without any data" );
        return mesh_vertices;
    }

    void _CollectMeshVerticesFromFile( std::vector<TVec3>& mesh_vertices, const TShapeData& data )
    {
        const std::string filepath = data.mesh_data.filename;
        auto assimp_scene = std::unique_ptr<const aiScene,aiSceneDeleter>( aiImportFile( filepath.c_str(),
                                                                                aiProcessPreset_TargetRealtime_MaxQuality |
                                                                                aiProcess_PreTransformVertices ) );
        if ( !assimp_scene )
        {
            LOCO_CORE_ERROR( "_CollectMeshVerticesFromFile >>> Couldn't open model {0}", filepath );
            return;
        }

        std::stack<const aiNode*> dfs_traversal;
        dfs_traversal.push( assimp_scene->mRootNode );
        while ( !dfs_traversal.empty() )
        {
            auto assimp_node = dfs_traversal.top();
            dfs_traversal.pop();
            if ( !assimp_node )
                continue;

            // It's enough with the vertex data, as we'll only construct the convex-hull
            for ( ssize_t i = 0; i < assimp_node->mNumMeshes; i++ )
            {
                const aiMesh* assimp_mesh = assimp_scene->mMeshes[assimp_node->mMeshes[i]];
                for ( ssize_t v = 0; v < assimp_mesh->mNumVertices; v++ )
                    mesh_vertices.push_back( { data.size.x() * assimp_mesh->mVertices[v].x,
                                               data.size.y() * assimp_mesh->mVertices[v].y,
                                               data.size.z() * assimp_mesh->mVertices[v].z } );
            }

            for ( ssize_t i = 0; i < assimp_node->mNumChildren; i++ )
                dfs_traversal.push( assimp_node->mChildren[i] );
        }
    }

    void _CollectMeshVerticesFromUser( std::vector<TVec3>& mesh_vertices, const TShapeData& data )
    {
        // It's enough with the vertex data, as we'll only construct the convex-hull
        if ( data.mesh_data.vertices.size() % 3 != 0 )
            LOCO_CORE_ERROR( "_CollectMeshVerticesFromUser >>> There must be 3 elements per vertex" );

        const std::vector<float>& vertices = data.mesh_data.vertices;
        const ssize_t num_vertices = vertices.size() / 3;
        mesh_vertices.reserve( num_vertices );
        for ( ssize_t i = 0; i < num_vertices; i++ )
            mesh_vertices.push_back( { data.size.x() * vertices[3 * i + 0],
                                       data.size.y() * vertices[3 * i + 1],
                                       data.size.z() * vertices[3 * i + 2] } );
    }

    void aiSceneDeleter::operator() ( const aiScene* assimp_scene ) const
    {
        aiReleaseImport( assimp_scene );
    }
}}