message( "LOCO::PHYSX::tests::cpp >>> Configuring C/C++ loco-physx-specific tests" )

include_directories( "${LOCO_PHYSX_INCLUDE_DIRS}" )

function( FcnBuildPhysxTestSpecific sources_list executable_name )
    add_executable( ${executable_name} ${sources_list} )
    target_link_libraries( ${executable_name} locoPhysicsPHYSX loco_core gtest_main )
    add_test( NAME "${executable_name}_test" COMMAND "${executable_name}" )
endfunction()

FILE( GLOB TestPhysxSources *.cpp )

foreach( test_physx_file ${TestPhysxSources} )
    string( REPLACE ".cpp" "" executable_long_name ${test_physx_file} )
    get_filename_component( executable_name ${executable_long_name} NAME )
    FcnBuildPhysxTestSpecific( ${test_physx_file} ${executable_name} )
endforeach( test_physx_file )