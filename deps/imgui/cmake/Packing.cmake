#this should have at least one - in it
set( CPACK_PACKAGE_NAME ${PROJECT_NAME} CACHE STRING "imgui Debian Package" )

# which is useful in case of packing only selected components instead of the whole thing
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "imgui Debian Package"
	CACHE STRING "imgui Debian Package"
)

#Name of the company
set(CPACK_PACKAGE_VENDOR "MROVER")

#Makes it so special characters are included in variable names
set(CPACK_VERBATIM_VARIABLES YES)

#Set up the directories for where the package will install
set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
set(CPACK_OUTPUT_FILE_PREFIX "../../../pkg/")

# https://unix.stackexchange.com/a/11552/254512
#set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/some")#/${CMAKE_PROJECT_VERSION}")

#These are actually required for CPACK to execute properly
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})

set(CPACK_PACKAGE_CONTACT "PLEASEDONTCONTACTME@THISISNOTAREALEMAIL.net")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "John A")

set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

# package name for deb. If set, then instead of some-application-0.9.2-Linux.deb
# you'll get some-application_0.9.2_amd64.deb (note the underscores too)
set(CPACK_DEBIAN_FILE_NAME "libimgui-dev.deb")
# that is if you want every group to have its own package,
# although the same will happen if this is not set (so it defaults to ONE_PER_GROUP)
# and CPACK_DEB_COMPONENT_INSTALL is set to YES
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)#ONE_PER_GROUP)
# without this you won't be able to pack only specified component
set(CPACK_DEB_COMPONENT_INSTALL YES)

include(CPack)
message(STATUS "Components to pack: ${CPACK_COMPONENTS_ALL}")