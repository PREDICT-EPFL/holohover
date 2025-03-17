execute_process(COMMAND git log --pretty=format:'%h' -n 1
                OUTPUT_VARIABLE GIT_REV
                ERROR_QUIET)

# Check whether we got any revision (which isn't
# always the case, e.g. when someone downloaded a zip
# file from Github instead of a checkout)
if ("${GIT_REV}" STREQUAL "")
    set(GIT_REV "N/A")
    set(GIT_DIFF "")
    set(GIT_TAG "N/A")
    set(GIT_BRANCH "N/A")
else()
    execute_process(
        COMMAND bash -c "git describe --abbrev=10 --dirty='*' --always --tags"
        OUTPUT_VARIABLE GIT_TAG)

    string(STRIP "${GIT_TAG}" GIT_TAG)

    execute_process(
        COMMAND bash -c "git describe HEAD --always | xargs git show -s --format=format:%cI"
        OUTPUT_VARIABLE GIT_DATE)

    string(STRIP "${GIT_DATE}" GIT_DATE)

    set(GIT_VERSION "${GIT_TAG}_${GIT_DATE}")
endif()

set(VERSION "const char* GIT_TAG=\"${GIT_VERSION}\";")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/version.c)
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/version.c VERSION_)
else()
    set(VERSION_ "")
endif()

if (NOT "${VERSION}" STREQUAL "${VERSION_}")
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/version.c "${VERSION}")
endif()
