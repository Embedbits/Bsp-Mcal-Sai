#======================== Unit tests configuration ============================#
# Unit tests of Sai module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).
#
# The test set is independent - it lists everything it needs: code under test,
# mocked headers, host emulation of the target and libraries built for host.

get_filename_component(SAI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${SAI_DIR}/.." ABSOLUTE)

UnitTesting_Add_Test(
    NAME            Sai
    TEST_SOURCE     Test_Sai.c
    UUT_SOURCES     ${SAI_DIR}/Sai.c
                    ${SAI_DIR}/Sai_Cfg.c
                    ${SAI_DIR}/Sai_Reg.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Gpio/Gpio_Port.h
                    ${MCAL_DIR}/Gpdma/Gpdma_Port.h
    INCLUDE_DIRS    ${SAI_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Gpio
                    ${MCAL_DIR}/Gpdma
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
    RUN_SERIAL
)
