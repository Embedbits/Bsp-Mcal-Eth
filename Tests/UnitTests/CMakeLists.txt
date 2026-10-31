#======================== Unit tests configuration ============================#
# Unit tests of Eth module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(ETH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${ETH_DIR}/.." ABSOLUTE)

# Both sources of the module are compiled for every line; on a line without ETH (STM32H503,
# STM32H523, STM32H533, STM32H562) every public function of Eth.c returns ETH_REQUEST_ERROR
# (peripheral check defined(ETH)). Self-clearing bits (DMAMR.SWR, MACMDIOAR.MB, MACTSCR.TSINIT/TSUPDT/TSADDREG) are
# not emulated by plain RegMem; the tests register a HW model (RegMem_Set_ModelActive) that
# clears them after the store, the same approach as the Wwdg / DbgMcu / VrefBuf unit tests.
UnitTesting_Add_Test(
    NAME            Eth
    TEST_SOURCE     Test_Eth.c
    UUT_SOURCES     ${ETH_DIR}/Eth.c
                    ${ETH_DIR}/Eth_Reg.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Gpio/Gpio_Port.h
                    ${MCAL_DIR}/Sbs/Sbs_Port.h
                    ${MCAL_DIR}/Cache/Cache_Port.h
    INCLUDE_DIRS    ${ETH_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Gpio
                    ${MCAL_DIR}/Sbs
                    ${MCAL_DIR}/Cache
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
    RUN_SERIAL
)
