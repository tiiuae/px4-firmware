/****************************************************************************
 * The bootloader touches no bus, but px4_platform links board_bus_info on
 * every NuttX build, so it needs something to link against. The real bus
 * tables do not compile here: their constexpr checks depend on board config
 * the bootloader does not set.
 ****************************************************************************/

/* Keeps the translation unit non-empty for compilers that object. */
const int board_bus_info_unused;
