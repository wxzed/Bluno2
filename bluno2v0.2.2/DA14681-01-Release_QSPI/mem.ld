MEMORY
{
        ROM (rx) : ORIGIN = (0x8000000 + (0)), LENGTH = (128 * 1024)
        RetRAM0 (rwx): ORIGIN = (0x7FC0000) + (( 32 * 1024) + ( 32 * 1024)), LENGTH = (( 64 * 1024))
        RetRAM1 (rwx): ORIGIN = ((0x7FC0000) + 0x100), LENGTH = (( 32 * 1024) - 0x100)
        RAM (rw) : ORIGIN = (0x7FC0000) + (( 32 * 1024)), LENGTH = (( 32 * 1024))
}
