Import("env")

#
# Dump build environment (for debug)
# print(env.Dump())
#

flags = [
    "-mfloat-abi=hard",
    "-mfpu=fpv4-sp-d16",
    "-u_printf_float",
    "-u_scanf_float"
]
env.Append(CCFLAGS=flags, LINKFLAGS=flags)