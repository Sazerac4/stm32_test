const std = @import("std");

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.
pub fn build(b: *std.Build) void {
    //b.verbose_cc = true;
    b.verbose_link = true;
    //b.verbose_air = true;
    //b.verbose = true;

    const prj_name = "test";

    const target = .{
        .cpu_arch = .thumb, // ARMv7
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 }, // STM32F446RE
        .os_tag = .freestanding, // running in bare metal
        .abi = .eabihf, // no libc (noneabi) with hardware floating point (hf)
    };
    const asm_sources = [_][]const u8{"startup_stm32f446xx.s"};

    //const asm_flags = [_][]const u8{
    //    "-fdata-sections",
    //    "-ffunction-sections",
    //    "-Wall",
    //    "-Wextra",
    //    "-Werror",
    //    "-pedantic",
    //    "-fstack-usage"
    //};

    const c_includes = [_][]const u8{
        "./Core/Inc",
        "./Drivers/STM32F4xx_HAL_Driver/Inc",
        "./Drivers/STM32F4xx_HAL_Driver/Inc/Legacy",
        "./Drivers/CMSIS/Device/ST/STM32F4xx/Include",
        "./Drivers/CMSIS/Include",
    };

    const c_sources_drivers = [_][]const u8{
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c",
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c",
    };
    // "-Wno-unused-parameter",
    const c_sources_drivers_compile_flags = [_][]const u8{
        "-std=gnu11",
        "-DUSE_HAL_DRIVER",
        "-DSTM32F446xx",
        "-ffunction-sections",
        "-fdata-sections",
        "-nostdlib",
        "-nostdinc",
        "-Wall",
        //"-Werror",
        "-Wextra",
        "-pedantic",
        "-fstack-usage",
        "-mthumb",
    };

    const c_sources_core = [_][]const u8{
        "./Core/Src/system_stm32f4xx.c",
        "./Core/Src/stm32f4xx_it.c",
        "./Core/Src/stm32f4xx_hal_msp.c",
    };
    const c_sources_app = [_][]const u8{
        "./Core/Src/main.c",
        "./Core/Src/MedianFilter.c",
    };

    //const c_compile_flags = [_][]const u8{ "-DUSE_HAL_DRIVER", "-DSTM32F446xx", "-std=gnu11", "-Wall", "-Werror", "-Wextra", "-pedantic", "-fstack-usage", "-fdata-sections", "-ffunction-sections" };
    const c_compile_flags = [_][]const u8{ "-DUSE_HAL_DRIVER", "-DSTM32F446xx", "-std=gnu11", "-Wall", "-Wextra", "-pedantic", "-fstack-usage", "-fdata-sections", "-ffunction-sections" };
    //const c_compile_flags = [_][]const u8{ "-DUSE_HAL_DRIVER", "-DSTM32F446xx", "-fstack-usage", "-fdata-sections", "-ffunction-sections" };

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    const elf = b.addExecutable(.{
        .name = prj_name ++ ".elf",
        .target = target,
        .optimize = optimize, // use optimization given by user
        //.strip = false, // do not strip debug symbols
        .linkage = .static, // static linking
        .link_libc = false, // will link against newlib_nano
        .single_threaded = true, // single core cpu
    });

    // Add necessary libraries and link to them
    elf.addIncludePath(.{ .path = "/usr/arm-none-eabi/include" });
    elf.addIncludePath(.{ .path = "/usr/arm-none-eabi/include/newlib-nano" });
    elf.addObjectFile(.{ .path = "/usr/arm-none-eabi/lib/thumb/v7e-m+fp/hard/libnosys.a" });
    elf.addObjectFile(.{ .path = "/usr/arm-none-eabi/lib/thumb/v7e-m+fp/hard/libc_nano.a" });
    elf.addObjectFile(.{ .path = "/usr/arm-none-eabi/lib/thumb/v7e-m+fp/hard/libm.a" });
    //elf.addObjectFile(.{ .path = "/usr/arm-none-eabi/lib/thumb/v7e-m+fp/hard/libgcc.a" });
    //elf.addObjectFile(.{ .path = "/usr/arm-none-eabi/lib/thumb/v7e-m+fp/hard/crti.o" });
    //elf.addObjectFile(.{ .path = "/usr/arm-none-eabi/lib/thumb/v7e-m+fp/hard/crtbegin.o" });

    // C runtime 0 contains _start function, initializes stack and libc
    // runtime, then calls _main
    // Cruntime 0: This object is expected to contain the _start symbol which
    // takes care of bootstrapping the initial execution of the program.
    // this object initializes very early ABI requirements
    // (like the stack or frame pointer), setting up the argc/argv/env values, and
    // then passing pointers to the init/fini/main funcs to the internal libc main
    // which in turn does more general bootstrapping before finally calling the real
    // main function.
    elf.addObjectFile(.{ .path = "/usr/arm-none-eabi/lib/thumb/v7e-m+fp/hard/crt0.o" });

    // crti (cruntime init) Defines the function prologs for the .init and
    // .fini sections (with the _init and _fini symbols respectively).  This
    // way they can be called directly. They contain constructors and destructors
    // for global structs
    elf.addObjectFile(.{ .path = "/usr/lib/gcc/arm-none-eabi/13.2.0/thumb/v7e-m+fp/hard/crti.o" });
    //  Defines the function epilogs for the .init/.fini sections
    //elf.addObjectFile(.{ .path = "/usr/lib/gcc/arm-none-eabi/13.2.0/thumb/v7e-m+fp/hard/crtn.o" });

    // GCC uses this to find the start of the constructors
    elf.addObjectFile(.{ .path = "/usr/lib/gcc/arm-none-eabi/13.2.0/thumb/v7e-m+fp/hard/crtbegin.o" });
    // GCC uses this to find the start of the destructors.
    //elf.addObjectFile(.{ .path = "/usr/lib/gcc/arm-none-eabi/13.2.0/thumb/v7e-m+fp/hard/crtend.o" });

    // Add C source files
    elf.addCSourceFiles(&c_sources_drivers, &c_sources_drivers_compile_flags);
    elf.addCSourceFiles(&c_sources_core, &c_compile_flags);
    elf.addCSourceFiles(&c_sources_app, &c_compile_flags);

    // Add Assembly sources
    for (asm_sources) |path| {
        elf.addAssemblyFile(.{ .path = path });
    }
    // Add C headers include dirs
    for (c_includes) |path| {
        elf.addIncludePath(.{ .path = path });
    }

    // Set Entry Point of the firmware
    elf.entry_symbol_name = "Reset_Handler";

    // Set linker script file
    elf.setLinkerScriptPath(.{ .path = "./STM32F446RETx_FLASH.ld" });
    elf.setVerboseLink(true);

    b.default_step.dependOn(&elf.step);
    b.installArtifact(elf);
}
