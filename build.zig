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
    const gcc_version = "13.2.1";
    const gcc_path = "/opt/dev/xpack-arm-none-eabi-gcc-13.2.1-1.1/";
    const gcc_arch = "v7e-m+fp";

    const query: std.zig.CrossTarget = .{
        .cpu_arch = .thumb,
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
        //.cpu_features_add = std.Target.arm.featureSet(&[_]std.Target.arm.Feature{std.Target.arm.Feature.}),
        .os_tag = .freestanding,
        .os_version_min = undefined,
        .os_version_max = undefined,
        .abi = .eabihf,
        .glibc_version = null,
    };
    const target = b.resolveTargetQuery(query);
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
        "-Og",
        "-gdwarf-2",
        "-ggdb3",
        "-std=gnu11",
        "-DUSE_HAL_DRIVER",
        "-DSTM32F446xx",
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
        "./Core/Src/sysmem.c",
        "./Core/Src/syscalls.c",
    };

    const c_compile_flags = [_][]const u8{ "-Og", "-gdwarf-2", "-ggdb3", "-DUSE_HAL_DRIVER", "-DSTM32F446xx", "-std=gnu11", "-Wall", "-Wextra", "-pedantic", "-fstack-usage" };

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    const elf = b.addExecutable(.{
        .name = prj_name ++ ".elf",
        .target = target,
        .optimize = optimize, // use optimization given by user
        .strip = false, // do not strip debug symbols
        .linkage = .static, // static linking
        .link_libc = false, // will link against newlib_nano
        .single_threaded = true, // single core cpu
    });

    //////////////////////////////////////////////////////////////////
    // Manually including libraries bundled with arm-none-eabi-gcc
    elf.addLibraryPath(.{ .path = gcc_path ++ "arm-none-eabi/lib/thumb/" ++ gcc_arch ++ "/hard" });
    elf.addLibraryPath(.{ .path = gcc_path ++ "lib/gcc/arm-none-eabi/" ++ gcc_version ++ "/thumb/" ++ gcc_arch ++ "/hard" });
    elf.addSystemIncludePath(.{ .path = gcc_path ++ "arm-none-eabi/include" });
    //elf.linkSystemLibrary("nosys"); // "-lnosys",
    //elf.linkSystemLibrary("c_nano"); // "-lc_nano"
    elf.linkSystemLibrary("g_nano"); // "-lg_nano" //NOTE: Same as c_nano but with debug symbol
    elf.linkSystemLibrary("m"); // "-lm"
    elf.linkSystemLibrary("gcc"); // "-lgcc"
    //elf.forceUndefinedSymbol("_printf_float"); // Allow float formating (printf, sprintf, ...)

    // Manually include C runtime objects bundled with arm-none-eabi-gcc
    elf.addObjectFile(.{ .path = gcc_path ++ "arm-none-eabi/lib/thumb/" ++ gcc_arch ++ "/hard/crt0.o" });
    elf.addObjectFile(.{ .path = gcc_path ++ "/lib/gcc/arm-none-eabi/" ++ gcc_version ++ "/thumb/" ++ gcc_arch ++ "/hard/crti.o" });
    elf.addObjectFile(.{ .path = gcc_path ++ "/lib/gcc/arm-none-eabi/" ++ gcc_version ++ "/thumb/" ++ gcc_arch ++ "/hard/crtbegin.o" });
    elf.addObjectFile(.{ .path = gcc_path ++ "/lib/gcc/arm-none-eabi/" ++ gcc_version ++ "/thumb/" ++ gcc_arch ++ "/hard/crtend.o" });
    elf.addObjectFile(.{ .path = gcc_path ++ "/lib/gcc/arm-none-eabi/" ++ gcc_version ++ "/thumb/" ++ gcc_arch ++ "/hard/crtn.o" });

    //////////////////////////////////////////////////////////////////
    elf.entry = .{ .symbol_name = "Reset_Handler" }; // Set Entry Point of the firmware (Already set in the linker script)
    elf.want_lto = false; // -flto
    elf.link_data_sections = true; // -fdata-sections
    elf.link_function_sections = true; // -ffunction-sections
    elf.link_gc_sections = true; // -Wl,--gc-sections

    // Add C source files

    elf.addCSourceFiles(.{
        .files = &c_sources_drivers,
        .flags = &c_sources_drivers_compile_flags,
    });
    elf.addCSourceFiles(.{
        .files = &c_sources_core,
        .flags = &c_compile_flags,
    });
    elf.addCSourceFiles(.{
        .files = &c_sources_app,
        .flags = &c_compile_flags,
    });

    // Add Assembly sources
    for (asm_sources) |path| {
        elf.addAssemblyFile(.{ .path = path });
    }
    // Add C headers include dirs
    for (c_includes) |path| {
        elf.addIncludePath(.{ .path = path });
    }

    // Set linker script file
    elf.setLinkerScriptPath(.{ .path = "./STM32F446RETx_FLASH.ld" });
    elf.setVerboseLink(true); //NOTE: Generate linker error : 'https://github.com/ziglang/zig/issues/19410'

    // Copy the bin out of the elf
    const bin = b.addObjCopy(elf.getEmittedBin(), .{
        .format = .bin,
    });
    bin.step.dependOn(&elf.step);
    const copy_bin = b.addInstallBinFile(bin.getOutput(), prj_name ++ ".bin");
    b.default_step.dependOn(&copy_bin.step);

    // Copy the bin out of the elf
    const hex = b.addObjCopy(elf.getEmittedBin(), .{
        .format = .hex,
    });
    hex.step.dependOn(&elf.step);
    const copy_hex = b.addInstallBinFile(hex.getOutput(), prj_name ++ ".hex");
    b.default_step.dependOn(&copy_hex.step);

    b.default_step.dependOn(&elf.step);
    b.installArtifact(elf);
}
