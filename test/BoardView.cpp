/*
 *  test/BoardView.cpp
 *  Copyright 2020-2021 ItJustWorksTM
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <catch2/catch_test_macros.hpp>
#include "SMCE/Board.hpp"
#include "SMCE/BoardConf.hpp"
#include "SMCE/BoardView.hpp"
#include "SMCE/Toolchain.hpp"
#include "defs.hpp"

using namespace std::literals;

constexpr std::byte operator""_b(char c) noexcept { return static_cast<std::byte>(c); }
constexpr std::byte operator""_b(unsigned long long c) noexcept { return static_cast<std::byte>(c); }

TEST_CASE("BoardView GPIO", "[BoardView]") {
    smce::Toolchain tc{SMCE_PATH};
    REQUIRE(!tc.check_suitable_environment());
    smce::Sketch sk{SKETCHES_PATH "pins", {.fqbn = "arduino:avr:nano"}};
    const auto ec = tc.compile(sk);
    if (ec)
        std::cerr << tc.build_log().second;
    REQUIRE_FALSE(ec);
    smce::Board br{};
    // clang-format off
    smce::BoardConfig bc{
        /* .pins = */{0, 2},
        /* .gpio_drivers = */{
            smce::BoardConfig::GpioDrivers{
                0,
                smce::BoardConfig::GpioDrivers::DigitalDriver{true, false},
                smce::BoardConfig::GpioDrivers::AnalogDriver{true, false}
            },
            smce::BoardConfig::GpioDrivers{
                2,
                smce::BoardConfig::GpioDrivers::DigitalDriver{false, true},
                smce::BoardConfig::GpioDrivers::AnalogDriver{false, true}
            },
        }
    };
    // clang-format on
    REQUIRE(br.configure(std::move(bc)));
    REQUIRE(br.attach_sketch(sk));
    REQUIRE(br.start());
    auto bv = br.view();
    REQUIRE(bv.valid());
    auto pin0 = bv.pins[0];
    REQUIRE(pin0.exists());
    auto pin0d = pin0.digital();
    REQUIRE(pin0d.exists());
    REQUIRE(pin0d.can_read());
    REQUIRE_FALSE(pin0d.can_write());
    auto pin0a = pin0.analog();
    REQUIRE(pin0a.exists());
    REQUIRE(pin0a.can_read());
    REQUIRE_FALSE(pin0a.can_write());
    auto pin1 = bv.pins[1];
    REQUIRE_FALSE(pin1.exists());
    auto pin2 = bv.pins[2];
    REQUIRE(pin2.exists());
    auto pin2d = pin2.digital();
    REQUIRE(pin2d.exists());
    REQUIRE_FALSE(pin2d.can_read());
    REQUIRE(pin2d.can_write());
    auto pin2a = pin2.analog();
    REQUIRE(pin2a.exists());
    REQUIRE_FALSE(pin2a.can_read());
    REQUIRE(pin2a.can_write());
    std::this_thread::sleep_for(1ms);

    pin0d.write(false);
    test_pin_delayable(pin2d, true, 16384, 1ms);
    pin0d.write(true);
    test_pin_delayable(pin2d, false, 16384, 1ms);
    REQUIRE(br.stop());
}

TEST_CASE("BoardView UART", "[BoardView]") {
    smce::Toolchain tc{SMCE_PATH};
    REQUIRE(!tc.check_suitable_environment());
    smce::Sketch sk{SKETCHES_PATH "uart", {.fqbn = "arduino:avr:nano"}};
    const auto ec = tc.compile(sk);
    if (ec)
        std::cerr << tc.build_log().second;
    REQUIRE_FALSE(ec);
    smce::Board br{};
    REQUIRE(br.configure({.uart_channels = {{/* uart channel with default values */}}}));
    REQUIRE(br.attach_sketch(sk));
    REQUIRE(br.start());

    auto bv = br.view();
    REQUIRE(bv.valid());
    auto uart0 = bv.uart_channels[0];
    REQUIRE(uart0.exists());
    REQUIRE(uart0.rx().exists());
    REQUIRE(uart0.tx().exists());
    auto uart1 = bv.uart_channels[1];
    REQUIRE_FALSE(uart1.exists());
    REQUIRE_FALSE(uart1.rx().exists());
    REQUIRE_FALSE(uart1.tx().exists());
    std::this_thread::sleep_for(1ms);

    std::array out = {'H', 'E', 'L', 'L', 'O', ' ', 'U', 'A', 'R', 'T', '\0'};
    std::array<char, out.size()> in{};
    REQUIRE(uart0.rx().write(out) == out.size());
    int ticks = 16'000;
    do {
        if (ticks-- == 0)
            FAIL("Timed out");
        std::this_thread::sleep_for(1ms);
    } while (uart0.tx().size() != in.size());
    REQUIRE(uart0.tx().front() == 'H');
    REQUIRE(uart0.tx().read(in) == in.size());
    REQUIRE(uart0.tx().front() == '\0');
    REQUIRE(uart0.tx().size() == 0);
    REQUIRE(in == out);

#if !MSVC_DEBUG
    std::reverse(out.begin(), out.end());
    REQUIRE(uart0.rx().write(out) == out.size());
    ticks = 16'000;
    do {
        if (ticks-- == 0)
            FAIL("Timed out");
        std::this_thread::sleep_for(1ms);
    } while (uart0.tx().size() != in.size());
    REQUIRE(uart0.tx().read(in) == in.size());
    REQUIRE(uart0.tx().size() == 0);
    REQUIRE(in == out);
#endif

    REQUIRE(br.stop());
}

// testing the RGB444 write function
TEST_CASE("BoardView FrameBuffer RGB444 Write", "[BoardView]") {
    // set up
    std::byte target[3]; // target
    std::byte black_bits[] = {0_b, 0_b};
    std::byte white_bits[] = {0xff_b, 0xf_b};
    std::byte red_bits[] = {0_b, 0xf_b};
    std::byte green_bits[] = {0xf0_b, 0_b};
    std::byte blue_bits[] = {0x0f_b, 0_b};
    std::byte brown_bits[] = {0x75_b, 0x8_b};
    std::byte purple_bits[] = {0x5c_b, 0xa_b};

    smce::convert_rgb444_to_rgb888(std::span{black_bits, 2}, target);
    REQUIRE(target[0] == 0_b); // red
    REQUIRE(target[1] == 0_b); // green
    REQUIRE(target[2] == 0_b); // blue

    smce::convert_rgb444_to_rgb888(std::span{white_bits, 2}, target);
    REQUIRE(target[0] == 255_b); // red
    REQUIRE(target[1] == 255_b); // green
    REQUIRE(target[2] == 255_b); // blue

    smce::convert_rgb444_to_rgb888(std::span{red_bits, 2}, target);
    REQUIRE(target[0] == 255_b); // red
    REQUIRE(target[1] == 0_b);   // green
    REQUIRE(target[2] == 0_b);   // blue

    smce::convert_rgb444_to_rgb888(std::span{green_bits, 2}, target);
    REQUIRE(target[0] == 0_b);   // red
    REQUIRE(target[1] == 255_b); // green
    REQUIRE(target[2] == 0_b);   // blue

    smce::convert_rgb444_to_rgb888(std::span{blue_bits, 2}, target);
    REQUIRE(target[0] == 0_b);   // red
    REQUIRE(target[1] == 0_b);   // green
    REQUIRE(target[2] == 255_b); // blue

    smce::convert_rgb444_to_rgb888(std::span{brown_bits, 2}, target);
    REQUIRE(target[0] == 0x88_b); // red
    REQUIRE(target[1] == 0x77_b); // green
    REQUIRE(target[2] == 0x55_b); // blue

    smce::convert_rgb444_to_rgb888(std::span{purple_bits, 2}, target);
    REQUIRE(target[0] == 0xaa_b); // red
    REQUIRE(target[1] == 0x55_b); // green
    REQUIRE(target[2] == 0xcc_b); // blue
}

// testing the RGB444 read function
TEST_CASE("BoardView FrameBuffer RGB444 Read", "[BoardView]") {
    // set up
    std::byte target[2]; // target
    std::byte black_bits[] = {0_b, 0_b, 0_b};
    std::byte white_bits[] = {0xff_b, 0xff_b, 0xff_b};
    std::byte red_bits[] = {0xff_b, 0_b, 0_b};
    std::byte green_bits[] = {0_b, 0xff_b, 0_b};
    std::byte blue_bits[] = {0x0_b, 0_b, 0xff_b};
    std::byte brown_bits[] = {0x95_b, 0x6c_b, 0x32_b};
    std::byte purple_bits[] = {0xa4_b, 0x52_b, 0xcd_b};

    smce::convert_rgb888_to_rgb444(black_bits, target);
    REQUIRE(target[1] == 0_b); // red
    REQUIRE(target[0] == 0_b); // green and blue

    smce::convert_rgb888_to_rgb444(white_bits, target);
    REQUIRE(target[1] == 0x0f_b); // red
    REQUIRE(target[0] == 0xff_b); // green and blue

    smce::convert_rgb888_to_rgb444(red_bits, target);
    REQUIRE(target[1] == 0x0f_b); // red
    REQUIRE(target[0] == 0_b);    // green and blue

    smce::convert_rgb888_to_rgb444(green_bits, target);
    REQUIRE(target[1] == 0_b);    // red
    REQUIRE(target[0] == 0xf0_b); // green and blue

    smce::convert_rgb888_to_rgb444(blue_bits, target);
    REQUIRE(target[1] == 0_b);    // red
    REQUIRE(target[0] == 0x0f_b); // green and blue

    smce::convert_rgb888_to_rgb444(brown_bits, target);
    REQUIRE(target[1] == 0x9_b);  // red
    REQUIRE(target[0] == 0x63_b); // green and blue

    smce::convert_rgb888_to_rgb444(purple_bits, target);
    REQUIRE(target[1] == 0xa_b);  // red
    REQUIRE(target[0] == 0x5c_b); // green and blue
}

// testing the RGB565 read function
TEST_CASE("BoardView FrameBuffer RGB565 Read", "[BoardView]") {
    // set up
    std::byte target[2]; // target
    std::byte black_bits[] = {0_b, 0_b, 0_b};
    std::byte white_bits[] = {0xff_b, 0xff_b, 0xff_b};
    std::byte red_bits[] = {0xff_b, 0_b, 0_b};
    std::byte green_bits[] = {0_b, 0xff_b, 0_b};
    std::byte blue_bits[] = {0x0_b, 0_b, 0xff_b};
    std::byte brown_bits[] = {0x95_b, 0x6c_b, 0x32_b};
    std::byte purple_bits[] = {0xa4_b, 0x52_b, 0xcd_b};

    smce::convert_rgb888_to_rgb565(black_bits, target);
    REQUIRE(target[1] == 0_b); // red
    REQUIRE(target[0] == 0_b); // green and blue

    smce::convert_rgb888_to_rgb565(white_bits, target);
    REQUIRE(target[1] == 0xff_b); // red
    REQUIRE(target[0] == 0xff_b); // green and blue

    smce::convert_rgb888_to_rgb565(red_bits, target);
    REQUIRE(target[1] == 0xf8_b); // red
    REQUIRE(target[0] == 0_b);    // green and blue

    smce::convert_rgb888_to_rgb565(green_bits, target);
    REQUIRE(target[1] == 0x07_b); // red
    REQUIRE(target[0] == 0xe0_b); // green and blue

    smce::convert_rgb888_to_rgb565(blue_bits, target);
    REQUIRE(target[1] == 0_b);    // red
    REQUIRE(target[0] == 0x1f_b); // green and blue

    smce::convert_rgb888_to_rgb565(brown_bits, target);
    REQUIRE(target[1] == 0x93_b); // red
    REQUIRE(target[0] == 0x66_b); // green and blue

    smce::convert_rgb888_to_rgb565(purple_bits, target);
    REQUIRE(target[1] == 0xa2_b); // red
    REQUIRE(target[0] == 0x99_b); // green and blue
}

// testing the RGB565 write function
TEST_CASE("BoardView FrameBuffer RGB565 Write", "[BoardView]") {
    // set up
    std::byte target[3];
    std::byte black_bits[] = {0_b, 0_b};
    std::byte white_bits[] = {0xff_b, 0xff_b};
    std::byte red_bits[] = {0x00_b, 0xf8_b};
    std::byte green_bits[] = {0xe0_b, 0x07_b};
    std::byte blue_bits[] = {0x1f_b, 0_b};
    std::byte brown_bits[] = {0x66_b, 0x93_b};
    std::byte purple_bits[] = {0x99_b, 0xa2_b};

    smce::convert_rgb565_to_rgb888(std::span{black_bits, 2}, target);
    REQUIRE(target[0] == 0_b); // red
    REQUIRE(target[1] == 0_b); // green
    REQUIRE(target[2] == 0_b); // blue

    smce::convert_rgb565_to_rgb888(std::span{white_bits, 2}, target);
    REQUIRE(target[0] == 255_b); // red
    REQUIRE(target[1] == 255_b); // green
    REQUIRE(target[2] == 255_b); // blue

    smce::convert_rgb565_to_rgb888(std::span{red_bits, 2}, target);
    REQUIRE(target[0] == 255_b); // red
    REQUIRE(target[1] == 0_b);   // green
    REQUIRE(target[2] == 0_b);   // blue

    smce::convert_rgb565_to_rgb888(std::span{green_bits, 2}, target);
    REQUIRE(target[0] == 0_b);   // red
    REQUIRE(target[1] == 255_b); // green
    REQUIRE(target[2] == 0_b);   // blue

    smce::convert_rgb565_to_rgb888(std::span{blue_bits, 2}, target);
    REQUIRE(target[0] == 0_b);   // red
    REQUIRE(target[1] == 0_b);   // green
    REQUIRE(target[2] == 255_b); // blue

    smce::convert_rgb565_to_rgb888(std::span{brown_bits, 2}, target);
    REQUIRE(target[0] == 0x94_b); // red
    REQUIRE(target[1] == 0x6d_b); // green
    REQUIRE(target[2] == 0x31_b); // blue

    smce::convert_rgb565_to_rgb888(std::span{purple_bits, 2}, target);
    REQUIRE(target[0] == 0xa5_b); // red
    REQUIRE(target[1] == 0x51_b); // green
    REQUIRE(target[2] == 0xce_b); // blue
}

TEST_CASE("BoardView RGB444 cvt", "[BoardView]") {
    smce::Toolchain tc{SMCE_PATH};
    REQUIRE(!tc.check_suitable_environment());
    smce::Sketch sk{SKETCHES_PATH "noop", {.fqbn = "arduino:avr:nano"}};
    const auto ec = tc.compile(sk);
    if (ec)
        std::cerr << tc.build_log().second;
    REQUIRE_FALSE(ec);
    smce::Board br{};
    REQUIRE(br.configure({.frame_buffers = {{}}}));
    REQUIRE(br.attach_sketch(sk));
    REQUIRE(br.start());
    auto bv = br.view();
    REQUIRE(bv.valid());
    REQUIRE(br.suspend());
    auto fb = bv.frame_buffers[0];
    REQUIRE(fb.exists());

    {
        constexpr std::size_t height = 1;
        constexpr std::size_t width = 1;

        constexpr std::array in = {'\xBC'_b, '\x0A'_b};
        constexpr std::array expected_out = {'\xAA'_b, '\xBB'_b, '\xCC'_b};
        static_assert(in.size() == expected_out.size() / 3 * 2);

        fb.set_height(height);
        fb.set_width(width);
        REQUIRE(fb.write_rgb444(in));

        std::array<std::byte, std::size(expected_out)> out;
        REQUIRE(fb.read_rgb888(out));
        REQUIRE(out == expected_out);
    }

    {
        constexpr std::size_t height = 2;
        constexpr std::size_t width = 2;

        constexpr std::array in = {'\x23'_b, '\xF1'_b, '\x56'_b, '\xF4'_b, '\x89'_b, '\xF7'_b, '\xBC'_b, '\xFA'_b};
        constexpr std::array expected_out = {'\x11'_b, '\x22'_b, '\x33'_b, '\x44'_b, '\x55'_b, '\x66'_b,
                                             '\x77'_b, '\x88'_b, '\x99'_b, '\xAA'_b, '\xBB'_b, '\xCC'_b};
        static_assert(in.size() == expected_out.size() / 3 * 2);

        fb.set_height(height);
        fb.set_width(width);
        fb.write_rgb444(in);

        std::array<std::byte, std::size(expected_out)> out;
        fb.read_rgb888(out);
        REQUIRE(out == expected_out);
    }

    {
        constexpr std::size_t height = 1;
        constexpr std::size_t width = 1;

        constexpr std::array in = {'\xAD'_b, '\xBE'_b, '\xCF'_b};
        constexpr std::array expected_out = {'\xBC'_b, '\x0A'_b};
        static_assert(expected_out.size() == in.size() / 3 * 2);

        fb.set_height(height);
        fb.set_width(width);
        REQUIRE(fb.write_rgb888(in));

        std::array<std::byte, std::size(expected_out)> out;
        REQUIRE(fb.read_rgb444(out));
        REQUIRE(out == expected_out);
    }

    {
        constexpr std::size_t height = 2;
        constexpr std::size_t width = 2;

        constexpr std::array in = {'\x1A'_b, '\x2B'_b, '\x3C'_b, '\x4D'_b, '\x5E'_b, '\x6F'_b,
                                   '\x7A'_b, '\x8B'_b, '\x9C'_b, '\xAD'_b, '\xBE'_b, '\xCF'_b};
        constexpr std::array expected_out = {'\x23'_b, '\x01'_b, '\x56'_b, '\x04'_b,
                                             '\x89'_b, '\x07'_b, '\xBC'_b, '\x0A'_b};
        static_assert(expected_out.size() == in.size() / 3 * 2);

        fb.set_height(height);
        fb.set_width(width);
        fb.write_rgb888(in);

        std::array<std::byte, std::size(expected_out)> out;
        fb.read_rgb444(out);
        REQUIRE(out == expected_out);
    }

    REQUIRE(br.stop());
}
