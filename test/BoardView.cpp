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
    auto pin0 = bv.pins[0].digital();
    REQUIRE(pin0.exists());
    auto pin1 = bv.pins[1].digital();
    REQUIRE_FALSE(pin1.exists());
    auto pin2 = bv.pins[2].digital();
    REQUIRE(pin2.exists());
    std::this_thread::sleep_for(1ms);

    pin0.write(false);
    test_pin_delayable(pin2, true, 16384, 1ms);
    pin0.write(true);
    test_pin_delayable(pin2, false, 16384, 1ms);
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
    REQUIRE(uart0.tx().read(in) == in.size());
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
    std::byte black_bits[] = {(std::byte)0, (std::byte)0};
    std::byte white_bits[] = {(std::byte)0xff, (std::byte)0xf};
    std::byte red_bits[] = {(std::byte)0, (std::byte)0xf};
    std::byte green_bits[] = {(std::byte)0xf0, (std::byte)0};
    std::byte blue_bits[] = {(std::byte)0x0f, (std::byte)0};
    std::byte brown_bits[] = {(std::byte)0x75, (std::byte)0x8};
    std::byte purple_bits[] = {(std::byte)0x5c, (std::byte)0xa};

    smce::convert_rgb444_to_rgb888(std::span{black_bits, 2}, target);
    REQUIRE((int)target[0] == 0); // red
    REQUIRE((int)target[1] == 0); // green
    REQUIRE((int)target[2] == 0); // blue

    smce::convert_rgb444_to_rgb888(std::span{white_bits, 2}, target);
    REQUIRE((int)target[0] == 255); // red
    REQUIRE((int)target[1] == 255); // green
    REQUIRE((int)target[2] == 255); // blue

    smce::convert_rgb444_to_rgb888(std::span{red_bits, 2}, target);
    REQUIRE((int)target[0] == 255); // red
    REQUIRE((int)target[1] == 0);   // green
    REQUIRE((int)target[2] == 0);   // blue

    smce::convert_rgb444_to_rgb888(std::span{green_bits, 2}, target);
    REQUIRE((int)target[0] == 0);   // red
    REQUIRE((int)target[1] == 255); // green
    REQUIRE((int)target[2] == 0);   // blue

    smce::convert_rgb444_to_rgb888(std::span{blue_bits, 2}, target);
    REQUIRE((int)target[0] == 0);   // red
    REQUIRE((int)target[1] == 0);   // green
    REQUIRE((int)target[2] == 255); // blue

    smce::convert_rgb444_to_rgb888(std::span{brown_bits, 2}, target);
    REQUIRE((int)target[0] == 0x88); // red
    REQUIRE((int)target[1] == 0x77); // green
    REQUIRE((int)target[2] == 0x55); // blue

    smce::convert_rgb444_to_rgb888(std::span{purple_bits, 2}, target);
    REQUIRE((int)target[0] == 0xaa); // red
    REQUIRE((int)target[1] == 0x55); // green
    REQUIRE((int)target[2] == 0xcc); // blue
}

// testing the RGB444 read function
TEST_CASE("BoardView FrameBuffer RGB444 Read", "[BoardView]") {
    // set up
    std::byte target[2]; // target
    std::byte black_bits[] = {(std::byte)0, (std::byte)0, (std::byte)0};
    std::byte white_bits[] = {(std::byte)0xff, (std::byte)0xff, (std::byte)0xff};
    std::byte red_bits[] = {(std::byte)0xff, (std::byte)0, (std::byte)0};
    std::byte green_bits[] = {(std::byte)0, (std::byte)0xff, (std::byte)0};
    std::byte blue_bits[] = {(std::byte)0x0, (std::byte)0, (std::byte)0xff};
    std::byte brown_bits[] = {(std::byte)0x95, (std::byte)0x6c, (std::byte)0x32};
    std::byte purple_bits[] = {(std::byte)0xa4, (std::byte)0x52, (std::byte)0xcd};

    smce::convert_rgb888_to_rgb444(black_bits, target);
    REQUIRE((int)target[1] == 0); // red
    REQUIRE((int)target[0] == 0); // green and blue

    smce::convert_rgb888_to_rgb444(white_bits, target);
    REQUIRE((int)target[1] == 0x0f); // red
    REQUIRE((int)target[0] == 0xff); // green and blue

    smce::convert_rgb888_to_rgb444(red_bits, target);
    REQUIRE((int)target[1] == 0x0f); // red
    REQUIRE((int)target[0] == 0);    // green and blue

    smce::convert_rgb888_to_rgb444(green_bits, target);
    REQUIRE((int)target[1] == 0);    // red
    REQUIRE((int)target[0] == 0xf0); // green and blue

    smce::convert_rgb888_to_rgb444(blue_bits, target);
    REQUIRE((int)target[1] == 0);    // red
    REQUIRE((int)target[0] == 0x0f); // green and blue

    smce::convert_rgb888_to_rgb444(brown_bits, target);
    REQUIRE((int)target[1] == 0x9);  // red
    REQUIRE((int)target[0] == 0x63); // green and blue

    smce::convert_rgb888_to_rgb444(purple_bits, target);
    REQUIRE((int)target[1] == 0xa);  // red
    REQUIRE((int)target[0] == 0x5c); // green and blue
}

// testing the RGB565 read function
TEST_CASE("BoardView FrameBuffer RGB565 Read", "[BoardView]") {
    // set up
    std::byte target[2]; // target
    std::byte black_bits[] = {(std::byte)0, (std::byte)0, (std::byte)0};
    std::byte white_bits[] = {(std::byte)0xff, (std::byte)0xff, (std::byte)0xff};
    std::byte red_bits[] = {(std::byte)0xff, (std::byte)0, (std::byte)0};
    std::byte green_bits[] = {(std::byte)0, (std::byte)0xff, (std::byte)0};
    std::byte blue_bits[] = {(std::byte)0x0, (std::byte)0, (std::byte)0xff};
    std::byte brown_bits[] = {(std::byte)0x95, (std::byte)0x6c, (std::byte)0x32};
    std::byte purple_bits[] = {(std::byte)0xa4, (std::byte)0x52, (std::byte)0xcd};

    smce::convert_rgb888_to_rgb565(black_bits, target);
    REQUIRE((int)target[1] == 0); // red
    REQUIRE((int)target[0] == 0); // green and blue

    smce::convert_rgb888_to_rgb565(white_bits, target);
    REQUIRE((int)target[1] == 0xff); // red
    REQUIRE((int)target[0] == 0xff); // green and blue

    smce::convert_rgb888_to_rgb565(red_bits, target);
    REQUIRE((int)target[1] == 0xf8); // red
    REQUIRE((int)target[0] == 0);    // green and blue

    smce::convert_rgb888_to_rgb565(green_bits, target);
    REQUIRE((int)target[1] == 0x07); // red
    REQUIRE((int)target[0] == 0xe0); // green and blue

    smce::convert_rgb888_to_rgb565(blue_bits, target);
    REQUIRE((int)target[1] == 0);    // red
    REQUIRE((int)target[0] == 0x1f); // green and blue

    smce::convert_rgb888_to_rgb565(brown_bits, target);
    REQUIRE((int)target[1] == 0x93); // red
    REQUIRE((int)target[0] == 0x66); // green and blue

    smce::convert_rgb888_to_rgb565(purple_bits, target);
    REQUIRE((int)target[1] == 0xa2); // red
    REQUIRE((int)target[0] == 0x99); // green and blue
}

// testing the RGB565 write function
TEST_CASE("BoardView FrameBuffer RGB565 Write", "[BoardView]") {
    // set up
    std::byte target[3];
    std::byte black_bits[] = {(std::byte)0, (std::byte)0};
    std::byte white_bits[] = {(std::byte)0xff, (std::byte)0xff};
    std::byte red_bits[] = {(std::byte)0x00, (std::byte)0xf8};
    std::byte green_bits[] = {(std::byte)0xe0, (std::byte)0x07};
    std::byte blue_bits[] = {(std::byte)0x1f, (std::byte)0};
    std::byte brown_bits[] = {(std::byte)0x66, (std::byte)0x93};
    std::byte purple_bits[] = {(std::byte)0x99, (std::byte)0xa2};

    smce::convert_rgb565_to_rgb888(std::span{black_bits, 2}, target);
    REQUIRE((int)target[0] == 0); // red
    REQUIRE((int)target[1] == 0); // green
    REQUIRE((int)target[2] == 0); // blue

    smce::convert_rgb565_to_rgb888(std::span{white_bits, 2}, target);
    REQUIRE((int)target[0] == 255); // red
    REQUIRE((int)target[1] == 255); // green
    REQUIRE((int)target[2] == 255); // blue

    smce::convert_rgb565_to_rgb888(std::span{red_bits, 2}, target);
    REQUIRE((int)target[0] == 255); // red
    REQUIRE((int)target[1] == 0);   // green
    REQUIRE((int)target[2] == 0);   // blue

    smce::convert_rgb565_to_rgb888(std::span{green_bits, 2}, target);
    REQUIRE((int)target[0] == 0);   // red
    REQUIRE((int)target[1] == 255); // green
    REQUIRE((int)target[2] == 0);   // blue

    smce::convert_rgb565_to_rgb888(std::span{blue_bits, 2}, target);
    REQUIRE((int)target[0] == 0);   // red
    REQUIRE((int)target[1] == 0);   // green
    REQUIRE((int)target[2] == 255); // blue

    smce::convert_rgb565_to_rgb888(std::span{brown_bits, 2}, target);
    REQUIRE((int)target[0] == 0x94); // red
    REQUIRE((int)target[1] == 0x6d); // green
    REQUIRE((int)target[2] == 0x31); // blue

    smce::convert_rgb565_to_rgb888(std::span{purple_bits, 2}, target);
    REQUIRE((int)target[0] == 0xa5); // red
    REQUIRE((int)target[1] == 0x51); // green
    REQUIRE((int)target[2] == 0xce); // blue
}
