/*
 *  BoardView.cpp
 *  Copyright 2021 ItJustWorksTM
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

#include "SMCE/BoardView.hpp"

#include <iterator>
#include <mutex>
#include <boost/date_time/microsec_time_clock.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include "SMCE/internal/BoardData.hpp"
#include "SMCE/internal/utils.hpp"

using microsec_clock = boost::date_time::microsec_clock<boost::posix_time::ptime>;

namespace smce {

[[nodiscard]] std::string_view BoardView::storage_get_root(Link link, std::uint16_t accessor) noexcept {
    if (!m_bdat)
        return {};
    using Bus = BoardData::DirectStorage::Bus;
    constexpr std::array<Bus, 3> link2bus{
        Bus{-1},  // UART
        Bus::SPI, // SPI
        Bus{-1},  // I2C
    };
    const auto bus = link2bus[static_cast<std::size_t>(link)];
    if (bus == Bus{-1})
        return {};

    const auto it = std::find_if(
        m_bdat->direct_storages.begin(), m_bdat->direct_storages.end(),
        [=](const BoardData::DirectStorage& ds) -> bool { return ds.bus == bus && ds.accessor == accessor; });
    if (it == m_bdat->direct_storages.end())
        return {};
    return it->root_dir;
}

[[nodiscard]] bool VirtualAnalogDriver::exists() noexcept { return m_bdat && m_idx < m_bdat->pins.size(); }

[[nodiscard]] bool VirtualAnalogDriver::can_read() noexcept { return exists() && m_bdat->pins[m_idx].can_analog_read; }

[[nodiscard]] bool VirtualAnalogDriver::can_write() noexcept {
    return exists() && m_bdat->pins[m_idx].can_analog_write;
}

[[nodiscard]] std::uint16_t VirtualAnalogDriver::read() noexcept {
    return exists() ? m_bdat->pins[m_idx].value.load() : 0;
}

void VirtualAnalogDriver::write(std::uint16_t value) noexcept {
    if (exists())
        m_bdat->pins[m_idx].value.store(value);
}

[[nodiscard]] bool VirtualDigitalDriver::exists() noexcept { return m_bdat && m_idx < m_bdat->pins.size(); }

[[nodiscard]] bool VirtualDigitalDriver::can_read() noexcept {
    return exists() && m_bdat->pins[m_idx].can_digital_read;
}

[[nodiscard]] bool VirtualDigitalDriver::can_write() noexcept {
    return exists() && m_bdat->pins[m_idx].can_digital_write;
}

[[nodiscard]] bool VirtualDigitalDriver::read() noexcept { return exists() && m_bdat->pins[m_idx].value.load(); }

void VirtualDigitalDriver::write(bool value) noexcept {
    if (exists())
        m_bdat->pins[m_idx].value.store(value ? 255 : 0);
}

[[nodiscard]] bool VirtualPin::exists() noexcept { return m_bdat && m_idx < m_bdat->pins.size(); }

[[nodiscard]] bool VirtualPin::locked() noexcept {
    return !exists() || m_bdat->pins[m_idx].active_driver != BoardData::Pin::ActiveDriver::gpio;
}

void VirtualPin::set_direction(DataDirection dir) noexcept {
    if (exists() && !locked())
        m_bdat->pins[m_idx].data_direction = static_cast<BoardData::Pin::DataDirection>(dir);
}

[[nodiscard]] auto VirtualPin::get_direction() noexcept -> DataDirection {
    return exists() && !locked() ? static_cast<DataDirection>(m_bdat->pins[m_idx].data_direction.load())
                                 : DataDirection::in;
}

VirtualPin VirtualPins::operator[](std::size_t pin_id) noexcept {
    if (!m_bdat)
        return {m_bdat, 0};
    const auto it = std::lower_bound(m_bdat->pins.begin(), m_bdat->pins.end(), pin_id,
                                     [](const auto& pin, std::size_t pin_id) { return pin.id < pin_id; });
    if (it != m_bdat->pins.end()) {
        if (const auto delta = std::distance(m_bdat->pins.begin(), it); delta >= 0 && m_bdat->pins[delta].id == pin_id)
            return {m_bdat, static_cast<std::size_t>(delta)};
    }
    return {nullptr, std::size_t(-1)};
}

[[nodiscard]] bool VirtualUartBuffer::exists() noexcept { return m_bdat && m_index < m_bdat->uart_channels.size(); }

[[nodiscard]] std::size_t VirtualUartBuffer::max_size() noexcept {
    // clang-format off
    return exists() ? (m_dir == Direction::rx
                           ? m_bdat->uart_channels[m_index].max_buffered_rx
                           : m_bdat->uart_channels[m_index].max_buffered_tx) : 0;
    // clang-format on
}

// Local function to get deque, mutex and mbuff
[[nodiscard]] static auto get_deque_mutex_mbuff(BoardData* bdat, size_t index, bool receive) {
    auto& chan = bdat->uart_channels[index];
    if (receive)
        return std::tie(chan.rx, chan.rx_mut, chan.max_buffered_rx);
    else
        return std::tie(chan.tx, chan.tx_mut, chan.max_buffered_tx);
}

[[nodiscard]] std::size_t VirtualUartBuffer::size() noexcept {
    if (!exists())
        return 0;
    auto [d, mut, ignored] = get_deque_mutex_mbuff(m_bdat, m_index, (m_dir == Direction::rx));
    if (!mut.timed_lock(microsec_clock::universal_time() + boost::posix_time::seconds{1}))
        return 0;
    std::lock_guard lg{mut, std::adopt_lock};
    const auto ret = d.size();
    return ret;
}

std::size_t VirtualUartBuffer::read(std::span<char> buf) noexcept {
    if (!exists())
        return 0;
    auto [d, mut, ignored] = get_deque_mutex_mbuff(m_bdat, m_index, (m_dir == Direction::rx));
    if (!mut.timed_lock(microsec_clock::universal_time() + boost::posix_time::seconds{1}))
        return 0;
    std::lock_guard lg{mut, std::adopt_lock};
    const std::size_t count = std::min(d.size(), buf.size());
    std::copy_n(d.begin(), count, buf.begin());
    d.erase(d.begin(), d.begin() + count);
    return count;
}

std::size_t VirtualUartBuffer::write(std::span<const char> buf) noexcept {
    if (!exists())
        return 0;
    auto [d, mut, max_buffered] = get_deque_mutex_mbuff(m_bdat, m_index, (m_dir == Direction::rx));
    if (!mut.timed_lock(microsec_clock::universal_time() + boost::posix_time::seconds{1}))
        return 0;
    std::lock_guard lg{mut, std::adopt_lock};
    const std::size_t count = std::min(
        std::clamp(max_buffered - d.size(), std::size_t{0}, static_cast<std::size_t>(max_buffered)), buf.size());
    std::copy_n(buf.begin(), count, std::back_inserter(d));
    return count;
}

[[nodiscard]] char VirtualUartBuffer::front() noexcept {
    if (!exists())
        return '\0';
    auto [d, mut, ignored] = get_deque_mutex_mbuff(m_bdat, m_index, (m_dir == Direction::rx));
    if (!mut.timed_lock(microsec_clock::universal_time() + boost::posix_time::seconds{1}))
        return 0;
    std::lock_guard lg{mut, std::adopt_lock};
    if (d.empty())
        return '\0';
    const char ret = d.front();
    return ret;
}

[[nodiscard]] bool VirtualUart::exists() noexcept { return m_bdat && m_index < m_bdat->uart_channels.size(); }

[[nodiscard]] bool VirtualUart::is_active() noexcept {
    return exists() && m_bdat->uart_channels[m_index].active.load();
}

void VirtualUart::set_active(bool value) noexcept {
    if (exists())
        m_bdat->uart_channels[m_index].active.store(value);
}

[[nodiscard]] VirtualUart VirtualUarts::operator[](std::size_t idx) noexcept {
    if (!m_bdat || m_bdat->uart_channels.size() <= idx)
        return VirtualUart{m_bdat, idx};
    return VirtualUart{m_bdat, idx};
}

[[nodiscard]] auto VirtualUarts::begin() noexcept -> Iterator { return Iterator{*this}; }

[[nodiscard]] auto VirtualUarts::end() noexcept -> Iterator { return Iterator{*this, size()}; }

[[nodiscard]] std::size_t VirtualUarts::size() noexcept { return m_bdat ? m_bdat->uart_channels.size() : 0; }

[[nodiscard]] VirtualUart VirtualUarts::Iterator::operator*() noexcept { return m_vu[m_index]; }

[[nodiscard]] bool FrameBuffer::exists() noexcept { return m_bdat && m_idx < m_bdat->frame_buffers.size(); }

[[nodiscard]] auto FrameBuffer::direction() noexcept -> Direction {
    return exists() ? Direction{static_cast<uint8_t>(m_bdat->frame_buffers[m_idx].direction)} : Direction::in;
}

[[nodiscard]] bool FrameBuffer::needs_horizontal_flip() noexcept {
    return exists() && m_bdat->frame_buffers[m_idx].transform.load().horiz_flip;
}

void FrameBuffer::needs_horizontal_flip(bool val) noexcept {
    if (!exists())
        return;

    auto trans = m_bdat->frame_buffers[m_idx].transform.load();
    trans.horiz_flip = val;
    m_bdat->frame_buffers[m_idx].transform.store(trans);
}

[[nodiscard]] bool FrameBuffer::needs_vertical_flip() noexcept {
    return exists() && m_bdat->frame_buffers[m_idx].transform.load().vert_flip;
}

void FrameBuffer::needs_vertical_flip(bool val) noexcept {
    if (!exists())
        return;

    auto trans = m_bdat->frame_buffers[m_idx].transform.load();
    trans.vert_flip = val;
    m_bdat->frame_buffers[m_idx].transform.store(trans);
}

[[nodiscard]] std::uint16_t FrameBuffer::get_width() noexcept {
    return exists() ? m_bdat->frame_buffers[m_idx].width.load() : 0;
}

void FrameBuffer::set_width(std::uint16_t width) noexcept {
    if (!exists())
        return;
    auto& fb = m_bdat->frame_buffers[m_idx];
    fb.width = width;
    fb.data.resize(width * fb.height * 3);
}

[[nodiscard]] std::uint16_t FrameBuffer::get_height() noexcept {
    return exists() ? m_bdat->frame_buffers[m_idx].height.load() : 0;
}

void FrameBuffer::set_height(std::uint16_t height) noexcept {
    if (!exists())
        return;
    auto& fb = m_bdat->frame_buffers[m_idx];
    fb.height = height;
    fb.data.resize(height * fb.width * 3);
}

[[nodiscard]] std::uint8_t FrameBuffer::get_freq() noexcept {
    return exists() ? m_bdat->frame_buffers[m_idx].freq.load() : 0;
}

void FrameBuffer::set_freq(std::uint8_t freq) noexcept {
    if (!exists())
        return;
    m_bdat->frame_buffers[m_idx].freq = freq;
}

bool FrameBuffer::write_rgb888(std::span<const std::byte> buf) {
    if (!exists())
        return false;

    auto& frame_buf = m_bdat->frame_buffers[m_idx];
    if (buf.size() != frame_buf.data.size())
        return false;

    [[maybe_unused]] std::lock_guard lk{frame_buf.data_mut};
    std::memcpy(frame_buf.data.data(), buf.data(), buf.size());
    return true;
}

bool FrameBuffer::read_rgb888(std::span<std::byte> buf) {
    if (!exists())
        return false;

    auto& frame_buf = m_bdat->frame_buffers[m_idx];
    if (buf.size() != frame_buf.data.size())
        return false;
    [[maybe_unused]] std::lock_guard lk{frame_buf.data_mut};
    std::memcpy(buf.data(), frame_buf.data.data(), buf.size());
    return true;
}

// convert 2bytes into 3bytes
void convert_rgb444_to_rgb888(std::span<const std::byte> source, std::byte* target) {
    // for each 2 bytes in the input write 3 bytes to the output
    for (auto i = source.begin(); i != source.end(); ++i) {
        std::byte left = *i++;
        std::byte right = *i;

        *target++ = (right << 4) | (right & (std::byte)0b00001111); // red
        *target++ = (left & (std::byte)0b11110000) | (left >> 4);   // green
        *target++ = (left << 4) | (left & (std::byte)0b00001111);   // blue
    }
}

// buf has the RGB444 format
// framebuffer always has RGB888 format
bool FrameBuffer::write_rgb444(std::span<const std::byte> source) {
    if (!exists())
        return false;

    auto& frame_buf = m_bdat->frame_buffers[m_idx];
    if (source.size() != frame_buf.data.size() / 3 * 2)
        return false;

    [[maybe_unused]] std::lock_guard lk{frame_buf.data_mut};

    // to is a pointer to the frame buffer data (a long array of bytes, where every 4 bytes is one pixel)
    std::byte* target = frame_buf.data.data();
    convert_rgb444_to_rgb888(source, target);
    return true;
}

// convert 3 bytes into 2 bytes
void convert_rgb888_to_rgb444(const std::byte* source, std::span<std::byte> target) {

    for (auto i = target.begin(); i != target.end(); ++i) {
        // read red/green/blue

        // pick values from the 24bits from frame buffer
        std::byte red = *source++;
        std::byte green = *source++;
        std::byte blue = *source++;

        // write to the target
        *i++ = (green & (std::byte)0b11110000) | ((blue & (std::byte)0b11110000) >> 4);
        *i = (red >> 4);
    }
}
// read from the frame buffer (rgb888) into the buf (rgb444)
bool FrameBuffer::read_rgb444(std::span<std::byte> target) {
    if (!exists())
        return false;

    auto& frame_buf = m_bdat->frame_buffers[m_idx];
    if (target.size() != frame_buf.data.size() / 3 * 2)
        return false;
    [[maybe_unused]] std::lock_guard lk{frame_buf.data_mut};

    const auto* source = frame_buf.data.data();
    convert_rgb888_to_rgb444(source, target);
    return true;
}

void convert_rgb565_to_rgb888(std::span<const std::byte> source, std::byte* target) {
    // read two bytes at the time
    for (auto i = source.begin(); i != source.end(); ++i) {
        std::byte left = *i++;
        std::byte right = *i;
        *target++ = (right & (std::byte)0b11111000) | (right >> 5); // red
        *target++ = ((right & (std::byte)0b00000111) << 5) | ((left & (std::byte)0b11100000) >> 3) |
                    ((right & (std::byte)0b00000110) >> 1);           // green
        *target++ = (left << 3) | ((left & (std::byte)0b11100) >> 2); // blue
    }
}
// Implementation of rgb565. conversion from RGB888 to RGB565
bool FrameBuffer::write_rgb565(std::span<const std::byte> source) {
    if (!exists())
        return false;

    auto& frame_buf = m_bdat->frame_buffers[m_idx];
    if (source.size() != frame_buf.data.size() / 2)
        return false;

    [[maybe_unused]] std::lock_guard lk{frame_buf.data_mut};

    // Unsure what this does, will try to remove it if it does not work.
    auto* target = frame_buf.data.data();

    // Example used to convert to rgb565 http://www.barth-dev.de/about-rgb565-and-how-to-convert-into-it/#
    convert_rgb565_to_rgb888(source, target);
    return true;
}

void convert_rgb888_to_rgb565(const std::byte* source, std::span<std::byte> target) {
    // read two bytes at the time
    for (auto i = target.begin(); i != target.end(); ++i) {
        // read red/green/blue

        // pick values from the 24bits from frame buffer
        std::byte red = *source++;
        std::byte green = *source++;
        std::byte blue = *source++;

        // write to the target
        *i++ = ((green & (std::byte)0b00011100) << 3) | ((blue & (std::byte)0b11111000) >> 3);
        *i = (red & (std::byte)0b11111000) | ((green & (std::byte)0b11100000) >> 5);
    }
}

// read from the frame buffer (rgb888) into the buf (rg565)
bool FrameBuffer::read_rgb565(std::span<std::byte> target) {
    if (!exists())
        return false;

    auto& frame_buf = m_bdat->frame_buffers[m_idx];
    if (target.size() != frame_buf.data.size())
        return false;
    [[maybe_unused]] std::lock_guard lk{frame_buf.data_mut};

    const auto* source = frame_buf.data.data();
    convert_rgb888_to_rgb565(source, target);

    return true;
}

void convert_rgb888_to_yuv422([[maybe_unused]] const std::byte* source, [[maybe_unused]] std::span<std::byte> target) {
    // TODO
    // https://en.wikipedia.org/wiki/YUV#Y%E2%80%B2UV422_to_RGB888_conversion
    // read two pixels at the time
    for (auto i = target.begin(); i != target.end();) {
        // read red/green/blue

        // pick values from the 24bits from frame buffer
        std::byte r1 = *source++;
        std::byte g1 = *source++;
        std::byte b1 = *source++;

        std::byte r2 = *source++;
        std::byte g2 = *source++;
        std::byte b2 = *source++;

        double y1 = 0.299 * (double)r1 + 0.587 * (double)g1 + 0.114 * (double)b1;
        double u1 = -0.169 * (double)r1 - 0.331 * (double)g1 + 0.499 * (double)b1 + 128;
        double v1 = 0.499 * (double)r1 - 0.418 * (double)g1 - 0.0813 * (double)b1 + 128;

        double y2 = 0.299 * (double)r2 + 0.587 * (double)g2 + 0.114 * (double)b2;
        double u2 = -0.169 * (double)r2 - 0.331 * (double)g2 + 0.499 * (double)b2 + 128;
        double v2 = 0.499 * (double)r2 - 0.418 * (double)g2 - 0.0813 * (double)b2 + 128;

        double u = (u1 + u2) / 2;
        double v = (v1 + v2) / 2;

        // write to the target
        *i++ = (std::byte)y1;
        *i++ = (std::byte)u;
        *i++ = (std::byte)y2;
        *i++ = (std::byte)v;
    }
}
void convert_yuv422_to_rgb888(std::span<const std::byte> source, std::byte* target) {
    // read four bytes at the time
    for (auto i = source.begin(); i != source.end();) {
        std::byte y1 = *i++;
        std::byte u = *i++;
        std::byte y2 = *i++;
        std::byte v = *i++;

        // Convert YUV to RGB of pixel 1
        double r1 = (double)y1 + 1.402 * ((double)v - 128);
        double g1 = (double)y1 - 0.344 * ((double)u - 128) - 0.714 * ((double)v - 128);
        double b1 = (double)y1 + 1.772 * ((double)u - 128);

        // Convert YUV to RGB of pixel 2
        double r2 = (double)y2 + 1.402 * ((double)v - 128);
        double g2 = (double)y2 - 0.344 * ((double)u - 128) - 0.714 * ((double)v - 128);
        double b2 = (double)y2 + 1.772 * ((double)u - 128);

        // Write R, G, B for pixel 1
        *target++ = (std::byte)std::clamp(r1, 0.0, 255.0);
        *target++ = (std::byte)std::clamp(g1, 0.0, 255.0);
        *target++ = (std::byte)std::clamp(b1, 0.0, 255.0);
        // Write R, G, B for pixel 2
        *target++ = (std::byte)std::clamp(r2, 0.0, 255.0);
        *target++ = (std::byte)std::clamp(g2, 0.0, 255.0);
        *target++ = (std::byte)std::clamp(b2, 0.0, 255.0);
    }
}

// YUV format
bool FrameBuffer::read_yuv422(std::span<std::byte> target) {
    if (!exists())
        return false;

    auto& frame_buf = m_bdat->frame_buffers[m_idx];
    if (target.size() != frame_buf.data.size() / 6 * 4)
        return false;
    [[maybe_unused]] std::lock_guard lk{frame_buf.data_mut};

    const auto* source = frame_buf.data.data();
    convert_rgb888_to_yuv422(source, target);

    return true;
}

// RGB888: R1 G1 B1 R2 G2 B2
// YUV422: Y1 U  Y2 V

bool FrameBuffer::write_yuv422(std::span<const std::byte> source) {
    if (!exists())
        return false;

    auto& frame_buf = m_bdat->frame_buffers[m_idx];
    if (source.size() != frame_buf.data.size() / 6 * 4)
        return false;

    [[maybe_unused]] std::lock_guard lk{frame_buf.data_mut};

    // Unsure what this does, will try to remove it if it does not work.
    auto* target = frame_buf.data.data();

    // Example used to convert to rgb565 http://www.barth-dev.de/about-rgb565-and-how-to-convert-into-it/#
    convert_yuv422_to_rgb888(source, target);
    return true;
}

// End of implementation of RGB565

FrameBuffer FrameBuffers::operator[](std::size_t key) noexcept {
    if (!m_bdat)
        return {m_bdat, 0};
    const auto it = std::lower_bound(m_bdat->frame_buffers.begin(), m_bdat->frame_buffers.end(), key,
                                     [](const auto& pin, std::size_t key) { return pin.key < key; });
    if (it != m_bdat->frame_buffers.end()) {
        if (const auto delta = std::distance(m_bdat->frame_buffers.begin(), it);
            delta >= 0 && m_bdat->frame_buffers[delta].key == key)
            return {m_bdat, static_cast<std::size_t>(delta)};
    }
    return {nullptr, std::size_t(-1)};
}

} // namespace smce
