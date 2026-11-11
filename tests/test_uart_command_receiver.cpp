#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <functional>
#include <vector>

#include "uart_command_receiver.h"
#include "crc.h"

namespace {
using Packet = std::array<uint8_t, 23>;

// Independent bitwise CRC implementation for constructing fake packets.
uint32_t WireCrc(const uint8_t* bytes, size_t size) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < size; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
        }
    }
    return ~crc;
}

void SetCrc(Packet& packet) {
    uint32_t crc = WireCrc(packet.data(), 19);
    for (size_t i = 0; i < 4; ++i) packet[19 + i] = crc >> (8 * i);
}

Packet MakePacket(uint16_t states) {
    Packet packet = {0x0D, 0xD0, 0xAD, 0xDE};
    for (size_t i = 0; i < 15; ++i) packet[4 + i] = (states >> i) & 1;
    SetCrc(packet);
    return packet;
}

uint16_t States(const GseCommand& command) {
    uint8_t bytes[sizeof(command)];
    memcpy(bytes, &command, sizeof(command));
    uint16_t states = 0;
    for (size_t i = 0; i < 15; ++i) states |= bytes[4 + i] << i;
    return states;
}

struct DmaSimulation {
    DMA_HandleTypeDef dma;
    UART_HandleTypeDef uart{&dma};
    UartCommandReceiver receiver{&uart};
    uint8_t* destination = nullptr;
    uint16_t size = 0;
    uint32_t primask = 0;
    bool running = false;
    int starts = 0;
    int aborts = 0;
    int errorClears = 0;
    int counterReads = 0;
    HAL_StatusTypeDef startStatus = HAL_OK;
    HAL_StatusTypeDef abortStatus = HAL_OK;
    std::function<void(int)> afterCounterRead;

    void ServiceInterrupt() {
        if (primask == 0 && dma.transferComplete) {
            dma.transferComplete = false; // HAL clears TC before running the callback.
            receiver.OnRxComplete();
        }
    }

    void Write(const uint8_t* bytes, size_t count, bool serviceInterrupt = true) {
        ASSERT_TRUE(running);
        for (size_t i = 0; i < count; ++i) {
            destination[size - dma.remaining] = bytes[i];
            if (--dma.remaining == 0) {
                dma.remaining = size;
                dma.transferComplete = true;
                if (serviceInterrupt) ServiceInterrupt();
            }
        }
    }

    void Write(const Packet& packet) { Write(packet.data(), packet.size()); }
    void Noise(size_t count) {
        std::vector<uint8_t> bytes(count, 0xAA);
        Write(bytes.data(), bytes.size());
    }
};

DmaSimulation* sim = nullptr;
}

extern "C" {
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef* uart) {
    ++sim->aborts;
    if (sim->abortStatus != HAL_OK) return sim->abortStatus;
    sim->running = false;
    sim->dma.transferComplete = false;
    uart->RxState = 0;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef* uart, uint8_t* data, uint16_t size) {
    ++sim->starts;
    if (sim->startStatus != HAL_OK) return sim->startStatus;
    sim->destination = data;
    sim->size = size;
    sim->dma.remaining = size;
    sim->dma.transferComplete = false;
    sim->dma.halfInterruptEnabled = true;
    sim->running = true;
    uart->RxState = 1;
    return HAL_OK;
}

uint32_t __get_PRIMASK() { return sim->primask; }
void __disable_irq() { sim->primask = 1; }
void __set_PRIMASK(uint32_t primask) {
    sim->primask = primask;
    sim->ServiceInterrupt();
}

uint32_t MockDmaCounter(DMA_HandleTypeDef* dma) {
    uint32_t remaining = dma->remaining;
    ++sim->counterReads;
    // Allow DMA to move after the register sample, including during a snapshot.
    if (sim->afterCounterRead) sim->afterCounterRead(sim->counterReads);
    return remaining;
}
void MockClearUartErrors(UART_HandleTypeDef*) { ++sim->errorClears; }
}

class UartCommandReceiverTest : public ::testing::Test {
  protected:
    DmaSimulation hardware;
    GseCommand command{};

    void SetUp() override {
        sim = &hardware;
        ASSERT_TRUE(hardware.receiver.Start());
    }
    void TearDown() override { sim = nullptr; }
    bool Read() { return hardware.receiver.ReadCommand(command); }
};

TEST_F(UartCommandReceiverTest, UsesDmaAndDoesNotRestartOnReads) {
    EXPECT_EQ(hardware.size, 1024);
    EXPECT_FALSE(hardware.dma.halfInterruptEnabled);
    EXPECT_FALSE(Read());
    hardware.Write(MakePacket(0x1234));
    EXPECT_TRUE(Read());
    EXPECT_FALSE(Read());
    EXPECT_EQ(hardware.starts, 1);
    EXPECT_EQ(hardware.aborts, 1);
}

TEST_F(UartCommandReceiverTest, PacketFormatAndCrcMatchProtocol) {
    EXPECT_EQ(sizeof(GseCommand), 23);
    EXPECT_EQ(offsetof(GseCommand, crc), 19);
    const uint8_t reference[] = "123456789";
    EXPECT_EQ(Crc32(reference, 9), 0xCBF43926u);
    hardware.Write(MakePacket(0x5555));
    ASSERT_TRUE(Read());
    EXPECT_EQ(command.magicHeader, 0xDEADD00Du);
    EXPECT_EQ(States(command), 0x5555);
    EXPECT_TRUE(command.igniter0Fire);
    EXPECT_FALSE(command.igniter1Fire);
    EXPECT_TRUE(command.alarm);
    EXPECT_FALSE(command.solenoidState0);
    EXPECT_TRUE(command.solenoidState11);
}

TEST_F(UartCommandReceiverTest, ConsumesOneCommandPerCallInFIFOOrder) {
    hardware.Write(MakePacket(1));
    hardware.Write(MakePacket(2));
    hardware.Write(MakePacket(3));
    for (uint16_t expected = 1; expected <= 3; ++expected) {
        ASSERT_TRUE(Read());
        EXPECT_EQ(States(command), expected);
    }
    EXPECT_FALSE(Read());
    EXPECT_EQ(States(command), 3);
}

TEST_F(UartCommandReceiverTest, PreservesPartialPacketAndHeaderAcrossSplits) {
    Packet packet = MakePacket(0x3456);
    for (size_t split = 1; split < packet.size(); ++split) {
        SCOPED_TRACE(split);
        hardware.Write(packet.data(), split);
        EXPECT_FALSE(Read());
        hardware.Write(packet.data() + split, packet.size() - split);
        ASSERT_TRUE(Read());
        EXPECT_EQ(States(command), 0x3456);
    }
}

TEST_F(UartCommandReceiverTest, IgnoresGarbageBeforeValidCommand) {
    hardware.Noise(101);
    hardware.Write(MakePacket(9));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 9);
}

TEST_F(UartCommandReceiverTest, PreservesPartialHeaderAfterGarbageAtBufferBoundary) {
    Packet packet = MakePacket(10);
    hardware.Noise(1022);
    hardware.Write(packet.data(), 2);
    EXPECT_FALSE(Read());
    hardware.Write(packet.data() + 2, packet.size() - 2);
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 10);
}

TEST_F(UartCommandReceiverTest, RejectsWrongHeaderEvenWithValidCrc) {
    Packet packet = MakePacket(7);
    packet[0] ^= 1; // Corrupts header on purpose
    SetCrc(packet);
    hardware.Write(packet);
    EXPECT_FALSE(Read());
    EXPECT_EQ(States(command), 0);
    hardware.Write(MakePacket(8));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 8);
}

TEST_F(UartCommandReceiverTest, RejectsBadCrcAndContinuesSearchingInSameRead) {
    Packet packet = MakePacket(7);
    packet.back() ^= 1; // Corrupts CRC
    hardware.Write(packet);
    hardware.Write(MakePacket(8));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 8); // Does it still read the next packet
}

TEST_F(UartCommandReceiverTest, ResynchronizesWhenNextHeaderOverlapsCorruptPacket) {
    // If say an error happens and partial command never finishes
    const uint8_t falseStart[] = {0x0D, 0xD0, 0xAD, 0xDE, 0xAA, 0xAA, 0xAA};
    hardware.Write(falseStart, sizeof(falseStart));
    hardware.Write(MakePacket(11));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 11);
}

TEST_F(UartCommandReceiverTest, RejectsInvalidBoolRepresentationEvenWithValidCrc) {
    // Extra check that all bools are 1s or 0s
    Packet packet = MakePacket(7);
    packet[4] = 2;
    SetCrc(packet);
    hardware.Write(packet);
    EXPECT_FALSE(Read());
    EXPECT_EQ(States(command), 0);
    hardware.Write(MakePacket(8));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 8);
}

TEST_F(UartCommandReceiverTest, InvalidBytesDontReplacePreviousAcceptedCommand) {
    hardware.Write(MakePacket(0x4321));
    ASSERT_TRUE(Read());
    Packet corrupt = MakePacket(0);
    corrupt[19] ^= 0x80;
    hardware.Write(corrupt);
    hardware.Noise(50);
    EXPECT_FALSE(Read());
    EXPECT_EQ(States(command), 0x4321); // The first valid command was still used while the bad bytes past it were ignored
}

TEST_F(UartCommandReceiverTest, ReadsHeadersAndPacketsAcrossBufferBoundary) {
    for (uint32_t offset : {1010u, 1022u}) {
        SCOPED_TRACE(offset);
        ASSERT_TRUE(hardware.receiver.Start());
        hardware.Noise(offset);
        EXPECT_FALSE(Read());
        hardware.Write(MakePacket(0x4567));
        ASSERT_TRUE(Read());
        EXPECT_EQ(States(command), 0x4567);
        EXPECT_FALSE(Read());
    }
}

TEST_F(UartCommandReceiverTest, FullBufferIsDistinguishableFromEmptyBuffer) {
    hardware.Write(MakePacket(17));
    hardware.Noise(1024 - sizeof(GseCommand));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 17);
    EXPECT_FALSE(Read());
}

TEST_F(UartCommandReceiverTest, OverflowRetainsNewestCommandsInOrder) {
    for (uint16_t i = 0; i < 90; ++i) hardware.Write(MakePacket(i));
    // 90 * 23 - 1024 is in the middle of command 45. Commands [46, 89] should survive.
    for (uint16_t expected = 46; expected < 90; ++expected) {
        ASSERT_TRUE(Read()) << expected; // Prints expected if assert fails
        EXPECT_EQ(States(command), expected);
    }
    EXPECT_FALSE(Read()); // There should be no more commands to read after number 89
}

TEST_F(UartCommandReceiverTest, OverflowRecoveryContinuesReceivingWithoutRestart) {
    hardware.Noise(3 * 1024 + 10);
    hardware.Write(MakePacket(31));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 31);
    hardware.Write(MakePacket(32));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 32);
    EXPECT_EQ(hardware.starts, 1);
}

TEST_F(UartCommandReceiverTest, CountsDMAWrapOncePerActualWrap) {
    hardware.Noise(1022);
    EXPECT_FALSE(Read());
    Packet packet = MakePacket(33);
    hardware.Write(packet.data(), packet.size(), false);
    ASSERT_TRUE(hardware.dma.transferComplete);
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 33);
    EXPECT_FALSE(hardware.dma.transferComplete);
    EXPECT_FALSE(Read());
    hardware.Write(MakePacket(34));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 34);
}

TEST_F(UartCommandReceiverTest, HandlesWrapDuringReadCycle) {
    hardware.Noise(1023);
    EXPECT_FALSE(Read());
    int trigger = hardware.counterReads + 1;
    hardware.afterCounterRead = [&, trigger](int reads) {
        if (reads == trigger) hardware.Noise(1);
    };
    EXPECT_FALSE(Read());
    hardware.afterCounterRead = {};
    hardware.Write(MakePacket(35));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 35);
}

TEST_F(UartCommandReceiverTest, DoesNotChaseBytesArrivingAfterReadSnapshot) {
    hardware.afterCounterRead = [&](int reads) {
        if (reads == 1) hardware.Write(MakePacket(36));
    };
    EXPECT_FALSE(Read());
    hardware.afterCounterRead = {};
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 36);
}

TEST_F(UartCommandReceiverTest, RejectsCandidateOverwrittenDuringProcessing) {
    hardware.Write(MakePacket(37));
    hardware.afterCounterRead = [&](int reads) {
        if (reads == 1) hardware.Noise(1024);
    };
    EXPECT_FALSE(Read());
    EXPECT_EQ(States(command), 0);
    hardware.afterCounterRead = {};
    hardware.Write(MakePacket(38));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 38);
}

TEST_F(UartCommandReceiverTest, ErrorCallbackDiscardsQueuedAndPartialData) {
    hardware.uart.gState = 42; // An ongoing TX must not be reset by RX recovery.
    Packet old = MakePacket(39);
    hardware.Write(old);
    hardware.Write(old.data(), 10);
    hardware.receiver.OnError();
    EXPECT_EQ(hardware.starts, 1);
    EXPECT_EQ(hardware.aborts, 1);
    EXPECT_FALSE(Read());
    EXPECT_EQ(hardware.starts, 2);
    EXPECT_EQ(hardware.uart.gState, 42u);
    hardware.Write(old.data() + 10, old.size() - 10);
    EXPECT_FALSE(Read());
    EXPECT_EQ(States(command), 0);
    hardware.Write(MakePacket(40));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 40);
}

TEST_F(UartCommandReceiverTest, RejectsCandidateIfErrorOccursDuringProcessing) {
    hardware.Write(MakePacket(41));
    hardware.afterCounterRead = [&](int reads) {
        if (reads == 2) hardware.receiver.OnError();
    };
    EXPECT_FALSE(Read());
    EXPECT_EQ(States(command), 0);
    hardware.afterCounterRead = {};
    EXPECT_FALSE(Read()); // Restart and discard interrupted data.
    hardware.Write(MakePacket(42));
    EXPECT_TRUE(Read());
}

TEST_F(UartCommandReceiverTest, RetriesFailedDmaStartOnFollowingTick) {
    hardware.receiver.OnError();
    hardware.startStatus = HAL_BUSY;
    EXPECT_FALSE(Read());
    EXPECT_EQ(hardware.starts, 2);
    EXPECT_FALSE(hardware.running);
    hardware.startStatus = HAL_OK;
    EXPECT_FALSE(Read());
    EXPECT_EQ(hardware.starts, 3);
    hardware.Write(MakePacket(43));
    ASSERT_TRUE(Read());
    EXPECT_EQ(States(command), 43);
}

TEST_F(UartCommandReceiverTest, RetriesFailedRxAbortBeforeRestarting) {
    hardware.receiver.OnError();
    hardware.abortStatus = HAL_TIMEOUT;
    EXPECT_FALSE(Read());
    EXPECT_EQ(hardware.starts, 1);
    hardware.abortStatus = HAL_OK;
    EXPECT_FALSE(Read());
    EXPECT_EQ(hardware.starts, 2);
    hardware.Write(MakePacket(44));
    EXPECT_TRUE(Read());
}

TEST_F(UartCommandReceiverTest, PreservesPreviouslyMaskedInterruptState) {
    hardware.Write(MakePacket(45));
    hardware.primask = 1;
    ASSERT_TRUE(Read());
    EXPECT_EQ(hardware.primask, 1u);
}