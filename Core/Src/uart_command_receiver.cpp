#include "uart_command_receiver.h"
#include "crc.h"
#include <cstddef>
#include <cstring>

constexpr uint8_t commandHeader[] = {0x0D, 0xD0, 0xAD, 0xDE}; // DEADD00D little-endian
constexpr size_t crcOffset = offsetof(GseCommand, crc);
static_assert(sizeof(GseCommand) == 23, "GseCommand packet format was expected to be 23 bytes (update tests if not)");
static_assert(crcOffset == 19, "CRC was expected to cover all fields except itself");

bool UartCommandReceiver::Start() {
    restartPending = true;
    // Cancel any transmissions before this starts or restarts
    if (HAL_UART_AbortReceive(uart) != HAL_OK) return false;

    dmaWraps = 0;
    readCount = 0;
    __HAL_UART_CLEAR_OREFLAG(uart); // Clears overrun flag if set (previously had this issue)
    restartPending = false;
    // Retry restart if receiving failed, const cast to strip volatile for function signature
    if (HAL_UART_Receive_DMA(uart, const_cast<uint8_t*>(buffer), bufferSize) != HAL_OK) {
        restartPending = true;
        return false;
    }
    // Only the full buffer interrupt is needed to count wraps (not using halfway interrupt)
    __HAL_DMA_DISABLE_IT(uart->hdmarx, DMA_IT_HT);
    return true;
}

uint32_t UartCommandReceiver::WriteCount() const {
    // Take a snapshot of the wrap count and write position of DMA controller
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t wraps = dmaWraps;
    uint32_t remaining = __HAL_DMA_GET_COUNTER(uart->hdmarx);
    // Transfer Complete (TC) flag denotes that wrap has occurred
    if (__HAL_DMA_GET_FLAG(uart->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(uart->hdmarx))) {
        ++wraps;
        // DMA continues during this short critical section, so re-read to prevent race
        // condition of wrapping occurring during this read
        remaining = __HAL_DMA_GET_COUNTER(uart->hdmarx);
    }
    __set_PRIMASK(primask);

    return wraps * bufferSize + (bufferSize - remaining) % bufferSize;
}

// Interrupt callbacks only update indices while allowing continued command reception.
// The wrap interrupt must run at least once per buffer cycle (~89 ms
// at 115200 baud), so don't disable interrupts for this long (lol if you do this it's cooked anyway)
void UartCommandReceiver::OnRxComplete() {
    ++dmaWraps;
}

// UART/DMA error recovery is deferred to ReadCommand
void UartCommandReceiver::OnError() {
    restartPending = true;
}

bool UartCommandReceiver::ReadCommand(GseCommand& command) {
    if (restartPending) {
        Start();
        return false;
    }

    // Limit each pass to the bytes present now to avoid racing with DMA which continues writing
    const uint32_t writeCount = WriteCount();
    if (writeCount - readCount > bufferSize) {
        // Old bytes were overwritten because we had a backlog > buffer size
        // Recover gracefully by searching the remaining window for the header
        readCount = writeCount - bufferSize;
    }

    // Skips if there's not a full command or greater to read
    while (writeCount - readCount >= sizeof(GseCommand)) {
        bool headerMatches = true;
        for (size_t i = 0; i < sizeof(commandHeader); ++i) {
            if (buffer[(readCount + i) % bufferSize] != commandHeader[i]) {
                headerMatches = false;
                break;
            }
        }
        if (!headerMatches) {
            // Real header could begin at any byte so we have to scan all of them (incrment by 1 only)
            // This may seem inefficient but re-reading bytes is very unlikely as we only read bytes that match header
            ++readCount;
            continue;
        }

        uint8_t packet[sizeof(GseCommand)];
        for (size_t i = 0; i < sizeof(packet); ++i) {
            // Copy from buffer with wrapping
            packet[i] = buffer[(readCount + i) % bufferSize];
        }

        // Reject a snapshot if DMA overwrote it
        const uint32_t currentWriteCount = WriteCount();
        if (currentWriteCount - readCount > bufferSize) {
            readCount = currentWriteCount - bufferSize;
            return false;
        }
        if (restartPending) return false; // Or if a UART error occurred mid-read

        // Validate CRC
        uint32_t receivedCrc;
        memcpy(&receivedCrc, packet + crcOffset, sizeof(receivedCrc));
        bool valid = memcmp(packet, commandHeader, sizeof(commandHeader)) == 0
                  && Crc32(packet, crcOffset) == receivedCrc;
        // Validate bools are 0 or 1 (additional misalignment protection)
        for (size_t i = sizeof(commandHeader); valid && i < crcOffset; ++i) {
            valid = packet[i] <= 1;
        }
        // At this point, we know the header, CRC, and bools are valid
        if (valid) {
            memcpy(&command, packet, sizeof(command));
            readCount += sizeof(command);
            return true;
        }
        // A corrupt candidate may overlap the next real header, so increment only by 1 (see above for explanation)
        ++readCount;
    }
    return false;
}
