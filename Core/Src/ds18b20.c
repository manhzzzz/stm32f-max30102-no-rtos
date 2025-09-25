#include "ds18b20.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*  ====== GHIM QUAN TRỌNG ======
    - Không dùng delay.h. HÀM NÀY do bạn cung cấp ở main.c (DWT-based).
*/
extern void delay_us(uint32_t us);

/* ========= OneWire low-level ========= */
static inline void ONEWIRE_LOW(void)  { HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET); }
static inline void ONEWIRE_HIGH(void) { HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);  }

static void ONEWIRE_Output(void) {
    GPIO_InitTypeDef G = {0};
    G.Pin   = DS18B20_PIN;
    G.Mode  = GPIO_MODE_OUTPUT_OD;     // Open-Drain (cần pull-up ngoài)
    G.Pull  = GPIO_NOPULL;
    G.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_PORT, &G);
}

static void ONEWIRE_Input(void) {
    GPIO_InitTypeDef G = {0};
    G.Pin  = DS18B20_PIN;
    G.Mode = GPIO_MODE_INPUT;
    G.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DS18B20_PORT, &G);
}

/* ========= 1-Wire core ========= */
static void ONEWIRE_WriteBit(uint8_t bit) {
    ONEWIRE_Output();
    ONEWIRE_LOW();
    if (bit) {
        // Write '1': pull low ~2-5us, thả ra đến hết slot (~60us)
        delay_us(3);
        ONEWIRE_Input();       // release
        delay_us(60);
    } else {
        // Write '0': giữ low ~60us
        delay_us(60);
        ONEWIRE_Input();       // release
        // recovery
        delay_us(2);
    }
}

static uint8_t ONEWIRE_ReadBit(void) {
    uint8_t bit;
    ONEWIRE_Output();
    ONEWIRE_LOW();
    delay_us(3);               // pull low rất ngắn
    ONEWIRE_Input();           // release để slave drive
    delay_us(10);              // sample ~10-15us
    bit = (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_SET);
    // kết thúc slot (~60us tổng)
    delay_us(50);
    return bit;
}

static void ONEWIRE_WriteByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        ONEWIRE_WriteBit(data & 0x01);
        data >>= 1;
    }
}

static uint8_t ONEWIRE_ReadByte(void) {
    uint8_t v = 0;
    for (int i = 0; i < 8; i++) {
        v >>= 1;
        if (ONEWIRE_ReadBit()) v |= 0x80;   // LSB first
    }
    return v;
}

/* Reset & presence detect: trả 1 nếu có thiết bị */
static uint8_t ONEWIRE_Reset(void) {
    uint8_t presence;
    ONEWIRE_Output();
    ONEWIRE_LOW();
    delay_us(480);
    ONEWIRE_Input();         // release
    delay_us(70);
    // Presence: slave kéo LOW
    presence = (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_RESET) ? 1 : 0;
    delay_us(410);
    return presence;
}

/* ========= DS18B20 High-level ========= */

// Trạng thái nội bộ để biến DS18B20_ReadTemp() thành non-blocking
static volatile uint8_t  s_conv_in_progress = 0;
static volatile uint32_t s_conv_start_ms    = 0;
static volatile float    s_last_temp_c      = 0.0f;

// Bắt đầu 1 lần chuyển đổi nhiệt độ (không chặn)
void DS18B20_StartConversion(void) {
    if (!ONEWIRE_Reset()) {
        // Không thấy DS18B20 trên bus: không khởi động được
        s_conv_in_progress = 0;
        return;
    }
    ONEWIRE_WriteByte(0xCC); // Skip ROM
    ONEWIRE_WriteByte(0x44); // Convert T
    s_conv_in_progress = 1;
    s_conv_start_ms    = HAL_GetTick();
}

// Kiểm tra đang bận (1=bận, 0=đã xong)
// Theo datasheet: trong lúc convert, đọc 1 "read time slot" sẽ trả 0; xong sẽ trả 1
uint8_t DS18B20_IsBusy(void) {
    // Nếu chưa từng start, coi như bận = 1 để code gọi StartConversion
    if (!s_conv_in_progress) return 1;
    return (ONEWIRE_ReadBit() == 0) ? 1 : 0;
}

// Đọc scratchpad ngay (KHÔNG delay) và trả về °C
float DS18B20_ReadTemp_NoDelay(void) {
    uint8_t tl, th;
    int16_t raw;
    if (!ONEWIRE_Reset()) {
        return s_last_temp_c; // giữ giá trị cũ nếu mất thiết bị
    }
    ONEWIRE_WriteByte(0xCC); // Skip ROM
    ONEWIRE_WriteByte(0xBE); // Read Scratchpad
    tl = ONEWIRE_ReadByte();
    th = ONEWIRE_ReadByte();
    // Có thể đọc thêm CRC nếu muốn (bỏ qua để tối ưu tốc độ)
    raw = (int16_t)((th << 8) | tl);
    return (float)raw / 16.0f;
}

/* API tiện dụng: KHÔNG chặn
   - Lần đầu gọi: khởi động convert & trả về giá trị gần nhất (0.0 nếu chưa có)
   - Các lần sau: nếu convert xong (hoặc timeout), đọc & cập nhật, rồi tự start vòng mới
   - Luôn trả về "giá trị đo gần nhất", KHÔNG chặn 750ms
*/
float DS18B20_ReadTemp(void) {
    uint32_t now = HAL_GetTick();

    // Nếu chưa start vòng nào, khởi động 1 vòng đo
    if (!s_conv_in_progress) {
        DS18B20_StartConversion();
        return s_last_temp_c;
    }

    // Nếu đã xong (hoặc timeout > 850ms), đọc giá trị và khởi động vòng mới
    if (!DS18B20_IsBusy() || (now - s_conv_start_ms) >= 850) {
        s_last_temp_c = DS18B20_ReadTemp_NoDelay();
        DS18B20_StartConversion();
    }
    return s_last_temp_c;
}

// Khởi tạo cơ bản: đảm bảo bus rảnh + khởi động vòng đo đầu tiên
void DS18B20_Init(void) {
    // Đưa line lên HIGH một nhịp
    ONEWIRE_Output();
    ONEWIRE_HIGH();
    delay_us(5);
    ONEWIRE_Input();

    // Thử reset để phát hiện thiết bị (không bắt buộc nhưng tốt cho debug)
    (void)ONEWIRE_Reset();

    // Bắt đầu chuyển đổi đầu tiên (non-blocking)
    DS18B20_StartConversion();
}

/* ====== GHI CHÚ ======
   - Nếu bạn muốn cách dùng "truyền thống":
       DS18B20_StartConversion();
       HAL_Delay(750);
       float t = DS18B20_ReadTemp_NoDelay();
     => vẫn OK.

   - Để mượt màn hình, trong vòng while() chỉ cần gọi:
       temperature = DS18B20_ReadTemp();
     => KHÔNG block, tự cập nhật ~mỗi 750ms khi xong.
*/
