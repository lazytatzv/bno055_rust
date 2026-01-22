#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    serial::config::Config,
    i2c::I2c,
};

// --- 設定 ---
const BNO055_ADDR: u8 = 0x28;
// I2Cクロック: 400kHz (高速モード) で通信時間を短縮
const I2C_SPEED: u32 = 400_000;
// UARTボーレート: 115200bps (50Hzなら十分。必要なら921600等へ上げる)
const UART_BAUD: u32 = 115_200;

// レジスタ定義
const REG_CHIP_ID: u8    = 0x00;
const REG_GYRO_DATA_X: u8 = 0x14; // ここから一括読み出し開始
const REG_CALIB_STAT: u8 = 0x35; // キャリブレーション状態
const REG_OPR_MODE: u8   = 0x3D;

const MODE_CONFIG: u8    = 0x00;
const MODE_NDOF: u8      = 0x0C;

// --- 送信パケット構造体 (25 bytes) ---
// ROS 2 (PC) 側で受け取るためのバイナリレイアウト
#[repr(C, packed)]
struct ImuPacket {
    header: [u8; 2],      // 0: 0xAA, 1: 0x55
    counter: u8,          // 2: パケットロスト検知用 (0-255)
    // --- Sensor Data (Little Endian i16) ---
    gyro_x: i16,          // 3-4
    gyro_y: i16,          // 5-6
    gyro_z: i16,          // 7-8
    quat_w: i16,          // 9-10 (BNO055はWが先頭)
    quat_x: i16,          // 11-12
    quat_y: i16,          // 13-14
    quat_z: i16,          // 15-16
    acc_lin_x: i16,       // 17-18 (Linear Acceleration)
    acc_lin_y: i16,       // 19-20
    acc_lin_z: i16,       // 21-22
    // --- Status ---
    calib_stat: u8,       // 23: Sys, Gyr, Acc, Mag
    checksum: u8,         // 24: sum(bytes[2..24])
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // 1. クロック設定 (84MHz)
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
    let mut delay = cp.SYST.delay(&clocks);

    // 2. GPIO & Peripherals
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    // UART (PC通信用)
    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();
    let mut uart = dp.USART2.serial(
        (tx_pin, rx_pin),
        Config::default().baudrate(UART_BAUD.bps()),
        &clocks,
    ).unwrap();

    // I2C (センサ用) - 高速モード400kHz
    let scl = gpiob.pb8.into_alternate().set_open_drain().internal_pull_up(true);
    let sda = gpiob.pb9.into_alternate().set_open_drain().internal_pull_up(true);
    let mut i2c = I2c::new(dp.I2C1, (scl, sda), I2C_SPEED.Hz(), &clocks);

    // --- BNO055 初期化シーケンス ---
    delay.delay_ms(700_u32); // 起動待ち

    // IDチェック
    let mut id_buf = [0u8; 1];
    loop {
        if i2c.write_read(BNO055_ADDR, &[REG_CHIP_ID], &mut id_buf).is_ok() && id_buf[0] == 0xA0 {
            break;
        }
        delay.delay_ms(100_u32);
    }

    // NDOFモード設定
    i2c.write(BNO055_ADDR, &[REG_OPR_MODE, MODE_CONFIG]).unwrap();
    delay.delay_ms(20_u32);
    i2c.write(BNO055_ADDR, &[REG_OPR_MODE, MODE_NDOF]).unwrap();
    delay.delay_ms(30_u32);

    let mut packet_counter: u8 = 0;

    // --- メインループ (50Hzターゲット) ---
    loop {
        // [Burst Read]
        // 0x14(Gyro) から 0x2D(Linear Accel) まで、間のEuler(0x1A)も含めて一気に読む。
        // サイズ: 0x2D - 0x14 + 1 = 26 bytes (0x1A-0x1Fの6バイトは捨てる)
        // さらにキャリブレーション(0x35)も欲しいが、離れているので別で読むか、
        // 効率重視で 0x14〜0x35 (34 bytes) 全部読んでしまうのが最強。
        // ここでは可読性とメモリ効率のバランスで「メインデータ(26byte)」と「Calib(1byte)」を2回で読む。
        
        let mut raw_buf = [0u8; 26]; // Gyro(6) + Euler(6) + Quat(8) + LinAcc(6)
        let mut calib_buf = [0u8; 1];

        // I2Cトランザクション
        let res_data = i2c.write_read(BNO055_ADDR, &[REG_GYRO_DATA_X], &mut raw_buf);
        let res_calib = i2c.write_read(BNO055_ADDR, &[REG_CALIB_STAT], &mut calib_buf);

        if res_data.is_ok() && res_calib.is_ok() {
            // パケット作成 (unsafeによる強制キャストで高速化)
            // バッファからデータを抽出 (Eulerデータ raw_buf[6..12] はスキップ)
            let mut packet = ImuPacket {
                header: [0xAA, 0x55],
                counter: packet_counter,
                
                // Gyro (0x14 ~ 0x19) -> buf[0..6]
                gyro_x: i16::from_le_bytes([raw_buf[0], raw_buf[1]]),
                gyro_y: i16::from_le_bytes([raw_buf[2], raw_buf[3]]),
                gyro_z: i16::from_le_bytes([raw_buf[4], raw_buf[5]]),

                // Quat (0x20 ~ 0x27) -> buf[12..20] (Euler 6byteを飛ばす)
                quat_w: i16::from_le_bytes([raw_buf[12], raw_buf[13]]),
                quat_x: i16::from_le_bytes([raw_buf[14], raw_buf[15]]),
                quat_y: i16::from_le_bytes([raw_buf[16], raw_buf[17]]),
                quat_z: i16::from_le_bytes([raw_buf[18], raw_buf[19]]),

                // Linear Accel (0x28 ~ 0x2D) -> buf[20..26]
                acc_lin_x: i16::from_le_bytes([raw_buf[20], raw_buf[21]]),
                acc_lin_y: i16::from_le_bytes([raw_buf[22], raw_buf[23]]),
                acc_lin_z: i16::from_le_bytes([raw_buf[24], raw_buf[25]]),

                calib_stat: calib_buf[0],
                checksum: 0,
            };

            // チェックサム計算 (CounterからCalibまでを単純加算)
            let packet_bytes: &[u8] = unsafe {
                core::slice::from_raw_parts(
                    &packet as *const _ as *const u8,
                    core::mem::size_of::<ImuPacket>(),
                )
            };
            
            // checksumフィールド(最後)の手前まで合計
            let mut sum: u8 = 0;
            for i in 2..24 { 
                sum = sum.wrapping_add(packet_bytes[i]);
            }
            packet.checksum = sum;

            // 送信
            for b in packet_bytes {
                nb::block!(uart.write(*b)).unwrap();
            }

            packet_counter = packet_counter.wrapping_add(1);
        }

        // 50Hz (20ms) ループ
        delay.delay_ms(20_u32);
    }
}
