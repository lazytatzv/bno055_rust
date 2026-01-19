#![no_std]
#![no_main]

use panic_probe as _; // パニック時にログを出す

use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
use hal::{pac, prelude::*};

use rtt_target::{rtt_init_print, rprintln};

// ★注意: さっき動いたアドレスにする (0x28 or 0x29)
const BNO055_ADDR: u8 = 0x28; 

const REG_CHIP_ID: u8    = 0x00; 
const REG_ACCEL_DATA: u8 = 0x08; // 加速度データ (X_LSB)
const REG_EULER_DATA: u8 = 0x1A; // 角度データ (Heading_LSB)

const REG_OPR_MODE: u8   = 0x3D; 
const REG_CALIB_STAT: u8 = 0x35; 

const CHIP_ID_VAL: u8     = 0xA0; 
const MODE_CONFIG: u8     = 0x00; 
const MODE_NDOF: u8       = 0x0C; 

#[entry]
fn main() -> ! {
    rtt_init_print!(); // 必須！

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
    let mut delay = cp.SYST.delay(&clocks);

    let gpiob = dp.GPIOB.split();

    // I2C1: PB8(SCL), PB9(SDA) + 内部プルアップ有効化
    let scl = gpiob.pb8.into_alternate().set_open_drain().internal_pull_up(true);
    let sda = gpiob.pb9.into_alternate().set_open_drain().internal_pull_up(true);

    let mut i2c = hal::i2c::I2c::new(
        dp.I2C1,
        (scl, sda),
        100.kHz(),
        &clocks,
    );

    delay.delay_ms(650_u32); // 起動待ち

    rprintln!("Searching for BNO055...");

    // IDチェック
    let mut id_buf = [0u8; 1];
    i2c.write_read(BNO055_ADDR, &[REG_CHIP_ID], &mut id_buf).unwrap();
    
    if id_buf[0] != CHIP_ID_VAL {
        panic!("Invalid Chip ID: {:#02x}", id_buf[0]);
    }
    rprintln!("BNO055 Found! ID: {:#02x}", id_buf[0]);

    // モード設定
    i2c.write(BNO055_ADDR, &[REG_OPR_MODE, MODE_CONFIG]).unwrap();
    delay.delay_ms(20_u32);
    i2c.write(BNO055_ADDR, &[REG_OPR_MODE, MODE_NDOF]).unwrap();
    delay.delay_ms(20_u32);

    loop {
        let mut buf = [0u8; 6];

        // --- 角度 (Euler) ---
        if i2c.write_read(BNO055_ADDR, &[REG_EULER_DATA], &mut buf).is_ok() {
            // リトルエンディアン変換 (Rustスタイル)
            let h_raw = i16::from_le_bytes([buf[0], buf[1]]);
            let r_raw = i16::from_le_bytes([buf[2], buf[3]]);
            let p_raw = i16::from_le_bytes([buf[4], buf[5]]);

            // 単位変換: 1度 = 16 LSB
            let heading = h_raw as f32 / 16.0;
            let roll    = r_raw as f32 / 16.0;
            let pitch   = p_raw as f32 / 16.0;

            rprintln!("Euler -> H: {:.1}, R: {:.1}, P: {:.1}", heading, roll, pitch);
        }

        // --- 加速度 (Accel) ---
        if i2c.write_read(BNO055_ADDR, &[REG_ACCEL_DATA], &mut buf).is_ok() {
            // リトルエンディアン変換
            let ax_raw = i16::from_le_bytes([buf[0], buf[1]]);
            let ay_raw = i16::from_le_bytes([buf[2], buf[3]]);
            let az_raw = i16::from_le_bytes([buf[4], buf[5]]);

            // 単位変換: 1m/s^2 = 100 LSB
            let ax = ax_raw as f32 / 100.0;
            let ay = ay_raw as f32 / 100.0;
            let az = az_raw as f32 / 100.0;

            rprintln!("Accel -> X: {:.2}, Y: {:.2}, Z: {:.2}", ax, ay, az);
        }

        let mut calib = [0u8; 1];
        if i2c.write_read(BNO055_ADDR, &[REG_CALIB_STAT], &mut calib).is_ok() {
            let v = calib[0];
            
            // 1バイトの中に2bitずつ4つの情報が詰まってるので分解する
            let sys = (v >> 6) & 0x03; // システム (0-3)
            let gyr = (v >> 4) & 0x03; // ジャイロ (0-3)
            let acc = (v >> 2) & 0x03; // 加速度 (0-3)
            let mag = v & 0x03;        // 地磁気 (0-3)

            rprintln!("Calib -> Sys:{}, Gyr:{}, Acc:{}, Mag:{}", sys, gyr, acc, mag);
        }

        delay.delay_ms(100_u32);
    }
}
