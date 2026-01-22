#![no_std]
#![no_main]

// --- インポート ---
use panic_halt as _; // パニック時は無限ループで停止（安全策）
use core::fmt::Write; // writeln! マクロを使用するために必要
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    serial::config::Config,
    i2c::I2c,
};

// --- BNO055 レジスタ定義 ---
// データシート: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
const BNO055_ADDR: u8    = 0x28; // COM3ピンがLOWの場合のアドレス (Highなら0x29)
const REG_CHIP_ID: u8    = 0x00; // チップIDレジスタ (固定値 0xA0 が入っているはず)
const REG_EULER_DATA: u8 = 0x1A; // オイラー角データの先頭アドレス (Heading LSB)
const REG_CALIB_STAT: u8 = 0x35; // キャリブレーションステータス
const REG_OPR_MODE: u8   = 0x3D; // 動作モード設定レジスタ
const REG_SYS_TRIGGER: u8 = 0x3F; // リセット等に使用

// --- 動作モード ---
const MODE_CONFIG: u8    = 0x00; // 設定モード (設定変更時はこれにする)
const MODE_NDOF: u8      = 0x0C; // 9軸フュージョンモード (推奨: 自動補正あり)

#[entry]
fn main() -> ! {
    // 1. 基本的な周辺機器の取得
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // 2. クロック設定
    // システムクロックを84MHzに設定（F446REの安定動作）
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
    
    // 遅延用タイマーの作成
    let mut delay = cp.SYST.delay(&clocks);

    // 3. GPIO設定
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    // --- UART設定 (PC通信用) ---
    // PA2: TX, PA3: RX
    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();
    
    let mut uart = dp.USART2.serial(
        (tx_pin, rx_pin),
        Config::default().baudrate(115_200.bps()),
        &clocks,
    ).unwrap();

    // --- I2C設定 (センサ通信用) ---
    // PB8: SCL, PB9: SDA
    // OpenDrainとPullUpはI2Cの必須要件
    let scl = gpiob.pb8.into_alternate().set_open_drain().internal_pull_up(true);
    let sda = gpiob.pb9.into_alternate().set_open_drain().internal_pull_up(true);
    
    // I2Cバスの作成 (100kHzは安定重視。高速化したい場合は400.kHz()へ)
    let mut i2c = I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &clocks);

    // --- 起動シーケンス開始 ---
    writeln!(uart, "\r\n=== BNO055 High-Spec Reader Start ===\r").unwrap();

    // BNO055は電源投入後、動作可能になるまで時間がかかる (約650ms)
    delay.delay_ms(700_u32);

    // 4. 接続確認 (Chip ID Check)
    let mut id_buf = [0u8; 1];
    loop {
        // デバイスが見つかるまで無限にリトライする（配線抜けなどを考慮）
        match i2c.write_read(BNO055_ADDR, &[REG_CHIP_ID], &mut id_buf) {
            Ok(_) if id_buf[0] == 0xA0 => {
                writeln!(uart, "Sensor Found! ID: 0xA0\r").unwrap();
                break;
            }
            _ => {
                writeln!(uart, "Waiting for BNO055... Check wiring.\r").unwrap();
                delay.delay_ms(500_u32);
            }
        }
    }

    // 5. センサのリセットとモード設定
    // 一旦CONFIGモードにする
    writeln!(uart, "Setting OPR Mode to CONFIG...\r").unwrap();
    i2c.write(BNO055_ADDR, &[REG_OPR_MODE, MODE_CONFIG]).unwrap();
    delay.delay_ms(20_u32); // モード切替時は待機が必要

    // 外部水晶の発振子を使う設定にする場合などはここに記述（今回は内部発振でシンプルに）

    // NDOFモード (9軸フュージョン) に設定
    writeln!(uart, "Setting OPR Mode to NDOF (Fusion)...\r").unwrap();
    i2c.write(BNO055_ADDR, &[REG_OPR_MODE, MODE_NDOF]).unwrap();
    delay.delay_ms(30_u32); // データシート記載の待機時間

    writeln!(uart, "Initialization Complete. Starting Loop.\r").unwrap();
    writeln!(uart, "Format: Heading,Roll,Pitch,SysCal,GyrCal,AccCal,MagCal\r").unwrap();

    // --- メインループ ---
    loop {
        // [データ取得]
        // オイラー角(6バイト) + 何か別のデータ、ではなく
        // ここでは「オイラー角」と「キャリブレーション」をそれぞれ取得します。
        
        let mut euler_buf = [0u8; 6];
        let mut calib_buf = [0u8; 1];
        
        // I2C通信は外部要因で失敗する可能性があるので Result をチェックする
        let euler_res = i2c.write_read(BNO055_ADDR, &[REG_EULER_DATA], &mut euler_buf);
        let calib_res = i2c.write_read(BNO055_ADDR, &[REG_CALIB_STAT], &mut calib_buf);

        match (euler_res, calib_res) {
            (Ok(_), Ok(_)) => {
                // --- 1. データの変換 (Raw -> Physical value) ---
                // BNO055のリトルエンディアンデータをi16に結合
                let h_raw = i16::from_le_bytes([euler_buf[0], euler_buf[1]]);
                let r_raw = i16::from_le_bytes([euler_buf[2], euler_buf[3]]);
                let p_raw = i16::from_le_bytes([euler_buf[4], euler_buf[5]]);

                // 単位変換: デフォルトでは 1 LSB = 1/16 度
                let heading = h_raw as f32 / 16.0;
                let roll    = r_raw as f32 / 16.0;
                let pitch   = p_raw as f32 / 16.0;

                // --- 2. キャリブレーション状態の分解 ---
                // 1バイトに [Sys:2bit][Gyr:2bit][Acc:2bit][Mag:2bit] の順で入っている
                let calib = calib_buf[0];
                let sys = (calib >> 6) & 0x03; // システム全体の信頼度 (0-3)
                let gyr = (calib >> 4) & 0x03;
                let acc = (calib >> 2) & 0x03;
                let mag = calib & 0x03;

                // --- 3. UART送信 (CSV形式) ---
                // フォーマット: H, R, P, Sys, Gyr, Acc, Mag
                // 小数点以下2桁まで表示
                writeln!(
                    uart,
                    "{:.2},{:.2},{:.2},{},{},{},{}\r",
                    heading, roll, pitch, sys, gyr, acc, mag
                ).unwrap();
            },
            _ => {
                // 通信エラー時
                writeln!(uart, "I2C Read Error!\r").unwrap();
            }
        }

        // --- ループ周期 ---
        // 100ms待機 (10Hz更新)
        // 早すぎてもシリアル通信が詰まるため、最初はこれくらいが適切
        delay.delay_ms(100_u32);
    }
}
