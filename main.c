// 送信機メインdsPIC
#define FCY 69784687UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "hard_i2c.h"
#include "lcd_i2c.h"
#include "twe_lite.h"


typedef union tagHL16 {
    uint16_t HL;
    struct {
        uint8_t L;
        uint8_t H;
    };
    struct {
        unsigned :7;
        unsigned S:1;
        unsigned :7;
        unsigned T:1;
    };
} HL16;


typedef union tagHL32 {
    uint32_t HL;
    struct {
        uint16_t L;
        uint16_t H;
    };
} HL32;


uint16_t in2std(uint16_t b, uint16_t n, uint16_t f, uint16_t in, uint16_t center, uint16_t minmax);
uint8_t cal_move(uint8_t k0v, uint8_t k1v, uint8_t k2v, uint8_t k3v);
void main_loop(void);
void clear_buf(void);
void get_sub_keys(void);
void get_keys(void);
char get_calibration(void);
char test_input_keys(void);
void menu(char page);


// ニュートラルとみなす入力信号振れ
#define CENTER 50
// テールランプ点灯最少減速量
#define LIMIT_TAIL 128
// 過去これだけの減速量を判定に用いる
#define COUNT_TAIL 16
// 一度点灯すると最低でもこれだけ連続点灯
#define LED_CONT 8

uint16_t tail[COUNT_TAIL]; // 過去のv0
uint8_t tail_p; // 判定結果格納先

// ブザーを鳴らす長さ
#define LEN_BUZ 10
// エラー表示する長さ
#define LEN_ERR 200

char cnt_buz = 0; // ブザー鳴らす

// キャリブレーション結果
uint16_t adc_MAX_LX = 0;
uint16_t adc_MIN_LX = 0;
uint16_t adc_MID_LX = 0;
uint16_t adc_MAX_LY = 0;
uint16_t adc_MIN_LY = 0;
uint16_t adc_MID_LY = 0;
uint16_t adc_MAX_RX = 0;
uint16_t adc_MIN_RX = 0;
uint16_t adc_MID_RX = 0;
uint16_t adc_MAX_RY = 0;
uint16_t adc_MIN_RY = 0;
uint16_t adc_MID_RY = 0;


// AD変換値 DMAセットされる
uint16_t thro_in = 0; // スロットル入力値
uint16_t aile_in = 0; // エルロン入力値
uint16_t can_in = 0; // 砲身上下入力値
uint16_t ch1_in = 0; // 砲塔旋回入力値
uint16_t batt_master = 0; // 送信機バッテリー電圧
uint16_t trig = 0;
uint16_t push1 = 0;
uint16_t push2 = 0;

uint8_t tank = 0; // 戦車の種類 0:STANK 1:tiger1バトルタンク 2:機動戦闘車 3:tiger16

uint16_t batt_slave; // 受信機バッテリー電圧
uint8_t wifi_master; // 送信機からの電波の強度
uint8_t wifi_slave; // 受信機からの電波の強度

#define MAX_BUF 32
char buf[MAX_BUF];
uint8_t rsv[32];


uint8_t led_rest; // 尾灯の残り点灯時間（サーボパルス単位）
uint16_t thro_in; // スロットル入力値
uint16_t aile_in; // エルロン入力値
uint16_t ch1_in;  // 砲塔旋回入力値
uint16_t can_in;  // 砲身上下入力値
uint8_t left0;    // 左キャタ以前の値
uint8_t right0;   // 右キャタ以前の値
uint8_t left;  // 左キャタ 1:最大後退 128:ニュートラル 255:最大前進
uint8_t right; // 右キャタ 1:最大後退 128:ニュートラル 255:最大前進
uint8_t turn;  // 砲塔旋回 0:最大右旋回 128:ニュートラル 255:最大左旋回
uint8_t cannon;  // 砲身上下 0:最速DOWN 128:ニュートラル 255:最速UP
uint16_t v0;   // スロットル変換値 固定小数点 0=最大後退 4=ニュートラル 8=最大前進
uint16_t v1;   // エルロン変換値 固定小数点 0=左最大 4=ニュートラル 8=右最大
uint8_t led_on; // 尾灯を点灯させるなら1


uint8_t k0;    // 表オフセット値（参照４点中の左下）
uint8_t k1;    // 表オフセット値（参照４点中の右下）
uint8_t k2;    // 表オフセット値（参照４点中の左上）
uint8_t k3;    // 表オフセット値（参照４点中の右上）


uint8_t wifi_buzzer; // 受信状態ブザー 1:ON 0:OFF

uint16_t num_bb = 0; // 弾数
uint8_t temper = 0; // 温度
uint16_t coil = 0; // コイルガン


#define NUM_DATA 25
char data[NUM_DATA]; // キー状態
char data_idx; // 次に送信するのは data[data_idx]
//[0]  T-SPEED MID なら 1 そうでないなら 0   TRR-U      メニュー入り
//[1]  T-SPEED HIGH なら 1 そうでないなら 0  TRR-D      BEEP切り替え

//車高ニュートラルに
//[2]  左PAD押し込まれたら1                  TRR-L
//[3]  右PAD押し込まれたら1                  TRR-R


//[4]  ↑が押されたら1                       RigtU B
//[5]  ↓が押されたら1                       RigtD A

//[6]  ←が押されたら1                       LeftD ↑
//[7]  →が押されたら1                       LeftU ↓

//俯仰を水平に戻す
//[8]  TURBOキーが押されたら1                TRL-U
//[9]  CLEARキーが押されたら1                TRL-D

//レーザー切り替え（電源ＯＮ時に押されていれば車種変更）
//[10] LBキーが押されたら1                   TRL-L
//[11] RBキーが押されたら1                   TRL-R

//[12] Aが押されたら1                        A
//[13] Bが押されたら1　　　　　　　　　　　　R
//[14] Xが押されたら1                        LR
//[15] Yが押されたら1                        L


//thro_in, aile_in, can_in, ch1_in
//RY       RX       LY      LX
//         →が大           ←が大


// デジタルキー入力の安定化を行い、操作用の入力状態を作成する
#define NUM_KEYS 16
#define KEY_RESPONSE 3
char data_cnt_on[NUM_KEYS]; // キーが連続でONだった回数
char data_cnt_off[NUM_KEYS]; // キーが連続でOFFだった回数
char data_key[NUM_KEYS]; // キーの確定状態 1 または 0
char data_pre[NUM_KEYS]; // 直前の data_key[] の値
char data_push[NUM_KEYS]; // key=1 で pre=0 の場合のみ1 


// 文字列リソース
char msg_menu_b[] = {'M','E','N','U','=','T','R','R','-','U',0xce,0xde,0xc0,0xdd,0}; // MENU = TRR-Uﾎﾞﾀﾝ
char msg_push_b[] = {'B',0xce,0xde,0xc0,0xdd,0xa6,' ',0xb5,0xbc,0xc3,' ',0xb8,0xc0,0xde,0xbb,0xb2,0}; // Bﾎﾞﾀﾝｦ ｵｼﾃ ｸﾀﾞｻｲ
char msg_dont_t[] = {0xce,0xb6,0xc9,' ',0xce,0xde,0xc0,0xdd,' ',0xc6,' ',0xbb,0xdc,0xd7,0xbd,0xde,0}; // ﾎｶﾉ ﾎﾞﾀﾝ ﾆ ｻﾜﾗｽﾞ
char msg_sel_gd[] = {' ','u','s','e',' ','U','P',',','D','O','W','N',',','B',',','A',0}; // use UP,DOWN,B,A


// 左キャタの動き量テーブル
// [y+4][4-x]
uint8_t table_right[] = {
// -4
		128,96,64,32,1,1,1,1,1,
// -3
		128,104,80,56,32,32,32,32,32,
// -2
		128,112,96,80,64,64,64,64,64,
// -1
		128,120,112,104,96,96,96,96,96,
// 0
		128,128,128,128,128,128,128,128,128,
// +1
		128,136,144,152,160,160,160,160,160,
// +2
		128,144,160,176,192,192,192,192,192,
// +3
		128,152,176,200,224,224,224,224,224,
// +4
		128,160,192,224,255,255,255,255,255
};

// 右キャタの動き量テーブル
uint8_t table_left[] = {
// -4
		1,1,1,1,1,32,64,96,128,
// -3
		32,32,32,32,32,56,80,104,128,
// -2
		64,64,64,64,64,80,96,112,128,
// -1
		96,96,96,96,96,104,112,120,128,
// 0
		128,128,128,128,128,128,128,128,128,
// +1
		160,160,160,160,160,152,144,136,128,
// +2
		192,192,192,192,192,176,160,144,128,
// +3
		224,224,224,224,224,200,176,152,128,
// +4
		255,255,255,255,255,224,192,160,128
};


// Allocate and reserve a page of flash for this test to use.  The compiler/linker will reserve this for data and not place any code here.
static __prog__  uint8_t flashTestPage[FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS] __attribute__((space(prog),aligned(FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS)));

// We have detected a flash hardware error of some type.
static void FlashError()
{
    while (1) 
    { }
}

static void MiscompareError()
{
    while (1) 
    { }
}


// オートパワーオフタイマー
void int_timer(void) {
    LED_SetLow();
    POW_OUT_SetLow();
    while (1) ;
}


/////////////////////////////////////////
// キャリブレーション保存
// 失敗したら0以外を返す
/////////////////////////////////////////
char save_calibration() {
    uint32_t flash_storage_address;
    bool result;
    uint32_t write_data[6];
    uint32_t read_data[6];

    HL32 d;
    
    // Get flash page aligned address of flash reserved above for this test.
    flash_storage_address = FLASH_GetErasePageAddress((uint32_t)&flashTestPage[0]);

    FLASH_Unlock(FLASH_UNLOCK_KEY);

    result = FLASH_ErasePage(flash_storage_address);
    if (result == false) {
        FlashError();
        return 1;
    }
    
    // Fill first 4 flash words with data
    // For this product we must write two adjacent words at a one time.
    d.H = 0;
    d.L = adc_MAX_LX;
    d.HL <<= 12;
    d.L += (adc_MIN_LX & 0x0fff);
    write_data[0] = d.HL;

    d.H = 0;
    d.L = adc_MID_LX;
    d.HL <<= 12;
    d.L += (adc_MAX_LY & 0x0fff);
    write_data[1] = d.HL;

    d.H = 0;
    d.L = adc_MIN_LY;
    d.HL <<= 12;
    d.L += (adc_MID_LY & 0x0fff);
    write_data[2] = d.HL;

    d.H = 0;
    d.L = adc_MAX_RX;
    d.HL <<= 12;
    d.L += (adc_MIN_RX & 0x0fff);
    write_data[3] = d.HL;

    d.H = 0;
    d.L = adc_MID_RX;
    d.HL <<= 12;
    d.L += (adc_MAX_RY & 0x0fff);
    write_data[4] = d.HL;

    d.H = 0;
    d.L = adc_MIN_RY;
    d.HL <<= 12;
    d.L += (adc_MID_RY & 0x0fff);
    write_data[5] = d.HL;

    // For this product we must write two adjacent words at a one time.
    result  = FLASH_WriteDoubleWord24(flash_storage_address,   write_data[0], write_data[1]);
    result &= FLASH_WriteDoubleWord24(flash_storage_address + 4, write_data[2], write_data[3]);
    result &= FLASH_WriteDoubleWord24(flash_storage_address + 8, write_data[4], write_data[5]);
    if (result == false) {
        FlashError();
        return 1;
    }

    // Clear Key for NVM Commands so accidental call to flash routines will not corrupt flash
    FLASH_Lock();
    
    // read the flash words to verify the data
    read_data[0] = FLASH_ReadWord24(flash_storage_address);
    read_data[1] = FLASH_ReadWord24(flash_storage_address + 2);
    read_data[2] = FLASH_ReadWord24(flash_storage_address + 4 );
    read_data[3] = FLASH_ReadWord24(flash_storage_address + 6 );
    read_data[4] = FLASH_ReadWord24(flash_storage_address + 8 );
    read_data[5] = FLASH_ReadWord24(flash_storage_address + 10 );

    // Stop if the read data does not match the write data;
    if ( (write_data[0] != read_data[0]) ||
         (write_data[1] != read_data[1]) ||
         (write_data[2] != read_data[2]) ||
         (write_data[3] != read_data[3]) ||
         (write_data[4] != read_data[4]) ||
         (write_data[5] != read_data[5]) )
    {
        MiscompareError();    
        return 1;
    }
    
    return 0;
}


/////////////////////////////////////////
// キャリブレーション取得
// キャリブレーションが必要なら0以外を返す
/////////////////////////////////////////
char get_calibration() {
    uint32_t flash_storage_address;
    HL32 d;
    
    // Get flash page aligned address of flash reserved above for this test.
    flash_storage_address = FLASH_GetErasePageAddress((uint32_t)&flashTestPage[0]);
    
    d.HL = FLASH_ReadWord24(flash_storage_address);
    adc_MIN_LX = (d.L & 0x0fff);
    d.HL >>= 12;
    adc_MAX_LX = d.L;
    if (adc_MAX_LX < 2000) return 1;
    
    d.HL = FLASH_ReadWord24(flash_storage_address + 2);
    adc_MAX_LY = (d.L & 0x0fff);
    d.HL >>= 12;
    adc_MID_LX = d.L;
    if (adc_MAX_LY < 2000) return 1;
    
    d.HL = FLASH_ReadWord24(flash_storage_address + 4 );
    adc_MID_LY = (d.L & 0x0fff);
    d.HL >>= 12;
    adc_MIN_LY = d.L;
    if (adc_MID_LY < 1000) return 1;
    
    d.HL = FLASH_ReadWord24(flash_storage_address + 6 );
    adc_MIN_RX = (d.L & 0x0fff);
    d.HL >>= 12;
    adc_MAX_RX = d.L;
    if (adc_MAX_RX < 2000) return 1;
    
    d.HL = FLASH_ReadWord24(flash_storage_address + 8 );
    adc_MAX_RY = (d.L & 0x0fff);
    d.HL >>= 12;
    adc_MID_RX = d.L;
    if (adc_MAX_RY < 2000) return 1;
    
    d.HL = FLASH_ReadWord24(flash_storage_address + 10 );
    adc_MID_RY = (d.L & 0x0fff);
    d.HL >>= 12;
    adc_MIN_RY = d.L;
    if (adc_MID_RY < 1000) return 1;
    
    return 0;
}


// 入力AD値を標準値(0 to 8) に正規化
// b:最大後退値 n:ニュートラル値 f:最大前進値 in:入力AD値
// return 上位バイトが整数部 下位バイトが小数部
uint16_t in2std(uint16_t b, uint16_t n, uint16_t f, uint16_t in, uint16_t center, uint16_t minmax) {
    unsigned long acc;
    if (in > n) { // 前進時
        if (in >= (f - minmax)) return 0x0800;
        if (in <= (n + center)) return 0x0400;
        acc = (unsigned long)(in - (n + center));
        acc <<= 10;
        acc /= ((f - minmax) - (n + center));
        uint16_t ret = (uint16_t)acc;
        return (ret + 0x0400);
    }
    else { // 後退時
        if (in <= (b + minmax)) return 0;
        if (in >= (n - center)) return 0x0400;
        acc = (unsigned long)(in - (b + minmax));
        acc <<= 10;
        acc /= ((n - center) - (b + minmax));
        return (uint16_t)acc;
    }
}


// 周囲４点の値 k0v k1v k2v k3v より内挿計算
// 内分点は v0 v1 の小数部を使う　２５６分の v0[0] v1[0]
uint8_t cal_move(uint8_t k0v, uint8_t k1v, uint8_t k2v, uint8_t k3v) {
    uint16_t v0s = v0 & 0xff;

    // k0v と k2v の内分点の値を計算し k02v へ格納
    HL16 k02v, K0v16; // 上位バイトが整数部で下位バイトが小数部
    k02v.H = 0;
    if (k2v > k0v) {
        k02v.L = k2v - k0v;
        k02v.HL *= v0s;
        k02v.H += k0v;
    }
    else {
        k02v.L = k0v - k2v;
        k02v.HL *= v0s;
        K0v16.H = k0v;
        K0v16.L = 0;
        k02v.HL = K0v16.HL - k02v.HL;
    }
    
    // k1v と k3v の内分点の値を計算し cal3 へ格納
    HL16 k13v, k1v16; // 上位バイトが整数部で下位バイトが小数部
    k13v.H = 0;
    if (k3v > k1v) {
        k13v.L = k3v - k1v;
        k13v.HL *= v0s;
        k13v.H += k1v;
    }
    else {
        k13v.L = k1v - k3v;
        k13v.HL *= v0s;
        k1v16.H = k1v;
        k1v16.L = 0;
        k13v.HL = k1v16.HL - k13v.HL;
    }

    // k02v と k13v の内分点の値を計算
    uint16_t v1s = v1 & 0xff;
    uint16_t v1sa;
    HL16 ab, ans;
    if (k13v.HL < k02v.HL) {
        ab.HL = k02v.HL - k13v.HL;
        v1sa = v1s * ab.H;
        ab.HL = v1s * ab.L;
        ans.HL = k13v.HL + v1sa + ab.H;
    }
    else {
        ab.HL = k13v.HL - k02v.HL;
        v1sa = v1s * ab.H;
        ab.HL = v1s * ab.L;
        ans.HL = k13v.HL - v1sa - ab.H;
    }

    if (ans.S) { // 四捨五入
        ans.H ++;
    }
    return ans.H;
}


void main_loop(void) {
	// 入力砲塔旋回を 0 to 255 に正規化 128=ニュートラル
    uint16_t v = in2std(adc_MIN_LX, adc_MID_LX, adc_MAX_LX, ch1_in, CENTER, CENTER);
    v >>= 3;
    if (v > 255) {
        turn = 255;
    }
    else {
        turn = (uint8_t)v;
    }

	// 入力砲身上下を 0 to 255 に正規化 128=ニュートラル
    v = in2std(adc_MIN_LY, adc_MID_LY, adc_MAX_LY, can_in, CENTER, CENTER);
    v >>= 3;
    if (v > 255) {
        cannon = 255;
    }
    else {
        cannon = (uint8_t)v;
    }

    if (tank == 0) { // S-TANK
    	// 入力スロットルを 0 to 255 に正規化 128=ニュートラル
        v = in2std(adc_MIN_RY, adc_MID_RY, adc_MAX_RY, thro_in, CENTER, CENTER);
        v >>= 3;
        if (v > 255) {
            left = 255;
        }
        else {
            left = (uint8_t)v;
        }

    	// 入力エルロンを 0 to 255 に正規化 128=ニュートラル
        v = in2std(adc_MIN_RX, adc_MID_RX, adc_MAX_RX, aile_in, CENTER, CENTER);
        v >>= 3;
        if (v > 255) {
            right = 255;
        }
        else {
            right = (uint8_t)v;
        }
        led_on = 0;
    }
    else if (tank == 1) { // Tiger1
        // 入力スロットル量を 0 to 8 に正規化 4=ニュートラル　整数部8bit+小数部8bit
        v0 = in2std(adc_MIN_RY, adc_MID_RY, adc_MAX_RY, thro_in, CENTER, CENTER);

        // 入力エルロン量を 0 to 8 に正規化 4=ニュートラル　整数部8bit+小数部8bit
        v1 = in2std(adc_MIN_RX, adc_MID_RX, adc_MAX_RX, aile_in, CENTER, CENTER);

        // 入力座標周囲４点の値をキャタ移動速度テーブルのどの位置から引いてくれば良いか計算し、
        // k0 左下 k1 右下 k2 左上 k3 右上　に格納する
        uint8_t w = (uint8_t)((v0 >> 8) * 9 + 8 - (v1 >> 8));
        k1 = k3 = w;
        if (v1 < 0x0800) {
            w--;
        }
        k0 = k2 = w;
        if (v0 < 0x0800) {
            k2 = k0 + 9;
            k3 = k1 + 9;
        }
        // 左キャタの移動速度を計算    
        left = cal_move(table_left[k0], table_left[k1], table_left[k2], table_left[k3]);
        // 右キャタの移動速度を計算
        right = cal_move(table_right[k0], table_right[k1], table_right[k2], table_right[k3]);

        // テールランプ（下ＬＥＤ）点灯処理
        uint8_t led = 0;
    
        //if ((data_key[4] == 1) || (data_key[5] == 1)) { // 右側ボタンが押されていれば、
        if (v0 == 0x0400) { // スロットルが中立
            led = 1;
            uint16_t ww;
            // 超信地旋回
            if (v1 > 0x400) {
                v1 -= 0x400;
                ww = (v1 / 11);
                left = (uint8_t)(128 + ww);
            }
            else {
                v1 = 0x400 - v1;
                ww = (v1 / 11);
                left = (uint8_t)(128 - ww);
            }
            right = (uint8_t)((256 - left) & 0xff);
            // 砲身上下動停止
            //cannon = 128;
        }

        if (v0 <= 0x0400) led = 1; // 停止またはバックなら点灯
        if (tail[tail_p] > (v0 + LIMIT_TAIL)) {
            led = 1;
        }
        tail[tail_p ++] = v0;
        if (tail_p == COUNT_TAIL) tail_p = 0;
        if (led == 1) {
            led_rest = LED_CONT;
        }
        if (led_rest > 0) {
            led_rest --;
            led_on = 1;
        }
        else {
            led_on = 0;
        }
    }
    else { // 機動戦闘車 or Tiger16
    	// 入力スロットルを 0 to 255 に正規化 128=ニュートラル
        v = in2std(adc_MIN_RY, adc_MID_RY, adc_MAX_RY, thro_in, CENTER, CENTER);
        v >>= 3;
        if (v > 255) {
            left = 255;
        }
        else {
            left = (uint8_t)v;
        }

    	// 入力エルロンを 0 to 255 に正規化 128=ニュートラル
        v = in2std(adc_MIN_RX, adc_MID_RX, adc_MAX_RX, aile_in, CENTER, CENTER);
        v >>= 3;
        if (v > 255) {
            right = 255;
        }
        else {
            right = (uint8_t)v;
        }
        led_on = 0;
    }
    if ((turn != 128) || (cannon != 128) || (left != 128) || (right != 128)) {
        TMR2 = 0;
    }
}


uint16_t cnt_pow_on = 0; // 電源ボタンが連続して押された回数
uint16_t cnt_pow_off = 0; // 電源ボタンが連続して離された回数


// 電源ボタンの状態を確認
void check_power(uint16_t waitms) {
    if (POW_IN_GetValue()) {
        cnt_pow_off ++;
        if (cnt_pow_off == 0) cnt_pow_off ++;
        cnt_pow_on = 0;
    }
    else {
        cnt_pow_on ++;
        if (cnt_pow_on == 0) cnt_pow_on ++;
        cnt_pow_off = 0;
    }
    if (waitms) __delay_ms(waitms);
}


void clear_buf(void) {
    uint8_t c; 
    for (c=0; c<MAX_BUF; c++) buf[c] = 0;
}


void get_sub_keys(void) {
    uint16_t d;

    d = push1;
    if (d > 3167) {
        data[6] = data[7] = data[8] = data[9] = data[10] = data[11] = 0;
    }
    else if (d > 2052) {
        data[6] = 1;
        data[7] = data[8] = data[9] = data[10] = data[11] = 0;
    }
    else if (d > 1488) {
        data[7] = 1;
        data[6] = data[8] = data[9] = data[10] = data[11] = 0;
    }
    else if (d > 928) {
        data[10] = 1;
        data[6] = data[7] = data[8] = data[9] = data[11] = 0;
    }
    else if (d > 558) {
        data[11] = 1;
        data[6] = data[7] = data[8] = data[9] = data[10] = 0;
    }
    else if (d > 186) {
        data[9] = 1;
        data[6] = data[7] = data[8] = data[10] = data[11] = 0;
    }
    else {
        data[8] = 1;
        data[6] = data[7] = data[9] = data[10] = data[11] = 0;
    }
    
    d = push2;
    if (d > 3171) {
        data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = 0;
    }
    else if (d > 2053) {
        data[5] = 1;
        data[0] = data[1] = data[2] = data[3] = data[4] = 0;
    }
    else if (d > 1675) {
        data[4] = 1;
        data[0] = data[1] = data[2] = data[3] = data[5] = 0;
    }
    else if (d > 1301) {
        data[1] = 1;
        data[0] = data[2] = data[3] = data[4] = data[5] = 0;
    }
    else if (d > 928) {
        data[0] = 1;
        data[1] = data[2] = data[3] = data[4] = data[5] = 0;
    }
    else if (d > 558) {
        data[2] = 1;
        data[0] = data[1] = data[3] = data[4] = data[5] = 0;
    }
    else {
        data[3] = 1;
        data[0] = data[1] = data[2] = data[4] = data[5] = 0;
    }

    data[12] = EXT_GetValue();

    d = trig;
    if (d > 3427) {
        data[13] = data[14] = data[15] = 0;
    }
    else if (d > 2510) {
        data[15] = 1;
        data[13] = data[14] = 0;
    }
    else if (d > 2025) {
        data[13] = 1;
        data[14] = data[15] = 0;
    }
    else {
        data[13] = data[14] = data[15] = 1;
    }
}


// キーの安定化取得
void get_keys(void) {
    get_sub_keys();
    uint8_t k;
    for (k=0; k<NUM_KEYS; k++) {
        if (data[k] == 1) {
            data_cnt_off[k] = 0;
            if (data_cnt_on[k] < KEY_RESPONSE) {
                data_cnt_on[k] ++;
            }
            if (data_cnt_on[k] == KEY_RESPONSE) {
                data_key[k] = 1;
            }
        }
        else {
            data_cnt_on[k] = 0;
            if (data_cnt_off[k] < KEY_RESPONSE) {
                data_cnt_off[k] ++;
            }
            if (data_cnt_off[k] == KEY_RESPONSE) {
                data_key[k] = 0;
            }
        }
        if ((data_key[k] == 1) && (data_pre[k] == 0)) {
            data_push[k] = 1;
        }
        else {
            data_push[k] = 0;
        }
        if (data_key[k] != data_pre[k]) {
            TMR2 = 0;
        }
        data_pre[k] = data_key[k];
    }
}


/////////////////////////////////////////
// キー入力テスト
/////////////////////////////////////////
char test_input_keys(void) {
    char buz = 0;
    get_sub_keys();

    if (data[0]) {
        memcpy(&buf[6], "TRR-U ", 6);
    }
    else if (data[1]) {
        memcpy(&buf[6], "TRR-D ", 6);
    }
    else if (data[2]) {
        memcpy(&buf[6], "TRR-L ", 6);
    }
    else if (data[3]) {
        memcpy(&buf[6], "TRR-R ", 6);
    }
    else if (data[4]) {
        memcpy(&buf[6], "RigtU ", 6);
        buz = LEN_BUZ;
    }
    else if (data[5]) {
        memcpy(&buf[6], "RigtD ", 6);
    }
    else {
        memcpy(&buf[6], "----- ", 6);
    }

    if (data[6]) {
        memcpy(buf, "LeftD ", 6);
    }
    else if (data[7]) {
        memcpy(buf, "LeftU ", 6);
    }
    else if (data[8]) {
        memcpy(buf, "TRL-U ", 6);
    }
    else if (data[9]) {
        memcpy(buf, "TRL-D ", 6);
    }
    else if (data[10]) {
        memcpy(buf, "TRL-L ", 6);
    }
    else if (data[11]) {
        memcpy(buf, "TRL-R ", 6);
    }
    else {
        memcpy(buf, "----- ", 6);
    }

    if (data[12]) {
        buf[12] = 'A';
    }
    else {
        buf[12] = '-';
    }
    buf[13] = ' ';
    if (data[15]) {
        buf[14] = 'L';
    }
    else {
        buf[14] = '-';
    }
    if (data[13]) {
        buf[15] = 'R';
    }
    else {
        buf[15] = '-';
    }
    buf[16] = 0;
    
    LCD_i2C_cmd(0x80);
    LCD_i2C_data(buf);

    main_loop();
    uint16_t v = 0;
    if (left > 128) {
        v = left - 128;
    }
    else {
        v = 128 - left;
    }
    PWM_DutyCycleSet(PWM_GENERATOR_1, v << 4);

    v = 0;
    if (right > 128) {
        v = right - 128;
    }
    else {
        v = 128 - right;
    }
    PWM_DutyCycleSet(PWM_GENERATOR_2, v << 4);
    
    LCD_i2C_cmd(0xC0);
    //sprintf(buf, "%4d%4d%4d%4d", can_in, ch1_in, thro_in, aile_in);
    sprintf(buf, "%4d%4d%4d%4d", turn, cannon, left, right);
    LCD_i2C_data(buf);
    return buz;
}


/////////////////////////////////////////
// 電源ボタンが押されたか調べ
// 押されていれば電源ＯＦＦにする
/////////////////////////////////////////
void check_off(void) {
    check_power(0);
    if (cnt_pow_on > 100) {
        LED_SetLow();
        if (cnt_pow_off < 10) {
            check_power(10);
        }
        POW_OUT_SetLow();
        while (1) ;
    }
}


/////////////////////////////////////////
// ブザー指示されていれば、ブザー鳴らす
/////////////////////////////////////////
void check_buzzer(void) {
    if (cnt_buz == 0) {
        BUZZER_SetLow();
    }
    else {
        cnt_buz --;
        if (cnt_buz & 1) {
            BUZZER_SetHigh();
        }
        else {
            BUZZER_SetLow();
        }
    }
}


/////////////////////////////////////////
// メニュー
// page: 初期ページ
/////////////////////////////////////////
void menu(char page) {
    char page_pre = 0; // 直前のメニューページ
    uint16_t cnt_err = 0; // エラー表示する
    cnt_buz = 0;
    uint32_t sum;
    char buf_err[17];
    char c;
    
    while (1) {
        check_off();
        check_buzzer();
        
        if ((page >= 11) && (page <= 21)) {
            if (cnt_err > 0) {
                LCD_i2C_cmd(0xC0);
                cnt_err --;
                if (cnt_err == 0) {
                    LCD_i2C_data(msg_push_b);
                }
                else {
                    LCD_i2C_data(buf_err);
                }                    
            }
        }
 
        get_keys(); // キー入力
        switch (page) {
            case 1: // キャリブレーション
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data("1. calibration  ");
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_sel_gd);
                }
                if (data_push[7]) { // ↑
                    page = 3;
                }
                if (data_push[6]) { // ↓
                    page = 2;
                }
                if (data_push[4]) { // B
                    page = 11;
                }
                if (data_push[5]) { // A
                    return;
                }
                break;
            case 2: // リバース設定
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data("2. set norm/rev ");
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_sel_gd);
                }
                if (data_push[7]) { // ↑
                    page = 1;
                }
                if (data_push[6]) { // ↓
                    page = 3;
                }
                if (data_push[4]) { // B
                }
                if (data_push[5]) { // A
                    return;
                }
                break;
            case 3: // キー入力テスト
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data("3. test keys    ");
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_sel_gd);
                }
                if (data_push[7]) { // ↑
                    page = 2;
                }
                if (data_push[6]) { // ↓
                    page = 1;
                }
                if (data_push[4]) { // B
                    page = 31;
                }
                if (data_push[5]) { // A
                    return;
                }
                break;
            case 11: // 全ニュートラル 
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data(msg_dont_t);
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_push_b);
                }
                if (data_push[4]) { // B
                    cnt_buz = LEN_BUZ;
                    cnt_err = 0;

                    sum = 0;
                    for (c=0; c<64; c++) {
                        adc_MID_LX = ch1_in;
                        if (adc_MID_LX < 1600) {
                            cnt_err = LEN_ERR;
                            sprintf(buf_err, "MID_LX=%4d<1600", adc_MID_LX);
                        }
                        if (adc_MID_LX > 2400) {
                            cnt_err = LEN_ERR;
                            sprintf(buf_err, "MID_LX=%4d>2400", adc_MID_LX);
                        }
                        sum += adc_MID_LX;
                        __delay_us(100)
                    }
                    adc_MID_LX = (uint16_t)(sum >> 6);

                    sum = 0;
                    for (c=0; c<64; c++) {
                        adc_MID_LY = can_in;
                        if (adc_MID_LY < 1600) {
                            cnt_err = LEN_ERR;
                            sprintf(buf_err, "MID_LY=%4d<1600", adc_MID_LY);
                        }
                        if (adc_MID_LY > 2400) {
                            cnt_err = LEN_ERR;
                            sprintf(buf_err, "MID_LY=%4d>2400", adc_MID_LY);
                        }
                        sum += adc_MID_LY;
                        __delay_us(100)
                    }
                    adc_MID_LY = (uint16_t)(sum >> 6);

                    sum = 0;
                    for (c=0; c<64; c++) {
                        adc_MID_RX = aile_in;
                        if (adc_MID_RX < 1600) {
                            cnt_err = LEN_ERR;
                            sprintf(buf_err, "MID_RX=%4d<1600", adc_MID_RX);
                        }
                        if (adc_MID_RX > 2400) {
                            cnt_err = LEN_ERR;
                            sprintf(buf_err, "MID_RX=%4d>2400", adc_MID_RX);
                        }
                        sum += adc_MID_RX;
                        __delay_us(100)
                    }
                    adc_MID_RX = (uint16_t)(sum >> 6);

                    sum = 0;
                    for (c=0; c<64; c++) {
                        adc_MID_RY = thro_in;
                        if (adc_MID_RY < 1600) {
                            cnt_err = LEN_ERR;
                            sprintf(buf_err, "MID_RY=%4d<1600", adc_MID_RY);
                        }
                        if (adc_MID_RY > 2400) {
                            cnt_err = LEN_ERR;
                            sprintf(buf_err, "MID_RY=%4d>2400", adc_MID_RY);
                        }
                        sum += adc_MID_RY;
                        __delay_us(100)
                    }
                    adc_MID_RY = (uint16_t)(sum >> 6);

                    if (cnt_err == 0) {
                        page = 14;
                    }
                }
                else if (data_push[5]) { // A
                    page = 1;
                }
                break;
            case 14: // LY_MAX
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data("turn L up & ");
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_push_b);
                    cnt_err = 0;
                }
                LCD_i2C_cmd(0x8c);
                sprintf(buf, "%4d", can_in);
                LCD_i2C_data(buf);
                if (data_push[4]) { // B
                    cnt_buz = LEN_BUZ;
                    adc_MAX_LY = can_in;
                    if (adc_MAX_LY < (adc_MID_LY + 512)) {
                        cnt_err = LEN_ERR;
                        sprintf(buf_err, "MAX_LY=%4d<%4d", adc_MAX_LY, (adc_MID_LY + 512));
                    }
                    else {
                        page = 15;
                    }
                }
                else if (data_push[5]) { // A
                    page = 11;
                }
                break;
            case 15: // LY_MIN
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data("turn L down&");
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_push_b);
                    cnt_err = 0;
                }
                LCD_i2C_cmd(0x8c);
                sprintf(buf, "%4d", can_in);
                LCD_i2C_data(buf);
                if (data_push[4]) { // B
                    cnt_buz = LEN_BUZ;
                    adc_MIN_LY = can_in;
                    if (adc_MIN_LY > (adc_MID_LY - 512)) {
                        cnt_err = LEN_ERR;
                        sprintf(buf_err, "MIX_LY=%4d>%4d", adc_MIN_LY, (adc_MID_LY - 512));
                    }
                    else {
                        page = 16;
                    }
                }
                else if (data_push[5]) { // A
                    page = 14;
                }
                break;
            case 16: // LX_MAX
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data("turn L left&");
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_push_b);
                    cnt_err = 0;
                }
                LCD_i2C_cmd(0x8c);
                sprintf(buf, "%4d", ch1_in);
                LCD_i2C_data(buf);
                if (data_push[4]) { // B
                    cnt_buz = LEN_BUZ;
                    adc_MAX_LX = ch1_in;
                    if (adc_MAX_LX < (adc_MID_LX + 512)) {
                        cnt_err = LEN_ERR;
                        sprintf(buf_err, "MAX_LX=%4d<%4d", adc_MAX_LX, (adc_MID_LX + 512));
                    }
                    else {
                        page = 17;
                    }
                }
                else if (data_push[5]) { // A
                    page = 15;
                }
                break;
            case 17: // LX_MIN
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data("turn L right");
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_push_b);
                    cnt_err = 0;
                }
                LCD_i2C_cmd(0x8c);
                sprintf(buf, "%4d", ch1_in);
                LCD_i2C_data(buf);
                if (data_push[4]) { // B
                    cnt_buz = LEN_BUZ;
                    adc_MIN_LX = ch1_in;
                    if (adc_MIN_LX > (adc_MID_LX - 512)) {
                        cnt_err = LEN_ERR;
                        sprintf(buf_err, "MIN_LX=%4d>%4d", adc_MIN_LX, (adc_MID_LX - 518));
                    }
                    else {
                        page = 18;
                    }
                }
                else if (data_push[5]) { // A
                    page = 16;
                }
                break;
            case 18: // RY_MAX
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data("turn R up & ");
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_push_b);
                    cnt_err = 0;
                }
                LCD_i2C_cmd(0x8c);
                sprintf(buf, "%4d", thro_in);
                LCD_i2C_data(buf);
                if (data_push[4]) { // B
                    cnt_buz = LEN_BUZ;
                    adc_MAX_RY = thro_in;
                    if (adc_MAX_RY < (adc_MID_RY + 512)) {
                        cnt_err = LEN_ERR;
                        sprintf(buf_err, "MAX_RY=%4d<%4d", adc_MAX_RY, (adc_MID_RY + 512));
                    }
                    else {
                        page = 19;
                    }
                }
                else if (data_push[5]) { // A
                    page = 17;
                }
                break;
            case 19: // RY_MIN
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data("turn R down&");
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_push_b);
                    cnt_err = 0;
                }
                LCD_i2C_cmd(0x8c);
                sprintf(buf, "%4d", thro_in);
                LCD_i2C_data(buf);
                if (data_push[4]) { // B
                    cnt_buz = LEN_BUZ;
                    adc_MIN_RY = thro_in;
                    if (adc_MIN_RY > (adc_MID_RY - 512)) {
                        cnt_err = LEN_ERR;
                        sprintf(buf_err, "MIN_RY=%4d>%4d", adc_MIN_RY, (adc_MID_RY - 512));
                    }
                    else {
                        page = 20;
                    }
                }
                else if (data_push[5]) { // A
                    page = 18;
                }
                break;
            case 20: // RX_MAX
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data("turn R left&");
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_push_b);
                    cnt_err = 0;
                }
                LCD_i2C_cmd(0x8c);
                sprintf(buf, "%4d", aile_in);
                LCD_i2C_data(buf);
                if (data_push[4]) { // B
                    cnt_buz = LEN_BUZ;
                    adc_MIN_RX = aile_in;
                    if (adc_MIN_RX > (adc_MID_RX - 512)) {
                        cnt_err = LEN_ERR;
                        sprintf(buf_err, "MIN_RX=%4d>%4d", adc_MIN_RX, (adc_MID_RX - 512));
                    }
                    else {
                        page = 21;
                    }
                }
                else if (data_push[5]) { // A
                    page = 19;
                }
                break;
            case 21: // RX_MIN
                if (page_pre != page) {
                    page_pre = page;
                    LCD_i2C_cmd(0x80);
                    LCD_i2C_data("turn R right&");
                    LCD_i2C_cmd(0xC0);
                    LCD_i2C_data(msg_push_b);
                    cnt_err = 0;
                }
                LCD_i2C_cmd(0x8c);
                sprintf(buf, "%4d", aile_in);
                LCD_i2C_data(buf);
                if (data_push[4]) { // B
                    cnt_buz = LEN_BUZ;
                    adc_MAX_RX = aile_in;
                    if (adc_MAX_RX < (adc_MID_RX + 512)) {
                        cnt_err = LEN_ERR;
                        sprintf(buf_err, "MAX_RX=%4d<%4d", adc_MAX_RX, (adc_MID_RX + 512));
                    }
                    else if (save_calibration()) {
                        cnt_err = LEN_ERR;
                        sprintf(buf_err, "** SAVE ERROR **");
                    }
                    else {
                        page = 1;
                    }
                }
                else if (data_push[5]) { // A
                    page = 20;
                }
                break;
            case 31: // キー入力テスト画面
                if (page_pre != page) {
                    page_pre = page;
                }
                cnt_buz = test_input_keys();
                if (data_push[5]) { // A
                    cnt_buz = 0;
                    PWM_DutyCycleSet(PWM_GENERATOR_1, 0);
                    PWM_DutyCycleSet(PWM_GENERATOR_2, 0);
                    page = 3;
                }
                break;
        }
        __delay_ms(10);
    }
}


int main(void)
{
    uint16_t i;
    uint8_t c;
    uint8_t pwm_fire = 0; // 射撃に伴う振動の期間

    // initialize the device
    SYSTEM_Initialize();
    POW_OUT_SetHigh();
    i2c1_driver_driver_open();
    i2c1_driver_initSlaveHardware();
    PWM_Enable();
    PWM_DutyCycleSet(PWM_GENERATOR_1, 0);
    PWM_DutyCycleSet(PWM_GENERATOR_2, 0);
    set_rsv_buf(rsv, 32);
    UART1_SetRxInterruptHandler(TWE_rsv_int);
    TMR2_SetInterruptHandler(int_timer);
    
    DMA_ChannelEnable(DMA_CHANNEL_0);
    DMA_PeripheralAddressSet(DMA_CHANNEL_0, (volatile unsigned int) &ADC1BUF0);
    DMA_StartAddressASet(DMA_CHANNEL_0, (uint16_t)(&thro_in));        

    uint8_t k;
    for (k=0; k<NUM_KEYS; k++) {
        data_cnt_on[k] = 0; // キーが連続でONだった回数
        data_cnt_off[k] = 0; // キーが連続でOFFだった回数
        data_key[k] = 0; // キーの確定状態 1 または 0
        data_pre[k] = 0; // 直前の data_key[] の値
        data_push[k] = 0; // key=1 で pre=0 の場合のみ1 
    }
    for (k=0; k<COUNT_TAIL; k++) {
        tail[k] = 0;
    }
    tail_p = 0;
    wifi_buzzer = 1;

    __delay_ms(100); // I2C バス安定化待ち    
    LCD_i2c_init(34);
    LED_SetHigh();
    
    get_keys();
    tank = 0;
    if (data[11]) { // 左下押しながら
        if (data[12]) { // 左スイッチが下
            tank = 1;
        }
        else {   
            tank = 0;
        }
    }
    else {
        if (data[12]) { // 左スイッチが下
            tank = 3;
        }
        else {   
            tank = 2;
        }
    }
    
    // TWE LITE モジュールと通信
    uint8_t *cp = (uint8_t *)buf;
    cp[0] = 0xdb;
    cp[1] = 0xf3; // 設定取得
    cp[2] = 0x00; // アプリケーションID
    TWE_send(3, cp);
    
    uint8_t n = 0;
    for (i=0; i<30000; i++) {
        n = get_rsv_size();
        if (n == 7) { // 受信できた
            if ((rsv[0] == 0xdb) && (rsv[1] == 0xf3) && (rsv[2] == 0)) {
                LCD_i2C_cmd(0x80);
                sprintf(buf, "APP ID= %02X%02X%02X%02X", rsv[3], rsv[4], rsv[5], rsv[6]);
                LCD_i2C_data(buf);
                LCD_i2C_cmd(0xC0);
                LCD_i2C_data(msg_menu_b);
                sprintf(buf, "%2d", tank);
                LCD_i2C_data(buf);
            }
            clear_rsv_size();
            break;
        }
    }
    if (n == 0) { // 受信失敗
        LCD_i2C_cmd(0x82);
        LCD_i2C_data("TWE LITE RED");
        LCD_i2C_cmd(0xC0);
        char str[] = {0xc2,0xb3,0xbc,0xdd,' ',0xbc,0xaf,0xca,0xdf,0xb2}; // ﾂｳｼﾝ ｼｯﾊﾟｲ
        LCD_i2C_data(str);
        sprintf(buf, " n=%d", n);
        LCD_i2C_data(buf);
        while (POW_IN_GetValue()) {
            BUZZER_SetLow();
            __delay_us(500)
            BUZZER_SetLow();
            __delay_us(500)
        }
        POW_OUT_SetLow();
    }
    else {
        while (cnt_pow_off < 10) {
            check_power(5); // 電源ボタンが離されるのを待つ
            if (cnt_pow_on > 1000) {
                break;
            }
        }
    }
    clear_rsv_size();

    char menu_in = 0;
    while (get_calibration()) { // キャリブレーション結果が保存されていない
        menu(11); // 強制キャリブレーション
        menu_in = 1;
    }

    // 最初からトリガーが押されていないことの確認
    get_keys();
    if (data[13] == 1) { // トリガー押されている    
        while ((data[13] == 1) || (data_key[13] == 1)) {
            k ++;
            if (k & 1) {
                BUZZER_SetHigh();
            }
            else {
                BUZZER_SetLow();
            }
            LCD_i2C_cmd(0x80);
            LCD_i2C_data("TRIGGER BUTTON R");
            LCD_i2C_cmd(0xC0);
            char msg_alert1[] = {' ',' ',' ',0xb5,0xbb,0xda,0xc0,' ',0xcf,0xcf,0xc3,0xde,0xbd,' ',' ',' '}; // ｵｻﾚﾀ ﾏﾏﾃﾞｽ
            LCD_i2C_data(msg_alert1);
            get_keys();
        }    
    }  
    if (data[15] == 1) { // トリガー押されている    
        while ((data[15] == 1) || (data_key[15] == 1)) {
            k ++;
            if (k & 1) {
                BUZZER_SetHigh();
            }
            else {
                BUZZER_SetLow();
            }
            LCD_i2C_cmd(0x80);
            LCD_i2C_data("TRIGGER BUTTON L");
            LCD_i2C_cmd(0xC0);
            char msg_alert1[] = {' ',' ',' ',0xb5,0xbb,0xda,0xc0,' ',0xcf,0xcf,0xc3,0xde,0xbd,' ',' ',' '}; // ｵｻﾚﾀ ﾏﾏﾃﾞｽ
            LCD_i2C_data(msg_alert1);
            get_keys();
        }    
    }  

    BUZZER_SetLow();
    LCD_i2C_cmd(0x01); // クリア
    
    uint8_t id = 0; // 応答ID

    left0 = right0 = left = right = 128; // キャタ移動ニュートラル
    batt_slave = 0;
    wifi_master = wifi_slave = 0;
    num_bb = 0; // 弾数
    temper = 0; // 温度

    // ブザー切り替え
    uint8_t buz0 = 0;
    uint8_t buz1 = 0;
    cnt_buz = 0;
    
    while (1) {
        check_off();
        check_buzzer();
        get_keys();
        main_loop();

        if (tank == 1) { // バトルタンク
        }
        else if (tank == 2) { // 機動戦闘車
            if (data[13]) {
                pwm_fire = 20;
            }
            signed short p1, w1, w2, f1;
            if (pwm_fire) {
                pwm_fire --;
                f1 = 6400;
            }
            else {
                f1 = 0;
            }       
            p1 = (signed short)left;
            p1 -= 128;
            if (p1 < 0) p1 = (-p1);
            p1 <<= 5;
            w1 = p1;
            w2 = p1;
            if (w1 > 6400) w1 = 6400;
            if (w2 > 6400) w2 = 6400;
            if (w1 < 0) w1 = 0;
            if (f1) {
                w1 = f1;
            }
            PWM_DutyCycleSet(PWM_GENERATOR_1, w1);
            PWM_DutyCycleSet(PWM_GENERATOR_2, w1);
        }
        else  if (tank == 3) { // tiger16

        }
        else {// S-TANK
            if (data[14]) {
                pwm_fire = 20;
            }
            else if (data[13]) {
                pwm_fire = 20;
            }
            else if (data[15]) {
                pwm_fire = 60;
            }
            signed short p1, p2, w1, w2, f1;
            if (pwm_fire) {
                pwm_fire --;
                f1 = 6400;
            }
            else {
                f1 = 0;
            }       
            p1 = (signed short)left;
            p1 -= 128;
            if (p1 < 0) p1 = (-p1);
            p1 <<= 5;
            p2 = (signed short)turn;
            p2 -= 128;
            p2 <<= 4;
            w1 = p1 - p2;
            w2 = p1 + p2;
            if (w1 > 6400) w1 = 6400;
            if (w2 > 6400) w2 = 6400;
            if (w1 < 0) w1 = 0;
            if (w2 < 0) w2 = 0;
            if (f1) {
                w1 = w2 = f1;
            }
            PWM_DutyCycleSet(PWM_GENERATOR_1, w1);
            PWM_DutyCycleSet(PWM_GENERATOR_2, w2);
        }
        buz1 = data_key[1]; // TRR-D
        if (buz1) { // 押された
            if (buz0 == 0) {
                wifi_buzzer ^= 1;
            }
        }        
        if (data_key[0]) { // TRR-U
            menu(1); // 通常にメニュー入り
        }
        
        // リモートセンシング
        n = get_rsv_size();
        if (n > 15) { // 受信できた
            // シリアルIDの確認はしない（アプリIDでのフィルターで良しとする）
            char ok = 1;
            if (tank == 1) { // バトルタンク
                //if (rsv[3] != 0x82) ok = 0;
                //if (rsv[4] != 0x02) ok = 0;
                //if (rsv[5] != 0x33) ok = 0;
                //if (rsv[6] != 0xD6) ok = 0;
            }
            else if (tank == 2) { // 機動戦闘車
                //ok = 0;
                HL16 dt;
                dt.L = rsv[17];
                dt.H = rsv[18];
                coil = dt.HL;
            }
            else if (tank == 3) {
                if (ok == 1) {
                    HL16 v;
                    v.H = rsv[17];
                    v.L = rsv[18];
                    num_bb = v.HL; // 弾数
                    temper = rsv[19]; // 温度
                }
            }
            else { // S-TANK
                //if (rsv[3] != 0x82) ok = 0;
                //if (rsv[4] != 0x02) ok = 0;
                //if (rsv[5] != 0x8D) ok = 0;
                //if (rsv[6] != 0x8D) ok = 0;
                if (ok == 1) {
                    num_bb = rsv[17]; // 弾数
                    temper = rsv[18]; // 温度
                }
            }

            if (ok == 1) {
                if (wifi_buzzer) {
                    cnt_buz = LEN_BUZ;
                }
                wifi_slave = rsv[11];
                wifi_master = rsv[14];
                HL16 v;
                v.H = rsv[15];
                v.L = rsv[16];
                batt_slave = v.HL;
            }
            else {
                cnt_buz = 0;
            }
        }
        else {
            cnt_buz = 0;
        }
        clear_rsv_size();

        
        HL16 b; // バッテリー電圧
        uint16_t vv = (uint16_t)(((long)7500 * batt_master) >> 12); // 1mV単位に変換
        b.HL = (vv / 10); // 0.01V単位に変換
        if ((vv % 10) >= 5) b.HL ++; // 四捨五入
        uint16_t nm = b.HL/100;
        uint16_t ns = batt_slave/100;

        LCD_i2C_cmd(0x80);
        if (tank == 1) { // バトルタンク
            sprintf(buf, "%2u.%02uV %3uR ", nm ,b.HL - nm * 100, wifi_master);
            LCD_i2C_data(buf);
            if (wifi_buzzer) {
                LCD_i2C_data("BUZ ");
            }
            else {
                LCD_i2C_data("Boff");
            }
            LCD_i2C_cmd(0xC0);
            sprintf(buf, "%2u.%02uV %3uR %2uBB", ns ,batt_slave - ns * 100, wifi_slave, num_bb);
            LCD_i2C_data(buf);
        }
        else if (tank == 2) { // 機動戦闘車
            uint16_t cd = coil & 4095;
            uint16_t cv = cd/10;
            char info[8];
            info[0] = 0;
            if (coil & 0x8000) {
                strcpy(info, "CHARGE");
            }
            if (coil & 0x2000) {
                strcpy(info, "EMPTY");
            }
            if (coil & 0x4000) {
                strcpy(info, "BREAK");
            }
            sprintf(buf, "%2u.%02uV %5s %3u", nm ,b.HL - nm * 100, info, wifi_master);
            LCD_i2C_data(buf);
            LCD_i2C_cmd(0xC0);
            sprintf(buf, "%2u.%02u %3u.%01uV %3u", ns ,batt_slave - ns * 100, cv, cd - cv * 10, wifi_slave);
            LCD_i2C_data(buf);
        }
        else if (tank == 3) { // tiger16
            sprintf(buf, "%2u.%02uV %3uR%4dC", nm ,b.HL - nm * 100, wifi_master, (signed char)temper);
            LCD_i2C_data(buf);
            LCD_i2C_cmd(0xC0);
            sprintf(buf, "%2u.%02uV %3uR%4uB", ns ,batt_slave - ns * 100, wifi_slave, num_bb);
            LCD_i2C_data(buf);
        }
        else { // S-TANK
            sprintf(buf, "%2u.%02uV %3uR %3uC", nm ,b.HL - nm * 100, wifi_master, temper);
            LCD_i2C_data(buf);
            LCD_i2C_cmd(0xC0);
            if (batt_slave == 65535) {
                sprintf(buf, " *LOW* %3uR %3uB", wifi_slave, num_bb);
                cnt_buz = LEN_BUZ;
            }
            else {
                sprintf(buf, "%2u.%02uV %3uR %3uB", ns ,batt_slave - ns * 100, wifi_slave, num_bb);
            }
            LCD_i2C_data(buf);
        }

        // 送信
        cp[0] = 0x78; // 全子機あて
        cp[1] = 0xA0; // 拡張形式
        cp[2] = id ++; // 応答ID
        cp[3] = 0xFF; // オプション無し

        // bit0  TRR-U（メニュー入り）
        // bit1  TRR-D（BEEP切り替え）
        // bit2  TRR-L（車高ニュートラル）
        // bit3  TRR-R（車高ニュートラル）
        // bit4  RigtU B（ドーザーブレード展開）
        // bit5  RigtD A（ドーザーブレード収納）
        // bit6  LeftD ↑（前照灯明るく）
        // bit7  LeftU ↓（前照灯暗く）
        uint8_t v = 0;
        for (c=7; c>=2; c--) {
            v |= data[c];
            v <<= 1;
        }
        v <<= 1;
        v |= led_on;
        cp[4] = v;

        // bit0  TRL-U キーが押されたら1（俯仰を水平に戻す）
        // bit1  TRL-D キーが押されたら1（俯仰を水平に戻す）
        // bit2  TRL-L が押されたら1（レーザー切り替え）
        // bit3  TRL-R が押されたら1（レーザー切り替え）
        // bit4  カメラ切り替えスイッチが押されたら1
        // bit5  Rが押されたら1（セミオート射撃）
        // bit6  RLが押されたら1（フルオート射撃）
        // bit7  Lが押されたら1（３点バースト射撃）
        v = data[15];
        for (c=14; c>=8; c--) {
            v <<= 1;
            v |= data[c];
        }
        cp[5] = v;
        
        // 元LT（拡張用） 
        cp[6] = 0;
        // 元RT（拡張用）
        cp[7] = 0;
        // 左キャタ
        cp[8] = left;
        // 右キャタ
        cp[9] = right;
        // 砲塔旋回
        cp[10] = turn;
        // 砲身上下
        cp[11] = cannon;
        TWE_send(12, cp);

        buz0 = buz1;
	}
    
    return 1; 
}
/**
 End of File
*/

