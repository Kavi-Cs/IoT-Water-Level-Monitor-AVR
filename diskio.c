#include <avr/io.h>
#include "diskio.h"

// SD Card Pin Configurations (ATmega328P)
#define SD_CS_PIN         PB2
#define SPI_MOSI          PB3
#define SPI_MISO          PB4
#define SPI_SCK           PB5

#define CS_LOW()        PORTB &= ~(1<<SD_CS_PIN)
#define CS_HIGH()       PORTB |= (1<<SD_CS_PIN)

// SPI පද්ධතිය ආරම්භ කිරීම
static void spi_init(void) {
    DDRB |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<SD_CS_PIN);
    DDRB &= ~(1<<SPI_MISO);
    
    // SD Module එක ස්ථාවර කරන්න internal pull-up එක ඔන් කිරීම
    PORTB |= (1<<SPI_MISO); 
    
    CS_HIGH();
    // 10MHz එක 128න් බෙදා වේගය 78kHz දක්වා අඩු කිරීම (Card Initialization වලට අත්‍යවශ්‍යයි)
    SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<SPR0); 
    SPSR = 0;
}

// SPI හරහා ඩේටා බයිට් එකක් යැවීම සහ ලබාගැනීම
static BYTE spi_transfer(BYTE data) {
    SPDR = data;
    while(!(SPSR & (1<<SPIF)));
    return SPDR;
}

// SD කාඩ් එකට කමාන්ඩ් (Commands) යැවීම
static BYTE send_cmd(BYTE cmd, DWORD arg) {
    BYTE n, res;
    
    // ACMD කමාන්ඩ් එකක් නම් ඊට කලින් අනිවාර්යයෙන්ම CMD55 (0x77) යැවිය යුතුය
    if (cmd & 0x80) {
        cmd &= 0x7F;
        res = send_cmd(0x77, 0); // CMD55 යැවීම
        if (res > 1) return res;
    }
    
    CS_HIGH(); spi_transfer(0xFF);
    CS_LOW(); spi_transfer(0xFF);
    spi_transfer(cmd);
    spi_transfer((BYTE)(arg >> 24));
    spi_transfer((BYTE)(arg >> 16));
    spi_transfer((BYTE)(arg >> 8));
    spi_transfer((BYTE)arg);
    
    n = 0x01;
    if (cmd == 0x40) n = 0x95; // CMD0 CRC
    if (cmd == 0x48) n = 0x87; // CMD8 CRC
    spi_transfer(n);
    
    n = 10;
    do { res = spi_transfer(0xFF); } while ((res & 0x80) && --n);
    return res;
}

// SD කාඩ් එක පණගැන්වීම (Initialization)
DSTATUS disk_initialize(void) {
    BYTE n, cmd, ty, ocr[4]; WORD tmr;
    
    spi_init();
    for (n = 10; n; n--) spi_transfer(0xFF);
    ty = 0;
    
    if (send_cmd(0x40, 0) == 1) { // CMD0
        if (send_cmd(0x48, 0x1AA) == 1) { // CMD8
            for (n = 0; n < 4; n++) ocr[n] = spi_transfer(0xFF);
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                for (tmr = 10000; tmr && send_cmd(0x80 | 0x69, 1UL << 30); tmr--); // ACMD41
                if (tmr && send_cmd(0x7A, 0) == 0) { // CMD58
                    for (n = 0; n < 4; n++) ocr[n] = spi_transfer(0xFF);
                    ty = (ocr[0] & 0x40) ? 6 : 2; // SDv2 (HC or SC)
                }
            }
        } else {
            cmd = (send_cmd(0x7A, 0) <= 1) ? 0x69 : 0x41;
            for (tmr = 10000; tmr && send_cmd(0x80 | cmd, 0); tmr--);
            if (tmr || send_cmd(0x50, 512) != 0) ty = 0;
        }
    }
    
    CS_HIGH(); spi_transfer(0xFF);
    
    // HIGH SPEED SWITCH: කාඩ් එක සාර්ථකව පණ ගැන්වුණා නම්, SPI වේගය උපරිම (5MHz) දක්වා වැඩි කිරීම
    if (ty) {
        SPCR = (1<<SPE) | (1<<MSTR); // Prescaler f_osc/4 (2.5MHz)
        SPSR |= (1<<SPI2X);          // Double Speed Mode On (5MHz)
    }
    
    return ty ? 0 : STA_NOINIT;
}

// SD කාඩ් එකෙන් දත්ත කියවීම (Read Sector)
DRESULT disk_readp(BYTE* buff, DWORD lba, UINT ofs, UINT cnt) {
    DRESULT res; BYTE rc; WORD bc;
    if (send_cmd(0x51, lba) == 0) { // CMD17 (Read Single Block)
        WORD tmr = 10000;
        do { rc = spi_transfer(0xFF); } while (rc == 0xFF && --tmr);
        if (rc == 0xFE) {
            bc = 514 - ofs - cnt;
            if (ofs) { do spi_transfer(0xFF); while (--ofs); }
            if (buff) { do *buff++ = spi_transfer(0xFF); while (--cnt); }
            else { do spi_transfer(0xFF); while (--cnt); }
            do spi_transfer(0xFF); while (--bc);
            res = RES_OK;
        } else res = RES_ERROR;
    } else res = RES_ERROR;
    CS_HIGH(); spi_transfer(0xFF);
    return res;
}

// 🎯 ULTIMATE ROBUST FIX: SD කාඩ් එකට දත්ත ලිවීම (Write Sector) - Extended Timeout Included
DRESULT disk_writep(const BYTE* buff, DWORD sc) {
    DRESULT res; 
    UINT bc; 
    DWORD tmr; /* Timeout එක ලොකු කරන්න DWORD කළා */
    static WORD wc; /* Write byte counter */

    res = RES_ERROR;

    if (buff) { /* 1. දත්ත (Data bytes) කාඩ් එකට යැවීම */
        bc = (UINT)sc;
        while (bc && wc) { 
            spi_transfer(*buff++);
            wc--; 
            bc--;
        }
        res = RES_OK;
    } else {
        if (sc) { /* 2. දත්ත ලිවීම ආරම්භ කිරීම (Initiate write process) */
            if (send_cmd(0x58, sc) == 0) { /* WRITE_BLOCK (CMD24) එක යැවීම */
                spi_transfer(0xFF); 
                spi_transfer(0xFE); /* Data token එක යැවීම */
                wc = 512; /* Sector size එක 512 ලෙස සැකසීම */
                res = RES_OK;
            }
        } else { /* 3. දත්ත ලිවීම අවසන් කිරීම (Finalize write process) */
            /* Sector එකේ ඉතිරි වෙන හැම බයිට් එකක්ම 0 වලින් පුරවා අවසන් කිරීම */
            while (wc > 0) {
                spi_transfer(0);
                wc--;
            }
            
            spi_transfer(0xFF); 
            spi_transfer(0xFF); /* Dummy CRC බයිට්ස් 2ක් යැවීම */
            
            /* කාඩ් එකෙන් Data Response Token එක ලැබෙනකන් පොඩ්ඩක් ලූප් එකක බලන් ඉමු */
            BYTE resp = 0xFF;
            for (tmr = 500; tmr; tmr--) {
                resp = spi_transfer(0xFF);
                if (resp != 0xFF) break; 
            }
            
            /* ආපු රිප්ලයි එකෙන් ඩේටා ටික කාඩ් එක පිළිගත්තාද (Data Accepted - 0x05) කියා බැලීම */
            if ((resp & 0x1F) == 0x05) { 
                /* ⚠️ Extended Timeout: කාඩ් එක දත්ත ටික ලියා ඉවර වනතුරු (Busy තත්ත්වය ඉවර වනතුරු) වැඩි වෙලාවක් බලා සිටීම */
                for (tmr = 500000; tmr; tmr--) {
                    if (spi_transfer(0xFF) == 0xFF) { /* MISO ලයින් එක High (0xFF) වුනොත් කාඩ් එක ලියා ඉවරයි */
                        res = RES_OK;
                        break;
                    }
                }
            }
        }
        CS_HIGH(); 
        spi_transfer(0xFF);
    }
    return res;
}